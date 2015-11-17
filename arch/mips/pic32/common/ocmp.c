/*
 * PIC32 General Purpose Output Compare Driver.
 *
 * Purna Chandra Mandal <purna.mandal@microchip.com>
 * Copyright (C) 2015 Microchip Technology Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <asm/mips-boards/generic.h>
#include <asm/mach-pic32/ocmp.h>
#include <asm/mach-pic32/pbtimer.h>
#include <asm/mach-pic32/pic32.h>

struct pic32_ocmp {
	/* provided by OF */
	unsigned int id;
	struct device_node *np;
	void __iomem *regs;
	unsigned int irq;
	unsigned long capability;
	/* internal */
	struct list_head link;
	struct mutex _mutex; /* multi client*/
	spinlock_t lock;
	unsigned long flags;
	struct pic32_pb_timer *tmr;
};

/* Output Compare Registers */
#define OCCON		0x00
#define OCR		0x10
#define OCRS		0x20

/* Output Compare Control Register fields */
#define OCCON_OCM		0x07    /* select OC operating mode */
#define OCCON_OCM_SHIFT		0
#define OCCON_ON		BIT(15) /* enable/disable module */
#define OCCON_OC32		BIT(5)  /* 32-bit dual module */
#define OCCON_OCFLT		BIT(4)  /* detect PWM fault */
#define OCCON_OCTSEL		BIT(3)  /* select Time source(TimerY) */

/* OC capability/flags */
#define PIC32_OC_BUSY		BIT(0)
#define PIC32_OC_32BIT		BIT(1)
#define PIC32_OC_PWM		BIT(2)
#define PIC32_OC_TRIG_ADC	BIT(3)

#define oc_cap_32bit(__oc)	((__oc)->capability & PIC32_OC_32BIT)
#define oc_is_32bit(__oc)	((__oc)->flags & PIC32_OC_32BIT)
#define oc_is_busy(__oc)	((__oc)->flags & PIC32_OC_BUSY)
#define oc_set_busy(__oc)	((__oc)->flags |= PIC32_OC_BUSY)
#define oc_clr_busy(__oc)	((__oc)->flags &= ~PIC32_OC_BUSY)

#ifdef DEBUG
#define dbg_oc(fmt, args...) \
	pr_info("%s:%d::" fmt, __func__, __LINE__, ##args)
#else
#define dbg_oc(fmt, args...)
#endif

static inline void oc_writew(u16 v, struct pic32_ocmp *oc, unsigned long offset)
{
	__raw_writew(v, oc->regs + offset);
}

static inline u16 oc_readw(struct pic32_ocmp *oc, unsigned long offset)
{
	return __raw_readw(oc->regs + offset);
}

static inline void oc_writel(u32 v, struct pic32_ocmp *oc, unsigned long offset)
{
	__raw_writel(v, oc->regs + offset);
}

static inline u32 oc_readl(struct pic32_ocmp *oc, unsigned long offset)
{
	return __raw_readl(oc->regs + offset);
}

static inline void oc_disable(struct pic32_ocmp *oc)
{
	oc_writel(OCCON_ON, oc, PIC32_CLR(OCCON));
}

static inline void oc_enable(struct pic32_ocmp *oc)
{
	oc_writel(OCCON_ON, oc, PIC32_SET(OCCON));
}

static inline void oc_set_32bit(u8 v, struct pic32_ocmp *oc)
{
	if (v)
		oc_writel(OCCON_OC32, oc, PIC32_SET(OCCON));
	else
		oc_writel(OCCON_OC32, oc, PIC32_CLR(OCCON));
}

static inline void oc_set_timer_id(u8 v, struct pic32_ocmp *oc)
{
	if (v & 1) /* TimerY is time-base */
		oc_writel(OCCON_OCTSEL, oc, PIC32_SET(OCCON));
	else
		oc_writel(OCCON_OCTSEL, oc, PIC32_CLR(OCCON));
}

static inline void oc_set_mode(u8 mode, struct pic32_ocmp *oc)
{
	u32 v;

	v = oc_readl(oc, OCCON);
	v &= ~(OCCON_OCM << OCCON_OCM_SHIFT);
	v |= (mode << OCCON_OCM_SHIFT);
	if (v != oc_readl(oc, OCCON))
		oc_writel(v, oc, OCCON);
}

static inline u8 oc_get_mode(struct pic32_ocmp *oc)
{
	u32 v;

	v = oc_readl(oc, OCCON);
	return (v >> OCCON_OCM_SHIFT) & OCCON_OCM;
}

static inline int oc_is_fault(struct pic32_ocmp *oc)
{
	return !!(oc_readl(oc, OCCON) & OCCON_OCFLT);
}

static inline u32 oc_read_comp(struct pic32_ocmp *oc)
{
	return oc_readl(oc, OCR);
}

static inline u32 oc_read_sec_comp(struct pic32_ocmp *oc)
{
	return oc_readl(oc, OCRS);
}

static inline void oc_write_comp(u32 v, struct pic32_ocmp *oc)
{
	oc_writel(v, oc, OCR);
}

static inline void oc_write_sec_comp(u32 v, struct pic32_ocmp *oc)
{
	oc_writel(v, oc, OCRS);
}

#define __oc_get_name(__oc)	((__oc)->np->name)

#define __oc_lock(__oc, __flags) \
	spin_lock_irqsave(&(__oc)->lock, (__flags))

#define __oc_unlock(__oc, __flags) \
	spin_unlock_irqrestore(&(__oc)->lock, (__flags))

static int __oc_match_by_id(struct pic32_ocmp *oc, void *data)
{
	int id = *((int *)data);
	int ret = (id == oc->id);

	if (ret)
		oc->flags = oc->capability;

	return ret;
}

static int __oc_match_by_cap(struct pic32_ocmp *oc, void *data)
{
	int cap = *((int *)data);
	int ret = ((cap & oc->capability) == cap);

	if (ret)
		oc->flags = cap;

	return ret;
}

static int __oc_match_by_node(struct pic32_ocmp *oc, void *data)
{
	struct device_node *np = (struct device_node *)data;
	int ret = (np == oc->np);

	dbg_oc("%s, np %s\n", __oc_get_name(oc), np->name);
	if (ret)
		oc->flags = oc->capability;

	return ret;
}

static int __oc_match_any(struct pic32_ocmp *oc, void *data)
{
	oc->flags = oc->capability;
	return 1;
}

static LIST_HEAD(pic32_oc_list);

static struct pic32_ocmp *oc_request(
	int (*match)(struct pic32_ocmp *, void *), void *data)
{
	struct pic32_ocmp *oc, *next;
	unsigned long flags;
	int found = 0;

	list_for_each_entry_safe(oc, next, &pic32_oc_list, link) {
		mutex_lock(&oc->_mutex);

		/* ignore, if busy */
		if (oc_is_busy(oc)) {
			mutex_unlock(&oc->_mutex);
			continue;
		}

		found = match(oc, data);
		if (!found) {
			mutex_unlock(&oc->_mutex);
			continue;
		}

		/* match found */

		/* lock OC */
		__oc_lock(oc, flags);

		/* mark in-use */
		oc_set_busy(oc);

		/* disable OC */
		oc_disable(oc);

		/* enable 32-bit OC mode, if asked */
		if (oc_is_32bit(oc))
			oc_set_32bit(1, oc);
		else
			oc_set_32bit(0, oc);

		/* unlock */
		__oc_unlock(oc, flags);
		mutex_unlock(&oc->_mutex);
		break;
	}

	dbg_oc("%s: found: %s\n", __func__, found ? __oc_get_name(oc) : "no");
	return found ? oc : ERR_PTR(-EBUSY);
}

struct pic32_ocmp *pic32_oc_request_specific(int id)
{
	return oc_request(__oc_match_by_id, (void *)&id);
}
EXPORT_SYMBOL(pic32_oc_request_specific);

struct pic32_ocmp *pic32_oc_request_by_cap(int cap)
{
	return oc_request(__oc_match_by_cap, (void *)&cap);
}
EXPORT_SYMBOL(pic32_oc_request_by_cap);

struct pic32_ocmp *pic32_oc_request_by_node(struct device_node *np)
{
	int rc;
	const char *prop = "microchip,ocmp";
	struct of_phandle_args spec;

	rc = of_parse_phandle_with_args(np, prop, "#oc-cells", 0, &spec);
	if (rc)
		return ERR_PTR(rc);

	dbg_oc("np %s\n", np->name);
	return oc_request(__oc_match_by_node, (void *)spec.np);
}
EXPORT_SYMBOL(pic32_oc_request_by_node);

struct pic32_ocmp *pic32_oc_request_any(void)
{
	return oc_request(__oc_match_any, (void *)NULL);
}
EXPORT_SYMBOL(pic32_oc_request_any);

int pic32_oc_free(struct pic32_ocmp *oc)
{
	if (unlikely(!oc))
		return -EINVAL;

	mutex_lock(&oc->_mutex);

	/* disable oc */
	oc_disable(oc);
	oc_set_mode(PIC32_OCM_NONE, oc);

	/* free this instance */
	oc_clr_busy(oc);

	oc->flags = 0;

	mutex_unlock(&oc->_mutex);

	return 0;
}
EXPORT_SYMBOL(pic32_oc_free);

/* pic32_oc_set_time_base - set timebase.
 *
 * Here timebase is pic32 general purpose timer whose running counter
 * will be compared against OC comp & sec-comp register.
 */
int pic32_oc_set_time_base(struct pic32_ocmp *oc, struct pic32_pb_timer *timer)
{
	mutex_lock(&oc->_mutex);

	oc->tmr = timer;
	/* set time src bit of OCCON depending on timer->id */
	oc_set_timer_id(timer->id, oc);

	mutex_unlock(&oc->_mutex);

	return 0;
}
EXPORT_SYMBOL(pic32_oc_set_time_base);

/* pic32_oc_settime - set timeout (nanosecs) & mode of operation
 */
int pic32_oc_settime(struct pic32_ocmp *oc, int mode, uint64_t timeout_nsec)
{
	u32 dty;
	unsigned long rate;

	if (unlikely(mode >= PIC32_OCM_MAX)) {
		pr_err("%s: invalid mode %d\n", __oc_get_name(oc), mode);
		return -EINVAL;
	}

	if (unlikely(!oc->tmr)) {
		pr_err("%s: time base is not set\n", __oc_get_name(oc));
		return -EPERM;
	}

	/* get timer rate */
	rate = pic32_pb_timer_get_rate(oc->tmr);

	/* convert timeout(nsecs) to timer count */
	dty = __clk_timeout_ns_to_period(timeout_nsec, rate);

	mutex_lock(&oc->_mutex);
	oc_set_mode(mode, oc);

	switch (mode) {
	case PIC32_OCM_NONE:
		break;
	case PIC32_OCM_TRANSITION_HIGH:
	case PIC32_OCM_TRANSITION_LOW:
	case PIC32_OCM_TRANSITION_TOGGLE:
		oc_write_comp(dty, oc);
		break;
	case PIC32_OCM_PULSE_CONTINUOUS:
	case PIC32_OCM_PULSE_ONE:
		oc_write_comp(0, oc);
		oc_write_sec_comp(dty, oc);
		break;
	case PIC32_OCM_PWM_DISABLED_FAULT:
	case PIC32_OCM_PWM_ENABLED_FAULT:
		oc_write_comp(0, oc);
		oc_write_sec_comp(dty, oc);
		break;
	default:
		pr_err("oc: incorrect OC mode requested\n");
		break;
	}

	mutex_unlock(&oc->_mutex);
	return 0;
}
EXPORT_SYMBOL(pic32_oc_settime);

/* pic32_oc_gettime - get timeout (nanosecs) programmed.
 */
int pic32_oc_gettime(struct pic32_ocmp *oc,
		     u64 *comp_p,
		     u64 *sec_comp_p)
{
	unsigned long rate;
	u32 count2, count;

	if (!oc)
		return -EINVAL;

	mutex_lock(&oc->_mutex);

	/* get timer rate */
	rate = pic32_pb_timer_get_rate(oc->tmr);

	dbg_oc("(%s)\n", __oc_get_name(oc));

	/* read count2 and count */
	count2 = oc_read_sec_comp(oc);
	count = oc_read_comp(oc);

	if (count2 == 0) {
		*sec_comp_p = 0;
		goto out_unlock;
	}

	/* calc time elapsed since last match. */
	if (comp_p)
		*comp_p = __clk_period_to_timeout_ns(count, rate);

	/* calc timeout interval programmed. */
	if (sec_comp_p)
		*sec_comp_p = __clk_period_to_timeout_ns(count2, rate);

	dbg_oc("count %x, count2 %x", count, count2);
out_unlock:
	mutex_unlock(&oc->_mutex);
	return 0;
}
EXPORT_SYMBOL(pic32_oc_gettime);

/* pic32_oc_start - starts configured OC.
 *
 * This function can be called from irq context.
 */
int pic32_oc_start(struct pic32_ocmp *oc)
{
	unsigned long flags;

	if (IS_ERR_OR_NULL(oc))
		return -EINVAL;

	dbg_oc("(%s)\n", __oc_get_name(oc));

	__oc_lock(oc, flags);
	oc_enable(oc);
	__oc_unlock(oc, flags);

	return 0;
}
EXPORT_SYMBOL(pic32_oc_start);

/* pic32_oc_stop - stop running OC.
 *
 * This function can be called from irq context.
 */
int pic32_oc_stop(struct pic32_ocmp *oc)
{
	unsigned long flags;

	if (IS_ERR_OR_NULL(oc))
		return -EINVAL;

	dbg_oc("(%s)\n", __oc_get_name(oc));

	__oc_lock(oc, flags);
	oc_disable(oc);
	__oc_unlock(oc, flags);

	return 0;
}
EXPORT_SYMBOL(pic32_oc_stop);

int pic32_oc_get_irq(struct pic32_ocmp *oc)
{
	if (IS_ERR_OR_NULL(oc))
		return -EINVAL;

	return oc->irq;
}
EXPORT_SYMBOL(pic32_oc_get_irq);

static int of_oc_setup_one(struct device_node *np, const void *data)
{
	struct pic32_ocmp *oc;
	int ret;

	oc = kzalloc(sizeof(*oc), GFP_KERNEL);
	if (!oc)
		return -ENOMEM;

	oc->np = of_node_get(np);

	oc->regs = of_iomap(np, 0);
	if (!oc->regs) {
		pr_err("Unable to map regs for %s", np->name);
		ret = -ENOMEM;
		goto out_oc;
	}

	ret = of_property_read_u32(np, "oc-id", &oc->id);
	if (ret) {
		pr_err("%s: Failed to read oc-id\n", np->name);
		goto out_oc;
	}

	spin_lock_init(&oc->lock);
	mutex_init(&oc->_mutex);

	/* disable OC */
	oc_disable(oc);

	if (of_find_property(np, "microchip,oc-32bit", NULL))
		oc->capability |= PIC32_OC_32BIT;

	if (of_find_property(np, "microchip,oc-pwm", NULL))
		oc->capability |= PIC32_OC_PWM;

	if (of_find_property(np, "microchip,oc-adc", NULL))
		oc->capability |= PIC32_OC_TRIG_ADC;

	/* install interrupt */
	oc->irq = irq_of_parse_and_map(np, 0);

	list_add_tail(&oc->link, &pic32_oc_list);
	return 0;

out_oc:
	of_node_put(oc->np);
	kfree(oc);
	return ret;
}

static const struct of_device_id pic32_oc_match[] = {
	{ .compatible = "microchip,pic32-ocmp",},
	{},
};

void __init of_pic32_oc_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;

	for_each_matching_node_and_match(np, pic32_oc_match, &match)
		of_oc_setup_one(np, match->data);
}
