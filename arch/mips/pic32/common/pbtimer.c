/*
 * PIC32 General Purpose Timer Driver.
 *
 * Copyright (c) 2014, Microchip Technology Inc.
 *      Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * Licensed under GPLv2.
 */

#include <linux/time.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/seq_file.h>

#include <asm/mach-pic32/pic32.h>
#include <asm/mach-pic32/pbtimer.h>

#define PIC32_TIMER_UPPER	0x0200

#if 0
#define dbg_timer(fmt, args...) \
	pr_info("%s:%d::" fmt, __func__, __LINE__, ##args)
#else
#define dbg_timer(fmt, args...)
#endif

#define __timer_lock(__timer, __flags) \
	spin_lock_irqsave(&(__timer)->lock, (__flags))

#define __timer_unlock(__timer, __flags) \
	spin_unlock_irqrestore(&(__timer)->lock, (__flags))

#define __timer_name(__timer) ((__timer)->np->name)

struct timer_ops {
	void (*write_count)(u32 v, struct pic32_pb_timer *tmr);
	void (*write_period)(u32 v, struct pic32_pb_timer *tmr);
	u32 (*read_count)(struct pic32_pb_timer *tmr);
	u32 (*read_period)(struct pic32_pb_timer *tmr);
};

u32 pbt_read_count(struct pic32_pb_timer *timer)
{
	return timer->ops->read_count(timer);
}
EXPORT_SYMBOL(pbt_read_count);

u32 pbt_read_period(struct pic32_pb_timer *timer)
{
	return timer->ops->read_period(timer);
}
EXPORT_SYMBOL(pbt_read_period);

void pbt_write_count(u32 v, struct pic32_pb_timer *timer)
{
	timer->ops->write_count(v, timer);

	/* NOTE: Writing to TMRCNT reg clears prescalers.
	 * So take proper care to restore prescaler once done.
	 */
	if (timer->capability & PIC32_TIMER_PS_FIXUP_COUNT)
		pbt_write_prescaler(timer->save_prescaler, timer);
}
EXPORT_SYMBOL(pbt_write_count);

void pbt_write_period(u32 v, struct pic32_pb_timer *timer)
{
	timer->ops->write_period(v, timer);
}
EXPORT_SYMBOL(pbt_write_period);

static u32 read_pbt_count16(struct pic32_pb_timer *timer)
{
	return pbt_readw(timer, PIC32_TIMER_COUNT);
}

static void write_pbt_count16(u32 count, struct pic32_pb_timer *timer)
{
	pbt_writew(count, timer, PIC32_TIMER_COUNT);
}

static u32 read_pbt_period16(struct pic32_pb_timer *timer)
{
	return pbt_readw(timer, PIC32_TIMER_PERIOD);
}

static void write_pbt_period16(u32 v, struct pic32_pb_timer *timer)
{
	pbt_writew((u16) v, timer, PIC32_TIMER_PERIOD);
}

static u32 read_pbt_count_dual(struct pic32_pb_timer *timer)
{
	u32 v;

	v = pbt_readw(timer, PIC32_TIMER_COUNT + PIC32_TIMER_UPPER); /* upper */
	v <<= 16;
	v |= pbt_readw(timer, PIC32_TIMER_COUNT); /* lower */

	return v;
}

static void write_pbt_count_dual(u32 count, struct pic32_pb_timer *timer)
{
	pbt_writew(count, timer, PIC32_TIMER_COUNT);
	pbt_writew(count >> 16, timer, PIC32_TIMER_COUNT + PIC32_TIMER_UPPER);
}

static u32 read_pbt_period_dual(struct pic32_pb_timer *timer)
{
	u32 v;

	v = pbt_readw(timer, PIC32_TIMER_PERIOD + PIC32_TIMER_UPPER);
	v <<= 16;
	v |= pbt_readw(timer, PIC32_TIMER_PERIOD);

	return v;
}

static void write_pbt_period_dual(u32 v, struct pic32_pb_timer *timer)
{
	pbt_writew(v, timer, PIC32_TIMER_PERIOD);
	pbt_writew(v >> 16, timer, PIC32_TIMER_PERIOD + PIC32_TIMER_UPPER);
}

struct timer_ops pbt_ops16 = {
	.read_count = read_pbt_count16,
	.read_period = read_pbt_period16,
	.write_count = write_pbt_count16,
	.write_period = write_pbt_period16,
};

struct timer_ops pbt_ops32 = {
	.read_count = read_pbt_count_dual,
	.read_period = read_pbt_period_dual,
	.write_count = write_pbt_count_dual,
	.write_period = write_pbt_period_dual,
};

static inline struct pic32_pb_timer *clk_hw_to_pic32_pb_timer(struct clk_hw *hw)
{
	return container_of(hw, struct pic32_pb_timer, hw);
}

static int pbt_clk_set_parent(struct clk_hw *hw, u8 idx)
{
	u16 v;
	struct pic32_pb_timer *timer = clk_hw_to_pic32_pb_timer(hw);

	dbg_timer("(%s, parent_id: %d)\n", __timer_name(timer), idx);

	if (unlikely(idx >= PIC32_TIMER_CLK_SRC_MAX - 1))
		return -EINVAL;

	if (timer->clk_idx)
		idx = timer->clk_idx[idx];

	switch (idx) {
	case 0:
		pbt_writew(TIMER_CS, timer, PIC32_TIMER_CTRL_CLR);
		break;

	default:
		/* extended clock source */
		if (timer_is_type_a(timer) && (timer_version(timer) == 2)) {
			v = pbt_readw(timer, PIC32_TIMER_CTRL);
			idx = (idx >> 1) & TIMER_ECS;
			v &= ~(TIMER_ECS << TIMER_ECS_SHIFT);
			v |= (idx << TIMER_ECS_SHIFT);
			pbt_writew(v, timer, PIC32_TIMER_CTRL);
			dbg_timer("(%s), ecs %d\n", __timer_name(timer), idx);
		}

		pbt_writew(TIMER_CS, timer, PIC32_TIMER_CTRL_SET);
		break;
	}

	return 0;
}

static u8 pbt_clk_get_parent(struct clk_hw *hw)
{
	struct pic32_pb_timer *timer = clk_hw_to_pic32_pb_timer(hw);
	u16 i, v;
	u8 idx;

	v = pbt_readw(timer, PIC32_TIMER_CTRL);

	idx = (v & TIMER_CS) >> TIMER_CS_SHIFT;
	if (timer_is_type_a(timer) && (timer_version(timer) == 2))
		idx |= ((v >> TIMER_ECS_SHIFT) & TIMER_ECS) << 1;

	if (timer->clk_idx == NULL)
		goto done;

	for (i = 0; i < __clk_get_num_parents(hw->clk); i++) {
		if (idx == timer->clk_idx[i]) {
			idx = i;
			break;
		}
	}
done:
	dbg_timer("(%s): get_parent: indx %u\n", __timer_name(timer), idx);
	return idx;
}

static long pbt_clk_round_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long *parent_rate)
{
	struct pic32_pb_timer *timer = clk_hw_to_pic32_pb_timer(hw);
	unsigned int idx, div, delta;
	unsigned int best_delta = -1, best_idx = -1;
	unsigned long new_rate;

	/* rate = tck_rate / prescaler */
	for (idx = 0; idx < timer->num_dividers; idx++) {
		div = timer->dividers[idx];
		delta = abs(rate - (*parent_rate / div));
		if (delta < best_delta) {
			best_idx = idx;
			best_delta = delta;

			if (!delta)
				break;
		}
	}

	new_rate = *parent_rate / timer->dividers[best_idx];
	dbg_timer("%s: parent_rate %lu, target_rate %lu/index %d, rate = %lu\n",
		__timer_name(timer), *parent_rate, rate, best_idx, new_rate);

	return new_rate;
}

static int pbt_clk_set_rate(struct clk_hw *hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct pic32_pb_timer *timer = clk_hw_to_pic32_pb_timer(hw);
	unsigned int idx, div, delta;
	unsigned int best_delta = -1, best_idx = -1;
	unsigned long flags;

	for (idx = 0; idx < timer->num_dividers; idx++) {
		div = timer->dividers[idx];
		delta = abs(rate - (parent_rate / div));
		if (delta < best_delta) {
			best_idx = idx;
			best_delta = delta;
		}
	}

	dbg_timer("%s(%lu, prate %lu), new_rate %lu, delta %d, idx %d\n",
		__timer_name(timer), rate, parent_rate,
		parent_rate / timer->dividers[best_idx],
		best_delta, best_idx);

	__timer_lock(timer, flags);

	/* apply prescaler */
	pbt_write_prescaler(best_idx, timer);
	timer->save_prescaler = best_idx;

	__timer_unlock(timer, flags);

	return 0;
}

static unsigned long pbt_clk_get_rate(struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct pic32_pb_timer *timer = clk_hw_to_pic32_pb_timer(hw);
	unsigned long rate;
	u8 prescaler;

	/* rate = parent_rate / prescaler */
	prescaler = pbt_read_prescaler(timer);

	rate = parent_rate / timer->dividers[prescaler];

	dbg_timer("(%s, parent_rate = %lu) / rate %lu, idx %u\n",
		__timer_name(timer), parent_rate, rate, prescaler);

	return rate;
}

static struct clk_ops pb_timer_clk_ops = {
	.get_parent = pbt_clk_get_parent,
	.set_parent = pbt_clk_set_parent,
	.set_rate = pbt_clk_set_rate,
	.recalc_rate = pbt_clk_get_rate,
	.round_rate = pbt_clk_round_rate,
};

static int pic32_timer_cls_a_prescalers[] = {1, 8, 64, 256};
static int pic32_timer_cls_b_prescalers[] = {1, 2, 4, 8, 16, 32, 64, 256};

static int of_timer_clk_register(struct pic32_pb_timer *timer)
{
	struct clk_init_data init;
	struct device_node *np = timer->np;
	int ret, num, i;
	const char **parents;
	char clk_name[20];

	/* check input clock source count */
	num = of_clk_get_parent_count(np);
	if ((num < 0) || (num > PIC32_TIMER_CLK_SRC_MAX)) {
		pr_err("%s: clk_get_parent_count error\n", np->name);
		return -EINVAL;
	}

	parents = kcalloc(num, sizeof(const char *), GFP_KERNEL);
	if (!parents)
		return -ENOMEM;

	ret = pic32_of_clk_get_parent_indices(np, &timer->clk_idx, num);
	if (ret)
		goto err_parents;

	for (i = 0; i < num; i++)
		parents[i] = of_clk_get_parent_name(np, i);

	snprintf(clk_name, sizeof(clk_name), "%s_clk", np->name);
	init.name = clk_name;
	init.ops = &pb_timer_clk_ops;
	init.flags = CLK_IS_BASIC;
	init.parent_names = parents;
	init.num_parents = num;

	/* init timer parameters */
	timer->hw.init = &init;
	if (timer_is_type_a(timer)) {
		timer->dividers = pic32_timer_cls_a_prescalers;
		timer->num_dividers = ARRAY_SIZE(pic32_timer_cls_a_prescalers);
	} else {
		timer->dividers = pic32_timer_cls_b_prescalers;
		timer->num_dividers = ARRAY_SIZE(pic32_timer_cls_b_prescalers);
	}

	/* register timer clk */
	timer->clk = clk_register(NULL, &timer->hw);

	ret = PTR_ERR_OR_ZERO(timer->clk);
	if (ret) {
		kfree(timer->clk_idx);
		goto err_parents;
	}

err_parents:
	kfree(parents);
	return ret;
}

static LIST_HEAD(pic32_timers_list);

static int timer_match_by_id(struct pic32_pb_timer *timer, void *data)
{
	int id = *((int *)data);
	int ret = (id == timer->id);

	if (ret)
		timer->flags = timer->capability;

	return ret;
}

static int timer_match_by_cap(struct pic32_pb_timer *timer, void *data)
{
	int cap_arg = *((int *)data);

	return ((cap_arg & timer->capability) == cap_arg);
}

static int timer_match_by_node(struct pic32_pb_timer *timer, void *data)
{
	struct device_node *np = (struct device_node *)data;
	int ret = (np == timer->np);

	if (ret)
		timer->flags = timer->capability;

	return ret;
}

static int timer_match_any(struct pic32_pb_timer *timer, void *data)
{
	timer->flags = timer->capability;
	return 1;
}

static struct pic32_pb_timer *pb_timer_request(
	int (*match)(struct pic32_pb_timer *t, void *data), void *data)
{
	struct pic32_pb_timer *timer, *next;
	unsigned long flags;
	int found = 0;

	list_for_each_entry_safe(timer, next, &pic32_timers_list, link) {

		mutex_lock(&timer->mutex);

		found = match(timer, data);

		/* a match found */
		if (found) {

			/* multi-client timer ? */
			if (++timer->usage_count > 1) {
				mutex_unlock(&timer->mutex);
				break;
			}

			clk_enable(timer->clk);

			/* lock timer */
			__timer_lock(timer, flags);

			/* mark busy */
			timer->flags |= PIC32_TIMER_BUSY;

			/* disable timer */
			pbt_disable(timer);

			/* enable 32-bit timer mode, if asked */
			if (timer_is_32bit(timer)) {
				timer->ops = &pbt_ops32;
				pbt_set_32bit(1, timer);
			} else {
				timer->ops = &pbt_ops16;
				pbt_set_32bit(0, timer);
			}

			/* unlock */
			__timer_unlock(timer, flags);

			mutex_unlock(&timer->mutex);
			break;
		}
		mutex_unlock(&timer->mutex);
	}

	dbg_timer("%s:found %s\n", __func__, found ? __timer_name(timer) : "n");
	return found ? timer : ERR_PTR(-EBUSY);
}

struct pic32_pb_timer *pic32_pb_timer_request_specific(int id)
{
	return pb_timer_request(timer_match_by_id, (void *)&id);
}
EXPORT_SYMBOL(pic32_pb_timer_request_specific);

struct pic32_pb_timer *pic32_pb_timer_request_by_cap(int cap)
{
	return pb_timer_request(timer_match_by_cap, (void *)&cap);
}
EXPORT_SYMBOL(pic32_pb_timer_request_by_cap);

struct pic32_pb_timer *pic32_pb_timer_request_by_node(struct device_node *np)
{
	int ret;
	struct of_phandle_args spec;

	ret = of_parse_phandle_with_args(np, "microchip,timer",
		"#timer-cells", 0, &spec);
	if (ret)
		return ERR_PTR(ret);

	dbg_timer("np %s\n", np->name);
	return pb_timer_request(timer_match_by_node, (void *)spec.np);
}
EXPORT_SYMBOL(pic32_pb_timer_request_by_node);

struct pic32_pb_timer *pic32_pb_timer_request_any(void)
{
	return pb_timer_request(timer_match_any, (void *)NULL);
}
EXPORT_SYMBOL(pic32_pb_timer_request_any);

int pic32_pb_timer_free(struct pic32_pb_timer *timer)
{
	if (unlikely(!timer))
		return -EINVAL;

	mutex_lock(&timer->mutex);
	if (WARN_ON(timer->usage_count == 0))
		goto out_unlock;

	if (--timer->usage_count > 0)
		goto out_unlock;

	/* disable timer */
	pbt_disable(timer);

	/* free this instance */
	timer->flags &= ~PIC32_TIMER_BUSY;

	/* 32bit mode enabled? reset 32-bit mode. */
	if (timer_is_32bit(timer)) {
		timer->flags &= ~PIC32_TIMER_32BIT;
		pbt_set_32bit(0, timer);
	}

	if (timer_is_gated(timer)) {
		pbt_disable_gate(timer);
		timer->flags &= ~PIC32_TIMER_GATED;
	}

	timer->flags = 0;

	clk_disable(timer->clk);

out_unlock:
	mutex_unlock(&timer->mutex);
	return 0;
}
EXPORT_SYMBOL(pic32_pb_timer_free);

static int pb_timer_set_timeout(struct pic32_pb_timer *timer,
	u64 timeout_nsec)
{
	unsigned long rate = clk_get_rate(timer->clk);
	u64 max_timeout, timeout;
	u32 period, delta;
	unsigned long flags;

	/* calc max timeout supported by current prescaler & configuration. */
	max_timeout = pb_timer_clk_get_max_timeout(timer, rate);
	if (max_timeout < timeout_nsec) {
		pr_err("pic32-timer: couldn't support requested timeout %llu\n",
			timeout_nsec);
		return -EINVAL;
	}

	/* calc required period from input timeout(nsecs) */
	period = __clk_timeout_ns_to_period(timeout_nsec, rate);

	/* recalc in reverse which will give us the best picture */
	timeout = __clk_period_to_timeout_ns(period, rate);
	delta = abs(timeout - timeout_nsec);

	dbg_timer("%s: target_timeout %llu, timeout %llu, delta %u\n",
		__timer_name(timer), timeout_nsec, timeout, delta);

	__timer_lock(timer, flags);

	/* apply period  & count */
	pbt_write_period(period, timer);
	pbt_write_count(0, timer);

	__timer_unlock(timer, flags);

	return 0;
}

static long __determine_clk_rate_from_timeout(struct pic32_pb_timer *timer,
	u64 timeout_nsec, int algo, struct clk **parent_clk_p)
{
	unsigned long rate;
	u32 period, best_period;
	unsigned int idx, c, best_div_idx = -1;
	unsigned int delta = -1, best_delta = -1;
	unsigned long parent_rate, best_parent_rate;
	u64 max_timeout, timeout, best_timeout = -1;
	struct clk *parent_clk, *best_clk = NULL;

	for (c = 0; (c < __clk_get_num_parents(timer->clk)) && delta; c++) {

		parent_clk = clk_get_parent_by_index(timer->clk, c);
		parent_rate = clk_get_rate(parent_clk);
		dbg_timer("parent_clk %s, parent_rate %lu\n",
			__clk_get_name(parent_clk), parent_rate);

		for (idx = 0; idx < timer->num_dividers; idx++) {

			/* rate = tck_rate / prescaler */
			rate = parent_rate / timer->dividers[idx];

			/* calc max timeout achievable with the prescaler */
			max_timeout = pb_timer_clk_get_max_timeout(timer, rate);
			if (max_timeout < timeout_nsec) {
				dbg_timer("max_timeout %llu < timeout %llu.\n",
					max_timeout, timeout_nsec);
				continue;
			}

			/* calculate timer period from specified timeout */
			period = __clk_timeout_ns_to_period(timeout_nsec, rate);

			/* recalc back to get best approx */
			timeout = __clk_period_to_timeout_ns(period, rate);

			delta = abs(timeout - timeout_nsec);
			if (delta < best_delta) {
				best_delta = delta;
				best_period = period;
				best_div_idx = idx;
				best_timeout = timeout;
				best_parent_rate = parent_rate;
				best_clk = parent_clk;

				if (delta == 0)
					break;
			}

			dbg_timer("rate %lu, count %u/ timeout %llu, delt %u\n",
				rate, period, timeout, delta);

			if (algo == PIC32_TIMER_PRESCALE_HIGH_RES)
				break;

		}
	}

	dbg_timer("parent(%s, %lu), best_idx %d, best_timeout %llu, delta %u\n",
		best_clk ? __clk_get_name(best_clk) : NULL, best_parent_rate,
		best_div_idx, best_timeout, best_delta);

	if (unlikely(best_delta == -1)) {
		pr_err("pic32-%s: timeout %lluns not supported\n",
			__timer_name(timer), timeout_nsec);
		return -EINVAL;
	}

	if (parent_clk_p)
		*parent_clk_p = best_clk;

	return best_parent_rate / timer->dividers[best_div_idx];
}

int pic32_pb_timer_settime(struct pic32_pb_timer *timer,
	unsigned long flags, u64 timeout_nsec)
{
	int ret = 0;
	long rate, algo_flag;
	struct clk *parent_clk;

	mutex_lock(&timer->mutex);
	if (WARN_ON(timer->enable_count > 0))
		goto out_done;

	if (flags & PIC32_TIMER_MAY_RATE) {
		algo_flag = flags & (PIC32_TIMER_PRESCALE_HIGH_RES|
					PIC32_TIMER_PRESCALE_BEST_MATCH);
		rate = __determine_clk_rate_from_timeout(timer,
				timeout_nsec, algo_flag, &parent_clk);
		if (rate <= 0)
			goto out_done;

		if (rate != clk_get_rate(timer->clk)) {
			clk_set_parent(timer->clk, parent_clk);
			clk_set_rate(timer->clk, rate);
		}
	}

	ret = pb_timer_set_timeout(timer, timeout_nsec);
	if (unlikely(ret))
		goto out_done;

	/* just cache new flags; next operations will use these flags. */
	timer->flags |= flags;

out_done:
	mutex_unlock(&timer->mutex);
	return ret;
}
EXPORT_SYMBOL(pic32_pb_timer_settime);

int pic32_pb_timer_gettime(struct pic32_pb_timer *timer, u64 *timeout,
	u64 *elapsed)
{
	unsigned long rate;
	u32 period, count;

	if (!timer)
		return 0;

	mutex_lock(&timer->mutex);

	/* read period and count */
	period = pbt_read_period(timer);
	count = pbt_read_count(timer);

	if (period == 0) {
		if (timeout)
			*timeout = 0;
		goto out_unlock;
	}

	/* calc time elapsed since last match. */
	rate = clk_get_rate(timer->clk);
	if (elapsed)
		*elapsed = __clk_period_to_timeout_ns(count, rate);

	/* calc timeout interval programmed. */
	if (timeout)
		*timeout = __clk_period_to_timeout_ns(period, rate);

out_unlock:
	mutex_unlock(&timer->mutex);
	return 0;
}
EXPORT_SYMBOL(pic32_pb_timer_gettime);

/* pic32_pb_timer_start - starts configured pbtimer.
 *
 * This function can be called from irq context.
 */
int pic32_pb_timer_start(struct pic32_pb_timer *timer)
{
	u8 parent_idx;
	int handle_intr = 0;
	unsigned long flags;

	if (IS_ERR_OR_NULL(timer))
		return -EINVAL;

	dbg_timer("(%s)\n", __timer_name(timer));

	__timer_lock(timer, flags);
	if (++timer->enable_count > 1)
		goto out_unlock;

	/* For continueous timing mode there is no specific case to handle.
	 * But in all other cases install interrupt handler and perform atleast
	 * the followings:
	 * - oneshot mode, disable timer in isr.
	 * - gated mode, *auto* disabled and note count.
	 */

	pbt_disable_gate(timer);

	if (timer_is_gated(timer)) {
		/* Sanity, gating is not allowed with external clk */
		parent_idx = pbt_clk_get_parent(&timer->hw);
		if (parent_idx != 0) {
			pr_err("pb-timer:gating not allowed for ext-clk\n");
			clk_disable(timer->clk);
			--timer->enable_count;
			goto out_unlock;
		}

		pbt_enable_gate(timer);
		handle_intr = 1;
	}

	if (timer_is_oneshot(timer))
		handle_intr = 1;

#ifdef PIC32_TIMER_HANDLE_IRQ
	if (timer->irq) {
		if (handle_intr)
			enable_irq(timer->irq);
		else
			disable_irq_nosync(timer->irq);
	}
#endif
	pbt_enable(timer);

out_unlock:
	__timer_unlock(timer, flags);
	return 0;
}
EXPORT_SYMBOL(pic32_pb_timer_start);

/* pic32_pb_timer_stop - stops running pbtimer.
 *
 * This function can be called from irq context.
 */
int pic32_pb_timer_stop(struct pic32_pb_timer *timer)
{
	unsigned long flags;

	if (IS_ERR_OR_NULL(timer))
		return -EINVAL;

	dbg_timer("(%s)\n", __timer_name(timer));

	__timer_lock(timer, flags);

	if (WARN_ON(timer->enable_count == 0))
		goto out_unlock;

	if (--timer->enable_count > 0)
		goto out_unlock;

#ifdef PIC32_TIMER_HANDLE_IRQ
	if (timer->irq)
		disable_irq_nosync(timer->irq);
#endif
	pbt_disable(timer);

out_unlock:
	__timer_unlock(timer, flags);

	return 0;
}
EXPORT_SYMBOL(pic32_pb_timer_stop);

#ifdef PIC32_TIMER_HANDLE_IRQ
static irqreturn_t pbt_irq_handler(int irq, void *dev_id)
{
	u32 count;
	struct pic32_pb_timer *timer = (struct pic32_pb_timer *)dev_id;

	timer->overrun++;

	/* oneshot mode */
	if (timer_is_oneshot(timer)) {
		pbt_disable(timer);
		pbt_write_period(0, timer);
	}

	/* gated timer */
	if (timer_is_gated(timer)) {
		count = pbt_read_count(timer);
		pbt_disable(timer);
		pr_info("%s:gated accumulation count %08x\n",
			__timer_name(timer), count);
	}

	return IRQ_RETVAL(1);
}
#endif

static int of_pb_timer_setup(struct device_node *np, const void *data)
{
	struct pic32_pb_timer *timer;
	struct pbtimer_platform_data *pdata;
	unsigned long rate;
	int ret;

	pdata = (struct pbtimer_platform_data *)data;

	dbg_timer("np %s\n", np->name);

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (timer == NULL)
		return -ENOMEM;

	timer->np = of_node_get(np);

	timer->base = of_iomap(np, 0);
	if (timer->base == NULL) {
		pr_err("Unable to map regs for %s", np->name);
		ret = -ENOMEM;
		goto out_timer;
	}

	ret = of_property_read_u32(np, "timer-id", &timer->id);
	if (ret) {
		pr_err("%s: Failed to read timer-id\n", np->name);
		goto out_timer;
	}

	timer->capability = pdata->timer_capability;

	spin_lock_init(&timer->lock);
	mutex_init(&timer->mutex);

	dbg_timer("np %s\n", np->name);

	if (of_find_property(np, "microchip,timer-typeA", NULL))
		timer->capability |= PIC32_TIMER_CLASS_A;

	if (of_find_property(np, "microchip,timer-async", NULL))
		timer->capability |= PIC32_TIMER_ASYNC;

	if (of_find_property(np, "microchip,timer-32bit", NULL))
		timer->capability |= PIC32_TIMER_32BIT;

	if (of_find_property(np, "microchip,timer-gated", NULL))
		timer->capability |= PIC32_TIMER_GATED;

	if (of_find_property(np, "microchip,timer-adc", NULL))
		timer->capability |= PIC32_TIMER_TRIG_ADC;

	if (of_find_property(np, "microchip,timer-oneshot", NULL))
		timer->capability |= PIC32_TIMER_ONESHOT;

	if (!of_property_read_u32(np, "microchip,timer-version", &ret))
		timer->capability |= (ret << PIC32_TIMER_VER_SHIFT);

	if (timer_cap_32bit(timer))
		timer->ops = &pbt_ops32;
	else
		timer->ops = &pbt_ops16;

	ret = of_timer_clk_register(timer);
	if (ret) {
		pr_err("%s: memory alloc failed!\n", __func__);
		goto out_unmap;
	}

	/* install interrupt */
	timer->irq = irq_of_parse_and_map(np, 0);

#ifdef PIC32_TIMER_HANDLE_IRQ
	if (timer->irq) {
		ret = request_irq(timer->irq, pbt_irq_handler, IRQF_SHARED,
			np->name, timer);
		if (ret)
			pr_err("%s: irq install failed!\n", __func__);
	}
#endif
	/* prepare clk */
	clk_prepare_enable(timer->clk);

	if (!of_property_read_u32(np, "clock-frequency", (u32 *)&rate)) {
		ret = clk_set_rate(timer->clk, rate);
		if (ret)
			pr_warn("%s: set_rate to %lu failed\n", np->name, rate);
	}

	/* save pre-scaler before disabling the timer */
	timer->save_prescaler = pbt_read_prescaler(timer);
	pbt_disable(timer);

	list_add_tail(&timer->link, &pic32_timers_list);
	return 0;

out_unmap:
	iounmap(timer->base);
out_timer:
	of_node_put(timer->np);
	kfree(timer);
	return ret;
}

#ifdef CONFIG_OF
#define PIC32_TIMER_DEFAULT_FLAGS \
	(PIC32_TIMER_PS_FIXUP_COUNT|PIC32_TIMER_PS_FIXUP_DISABLE)

static const struct pbtimer_platform_data pic32_data[] = {
	{ .timer_capability = PIC32_TIMER_DEFAULT_FLAGS, },
	{ .timer_capability = PIC32_TIMER_DEFAULT_FLAGS|PIC32_TIMER_CLASS_A,},
	{ .timer_capability = PIC32_TIMER_DEFAULT_FLAGS|PIC32_TIMER_TRIG_ADC,},
	{ .timer_capability = PIC32_TIMER_DEFAULT_FLAGS|PIC32_TIMER_32BIT, },
};

static const struct of_device_id pic32_timer_match[] = {
	{ .compatible = "microchip,pic32-timer", .data = &pic32_data[0], },
	{ .compatible = "microchip,pic32-timerA", .data = &pic32_data[1], },
	{ .compatible = "microchip,pic32-timer-adc", .data = &pic32_data[2],},
	{ .compatible = "microchip,pic32-timer32", .data = &pic32_data[3],},
	{},
};
#endif

void __init of_pic32_pb_timer_init(void)
{
	struct device_node *np;
	const struct of_device_id *match;

	for_each_matching_node_and_match(np, pic32_timer_match, &match)
		of_pb_timer_setup(np, match->data);

}

#ifdef CONFIG_DEBUG_FS
static int pb_timer_summary_show(struct seq_file *s, void *data)
{
	int i, p;
	struct clk *c, *parent_clk;
	struct pic32_pb_timer *timer = (struct pic32_pb_timer *)s->private;

	if (!timer)
		return -EINVAL;

	parent_clk = clk_get_parent(timer->clk);
	seq_printf(s, "%s:%lukH, parent:%lukH, div:%u, PRx:0x%08x, TMRx:0x%08x",
		__clk_get_name(timer->clk), clk_get_rate(timer->clk) / 1000,
		clk_get_rate(parent_clk) / 1000,
		timer->dividers[pbt_read_prescaler(timer)],
		pbt_read_period(timer), pbt_read_count(timer));

	seq_puts(s, "\n - - parents - -\n");
	for (i = 0; i < __clk_get_num_parents(timer->clk); i++) {
		p = timer->clk_idx ? timer->clk_idx[i] : i;
		c = clk_get_parent_by_index(timer->clk, i);
		seq_printf(s, "<%d> %s%c\n", p, __clk_get_name(c),
			(c == parent_clk) ? '*' : ' ');
	}

	seq_puts(s, "\n");

	return 0;
}

static int pb_timer_summary_open(struct inode *inode, struct file *file)
{
	return single_open(file, pb_timer_summary_show, inode->i_private);
}

static const struct file_operations pb_timer_summary_fops = {
	.open		= pb_timer_summary_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct debugfs_reg32 pb_timer_regset[] = {
	{ .name = "TxCON", .offset = PIC32_TIMER_CTRL,},
	{ .name = "TxCOUNT", .offset = PIC32_TIMER_COUNT,},
	{ .name = "TxPERIOD", .offset = PIC32_TIMER_PERIOD,},
};

static int debugfs_timeout_set(void *data, u64 val)
{
	struct pic32_pb_timer *timer = (struct pic32_pb_timer *)data;

	return pic32_pb_timer_settime(timer, PIC32_TIMER_MAY_RATE,
		(u64)(val * NSEC_PER_MSEC));
}

static int debugfs_timeout_get(void *data, u64 *val)
{
	pic32_pb_timer_gettime((struct pic32_pb_timer *)data, val, NULL);
	do_div(*val, NSEC_PER_MSEC);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_timeout,
	debugfs_timeout_get, debugfs_timeout_set, "%llu\n");

static int debugfs_enable_set(void *data, u64 val)
{
	int ret;
	struct pic32_pb_timer *timer = (struct pic32_pb_timer *)data;

	if (val)
		ret = pic32_pb_timer_start(timer);
	else
		ret = pic32_pb_timer_stop(timer);

	return ret;
}

static int debugfs_enable_get(void *data, u64 *val)
{
	*val = pbt_is_enabled((struct pic32_pb_timer *)data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_enable,
	debugfs_enable_get, debugfs_enable_set, "%llu\n");

static int debugfs_request_set(void *data, u64 val)
{
	struct pic32_pb_timer *timer = (struct pic32_pb_timer *)data;

	if (val)
		return (pic32_pb_timer_request_specific(timer->id) != timer);

	pic32_pb_timer_free(timer);

	return 0;
}

static int debugfs_is_ready(void *data, u64 *val)
{
	*val = timer_is_busy((struct pic32_pb_timer *)data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_request,
	debugfs_is_ready, debugfs_request_set, "%llu\n");

#define DEFINE_TIMER_DEBUGFS_RW(_pfx)					\
static int debugfs_set_##_pfx(void *data, u64 val)			\
{									\
	pbt_write_##_pfx((u32) val, (struct pic32_pb_timer *)data);	\
	return 0;							\
}									\
									\
static int debugfs_get_##_pfx(void *data, u64 *val)			\
{									\
	*val = pbt_read_##_pfx((struct pic32_pb_timer *)data);		\
	return 0;							\
}									\
									\
DEFINE_SIMPLE_ATTRIBUTE(fops_##_pfx,					\
	debugfs_get_##_pfx, debugfs_set_##_pfx, "%llu\n")		\

DEFINE_TIMER_DEBUGFS_RW(count);
DEFINE_TIMER_DEBUGFS_RW(period);
DEFINE_TIMER_DEBUGFS_RW(prescaler);

static ssize_t pb_timer_show_cap(struct file *file,
		char __user *userbuf, size_t count, loff_t *ppos)

{
	u32 len = 0, cap;
	char buf[64];

	cap = *(u32 *)file->private_data;

	len = snprintf(buf, sizeof(buf), "%s%s%s%s%s%s (0x%08x)\n",
		(cap & PIC32_TIMER_CLASS_A) ? "A" : "B",
		(cap & PIC32_TIMER_32BIT) ? "|32B" : "|16B",
		(cap & PIC32_TIMER_ASYNC) ? "|ASYN" : "|SYNC",
		(cap & PIC32_TIMER_CLK_EXT) ? "|EXT" : " ",
		(cap & PIC32_TIMER_GATED) ? "|GAT" : " ",
		(cap & PIC32_TIMER_TRIG_ADC) ? "|ADC" : " ",
		cap);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static const struct file_operations pb_timer_cap_fops = {
	.open		= simple_open,
	.read		= pb_timer_show_cap,
	.llseek		= default_llseek,
	.owner		= THIS_MODULE,
};

/* caller must hold prepare_lock */
static int pb_timer_debug_create_one(struct pic32_pb_timer *timer,
	struct dentry *pdentry)
{
	struct dentry *d;
	int ret = -ENOMEM;

	if (!timer || !pdentry) {
		ret = -EINVAL;
		goto out;
	}

	d = debugfs_create_dir(__timer_name(timer), pdentry);
	if (!d)
		goto out;

	timer->dentry = d;

	d = debugfs_create_u32("id", S_IRUGO, timer->dentry,
			(u32 *)&timer->id);
	if (!d)
		goto err_out;

	d = debugfs_create_u32_array("prescalers", S_IRUGO, timer->dentry,
			(u32 *)timer->dividers, timer->num_dividers);
	if (!d)
		goto err_out;

	d = debugfs_create_x32("regbase", S_IRUGO, timer->dentry,
			(u32 *)&timer->base);
	if (!d)
		goto err_out;

	timer->regs.base = timer->base;
	timer->regs.regs = pb_timer_regset;
	timer->regs.nregs = ARRAY_SIZE(pb_timer_regset);

	d = debugfs_create_regset32("regdump", S_IRUGO, timer->dentry,
			&timer->regs);
	if (!d)
		goto err_out;

	d = debugfs_create_file("summary", S_IRUGO, timer->dentry, timer,
			&pb_timer_summary_fops);
	if (!d)
		goto err_out;


	d = debugfs_create_file("capability", S_IRUGO, timer->dentry,
			&timer->capability, &pb_timer_cap_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("flags", S_IRUGO, timer->dentry, &timer->flags,
			&pb_timer_cap_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("request", S_IALLUGO, timer->dentry, timer,
			&fops_request);
	if (!d)
		goto err_out;


	d = debugfs_create_file("count", S_IALLUGO, timer->dentry, timer,
			&fops_count);
	if (!d)
		goto err_out;

	d = debugfs_create_file("period", S_IALLUGO, timer->dentry, timer,
			&fops_period);
	if (!d)
		goto err_out;

	d = debugfs_create_file("prescaler", S_IALLUGO, timer->dentry, timer,
			&fops_prescaler);
	if (!d)
		goto err_out;


	d = debugfs_create_file("timeout_msec", S_IALLUGO, timer->dentry, timer,
			&fops_timeout);
	if (!d)
		goto err_out;

	d = debugfs_create_file("enable", S_IALLUGO, timer->dentry, timer,
			&fops_enable);
	if (!d)
		goto err_out;
	return 0;

err_out:
	debugfs_remove_recursive(timer->dentry);
	timer->dentry = NULL;
out:
	return ret;
}

static struct dentry *pb_timer_debug_root;

static int pb_timer_debug_init(void)
{
	struct pic32_pb_timer *timer, *next;

	pb_timer_debug_root = debugfs_create_dir("pbtimer", NULL);
	if (IS_ERR(pb_timer_debug_root))
		return PTR_ERR(pb_timer_debug_root);

	list_for_each_entry_safe(timer, next, &pic32_timers_list, link)
		pb_timer_debug_create_one(timer, pb_timer_debug_root);

	return 0;
}
late_initcall(pb_timer_debug_init);

#endif /* CONFIG_DEBUG_FS */
