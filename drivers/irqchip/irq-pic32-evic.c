/*
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Copyright (c) 2004 MIPS Inc
 * Author: chris@mips.com
 *
 * Copyright (C) 2004, 06 Ralf Baechle <ralf@linux-mips.org>
 * Copyright (c) 2014 Cristian Birsan <cristian.birsan@microchip.com>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <asm/irq.h>
#include <asm/traps.h>
#include <dt-bindings/interrupt-controller/microchip,pic32mz-evic.h>

#include "irqchip.h"

struct irq_domain *evic_irq_domain;
static struct evic __iomem *evic_base;

static unsigned int *evic_irq_prio;

struct pic_reg {
	u32 val; /* value register*/
	u32 clr; /* clear register */
	u32 set; /* set register */
	u32 inv; /* inv register */
} __packed;

struct evic {
	struct pic_reg intcon;
	struct pic_reg priss;
	struct pic_reg intstat;
	struct pic_reg iptmr;
	struct pic_reg ifs[6];
	u32 reserved1[8];
	struct pic_reg iec[6];
	u32 reserved2[8];
	struct pic_reg ipc[48];
	u32 reserved3[64];
	u32 off[191];
} __packed;

static int get_ext_irq_index(irq_hw_number_t hw);
static void evic_set_ext_irq_polarity(int ext_irq, u32 type);

#define BIT_REG_MASK(bit, reg, mask)		\
	do {					\
		reg = bit/32;			\
		mask = 1 << (bit % 32);		\
	} while (0)

asmlinkage void plat_irq_dispatch(void)
{

	unsigned int irq, hwirq;
	u32 reg, mask;

	hwirq = readl(&evic_base->intstat.val) & 0xFF;

	/* Check if the interrupt was really triggered by hardware*/
	BIT_REG_MASK(hwirq, reg, mask);
	if (likely(readl(&evic_base->ifs[reg].val) &
			readl(&evic_base->iec[reg].val) & mask)) {
		irq = irq_linear_revmap(evic_irq_domain, hwirq);
		do_IRQ(irq);
	} else
		spurious_interrupt();
}

/* mask off an interrupt */
static inline void mask_pic32_irq(struct irq_data *irqd)
{
	u32 reg, mask;
	unsigned int hwirq = irqd_to_hwirq(irqd);

	BIT_REG_MASK(hwirq, reg, mask);
	writel(mask, &evic_base->iec[reg].clr);
}

/* unmask an interrupt */
static inline void unmask_pic32_irq(struct irq_data *irqd)
{
	u32 reg, mask;
	unsigned int hwirq = irqd_to_hwirq(irqd);

	BIT_REG_MASK(hwirq, reg, mask);
	writel(mask, &evic_base->iec[reg].set);
}

/* acknowledge an interrupt */
static void ack_pic32_irq(struct irq_data *irqd)
{
	u32 reg, mask;
	unsigned int hwirq = irqd_to_hwirq(irqd);

	BIT_REG_MASK(hwirq, reg, mask);
	writel(mask, &evic_base->ifs[reg].clr);
}

/* mask off and acknowledge an interrupt */
static inline void mask_ack_pic32_irq(struct irq_data *irqd)
{
	u32 reg, mask;
	unsigned int hwirq = irqd_to_hwirq(irqd);

	BIT_REG_MASK(hwirq, reg, mask);
	writel(mask, &evic_base->iec[reg].clr);
	writel(mask, &evic_base->ifs[reg].clr);
}

static int set_type_pic32_irq(struct irq_data *data, unsigned int flow_type)
{
	int index;

	switch (flow_type) {

	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		__irq_set_handler(data->irq,
				handle_edge_irq, 0, "edge");
		break;

	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		__irq_set_handler(data->irq,
				handle_fasteoi_irq, 0, "fasteoi");
		break;

	default:
		pr_err("Invalid interrupt type !\n");
		return -EINVAL;
	}

	/* set polarity for external interrupts only */
	index = get_ext_irq_index(data->hwirq);
	if (index >= 0)
		evic_set_ext_irq_polarity(index, flow_type);

	return IRQ_SET_MASK_OK;
}

static void pic32_bind_evic_interrupt(int irq, int set)
{
	writel(set, &evic_base->off[irq]);
}

static struct irq_chip pic32_irq_chip = {
	.name = "MICROCHIP EVIC",
	.irq_ack = ack_pic32_irq,
	.irq_mask = mask_pic32_irq,
	.irq_mask_ack = mask_ack_pic32_irq,
	.irq_unmask = unmask_pic32_irq,
	.irq_eoi = ack_pic32_irq,
	.irq_set_type = set_type_pic32_irq,
	.irq_enable = unmask_pic32_irq,
	.irq_disable = mask_pic32_irq,
};

static void evic_set_irq_priority(int irq, int priority)
{
	u32 reg, shift;

	reg = irq / 4;
	shift = (irq % 4) * 8;

	/* set priority */
	writel(INT_MASK << shift, &evic_base->ipc[reg].clr);
	writel(priority << shift, &evic_base->ipc[reg].set);
}

static void evic_set_ext_irq_polarity(int ext_irq, u32 type)
{
	if (WARN_ON(ext_irq >= NR_EXT_IRQS))
		return;
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		writel(1 << ext_irq, &evic_base->intcon.set);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		writel(1 << ext_irq, &evic_base->intcon.clr);
		break;
	default:
		pr_err("Invalid external interrupt polarity !\n");
	}
}

static int get_ext_irq_index(irq_hw_number_t hw)
{
	switch (hw) {
	case EXTERNAL_INTERRUPT_0:
		return 0;
	case EXTERNAL_INTERRUPT_1:
		return 1;
	case EXTERNAL_INTERRUPT_2:
		return 2;
	case EXTERNAL_INTERRUPT_3:
		return 3;
	case EXTERNAL_INTERRUPT_4:
		return 4;
	default:
		return -1;
	}
}

static int evic_intc_map(struct irq_domain *irqd, unsigned int virq,
			irq_hw_number_t hw)
{
	u32 reg, mask;

	irq_set_chip(virq, &pic32_irq_chip);

	BIT_REG_MASK(hw, reg, mask);

	/* disable */
	writel(mask, &evic_base->iec[reg].clr);

	/* clear flag */
	writel(mask, &evic_base->ifs[reg].clr);

	evic_set_irq_priority(hw, evic_irq_prio[hw]);

	return 0;
}

static int evic_irq_domain_xlate(struct irq_domain *d,
				struct device_node *ctrlr,
				const u32 *intspec,
				unsigned int intsize,
				irq_hw_number_t *out_hwirq,
				unsigned int *out_type)
{
	/* Check for number of params */
	if (WARN_ON(intsize < 3))
		return -EINVAL;
	if (WARN_ON(intspec[0] >= NR_IRQS))
		return -EINVAL;
	/* Check for correct priority settings */
	if (WARN_ON((intspec[1] < MICROCHIP_EVIC_MIN_PRIORITY)
			|| (intspec[1] > MICROCHIP_EVIC_MAX_PRIORITY)))
		return -EINVAL;

	*out_hwirq = intspec[0];

	evic_irq_prio[*out_hwirq] = intspec[1];

	*out_type = intspec[2];

	return 0;
}

static const struct irq_domain_ops evic_intc_irq_domain_ops = {
		.map = evic_intc_map,
		.xlate = evic_irq_domain_xlate,
};

#ifdef CONFIG_OF

static int __init
microchip_evic_of_init(struct device_node *node, struct device_node *parent)
{
	struct resource res;

	if (WARN_ON(!node))
		return -ENODEV;

	evic_irq_prio = kzalloc(NR_IRQS * sizeof(*evic_irq_prio),
				GFP_KERNEL);
	if (!evic_irq_prio)
		return -ENOMEM;

	evic_irq_prio[CORE_TIMER_INTERRUPT] = DEFAULT_INT_PRI; /* Default IRQ*/

	if (of_address_to_resource(node, 0, &res))
		panic("Failed to get evic memory range");

	if (request_mem_region(res.start, resource_size(&res),
				res.name) == NULL)
		panic("Failed to request evic memory");

	evic_base = ioremap_nocache(res.start, resource_size(&res));
	if (!evic_base)
		panic("Failed to remap evic memory");

	board_bind_eic_interrupt = &pic32_bind_evic_interrupt;

	evic_irq_domain = irq_domain_add_linear(node, NR_IRQS,
			&evic_intc_irq_domain_ops, NULL);
	if (!evic_irq_domain)
		panic("Failed to add linear irqdomain for EVIC");

	irq_set_default_host(evic_irq_domain);

	return 0;
}

IRQCHIP_DECLARE(microchip_evic, "microchip,evic-v2", microchip_evic_of_init);
#endif
