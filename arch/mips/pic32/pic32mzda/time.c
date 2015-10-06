/*
 * Joshua Henderson, joshua.henderson@microchip.com
 * Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
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
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <asm/irq.h>
#include <asm/cpu.h>
#include <asm/time.h>

#include <asm/mips-boards/generic.h>
#include <asm/mach-pic32/pbtimer.h>
#include <asm/mach-pic32/ocmp.h>

#include "common.h"
#include <dt-bindings/interrupt-controller/microchip,pic32mz-evic.h>

unsigned int get_c0_compare_int(void)
{
	int virq;

	virq = irq_create_mapping(evic_irq_domain, CORE_TIMER_INTERRUPT);
	irq_set_irq_type(virq, IRQ_TYPE_EDGE_RISING);
	return virq;
}

void __init plat_time_init(void)
{
	struct clk *clk;

	of_clk_init(NULL);
	clk = clk_get_sys("cpu_clk", NULL);
	if (IS_ERR(clk)) {
		clk = clk_get_sys("pb7_clk", NULL);
		if (IS_ERR(clk))
			panic("unable to get CPU clock, err=%ld", PTR_ERR(clk));
	}

	clk_prepare_enable(clk);
	pr_info("CPU Clock: %ldMHz\n", clk_get_rate(clk) / 1000000);
	mips_hpt_frequency = clk_get_rate(clk) / 2;

	of_pic32_pb_timer_init();
	of_pic32_oc_init();
	clocksource_of_init();
}
