/*
 * Joshua Henderson <joshua.henderson@microchip.com>
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
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/init.h>
#include <linux/irqchip/pic32-evic.h>
#include <linux/of.h>

#include <asm/time.h>
#include <asm/mach-pic32/pbtimer.h>
#include <asm/mach-pic32/ocmp.h>

#include "pic32mzda.h"

unsigned int get_c0_compare_int(void)
{
	return pic32_get_c0_compare_int();
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
