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
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/mc146818rtc.h>
#include <linux/clkdev.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/hardirq.h>
#include <asm/irq.h>
#include <asm/div64.h>
#include <asm/cpu.h>
#include <asm/time.h>

#include <asm/mips-boards/generic.h>
#include <asm/mach-pic32/pic32.h>
#include <asm/mach-pic32/pbtimer.h>

#include <asm/prom.h>
#include <dt-bindings/interrupt-controller/microchip,pic32mz-evic.h>

#include "common.h"

#define ICLK_MASK   0x00000080
#define PLLDIV_MASK 0x00000007
#define CUROSC_MASK 0x00000007
#define PLLMUL_MASK 0x0000007F
#define PB_MASK     0x00000007
#define FRC1        0
#define FRC2        7
#define SPLL        1
#define POSC        2
#define FRC_CLK     8000000

#define PIC32_POSC_FREQ         24000000

#define OSCCON         0x0000
#define SPLLCON        0x0020
#define PB1DIV         0x0140

extern void __init of_pic32_oc_init(void);

u32 pic32_get_cpuclk(void)
{
	u32 osc_freq = 0;
	u32 pllclk;
	u32 frcdivn;
	u32 osccon;
	u32 spllcon;
	int curr_osc;

	u32 plliclk;
	u32 pllidiv;
	u32 pllodiv;
	u32 pllmult;
	u32 frcdiv;

	void __iomem *osc_base = ioremap(PIC32_BASE_OSC, 0x200);

	osccon = __raw_readl(osc_base + OSCCON);
	spllcon = __raw_readl(osc_base + SPLLCON);

	plliclk = (spllcon & ICLK_MASK);
	pllidiv = ((spllcon >> 8) & PLLDIV_MASK) + 1;
	pllodiv = ((spllcon >> 24) & PLLDIV_MASK);
	pllmult = ((spllcon >> 16) & PLLMUL_MASK) + 1;
	frcdiv = ((osccon >> 24) & PLLDIV_MASK);

	pllclk = plliclk ? FRC_CLK : PIC32_POSC_FREQ;
	frcdivn = ((1 << frcdiv) + 1) + (128 * (frcdiv == 7));

	if (pllodiv < 2)
		pllodiv = 2;
	else if (pllodiv < 5)
		pllodiv = (1 << pllodiv);
	else
		pllodiv = 32;

	curr_osc = (int)((osccon >> 12) & CUROSC_MASK);

	switch (curr_osc) {
	case FRC1:
	case FRC2:
		osc_freq = FRC_CLK / frcdivn;
		break;
	case SPLL:
		osc_freq = ((pllclk / pllidiv) * pllmult) / pllodiv;
		break;
	case POSC:
		osc_freq = PIC32_POSC_FREQ;
		break;
	default:
		break;
	}

	iounmap(osc_base);

	return osc_freq;
}

u32 pic32_get_pbclk(int bus)
{
	u32 clk_freq;

	void __iomem *osc_base = ioremap(PIC32_BASE_OSC, 0x200);
	u32 pbxdiv = PB1DIV + ((bus - 1) * 0x10);
	u32 pbdiv = (__raw_readl(osc_base + pbxdiv) & PB_MASK) + 1;

	iounmap(osc_base);

	clk_freq = pic32_get_cpuclk();

	return clk_freq / pbdiv;
}

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
	clocksource_of_init();
}
