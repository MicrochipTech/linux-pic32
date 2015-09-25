/*
 * Joshua Henderson, <joshua.henderson@microchip.com>
 * Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#ifndef _ASM_MACH_PIC32_H
#define _ASM_MACH_PIC32_H

#include <asm/io.h>

/*
 * PIC32 register offsets for SET/CLR/INV where supported.
 */
#define PIC32_CLR(reg)		(reg + 0x04)
#define PIC32_SET(reg)		(reg + 0x08)
#define PIC32_INV(reg)		(reg + 0x0C)

/*
 * PIC32 Base Register Offsets
 */
#define PIC32_BASE_CONFIG	0x1f800000
#define PIC32_BASE_OSC		0x1f801200
#define PIC32_BASE_RESET	0x1f801240
#define PIC32_BASE_PPS		0x1f801400
#define PIC32_BASE_DMA		0x1f811000
#define PIC32_BASE_UART		0x1f822000
#define PIC32_BASE_PORT		0x1f860000
#define PIC32_BASE_DEVCFG2	0x1fc4ff44

/*
 * Register unlock sequence required for some registers.
 */
#define pic32_syskey_unlock()						\
	do {								\
		void __iomem *syskey = ioremap(PIC32_BASE_CONFIG +	\
					0x30, sizeof(u32));		\
		__raw_writel(0x00000000, syskey);			\
		__raw_writel(0xAA996655, syskey);			\
		__raw_writel(0x556699AA, syskey);			\
		iounmap(syskey);					\
	} while (0)

#endif /* _ASM_MACH_PIC32_H */
