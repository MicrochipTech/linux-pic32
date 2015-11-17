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
#ifndef PIC32MZDA_COMMON_H
#define PIC32MZDA_COMMON_H

extern struct irq_domain *evic_irq_domain;

/* early clock */
u32 pic32_get_pbclk(int bus);
u32 pic32_get_sysclk(void);

/* Default early console parameters */
#define EARLY_CONSOLE_PORT	1
#define EARLY_CONSOLE_BAUDRATE	115200

/* Device configuration */
int pic32_set_lcd_mode(int mode);

int pic32_set_sdhc_adma_fifo_threshold(u32 rthrs, u32 wthrs);

u32 pic32_get_boot_status(void);

u32 pic32_get_usb_pll_mode(void);

int pic32_disable_lcd(void);

int pic32_enable_lcd(void);

void __init pic32_config_init(void);

#endif
