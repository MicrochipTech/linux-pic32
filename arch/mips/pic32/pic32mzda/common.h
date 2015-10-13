/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2014 Joshua Henderson <joshua.henderson@microchip.com>
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
