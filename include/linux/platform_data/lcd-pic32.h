/*
 * Purna Chandra Mandal, purna.mandal@microchip.com
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
#ifndef __PIC32_LCD_PDATA_H__
#define __PIC32_LCD_PDATA_H__

/* LCD Controller info data structure, stored in device platform_data */
struct pic32_lcd_pdata {
	u8 default_bpp;
	int (*enable)(void);
	int (*disable)(void);
	int (*set_mode)(int mode);
};

#endif
