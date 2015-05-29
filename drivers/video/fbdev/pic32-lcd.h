/*
 * PIC32 LCD Frame Buffer Driver
 *
 * Joshua Henderson, <joshua.henderson@microchip.com>
 * Copyright (C) 2015 Microchip Technology Inc.  All rights reserved.
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
#ifndef PIC32_LCD_H
#define PIC32_LCD_H

/* Cursor */
#define PIC32_LCD_CURSOR
#define PIC32_LCD_CURSOR_IMAGE		0x800
#define PIC32_LCD_CURSOR_CLUT		0xA00
#define PIC32_LCD_CURSOR_WIDTH		32
#define PIC32_LCD_CURSOR_HEIGHT		32
#define PIC32_LCD_CURSOR_COLORS		16

/* Color Modes */
#define PIC32_LCD_MODE_LUT8		0x00
#define PIC32_LCD_MODE_RGBA5551		0x01
#define PIC32_LCD_MODE_RGBA8888		0x02
#define PIC32_LCD_MODE_RGB332		0x04
#define PIC32_LCD_MODE_RGB565		0x05
#define PIC32_LCD_MODE_ARGB8888		0x06
#define PIC32_LCD_MODE_L8		0x07
#define PIC32_LCD_MODE_L1		0x08
#define PIC32_LCD_MODE_L4		0x09
#define PIC32_LCD_MODE_YUYV		0x0A
#define PIC32_LCD_MODE_RGB888		0x0B

#define PIC32_LCD_REG_MODE		0x000
#define PIC32_LCD_REG_CLKCON		0x004
#define PIC32_LCD_REG_BGCOLOR		0x008
#define PIC32_LCD_REG_RESXY		0x00c
#define PIC32_LCD_REG_FRONTPORCHXY	0x014
#define PIC32_LCD_REG_BLANKINGXY	0x018
#define PIC32_LCD_REG_BACKPORCHXY	0x01c
#define PIC32_LCD_REG_CURSORXY		0x020
#define PIC32_LCD_REG_LAYER0_MODE	0x030
#define PIC32_LCD_REG_LAYER0_STARTXY	0x034
#define PIC32_LCD_REG_LAYER0_SIZEXY	0x038
#define PIC32_LCD_REG_LAYER0_BASEADDR	0x03c
#define PIC32_LCD_REG_LAYER0_STRIDE	0x040
#define PIC32_LCD_REG_LAYER0_RESXY	0x044
#define PIC32_LCD_REG_STATUS		0x0fc
#define PIC32_LCD_REG_INTERRUPT		0x0f8

#define PIC32_LCD_CONFIG_ENABLE		(1 << 31)
#define PIC32_LCD_CONFIG_CURSOR		(1 << 30)
#define PIC32_LCD_CONFIG_NEG_V		(1 << 28)
#define PIC32_LCD_CONFIG_NEG_H		(1 << 27)
#define PIC32_LCD_CONFIG_NEG_DE		(1 << 26)
#define PIC32_LCD_CONFIG_PCLKPOL	(1 << 22)
#define PIC32_LCD_CONFIG_GAMMA		(1 << 20)

#define FBIO_RAMP_SET			0x31

#endif
