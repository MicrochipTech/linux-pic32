/*
 *  Copyright (C) 2014, Joshua Henderson <joshua.henderson@microchip.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#ifndef __I2S_PIC32_HEADER__
#define __I2S_PIC32_HEADER__

#include <asm/mach-pic32/pic32.h>

#define SPICON				0x00
#define SPISTAT				0x10
#define SPIBUF				0x20
#define SPIBRG				0x30
#define SPICON2				0x40

#define SPICON_FRMEN			BIT(31)
#define SPICON_FRMSYNC			BIT(30)
#define SPICON_FRMPOL			BIT(29)
#define SPICON_MSSEN			BIT(28)
#define SPICON_FRMSYPW			BIT(27)
#define SPICON_FRMCNT_32BIT		(5 << 24)
#define SPICON_FRMCNT_16BIT		(4 << 24)
#define SPICON_FRMCNT_8BIT		(3 << 24)
#define SPICON_FRMCNT_4BIT		(2 << 24)
#define SPICON_FRMCNT_2BIT		(1 << 24)
#define SPICON_FRMCNT_1BIT		(0 << 24)
#define SPICON_MCLKSEL			BIT(23)
#define SPICON_SPIFE			BIT(17)
#define SPICON_ENHBUF			BIT(16)
#define SPICON_ON			BIT(15)
#define SPICON_SIDL			BIT(13)
#define SPICON_DISSDO			BIT(12)
#define SPICON_MODE_MASK		(3 << 10)
#define SPICON_MODE_16B_16		(0 << 10)
#define SPICON_MODE_16B_32		(1 << 10)
#define SPICON_MODE_32B_32		(2 << 10)
#define SPICON_MODE_24B_32		(3 << 10)
#define SPICON_SMP			BIT(9)
#define SPICON_CKE			BIT(8)
#define SPICON_SSEN			BIT(7)
#define SPICON_CKP			BIT(6)
#define SPICON_MSTEN			BIT(5)
#define SPICON_DISSDI			BIT(4)
#define SPICON_STXISEL_NOT_FULL		(3 << 2)
#define SPICON_STXISEL_HALF_EMPTY	(2 << 2)
#define SPICON_STXISEL_EMPTY		(1 << 2)
#define SPICON_STXISEL_COMPLETE		(0 << 2)
#define SPICON_SRXISEL_FULL		3
#define SPICON_SRXISEL_HALF_FULL	2
#define SPICON_SRXISEL_NOT_EMPTY	1
#define SPICON_SRXISEL_EMPTY		0

#define SPICON2_SPISGNEXT		BIT(15)
#define SPICON2_FRMERREN		BIT(12)
#define SPICON2_SPIROVEN		BIT(11)
#define SPICON2_SPITUREN		BIT(10)
#define SPICON2_IGNROV			BIT(9)
#define SPICON2_IGNTUR			BIT(8)
#define SPICON2_AUDEN			BIT(7)
#define SPICON2_AUDMONO			BIT(3)
#define SPICON2_AUDMOD_MASK		(3 << 0)
#define SPICON2_AUDMOD_PCM		(3 << 0)
#define SPICON2_AUDMOD_RJ		(2 << 0)
#define SPICON2_AUDMOD_LJ		(1 << 0)
#define SPICON2_AUDMOD_I2S		(0 << 0)

#define PIC32_I2S_REFCLK		0

#endif /* __I2S_PIC32_HEADER__ */
