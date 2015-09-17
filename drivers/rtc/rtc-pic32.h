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

#ifndef __ASM_ARCH_REGS_RTC_H
#define __ASM_ARCH_REGS_RTC_H __FILE__

#define PIC32_RTCCON		(0x00)
#define PIC32_RTCCON_ON		(1 << 15)
#define PIC32_RTCCON_SIDL	(1 << 13)
#define PIC32_RTCCON_RTCCLKSEL	(3 << 9)
#define PIC32_RTCCON_RTCCLKON	(1 << 6)
#define PIC32_RTCCON_RTCWREN	(1 << 3)
#define PIC32_RTCCON_RTCSYNC	(1 << 2)
#define PIC32_RTCCON_HALFSEC	(1 << 1)
#define PIC32_RTCCON_RTCOE	(1 << 0)

#define PIC32_RTCALRM		(0x10)
#define PIC32_RTCALRM_ALRMEN	(1 << 15)
#define PIC32_RTCALRM_CHIME	(1 << 14)
#define PIC32_RTCALRM_PIV	(1 << 13)
#define PIC32_RTCALRM_ALARMSYNC	(1 << 12)
#define PIC32_RTCALRM_AMASK	(0xF << 8)
#define PIC32_RTCALRM_ARPT	(0xFF << 0)

#define PIC32_RTCHOUR		0x23
#define PIC32_RTCMIN		0x22
#define PIC32_RTCSEC		0x21
#define PIC32_RTCYEAR		0x33
#define PIC32_RTCMON		0x32
#define PIC32_RTCDAY		0x31

#define PIC32_ALRMTIME		0x40
#define PIC32_ALRMDATE		0x50

#define PIC32_ALRMHOUR		0x43
#define PIC32_ALRMMIN		0x42
#define PIC32_ALRMSEC		0x41
#define PIC32_ALRMYEAR		0x53
#define PIC32_ALRMMON		0x52
#define PIC32_ALRMDAY		0x51

#endif /* __ASM_ARCH_REGS_RTC_H */
