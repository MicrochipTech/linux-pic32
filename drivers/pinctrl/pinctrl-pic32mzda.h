/*
 * pic32mz pinctrl definitions.
 *
 * Copyright (C) 2014 Microchip Technology, Inc.
 * Author: Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */

#ifndef __DT_PIC32MZ_PINCTRL_H__
#define __DT_PIC32MZ_PINCTRL_H__

#include "pinctrl-pic32.h"

/* struct pic32mz_pio - portX config register map
 * @ansel: operations of analog port pins
 * @tris: data direction register
 * @port: read from port pins
 * @lat: write to the port pins
 * @odc: digital or open-drain register
 * @cnpu: weak pull-up register
 * @cnpd: weak pull-down register
 * @cncon: change notification (CN) control register
 * @cnen: CN interrupt enable control bits
 * @cnstat: change occurred on corresponding pins
 **/
struct pic32mz_pio {
	struct pic32_reg ansel;
	struct pic32_reg tris;
	struct pic32_reg port;
	struct pic32_reg lat;
	struct pic32_reg odc;
	struct pic32_reg cnpu;
	struct pic32_reg cnpd;
	struct pic32_reg cncon;
	struct pic32_reg cnen;
	struct pic32_reg cnstat;
} __packed;

#endif /*__DT_PIC32MZ_PINCTRL_H__*/
