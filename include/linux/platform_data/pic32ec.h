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
#ifndef __PIC32EC_PDATA_H__
#define __PIC32EC_PDATA_H__

struct pic32ec_platform_data {
	u32		phy_mask;
	u8		is_rmii;	/* using RMII interface? */
	u8		rev_eth_addr;	/* reverse address byte order */
};

#endif
