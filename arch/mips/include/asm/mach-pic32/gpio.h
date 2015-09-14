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
#ifndef __ASM_MACH_PIC32_GPIO_H
#define __ASM_MACH_PIC32_GPIO_H

#define ARCH_NR_GPIOS 352

#define gpio_to_irq(gpio)	__gpio_to_irq(gpio)
#define gpio_get_value		__gpio_get_value
#define gpio_set_value		__gpio_set_value
#define gpio_cansleep		__gpio_cansleep

#include <asm-generic/gpio.h>

#endif /* __ASM_MACH_PIC32_GPIO_H */
