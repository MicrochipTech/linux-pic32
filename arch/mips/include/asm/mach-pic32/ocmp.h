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
#ifndef __PIC32_OCMP_H__
#define __PIC32_OCMP_H__

#include <linux/types.h>
#include <linux/init.h>

/* OCMP operation modes */
enum {
	PIC32_OCM_NONE,			/* disabled, but drains power */
	PIC32_OCM_TRANSITION_HIGH,	/* single match, OCx LOW to HIGH */
	PIC32_OCM_TRANSITION_LOW,	/* single match, OCx HIGH -> LOW */
	PIC32_OCM_TRANSITION_TOGGLE,	/* single match, OCx toggles */
	PIC32_OCM_PULSE_ONE,		/* dual match, generates one pulse */
	PIC32_OCM_PULSE_CONTINUOUS,	/* dual match, continuous pulse */
	PIC32_OCM_PWM_DISABLED_FAULT,	/* PWM mode, fault disabled */
	PIC32_OCM_PWM_ENABLED_FAULT,	/* PWM mode, fault enabled */
	PIC32_OCM_MAX,
};

struct pic32_ocmp;
struct pic32_pb_timer;

/* pic32_oc_request_by_node - get OCMP handle from device node. */
struct pic32_ocmp *pic32_oc_request_by_node(struct device_node *np);

/* pic32_oc_set_time_base - set timebase.
 *
 * @timer: pic32 general purpose timer running counter of which
 *         will be compared against OC comp1 & sec_comp register.
 */
int pic32_oc_set_time_base(struct pic32_ocmp *oc, struct pic32_pb_timer *timer);

/* pic32_oc_settime - set timeout and mode
 *
 * @mode    :one of PIC32_OCM_ enum
 * @timeout :timeout in nanosecs
 */
int pic32_oc_settime(struct pic32_ocmp *oc, int mode, uint64_t timeout_nsec);

/* pic32_oc_start - starts configured OC.
 *
 * This function can be called from irq context.
 */
int pic32_oc_start(struct pic32_ocmp *oc);

/* pic32_oc_stop - stop running OC.
 *
 * This function can be called from irq context.
 */
int pic32_oc_stop(struct pic32_ocmp *oc);

/* pic32_get_irq - get interrupt number for the OC. */
int pic32_oc_get_irq(struct pic32_ocmp *oc);

/* pic32_oc_free - release OC when all done */
int pic32_oc_free(struct pic32_ocmp *oc);

void __init of_pic32_oc_init(void);

#endif
