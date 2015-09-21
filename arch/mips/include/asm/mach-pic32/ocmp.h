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

/* OC Modes */
enum {
	PIC32_OCM_NONE, /* disabled, but continues to draw current */
	PIC32_OCM_TRANSITION_HIGH, /* single match mode, OCx transition
				      LOW -> HIGH */
	PIC32_OCM_TRANSITION_LOW, /* single match mode, OCx transition
				     HIGH -> LOW */
	PIC32_OCM_TRANSITION_TOGGLE,/* single match mode, OCx toggles from
				       INIT state */
	PIC32_OCM_PULSE_ONE, /* dual match mode, generates one pulse */
	PIC32_OCM_PULSE_CONTINUOUS, /* dual match mode, generates continuous
				       pulse */
	PIC32_OCM_PWM_DISABLED_FAULT, /* PWM mode, fault detection disabled */
	PIC32_OCM_PWM_ENABLED_FAULT,  /* PWM mode, fault detection enabled */
	PIC32_OCM_MAX,
};

struct pic32_ocmp;
struct pic32_pb_timer;
struct pic32_ocmp *pic32_oc_request_by_node(struct device_node *np);
int pic32_oc_free(struct pic32_ocmp *oc);
int pic32_oc_set_time_base(struct pic32_ocmp *oc, struct pic32_pb_timer *timer);
int pic32_oc_settime(struct pic32_ocmp *oc, int mode, uint64_t timeout_nsec);
int pic32_oc_start(struct pic32_ocmp *oc);
int pic32_oc_stop(struct pic32_ocmp *oc);
int pic32_oc_get_irq(struct pic32_ocmp *oc);
void __init of_pic32_oc_init(void);

#endif
