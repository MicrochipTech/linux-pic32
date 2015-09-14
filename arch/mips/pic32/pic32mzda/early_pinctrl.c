/*
 * PIC32mzda Early pinctrl configurations.
 *
 * Copyright (C) 2015 Microchip Technology, Inc.
 *
 * Authors:
 *   Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */
#include <asm/io.h>

#include "early_pinctrl.h"

enum pic32_ports {
	PIC32_PORTA = 0,
	PIC32_PORTB = 1,
	PIC32_PORTC = 2,
	PIC32_PORTD = 3,
	PIC32_PORTE = 4,
	PIC32_PORTF = 5,
	PIC32_PORTG = 6,
	PIC32_PORTH = 7,
	PIC32_PORTJ = 8,
	PIC32_PORTK = 9
};

static inline struct pic32_pio __iomem *
	pic32_map_port_addr(enum pic32_ports port)
{
	return ioremap_nocache(PIC32_BASE_PORT + (port * PORT_SIZE),
		sizeof(struct pic32_pio));
}

/* console, uart 2 - early pinmux */
void __init pic32mzda_earlyco_port3_pinctrl(void)
{
	struct pic32_ppsinr __iomem *ppsinr =
		ioremap_nocache(PPSIN_BASE, sizeof(*ppsinr));
	struct pic32_ppsoutr __iomem *ppsoutr =
		ioremap_nocache(PPSOUT_BASE, sizeof(*ppsoutr));
	struct pic32_pio __iomem *piob = pic32_map_port_addr(PIC32_PORTB);
	struct pic32_pio __iomem *piog = pic32_map_port_addr(PIC32_PORTG);

	BUG_ON(!ppsinr);
	BUG_ON(!ppsoutr);
	BUG_ON(!piob);
	BUG_ON(!piog);

	/* pins linkage */
	writel(0x02, &ppsoutr->rpg0r[9]);   /* TX (RPG9) */
	writel(0x05, &ppsinr->u2rxr);       /* RX (RPB0) */

	/* pins type: digital */
	writel((1 << 9), &piog->ansel.clr); /* TX (RPG9) */
	writel(1, &piob->ansel.clr);        /* RX (RPB0) */

	/* pins direction */
	writel((1 << 9), &piog->tris.clr); /* TX (RPG9): output  */
	writel(1, &piob->tris.set);        /* RX (RPB0): input */

	iounmap(ppsinr);
	iounmap(ppsoutr);
	iounmap(piob);
	iounmap(piog);
}
