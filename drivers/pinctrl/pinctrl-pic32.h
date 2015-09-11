/*
 * pic32 pinctrl core definitions.
 *
 * Copyright (C) 2014 Microchip Technology, Inc.
 * Author: Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */

#ifndef __DT_PIC32_PINCTRL_H__
#define __DT_PIC32_PINCTRL_H__

#include <linux/of_device.h>

#ifdef CONFIG_PINCTRL_PIC32MZ
#include "dt-bindings/pinctrl/pic32.h"
#elif defined(CONFIG_PINCTRL_PIC32MZDA)
#include "dt-bindings/pinctrl/pic32mzda.h"
#endif

#define MAX_PIO_BANKS	10
#define PINS_PER_BANK	32

/* device tree bindings */
enum pic32_pinctrl_props {
	PIC32_PINCTRL_PROP_SINGLE_PINS = 0,
	PIC32_PINCTRL_PROP_PINS,

	/* add above this line */
	PIC32_PINCTRL_PROP_LAST
};

struct pic32_pinctrl_prop {
	const char *name;
	const unsigned int narg;
};
#define PIC32_PINCTRL_DT_PROP(_name, _narg) { .name = _name, .narg = _narg }

#define PINMUX_MASK(off, size) (((1 << (size)) - 1) << (off))
#define PINMUX_FIELD(entry, off, size) \
		(((PINMUX_MASK(off, size)) & (entry)) >> (off))

#define PIC32_OFF_UNSPEC (~0UL)
#define PIC32_REG_SIZE 4

/* struct rpin_cod - remapable pin code
 * @bank: the pin's bank (PORT_A-to-PORT_J)
 * @pin: the pin (0-to-31)
 * @bucket: the bucket (BUCKET_A-to-BUCKET_D)
 * @dir: the pin direction (DIR_IN or DIR_OUT)
 * @cod: the code for input pinmux (COD(0x0)-to-COD(0xF))
 **/
struct rpin_cod {
	u32 bank:5,
	    pin:5,
	    bucket:5,
	    dir:1,
	    cod:16;
};

/* struct ppin_cod - peripheral pin code
 * @pin: peripherl pin code (e.g. PP_INT1, PP_T2CK...)
 * @bucket: the bucket (BUCKET_A-to-BUCKET_D)
 * @dir: the pin direction (DIR_IN or DIR_OUT)
 * @cod: the code for output pinmux (COD(0x0)-to-COD(0xF))
 **/
struct ppin_cod {
	u32 pin:10,
	    bucket:5,
	    dir:1,
	    cod:16;
};

/* struct pin_conf - pin configuration code
 * @dir: the pin direction (DIR_IN, DIR_OUT or DIR_NONE)
 * @mode: the pinmux mode (PIC32_PIN_MODE_NONE, PIC32_PIN_MODE_SOURCING...)
 **/
struct pin_conf {
	u32 dir:2,
	    conf:30;
};
#define PIC32_CHECK_PINCONF_CAPS(dev, pinconf, caps)			\
	do {								\
		if (!(pinconf & caps->pinconf_caps) &&			\
		    !(pinconf & caps->pinconf_outcaps) &&		\
		    !(pinconf & caps->pinconf_incaps)) {		\
									\
			dev_err(dev,					\
				"pin configuration not supported %lu\n",\
				pinconf);				\
			return -EINVAL;					\
		}							\
	} while (0)

/* struct pic32_reg - pic register
 * @val: register value
 * @clr: clear register value
 * @set: set register value
 * @inv: invert bits of register value
 **/
struct pic32_reg {
	u32 val;
	u32 clr;
	u32 set;
	u32 inv;
} __packed;
#define PIC32_PIO_REGS 4

/* enum pic32_pio_regs - pic32 regs for manipulate pin configuration; the
 *                       mapping may vary between pic32 flavors (e.g. MZ).
 **/
enum pic32_pio_regs {
	PIC32_UNKNOWN	= 0,

	PIC32_ANSEL	= 1,
	PIC32_TRIS	= 2,
	PIC32_PORT	= 3,
	PIC32_LAT	= 4,
	PIC32_ODC	= 5,
	PIC32_CNPU	= 6,
	PIC32_CNPD	= 7,
	PIC32_CNCON	= 8,
	PIC32_CNEN	= 9,
	PIC32_CNSTAT	= 10,

	PIC32_CNNE	= 11,
	PIC32_CNF	= 12,
	PIC32_SRCON1	= 13,
	PIC32_SRCON0	= 14,

	/* add above this line */
	PIC32_LAST
};
#define PIC32_CHECK_REG_OFF(reg, lookup, size)				\
	do {								\
		if (reg > size || lookup[reg] == PIC32_OFF_UNSPEC)	\
			BUG();						\
	} while (0)
#define PIC32_CNCON_BIT	15


/* struct pic32_pps_off - pps registers mapping
 * @ppsout_lookup_off: pps:out lookup registers offsets
 * @ppsout_bank_start: pps:out mapping bank start
 * @ppsout_bank_end: pps:out mapping bank end
 * @ppsout_pin_start: pps:out mapping pin start
 * @ppsout_pin_end: pps:out mapping pin end
 * @ppsout_map_pins: pps:out mapping pins per bank
 * @ppsin_lookup_off: pps:in lookup registers offsets
 * @ppsin_lookup_size: pps:in lookup map size
 **/
struct pic32_pps_off {
	unsigned (*ppsout_lookup_off)[MAX_PIO_BANKS][PINS_PER_BANK];
	unsigned (*ppsin_lookup_off)[PP_MAX];
};
#define PIC32_CHECK_PPSOUT_OFF(bank, pin, lookup)		\
	do {							\
		if (bank >= MAX_PIO_BANKS ||			\
		    pin > PINS_PER_BANK ||			\
		    lookup[bank][pin] == PIC32_OFF_UNSPEC)	\
			return -EINVAL;				\
	} while (0)
#define PIC32_CHECK_PPSIN_OFF(pin, lookup)			\
	do {							\
		if (lookup[pin] == PIC32_OFF_UNSPEC)		\
			return -EINVAL;				\
	} while (0)

/* struct pic32_caps - pin configuration capabilities
 * @pinconf_incaps: pin configuration input capabilities
 * @pinconf_outcaps: pin configuration output capabilities
 **/
struct pic32_caps {
	u32 pinconf_caps;
	u32 pinconf_incaps;
	u32 pinconf_outcaps;
};

int pic32_gpio_probe(struct platform_device *pdev,
		     unsigned (*reg_lookup_off)[], unsigned lookup_size);

int pic32_pinctrl_probe(struct platform_device *pdev,
			struct pic32_pps_off *pps_off,
			struct pic32_caps *caps);

#endif /*__DT_PIC32_PINCTRL_H__*/
