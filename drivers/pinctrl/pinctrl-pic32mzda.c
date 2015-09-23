/*
 * pic32mzda pinctrl driver.
 *
 * Copyright (C) 2015 Microchip Technology, Inc.
 * Author: Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>

#include "pinctrl-pic32.h"

/* PIC32MZDA PORT I/O: register offsets */
static unsigned pic32mzda_pio_lookup_off[PIC32_LAST] = {
	[PIC32_ANSEL]	= 0  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_TRIS]	= 1  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_PORT]	= 2  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_LAT]	= 3  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_ODC]	= 4  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNPU]	= 5  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNPD]	= 6  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNCON]	= 7  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNEN]	= 8  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNSTAT]	= 9  * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNNE]	= 10 * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_CNF]	= 11 * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_SRCON1]	= 12 * PIC32_PIO_REGS * PIC32_REG_SIZE,
	[PIC32_SRCON0]	= 13 * PIC32_PIO_REGS * PIC32_REG_SIZE,
};

static void pic32mzda_build_pio_lookup_off(
		unsigned (*pio_lookup_off)[PIC32_LAST])
{
	int i;

	/* Guard unsupported registers by flag PIC32_OFF_UNSPEC. */
	for (i = 0; i < PIC32_LAST; i++) {
		if ((*pio_lookup_off)[i] == 0 && i != PIC32_ANSEL)
			(*pio_lookup_off)[i] = PIC32_OFF_UNSPEC;
	}
}

static int pic32mzda_gpio_probe(struct platform_device *pdev)
{
	pic32mzda_build_pio_lookup_off(&pic32mzda_pio_lookup_off);

	return pic32_gpio_probe(pdev,
				&pic32mzda_pio_lookup_off,
				PIC32_LAST);
}

static const struct of_device_id pic32mzda_gpio_of_match[] = {
	{ .compatible = "microchip,pic32-gpio" },
	{ /* sentinel */ }
};

static struct platform_driver pic32mzda_gpio_driver = {
	.driver = {
		.name = "gpio-pic32mzda",
		.owner = THIS_MODULE,
		.of_match_table = pic32mzda_gpio_of_match,
	},
	.probe = pic32mzda_gpio_probe,
};

/* Peripheral Pin Select (input): register offsets */
static unsigned pic32mzda_ppsin_lookup_off[PP_MAX] = {
	/*[PP_INT0]	= 0  * PIC32_REG_SIZE, Not remappable */
	[PP_INT1]	= 1  * PIC32_REG_SIZE,
	[PP_INT2]	= 2  * PIC32_REG_SIZE,
	[PP_INT3]	= 3  * PIC32_REG_SIZE,
	[PP_INT4]	= 4  * PIC32_REG_SIZE,
	/*[PP_T1CK]	= 5  * PIC32_REG_SIZE, Not remappable */
	[PP_T2CK]	= 6  * PIC32_REG_SIZE,
	[PP_T3CK]	= 7  * PIC32_REG_SIZE,
	[PP_T4CK]	= 8  * PIC32_REG_SIZE,
	[PP_T5CK]	= 9  * PIC32_REG_SIZE,
	[PP_T6CK]	= 10 * PIC32_REG_SIZE,
	[PP_T7CK]	= 11 * PIC32_REG_SIZE,
	[PP_T8CK]	= 12 * PIC32_REG_SIZE,
	[PP_T9CK]	= 13 * PIC32_REG_SIZE,
	[PP_IC1]	= 14 * PIC32_REG_SIZE,
	[PP_IC2]	= 15 * PIC32_REG_SIZE,
	[PP_IC3]	= 16 * PIC32_REG_SIZE,
	[PP_IC4]	= 17 * PIC32_REG_SIZE,
	[PP_IC5]	= 18 * PIC32_REG_SIZE,
	[PP_IC6]	= 19 * PIC32_REG_SIZE,
	[PP_IC7]	= 20 * PIC32_REG_SIZE,
	[PP_IC8]	= 21 * PIC32_REG_SIZE,
	[PP_IC9]	= 22 * PIC32_REG_SIZE,
	/* rsv		= 23 */
	[PP_OCFA]	= 24 * PIC32_REG_SIZE,
	/*[PP_OCFB]	= 25 * PIC32_REG_SIZE, Not remappable */
	[PP_U1RX]	= 26 * PIC32_REG_SIZE,
	[PP_U1CTS]	= 27 * PIC32_REG_SIZE,
	[PP_U2RX]	= 28 * PIC32_REG_SIZE,
	[PP_U2CTS]	= 29 * PIC32_REG_SIZE,
	[PP_U3RX]	= 30 * PIC32_REG_SIZE,
	[PP_U3CTS]	= 31 * PIC32_REG_SIZE,
	[PP_U4RX]	= 32 * PIC32_REG_SIZE,
	[PP_U4CTS]	= 33 * PIC32_REG_SIZE,
	[PP_U5RX]	= 34 * PIC32_REG_SIZE,
	[PP_U5CTS]	= 35 * PIC32_REG_SIZE,
	[PP_U6RX]	= 36 * PIC32_REG_SIZE,
	[PP_U6CTS]	= 37 * PIC32_REG_SIZE,
	/*[PP_SCK1IN]	= 38 * PIC32_REG_SIZE, Not remappable */
	[PP_SDI1]	= 39 * PIC32_REG_SIZE,
	[PP_SS1]	= 40 * PIC32_REG_SIZE,
	/*[PP_SCK2IN]	= 41 * PIC32_REG_SIZE, Not remappable */
	[PP_SDI2]	= 42 * PIC32_REG_SIZE,
	[PP_SS2]	= 43 * PIC32_REG_SIZE,
	/*[PP_SCK3IN]	= 44 * PIC32_REG_SIZE, Not remappable */
	[PP_SDI3]	= 45 * PIC32_REG_SIZE,
	[PP_SS3]	= 46 * PIC32_REG_SIZE,
	/*[PP_SCK4IN]	= 47 * PIC32_REG_SIZE, Not remappable */
	[PP_SDI4]	= 48 * PIC32_REG_SIZE,
	[PP_SS4]	= 49 * PIC32_REG_SIZE,
	/*[PP_SCK5IN]	= 50 * PIC32_REG_SIZE, Not remappable */
	[PP_SDI5]	= 51 * PIC32_REG_SIZE,
	[PP_SS5]	= 52 * PIC32_REG_SIZE,
	/*[PP_SCK6IN]	= 53 * PIC32_REG_SIZE, Not remappable */
	[PP_SDI6]	= 54 * PIC32_REG_SIZE,
	[PP_SS6]	= 55 * PIC32_REG_SIZE,
	[PP_C1RX]	= 56 * PIC32_REG_SIZE,
	[PP_C2RX]	= 57 * PIC32_REG_SIZE,
	[PP_REFCLKI1]	= 58 * PIC32_REG_SIZE,
	/*[PP_REFCLKI2]	= 59 * PIC32_REG_SIZE, Not remappable */
	[PP_REFCLKI3]	= 60 * PIC32_REG_SIZE,
	[PP_REFCLKI4]	= 61 * PIC32_REG_SIZE
};

static void pic32mzda_build_ppsin_lookup_off(
		unsigned (*ppsin_lookup_off)[PP_MAX])
{
	int i;

	/* Guard unsupported configurations by flag PIC32_OFF_UNSPEC. */
	for (i = 0; i < PP_MAX; i++) {
		if ((*ppsin_lookup_off)[i] == 0)
			(*ppsin_lookup_off)[i] = PIC32_OFF_UNSPEC;
	}
}

/* Peripheral Pin Select (output): register offsets */
static unsigned pic32mzda_ppsout_lookup_off[MAX_PIO_BANKS][PINS_PER_BANK] = {
	[PORT_A][14] = (PORT_A * 16 + 14) * PIC32_REG_SIZE,
	[PORT_A][15] = (PORT_A * 16 + 15) * PIC32_REG_SIZE,

	[PORT_B][0]  = (PORT_B * 16 +  0) * PIC32_REG_SIZE,
	[PORT_B][1]  = (PORT_B * 16 +  1) * PIC32_REG_SIZE,
	[PORT_B][2]  = (PORT_B * 16 +  2) * PIC32_REG_SIZE,
	[PORT_B][3]  = (PORT_B * 16 +  3) * PIC32_REG_SIZE,
	[PORT_B][5]  = (PORT_B * 16 +  5) * PIC32_REG_SIZE,
	[PORT_B][6]  = (PORT_B * 16 +  6) * PIC32_REG_SIZE,
	[PORT_B][7]  = (PORT_B * 16 +  7) * PIC32_REG_SIZE,
	[PORT_B][8]  = (PORT_B * 16 +  8) * PIC32_REG_SIZE,
	[PORT_B][9]  = (PORT_B * 16 +  9) * PIC32_REG_SIZE,
	[PORT_B][10] = (PORT_B * 16 + 10) * PIC32_REG_SIZE,
	[PORT_B][15] = (PORT_B * 16 + 15) * PIC32_REG_SIZE,

	[PORT_C][1]  = (PORT_C * 16 +  1) * PIC32_REG_SIZE,
	[PORT_C][2]  = (PORT_C * 16 +  2) * PIC32_REG_SIZE,
	[PORT_C][3]  = (PORT_C * 16 +  3) * PIC32_REG_SIZE,
	[PORT_C][13] = (PORT_C * 16 +  13) * PIC32_REG_SIZE,
	[PORT_C][14] = (PORT_C * 16 +  14) * PIC32_REG_SIZE,

	[PORT_D][0]  = (PORT_D * 16 +  0) * PIC32_REG_SIZE,
	[PORT_D][2]  = (PORT_D * 16 +  2) * PIC32_REG_SIZE,
	[PORT_D][3]  = (PORT_D * 16 +  3) * PIC32_REG_SIZE,
	[PORT_D][4]  = (PORT_D * 16 +  4) * PIC32_REG_SIZE,
	[PORT_D][5]  = (PORT_D * 16 +  5) * PIC32_REG_SIZE,
	[PORT_D][6]  = (PORT_D * 16 +  6) * PIC32_REG_SIZE,
	[PORT_D][7]  = (PORT_D * 16 +  7) * PIC32_REG_SIZE,
	[PORT_D][9]  = (PORT_D * 16 +  9) * PIC32_REG_SIZE,
	[PORT_D][10] = (PORT_D * 16 + 10) * PIC32_REG_SIZE,
	[PORT_D][11] = (PORT_D * 16 + 11) * PIC32_REG_SIZE,
	[PORT_D][12] = (PORT_D * 16 + 12) * PIC32_REG_SIZE,
	[PORT_D][14] = (PORT_D * 16 + 14) * PIC32_REG_SIZE,
	[PORT_D][15] = (PORT_D * 16 + 15) * PIC32_REG_SIZE,

	[PORT_E][3]  = (PORT_E * 16 +  3) * PIC32_REG_SIZE,
	[PORT_E][5]  = (PORT_E * 16 +  5) * PIC32_REG_SIZE,
	[PORT_E][8]  = (PORT_E * 16 +  8) * PIC32_REG_SIZE,
	[PORT_E][9]  = (PORT_E * 16 +  9) * PIC32_REG_SIZE,

	[PORT_F][0]  = (PORT_F * 16 +  0) * PIC32_REG_SIZE,
	[PORT_F][1]  = (PORT_F * 16 +  1) * PIC32_REG_SIZE,
	[PORT_F][2]  = (PORT_F * 16 +  2) * PIC32_REG_SIZE,
	[PORT_F][3]  = (PORT_F * 16 +  3) * PIC32_REG_SIZE,
	[PORT_F][4]  = (PORT_F * 16 +  4) * PIC32_REG_SIZE,
	[PORT_F][5]  = (PORT_F * 16 +  5) * PIC32_REG_SIZE,
	[PORT_F][8]  = (PORT_F * 16 +  8) * PIC32_REG_SIZE,
	[PORT_F][12] = (PORT_F * 16 + 12) * PIC32_REG_SIZE,

	[PORT_G][0]  = (PORT_G * 16 +  0) * PIC32_REG_SIZE,
	[PORT_G][1]  = (PORT_G * 16 +  1) * PIC32_REG_SIZE,
	[PORT_G][7]  = (PORT_G * 16 +  7) * PIC32_REG_SIZE,
	[PORT_G][8]  = (PORT_G * 16 +  8) * PIC32_REG_SIZE,
	[PORT_G][9]  = (PORT_G * 16 +  9) * PIC32_REG_SIZE,
};

static void pic32mzda_build_ppsout_lookup_off(
		unsigned (*ppsout_lookup_off)[MAX_PIO_BANKS][PINS_PER_BANK])
{
	unsigned bank, pin;

	/* Guard unsupported configurations by flag PIC32_OFF_UNSPEC. */
	for (bank = 0; bank < MAX_PIO_BANKS; bank++) {
		for (pin = 0; pin < PINS_PER_BANK; pin++)
			if ((*ppsout_lookup_off)[bank][pin] == 0)
				(*ppsout_lookup_off)[bank][pin] =
							PIC32_OFF_UNSPEC;
	}
}

static struct pic32_pps_off pic32mzda_pps_off = {
	.ppsout_lookup_off = &pic32mzda_ppsout_lookup_off,
	.ppsin_lookup_off = &pic32mzda_ppsin_lookup_off,
};

static struct pic32_caps pic32mzda_caps = {
	.pinconf_caps = 0,
	.pinconf_incaps = 0,
	.pinconf_outcaps = 0
};

static void pic32mzda_build_caps(struct pic32_caps *caps)
{
	(caps->pinconf_caps)	= PIC32_PIN_CONF_NONE |
				  PIC32_PIN_CONF_OD |
				  PIC32_PIN_CONF_PU |
				  PIC32_PIN_CONF_PD |
				  PIC32_PIN_CONF_AN |
				  PIC32_PIN_CONF_DG;
	(caps->pinconf_outcaps) = PIC32_PIN_CONF_DOUT |
				  PIC32_PIN_CONF_OD_OUT |
				  PIC32_PIN_CONF_DG_OUT;
	(caps->pinconf_incaps) = PIC32_PIN_CONF_DIN |
				 PIC32_PIN_CONF_PU_IN |
				 PIC32_PIN_CONF_PD_IN |
				 PIC32_PIN_CONF_AN_IN |
				 PIC32_PIN_CONF_DG_IN;
}

static int pic32mzda_pinctrl_probe(struct platform_device *pdev)
{
	pic32mzda_build_caps(&pic32mzda_caps);

	return pic32_pinctrl_probe(pdev,
				   &pic32mzda_pps_off,
				   &pic32mzda_caps);
}

static const struct of_device_id pic32mzda_pinctrl_of_match[] = {
	{ .compatible = "microchip,pic32-pinctrl"},
	{ /* sentinel */ }
};

static struct platform_driver pic32mzda_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-pic32mzda",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pic32mzda_pinctrl_of_match),
	},
	.probe = pic32mzda_pinctrl_probe,
};

static int __init pic32mzda_pinctrl_init(void)
{
	int ret = 0;

	pic32mzda_build_ppsin_lookup_off(&pic32mzda_ppsin_lookup_off);
	pic32mzda_build_ppsout_lookup_off(&pic32mzda_ppsout_lookup_off);

	ret = platform_driver_register(&pic32mzda_gpio_driver);
	if (ret)
		return ret;

	return platform_driver_register(&pic32mzda_pinctrl_driver);
}
arch_initcall(pic32mzda_pinctrl_init);

static void __exit pic32mzda_pinctrl_exit(void)
{
	platform_driver_unregister(&pic32mzda_gpio_driver);
	platform_driver_unregister(&pic32mzda_pinctrl_driver);
}
module_exit(pic32mzda_pinctrl_exit);

MODULE_AUTHOR("Sorin-Andrei Pistirica <andrei.pistirica@microchip.com>");
MODULE_DESCRIPTION("Microchop pic32mzda pinctrl driver");
MODULE_LICENSE("GPL v2");
