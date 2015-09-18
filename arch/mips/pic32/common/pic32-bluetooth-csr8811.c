/* CSR Bluetooth GPIO control
 *
 * Cristian Birsan, cristian.birsan@microchip.com
 * Copyright (C) 2015 Microchip Technology Inc.  All rights reserved.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/rfkill.h>

static struct csr_bt {
	struct rfkill	*bt_rfkill;
	int				bt_nrst_gpio;
} csr_bt;

static int csr8811_bt_rfkill_set_power(void *data, bool blocked)
{
	/* rfkill_ops callback. Turn transmitter on when blocked is false */
	if (!blocked) {
		pr_info("[BT] Bluetooth Power On.\n");
		msleep(20);
		gpio_set_value(csr_bt.bt_nrst_gpio, 1);
		msleep(50);
	} else {
		pr_info("[BT] Bluetooth Power Off.\n");
		gpio_set_value(csr_bt.bt_nrst_gpio, 0);
	}
	return 0;
}

static const struct rfkill_ops csr8811_bt_rfkill_ops = {
	.set_block = csr8811_bt_rfkill_set_power,
};

static int csr8811_bluetooth_probe(struct platform_device *pdev)
{
	int ret;

	csr_bt.bt_nrst_gpio = of_get_named_gpio(pdev->dev.of_node,
			"bluetooth_nrst-gpios", 0);

	if (gpio_is_valid(csr_bt.bt_nrst_gpio)) {
			ret = devm_gpio_request(&pdev->dev,
					csr_bt.bt_nrst_gpio, "bt_nrst");
			if (ret) {
				dev_err(&pdev->dev, "Error requesting Bluetooth stby/rst gpio.\n");
				return -EINVAL;
			}
	}

	gpio_direction_output(csr_bt.bt_nrst_gpio, 0);

	csr_bt.bt_rfkill = rfkill_alloc("csr8811 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &csr8811_bt_rfkill_ops,
				NULL);

	if (unlikely(!csr_bt.bt_rfkill)) {
		dev_err(&pdev->dev, "[BT] bt_rfkill alloc failed.\n");
		devm_gpio_free(&pdev->dev, csr_bt.bt_nrst_gpio);
		return -ENOMEM;
	}

	/* The BT module is disabled at startup */
	rfkill_init_sw_state(csr_bt.bt_rfkill, 0);

	ret = rfkill_register(csr_bt.bt_rfkill);

	if (unlikely(ret)) {
		dev_err(&pdev->dev, "[BT] bt_rfkill register failed.\n");
		rfkill_destroy(csr_bt.bt_rfkill);
		devm_gpio_free(&pdev->dev, csr_bt.bt_nrst_gpio);
		return -ENODEV;
	}

	rfkill_set_sw_state(csr_bt.bt_rfkill, true);

	return ret;
}

static int csr8811_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(csr_bt.bt_rfkill);
	rfkill_destroy(csr_bt.bt_rfkill);

	return 0;
}

static const struct of_device_id csr8811_bluetooth_id[] = {
	{ .compatible = "csr,8811",},
	{ },
};
MODULE_DEVICE_TABLE(of, csr8811_bluetooth_id);

static struct platform_driver csr8811_bluetooth_driver = {
	.driver = {
		.name		= "csr8811_bluetooth",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(csr8811_bluetooth_id),
	},
	.probe = csr8811_bluetooth_probe,
	.remove = csr8811_bluetooth_remove,
};

module_platform_driver(csr8811_bluetooth_driver);

MODULE_DESCRIPTION("Microchip PIC32 CSR8811 bluetooth rfkill driver");
MODULE_AUTHOR("Cristian Birsan <cristian.birsan@microchip.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
