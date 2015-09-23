/*
 * Regulator driver for Microchip's Comparator Voltage Reference Module.
 *
 * Copyright (C) 2014 - Microchip Technology Inc.
 *
 * Author: Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

/* Regulator registers offset */
#define CVRCON_REG	0
#define CVRCON_CLR_REG	4
#define CVRCON_SET_REG	8

#define CVR		0xf
#define CVRSS		BIT(4)
#define VIN_SUPPLY_VDD	0
#define VIN_SUPPLY_VREF	1
#define CVRR		BIT(5)
#define CVRON		(BIT(15) | BIT(6)) /* ON + output enable */

struct pic32_regulator {
	void __iomem *regs;
	const void *pdata;
	int vout_range;
	int vin_supply;
};

static int reg_get_voltage_from_sel(struct pic32_regulator *reg, int select)
{
	int uV;

	if (reg->vout_range)
		uV = select * reg->vin_supply / 24;
	else
		uV = (reg->vin_supply / 4) +
			(select * reg->vin_supply / 32);

	return uV;
}

static inline int reg_get_voltage_sel(struct pic32_regulator *reg)
{
	return readl(reg->regs + CVRCON_REG) & CVR;
}

static int pic32_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct pic32_regulator *reg;

	reg = rdev_get_drvdata(rdev);
	return reg_get_voltage_sel(reg);
}

static int pic32_regulator_set_voltage_sel(struct regulator_dev *rdev,
					   unsigned selector)
{
	struct pic32_regulator *reg;
	unsigned long v;

	reg = rdev_get_drvdata(rdev);
	v = readl(reg->regs + CVRCON_REG);
	v &= ~CVR;
	v |= selector;
	writel(v, reg->regs + CVRCON_REG);

	return 0;
}

static int pic32_regulator_list_voltage(struct regulator_dev *rdev,
					unsigned selector)
{
	int uV;
	struct pic32_regulator *reg;

	reg = rdev_get_drvdata(rdev);
	if (selector >= rdev->desc->n_voltages)
		return -EINVAL;

	uV = reg_get_voltage_from_sel(reg, selector);
	return uV;
}

static int pic32_regulator_enable(struct regulator_dev *rdev)
{
	struct pic32_regulator *reg;

	reg = rdev_get_drvdata(rdev);
	writel(CVRON, reg->regs + CVRCON_SET_REG);

	return 0;
}

static int pic32_regulator_disable(struct regulator_dev *rdev)
{
	struct pic32_regulator *reg;

	reg = rdev_get_drvdata(rdev);
	writel(CVRON, reg->regs + CVRCON_CLR_REG);
	cpu_relax();

	return 0;
}

static int pic32_regulator_is_enabled(struct regulator_dev *rdev)
{
	u32 v;
	struct pic32_regulator *reg;

	reg = rdev_get_drvdata(rdev);
	v = readl(reg->regs + CVRCON_REG); /* CLR ON|CVROE */

	return (v & CVRON) == CVRON;
}

static struct regulator_ops pic32_regulator_voltage_ops = {
	.set_voltage_sel = pic32_regulator_set_voltage_sel,
	.get_voltage_sel = pic32_regulator_get_voltage_sel,
	.list_voltage    = pic32_regulator_list_voltage,
	.map_voltage     = regulator_map_voltage_iterate,
	.enable          = pic32_regulator_enable,
	.disable         = pic32_regulator_disable,
	.is_enabled      = pic32_regulator_is_enabled,
};

static const struct regulator_desc _reg_desc = {
	.name		= "pic32-regulator",
	.ops		= &pic32_regulator_voltage_ops,
	.type		= REGULATOR_VOLTAGE,
	.owner		= THIS_MODULE,
	.n_voltages	= 16,
};

static int pic32_regulator_probe(struct platform_device *pdev)
{
	u32 v;
	struct pic32_regulator *reg;
	struct resource *mem;
	struct regulator_dev *regulator;
	struct regulator_config config = { };
	struct device_node *np;
	int vout_range = 0;
	int uvin_supply, vin_supply_src = VIN_SUPPLY_VDD;

	reg = devm_kzalloc(&pdev->dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	np = pdev->dev.of_node;
	of_property_read_u32(np, "pic32,regulator-vin-select",
			     &vin_supply_src);

	switch (vin_supply_src) {
	case VIN_SUPPLY_VREF:
		break;
	case VIN_SUPPLY_VDD:
		break;
	default:
		dev_err(&pdev->dev, "incorrect input supply selection\n");
		return -EINVAL;
	}

	of_property_read_u32(np, "pic32,regulator-vin-uV", &uvin_supply);
	if (!uvin_supply) {
		dev_err(&pdev->dev, "incorrect input supply\n");
		return -EINVAL;
	}

	of_property_read_u32(np, "pic32,regulator-vout-select", &vout_range);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource ?\n");
		return -ENOENT;
	}

	reg->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (!reg->regs)
		return -ENOMEM;

	/* configure supply src */
	v = readl(reg->regs + CVRCON_REG);
	v &= ~CVRR;
	v |= vin_supply_src ? CVRR : 0;

	/* force voltage output range */
	reg->vout_range = vout_range;

	reg->vin_supply = uvin_supply;

	config.init_data = of_get_regulator_init_data(&pdev->dev, np,
						      &_reg_desc);
	if (!config.init_data)
		return -ENOMEM;

	config.of_node = np;
	config.dev = &pdev->dev;
	config.driver_data = reg;

	regulator = devm_regulator_register(&pdev->dev, &_reg_desc, &config);
	if (IS_ERR(regulator)) {
		dev_err(&pdev->dev, "Failed to register regulator %s\n",
			_reg_desc.name);
		return PTR_ERR(regulator);
	}

	return 0;
}

static const struct of_device_id pic32_of_match[] = {
	{ .compatible = "microchip,pic32-regulator",},
	{ },
};
MODULE_DEVICE_TABLE(of, pic32_of_match);

static struct platform_driver pic32_regulator_driver = {
	.driver = {
		.name		= "pic32-regulator",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_of_match),
	},
	.probe = pic32_regulator_probe,
};

module_platform_driver(pic32_regulator_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Purna Chandra Mandal <purna.mandal@microchip.com>");
MODULE_DESCRIPTION("Microchip PIC32 Comparator Voltage Reference Driver");
MODULE_ALIAS("platform:pic32-regulator");
