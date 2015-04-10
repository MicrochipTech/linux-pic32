/*
 *  PIC32 deadman timer driver
 *
 *  Copyright (C) 2014, Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/watchdog.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/mach-pic32/pic32.h>

/* Deadman Timer Regs */
#define DMTCON_REG	0x00
#define DMTPRECLR_REG	0x10
#define DMTCLR_REG	0x20
#define DMTSTAT_REG	0x30
#define DMTCNT_REG	0x40
#define DMTPSCNT_REG	0x60
#define DMTPSINTV_REG	0x70

/* Deadman Timer Regs fields */
#define DMT_ON			0x8000
#define DMT_STEP1_KEY		0x40
#define DMT_STEP1_KEY_BYTE	1
#define DMT_STEP2_KEY		0x08
#define DMTSTAT_WINOPN		0x01
#define DMTSTAT_EVENT		0x20
#define DMTSTAT_BAD2		0x40
#define DMTSTAT_BAD1		0x80

struct pic32_dmt {
	spinlock_t	lock;
	void __iomem	*regs;
	struct clk	*clk;
};

static inline int dmt_is_enabled(struct pic32_dmt *dmt)
{
	return readl(dmt->regs + DMTCON_REG) & DMT_ON;
}

static inline void dmt_enable(struct pic32_dmt *dmt)
{
	writel(DMT_ON, PIC32_SET(dmt->regs + DMTCON_REG));
}

static inline void dmt_disable(struct pic32_dmt *dmt)
{
	writel(DMT_ON, PIC32_CLR(dmt->regs + DMTCON_REG));
	cpu_relax();
}

static inline int dmt_bad_status(struct pic32_dmt *dmt)
{
	uint32_t val;

	val = readl(dmt->regs + DMTSTAT_REG);
	val &= (DMTSTAT_BAD1|DMTSTAT_BAD2|DMTSTAT_EVENT);
	if (val)
		pr_err("dmt: bad event generated: sts %08x\n", val);

	return val;
}

static inline int dmt_keepalive(struct pic32_dmt *dmt)
{
	writeb(DMT_STEP1_KEY, dmt->regs + DMTPRECLR_REG + DMT_STEP1_KEY_BYTE);
	writeb(DMT_STEP2_KEY, dmt->regs + DMTCLR_REG);

	/* check whether keys are latched correctly */
	return dmt_bad_status(dmt);
}

static inline u32 dmt_timeleft(struct pic32_dmt *dmt)
{
	u32 top = readl(dmt->regs + DMTPSCNT_REG);
	return top - readl(dmt->regs + DMTCNT_REG);
}

static inline u32 dmt_interval_time_to_clear(struct pic32_dmt *dmt)
{
	return readl(dmt->regs + DMTPSINTV_REG);
}

static inline u32 pic32_dmt_get_timeout_secs(struct pic32_dmt *dmt)
{
	return readl(dmt->regs + DMTPSCNT_REG) / clk_get_rate(dmt->clk);
}

static int pic32_dmt_start(struct watchdog_device *wdd)
{
	struct pic32_dmt *dmt = watchdog_get_drvdata(wdd);

	spin_lock(&dmt->lock);
	if (dmt_is_enabled(dmt))
		goto done;

	dmt_enable(dmt);
done:
	dmt_keepalive(dmt);
	spin_unlock(&dmt->lock);

	return 0;
}

static int pic32_dmt_stop(struct watchdog_device *wdd)
{
	struct pic32_dmt *dmt = watchdog_get_drvdata(wdd);

	spin_lock(&dmt->lock);
	dmt_disable(dmt);
	spin_unlock(&dmt->lock);

	return 0;
}

static int pic32_dmt_ping(struct watchdog_device *wdd)
{
	struct pic32_dmt *dmt = watchdog_get_drvdata(wdd);
	int ret;

	spin_lock(&dmt->lock);
	ret = dmt_keepalive(dmt);
	spin_unlock(&dmt->lock);

	return ret ? -EAGAIN : 0;
}

static unsigned int pic32_dmt_get_timeleft(struct watchdog_device *wdd)
{
	struct pic32_dmt *dmt = watchdog_get_drvdata(wdd);
	return dmt_timeleft(dmt);
}

static const struct watchdog_ops pic32_dmt_fops = {
	.owner		= THIS_MODULE,
	.start		= pic32_dmt_start,
	.stop		= pic32_dmt_stop,
	.get_timeleft	= pic32_dmt_get_timeleft,
	.ping		= pic32_dmt_ping,
};

static const struct watchdog_info pic32_dmt_ident = {
	.options	= WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
	.identity	= "PIC32 Deadman Timer",
};

static struct watchdog_device pic32_dmt_wdd = {
	.info		= &pic32_dmt_ident,
	.ops		= &pic32_dmt_fops,
	.min_timeout	= 1,
	.max_timeout	= UINT_MAX,
};

static int pic32_dmt_probe(struct platform_device *pdev)
{
	int ret;
	struct pic32_dmt *dmt;
	struct resource *mem;
	struct watchdog_device *wdd = &pic32_dmt_wdd;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	dmt = devm_kzalloc(&pdev->dev, sizeof(*dmt), GFP_KERNEL);
	if (IS_ERR(dmt))
		return PTR_ERR(dmt);

	dmt->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dmt->regs))
		return PTR_ERR(dmt->regs);

	dmt->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dmt->clk)) {
		dev_err(&pdev->dev, "clk not found\n");
		return PTR_ERR(dmt->clk);
	}

	ret = clk_prepare_enable(dmt->clk);
	if (ret)
		return ret;

	wdd->max_timeout /= clk_get_rate(dmt->clk);
	wdd->timeout = pic32_dmt_get_timeout_secs(dmt);
	if (!wdd->timeout) {
		dev_err(&pdev->dev,
			"timeout %dsec too small for DMT\n", wdd->timeout);
		ret = -EINVAL;
		goto out_disable_clk;
	}

	spin_lock_init(&dmt->lock);

	dev_info(&pdev->dev, "max_timeout %d, min_timeout %d, cur_timeout %d\n",
		wdd->max_timeout, wdd->min_timeout, wdd->timeout);
	ret = watchdog_register_device(wdd);
	if (ret) {
		dev_err(&pdev->dev, "watchdog register failed, err %d\n", ret);
		goto out_disable_clk;
	}

	watchdog_set_nowayout(wdd, WATCHDOG_NOWAYOUT);
	watchdog_set_drvdata(wdd, dmt);

	platform_set_drvdata(pdev, wdd);
	return 0;

out_disable_clk:
	clk_disable_unprepare(dmt->clk);
	return ret;
}

static int pic32_dmt_remove(struct platform_device *pdev)
{
	struct watchdog_device *wdd = platform_get_drvdata(pdev);
	struct pic32_dmt *dmt = watchdog_get_drvdata(wdd);

	clk_disable_unprepare(dmt->clk);
	watchdog_unregister_device(wdd);

	return 0;
}

static const struct of_device_id pic32_dmt_of_ids[] = {
	{ .compatible = "microchip,pic32-dmt",},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_dmt_of_ids);

static struct platform_driver pic32_dmt_driver = {
	.probe		= pic32_dmt_probe,
	.remove		= pic32_dmt_remove,
	.driver		= {
		.name		= "pi32-dmt",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_dmt_of_ids),
	}
};

module_platform_driver(pic32_dmt_driver);

MODULE_AUTHOR("Purna Chandra Mandal");
MODULE_DESCRIPTION("Microchip PIC32 DMT Driver");
MODULE_LICENSE("GPL");
