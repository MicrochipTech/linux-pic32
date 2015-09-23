/*
 *  PIC32 watchdog driver
 *
 *  Copyright (C) 2014, Joshua Henderson <joshua.henderson@microchip.com>
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

/* Watchdog Timer Registers */
#define WDTCON_REG	0x00
#define RESETCON_REG	0x10

/* Watchdog Timer Control Register fields */
#define WDTCON_WIN_EN		0x0001
#define WDTCON_RMCS_MASK	0x0003
#define WDTCON_RMCS_SHIFT	0x0006
#define WDTCON_RMPS_MASK	0x001F
#define WDTCON_RMPS_SHIFT	0x0008
#define WDTCON_ON		0x8000
#define WDTCON_CLR_KEY		0x5743

/* Reset Control Register fields for watchdog */
#define RESETCON_TIMEOUT_IDLE	0x0004
#define RESETCON_TIMEOUT_SLEEP	0x0008
#define RESETCON_WDT_TIMEOUT	0x0010

struct pic32_wdt {
	spinlock_t	lock; /* lock */
	void __iomem	*regs;
	struct clk	*clk;
	unsigned long	next_heartbeat;
	unsigned long	timeout;
};

static inline int wdt_is_enabled(struct pic32_wdt *wdt)
{
	return readl(wdt->regs + WDTCON_REG) & WDTCON_ON;
}

static inline int wdt_is_win_enabled(struct pic32_wdt *wdt)
{
	u32 v;

	v = readl(wdt->regs + WDTCON_REG);
	v &= WDTCON_WIN_EN;

	return v;
}

static inline void wdt_enable(struct pic32_wdt *wdt)
{
	writel(WDTCON_ON, PIC32_SET(wdt->regs + WDTCON_REG));
}

static inline void wdt_disable(struct pic32_wdt *wdt)
{
	writel(WDTCON_ON, PIC32_CLR(wdt->regs + WDTCON_REG));
	cpu_relax();
}

static inline u32 wdt_get_post_scaler(struct pic32_wdt *wdt)
{
	u32 v = readl(wdt->regs + WDTCON_REG);

	return (v >> WDTCON_RMPS_SHIFT) & WDTCON_RMPS_MASK;
}

static inline u32 wdt_get_clk_id(struct pic32_wdt *wdt)
{
	u32 v = readl(wdt->regs + WDTCON_REG);

	return (v >> WDTCON_RMCS_SHIFT) & WDTCON_RMCS_MASK;
}

static inline void wdt_keepalive(struct pic32_wdt *wdt)
{
	/* write key through single half-word */
	writew(WDTCON_CLR_KEY, wdt->regs + WDTCON_REG + 2);
}

static int pic32_wdt_bootstatus(struct pic32_wdt *wdt)
{
	u32 v = readl(wdt->regs + RESETCON_REG);

	return v & RESETCON_WDT_TIMEOUT;
}

static int pic32_wdt_get_timeout_secs(struct pic32_wdt *wdt)
{
	unsigned long rate;
	u32 period, ps, terminal;

	rate = clk_get_rate(wdt->clk);
	pr_debug("wdt: clk_id %d, clk_rate %lu (prescale)\n",
		 wdt_get_clk_id(wdt), rate);

	/* default, prescaler of 32 (i.e. div/32) is implicit. */
	rate >>= 5;

	/* calculate terminal count from postscaler. */
	ps = wdt_get_post_scaler(wdt);
	terminal = 1 << ps;

	/* find time taken (in secs) to reach terminal count */
	period = terminal / rate;
	pr_info("wdt: clk_rate %lu (postscale) / terminal %d, timeout %dsec\n",
		rate, terminal, period);

	return period;
}

static void pic32_wdt_keepalive(struct pic32_wdt *wdt)
{
	wdt->next_heartbeat = jiffies +
		msecs_to_jiffies(wdt->timeout * MSEC_PER_SEC);
	wdt_keepalive(wdt);
}

static int pic32_wdt_start(struct watchdog_device *wdd)
{
	struct pic32_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	if (wdt_is_enabled(wdt))
		goto done;

	wdt_enable(wdt);
done:
	pic32_wdt_keepalive(wdt);
	spin_unlock(&wdt->lock);

	return 0;
}

static int pic32_wdt_stop(struct watchdog_device *wdd)
{
	struct pic32_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	wdt_disable(wdt);
	spin_unlock(&wdt->lock);

	return 0;
}

static int pic32_wdt_ping(struct watchdog_device *wdd)
{
	struct pic32_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);
	pic32_wdt_keepalive(wdt);
	spin_unlock(&wdt->lock);

	return 0;
}

static unsigned int pic32_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct pic32_wdt *wdt = watchdog_get_drvdata(wdd);

	return jiffies_to_msecs(wdt->next_heartbeat - jiffies) / MSEC_PER_SEC;
}

static const struct watchdog_ops pic32_wdt_fops = {
	.owner		= THIS_MODULE,
	.start		= pic32_wdt_start,
	.stop		= pic32_wdt_stop,
	.get_timeleft	= pic32_wdt_get_timeleft,
	.ping		= pic32_wdt_ping,
};

static const struct watchdog_info pic32_wdt_ident = {
	.options = WDIOF_KEEPALIVEPING |
			WDIOF_MAGICCLOSE | WDIOF_CARDRESET,
	.identity = "PIC32 Watchdog",
};

static struct watchdog_device pic32_wdd = {
	.info		= &pic32_wdt_ident,
	.ops		= &pic32_wdt_fops,
	.min_timeout	= 1, /* 1-sec */
	.max_timeout	= 1048,
};

static const struct of_device_id pic32_wdt_dt_ids[] = {
	{ .compatible = "microchip,pic32-wdt-v2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_wdt_dt_ids);

static int pic32_wdt_drv_probe(struct platform_device *pdev)
{
	int ret;
	struct watchdog_device *wdd = &pic32_wdd;
	struct pic32_wdt *wdt;
	struct resource *mem;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem)
		return -EINVAL;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (IS_ERR(wdt))
		return PTR_ERR(wdt);

	wdt->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(wdt->regs))
		return PTR_ERR(wdt->regs);

	wdt->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(wdt->clk)) {
		dev_err(&pdev->dev, "clk not found\n");
		return PTR_ERR(wdt->clk);
	}

	ret = clk_prepare_enable(wdt->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk enable failed\n");
		return ret;
	}

	if (wdt_is_win_enabled(wdt)) {
		dev_err(&pdev->dev, "windowed-clear mode is not supported.\n");
		ret = -EOPNOTSUPP;
		goto out_disable_clk;
	}

	wdt->timeout = pic32_wdt_get_timeout_secs(wdt);
	spin_lock_init(&wdt->lock);

	wdd->timeout = wdt->timeout;
	wdd->bootstatus = pic32_wdt_bootstatus(wdt) ? WDIOF_CARDRESET : 0;

	ret = watchdog_register_device(wdd);
	if (ret) {
		dev_err(&pdev->dev, "watchdog register failed, err %d\n", ret);
		goto out_disable_clk;
	}

	watchdog_set_nowayout(wdd, WATCHDOG_NOWAYOUT);
	watchdog_set_drvdata(wdd, wdt);

	platform_set_drvdata(pdev, wdd);
	return 0;

out_disable_clk:
	clk_disable_unprepare(wdt->clk);

	return ret;
}

static int pic32_wdt_drv_remove(struct platform_device *pdev)
{
	struct watchdog_device *wdd = platform_get_drvdata(pdev);
	struct pic32_wdt *wdt = watchdog_get_drvdata(wdd);

	clk_disable_unprepare(wdt->clk);
	watchdog_unregister_device(wdd);

	return 0;
}

static struct platform_driver pic32_wdt_driver = {
	.probe		= pic32_wdt_drv_probe,
	.remove		= pic32_wdt_drv_remove,
	.driver		= {
		.name		= "pi32-wdt",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_wdt_dt_ids),
	}
};

module_platform_driver(pic32_wdt_driver);

MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_DESCRIPTION("Microchip PIC32 Watchdog Timer");
MODULE_LICENSE("GPL");
