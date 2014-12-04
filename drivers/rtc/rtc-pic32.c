/*
 * PIC32 RTCC driver
 *
 * Joshua Henderson, joshua.henderson@microchip.com
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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/clk.h>
#include <linux/log2.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/mach-pic32/pic32.h>

#include "rtc-pic32.h"

#define DRIVER_NAME "pic32-rtc"

static struct clk *rtc_clk;
static void __iomem *pic32_rtc_base;
static int pic32_rtc_alarmno;

static DEFINE_SPINLOCK(pic32_rtc_pie_lock);

static void pic32_rtc_alarm_clk_enable(bool enable)
{
	static DEFINE_SPINLOCK(pic32_rtc_alarm_clk_lock);
	static bool alarm_clk_enabled;
	unsigned long flags;

	spin_lock_irqsave(&pic32_rtc_alarm_clk_lock, flags);
	if (enable) {
		if (!alarm_clk_enabled) {
			clk_enable(rtc_clk);
			alarm_clk_enabled = true;
		}
	} else {
		if (alarm_clk_enabled) {
			clk_disable(rtc_clk);
			alarm_clk_enabled = false;
		}
	}
	spin_unlock_irqrestore(&pic32_rtc_alarm_clk_lock, flags);
}

static irqreturn_t pic32_rtc_alarmirq(int irq, void *id)
{
	struct rtc_device *rdev = id;

	clk_enable(rtc_clk);
	rtc_update_irq(rdev, 1, RTC_AF | RTC_IRQF);

	clk_disable(rtc_clk);

	pic32_rtc_alarm_clk_enable(false);

	return IRQ_HANDLED;
}

static int pic32_rtc_setaie(struct device *dev, unsigned int enabled)
{
	dev_dbg(dev, "%s: aie=%d\n", __func__, enabled);

	clk_enable(rtc_clk);

	writel(PIC32_RTCALRM_ALRMEN,
		pic32_rtc_base +
		(enabled ? PIC32_SET(PIC32_RTCALRM) :
			PIC32_CLR(PIC32_RTCALRM)));

	clk_disable(rtc_clk);

	pic32_rtc_alarm_clk_enable(enabled);

	return 0;
}

static int pic32_rtc_setfreq(struct device *dev, int freq)
{
	unsigned long flags;
	clk_enable(rtc_clk);

	spin_lock_irqsave(&pic32_rtc_pie_lock, flags);

	writel(PIC32_RTCALRM_AMASK, pic32_rtc_base + PIC32_CLR(PIC32_RTCALRM));
	writel(freq << 8, pic32_rtc_base + PIC32_SET(PIC32_RTCALRM));
	writel(PIC32_RTCALRM_CHIME, pic32_rtc_base + PIC32_SET(PIC32_RTCALRM));

	spin_unlock_irqrestore(&pic32_rtc_pie_lock, flags);

	clk_disable(rtc_clk);

	return 0;
}

static int pic32_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	unsigned int have_retried = 0;
	void __iomem *base = pic32_rtc_base;

	clk_enable(rtc_clk);
 retry_get_time:
	rtc_tm->tm_hour = readb(base + PIC32_RTCHOUR);
	rtc_tm->tm_min = readb(base + PIC32_RTCMIN);
	rtc_tm->tm_mon  = readb(base + PIC32_RTCMON);
	rtc_tm->tm_mday = readb(base + PIC32_RTCDAY);
	rtc_tm->tm_year = readb(base + PIC32_RTCYEAR);
	rtc_tm->tm_sec  = readb(base + PIC32_RTCSEC);

	/* the only way to work out whether the system was mid-update
	 * when we read it is to check the second counter, and if it
	 * is zero, then we re-try the entire read
	 */

	if (rtc_tm->tm_sec == 0 && !have_retried) {
		have_retried = 1;
		goto retry_get_time;
	}

	rtc_tm->tm_sec = bcd2bin(rtc_tm->tm_sec);
	rtc_tm->tm_min = bcd2bin(rtc_tm->tm_min);
	rtc_tm->tm_hour = bcd2bin(rtc_tm->tm_hour);
	rtc_tm->tm_mday = bcd2bin(rtc_tm->tm_mday);
	rtc_tm->tm_mon = bcd2bin(rtc_tm->tm_mon);
	rtc_tm->tm_year = bcd2bin(rtc_tm->tm_year);

	rtc_tm->tm_year += 100;

	dev_dbg(dev, "read time %04d.%02d.%02d %02d:%02d:%02d\n",
		 1900 + rtc_tm->tm_year, rtc_tm->tm_mon, rtc_tm->tm_mday,
		 rtc_tm->tm_hour, rtc_tm->tm_min, rtc_tm->tm_sec);

	rtc_tm->tm_mon -= 1;

	clk_disable(rtc_clk);
	return rtc_valid_tm(rtc_tm);
}

static int pic32_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	void __iomem *base = pic32_rtc_base;
	int year = tm->tm_year - 100;

	dev_dbg(dev, "set time %04d.%02d.%02d %02d:%02d:%02d\n",
		 1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		 tm->tm_hour, tm->tm_min, tm->tm_sec);

	if (year < 0 || year >= 100) {
		dev_err(dev, "rtc only supports 100 years\n");
		return -EINVAL;
	}

	clk_enable(rtc_clk);
	writeb(bin2bcd(tm->tm_sec),  base + PIC32_RTCSEC);
	writeb(bin2bcd(tm->tm_min),  base + PIC32_RTCMIN);
	writeb(bin2bcd(tm->tm_hour), base + PIC32_RTCHOUR);
	writeb(bin2bcd(tm->tm_mday), base + PIC32_RTCDAY);
	writeb(bin2bcd(tm->tm_mon + 1), base + PIC32_RTCMON);
	writeb(bin2bcd(year), base + PIC32_RTCYEAR);
	clk_disable(rtc_clk);

	return 0;
}

static int pic32_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *alm_tm = &alrm->time;
	void __iomem *base = pic32_rtc_base;
	unsigned int alm_en;

	clk_enable(rtc_clk);
	alm_tm->tm_sec  = readb(base + PIC32_ALRMSEC);
	alm_tm->tm_min  = readb(base + PIC32_ALRMMIN);
	alm_tm->tm_hour = readb(base + PIC32_ALRMHOUR);
	alm_tm->tm_mon  = readb(base + PIC32_ALRMMON);
	alm_tm->tm_mday = readb(base + PIC32_ALRMDAY);
	alm_tm->tm_year = readb(base + PIC32_ALRMYEAR);

	alm_en = readb(base + PIC32_RTCALRM);

	alrm->enabled = (alm_en & PIC32_RTCALRM_ALRMEN) ? 1 : 0;

	dev_dbg(dev, "read alarm %d, %04d.%02d.%02d %02d:%02d:%02d\n",
		 alm_en,
		 1900 + alm_tm->tm_year, alm_tm->tm_mon, alm_tm->tm_mday,
		 alm_tm->tm_hour, alm_tm->tm_min, alm_tm->tm_sec);

	alm_tm->tm_sec = bcd2bin(alm_tm->tm_sec);
	alm_tm->tm_min = bcd2bin(alm_tm->tm_min);
	alm_tm->tm_hour = bcd2bin(alm_tm->tm_hour);
	alm_tm->tm_mday = bcd2bin(alm_tm->tm_mday);
	alm_tm->tm_mon = bcd2bin(alm_tm->tm_mon);
	alm_tm->tm_mon -= 1;
	alm_tm->tm_year = bcd2bin(alm_tm->tm_year);

	clk_disable(rtc_clk);
	return 0;
}

static int pic32_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;
	void __iomem *base = pic32_rtc_base;

	clk_enable(rtc_clk);
	dev_dbg(dev, "pic32_rtc_setalarm: %d, %04d.%02d.%02d %02d:%02d:%02d\n",
		 alrm->enabled,
		 1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday,
		 tm->tm_hour, tm->tm_min, tm->tm_sec);

	writel(0x00, base + PIC32_ALRMTIME);
	writel(0x00, base + PIC32_ALRMDATE);

	if (tm->tm_sec < 60 && tm->tm_sec >= 0)
		writeb(bin2bcd(tm->tm_sec), base + PIC32_ALRMSEC);

	if (tm->tm_min < 60 && tm->tm_min >= 0)
		writeb(bin2bcd(tm->tm_min), base + PIC32_ALRMMIN);

	if (tm->tm_hour < 24 && tm->tm_hour >= 0)
		writeb(bin2bcd(tm->tm_hour), base + PIC32_ALRMHOUR);

	pic32_rtc_setaie(dev, alrm->enabled);

	clk_disable(rtc_clk);
	return 0;
}

static int pic32_rtc_proc(struct device *dev, struct seq_file *seq)
{
	unsigned int repeat;

	clk_enable(rtc_clk);

	repeat = readw(pic32_rtc_base + PIC32_RTCALRM);
	repeat &= PIC32_RTCALRM_ARPT;

	seq_printf(seq, "periodic_IRQ\t: %s\n", repeat  ? "yes" : "no");
	clk_disable(rtc_clk);
	return 0;

}

static const struct rtc_class_ops pic32_rtcops = {
	.read_time	= pic32_rtc_gettime,
	.set_time	= pic32_rtc_settime,
	.read_alarm	= pic32_rtc_getalarm,
	.set_alarm	= pic32_rtc_setalarm,
	.proc		= pic32_rtc_proc,
	.alarm_irq_enable = pic32_rtc_setaie,
};

static void pic32_rtc_enable(struct platform_device *pdev, int en)
{
	void __iomem *base = pic32_rtc_base;

	if (base == NULL)
		return;

	clk_enable(rtc_clk);
	if (!en) {
		writel(PIC32_RTCCON_ON,
			base + PIC32_CLR(PIC32_RTCCON));
	} else {

		pic32_syskey_unlock();

		writel(PIC32_RTCCON_RTCWREN, base + PIC32_SET(PIC32_RTCCON));

		writel(3 << 9, base + PIC32_CLR(PIC32_RTCCON));

		if ((readl(base + PIC32_RTCCON) & PIC32_RTCCON_ON) == 0) {
			dev_info(&pdev->dev, "rtc disabled, re-enabling\n");

			writel(PIC32_RTCCON_ON,
				base + PIC32_SET(PIC32_RTCCON));
		}
	}
	clk_disable(rtc_clk);
}

static int pic32_rtc_remove(struct platform_device *dev)
{
	pic32_rtc_setaie(&dev->dev, 0);

	clk_unprepare(rtc_clk);
	rtc_clk = NULL;

	return 0;
}

static int pic32_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	struct rtc_time rtc_tm;
	struct resource *res;
	int ret;

	dev_dbg(&pdev->dev, "%s: probe=%p\n", __func__, pdev);

	pic32_rtc_alarmno = platform_get_irq(pdev, 0);
	if (pic32_rtc_alarmno < 0) {
		dev_err(&pdev->dev, "no irq for alarm\n");
		return pic32_rtc_alarmno;
	}

	dev_dbg(&pdev->dev, "pic32_rtc: alarm irq %d\n",
		 pic32_rtc_alarmno);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pic32_rtc_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pic32_rtc_base))
		return PTR_ERR(pic32_rtc_base);

	rtc_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(rtc_clk)) {
		dev_err(&pdev->dev, "failed to find rtc clock source\n");
		ret = PTR_ERR(rtc_clk);
		rtc_clk = NULL;
		return ret;
	}

	clk_prepare_enable(rtc_clk);

	pic32_rtc_enable(pdev, 1);

	dev_dbg(&pdev->dev, "pic32_rtc: RTCCON=%02x\n",
		 readl(pic32_rtc_base + PIC32_RTCCON));

	device_init_wakeup(&pdev->dev, 1);

	rtc = devm_rtc_device_register(&pdev->dev, "pic32", &pic32_rtcops,
				  THIS_MODULE);

	if (IS_ERR(rtc)) {
		dev_err(&pdev->dev, "cannot attach rtc\n");
		ret = PTR_ERR(rtc);
		goto err_nortc;
	}

	pic32_rtc_gettime(NULL, &rtc_tm);

	if (rtc_valid_tm(&rtc_tm)) {
		rtc_tm.tm_year	= 100;
		rtc_tm.tm_mon	= 0;
		rtc_tm.tm_mday	= 1;
		rtc_tm.tm_hour	= 0;
		rtc_tm.tm_min	= 0;
		rtc_tm.tm_sec	= 0;

		pic32_rtc_settime(NULL, &rtc_tm);

		dev_warn(&pdev->dev,
			"warning: invalid RTC value so initializing it\n");
	}

	rtc->max_user_freq = 128;

	platform_set_drvdata(pdev, rtc);

	pic32_rtc_setfreq(&pdev->dev, 1);
	ret = devm_request_irq(&pdev->dev, pic32_rtc_alarmno, pic32_rtc_alarmirq,
			  0,  DRIVER_NAME " alarm", rtc);
	if (ret) {
		dev_err(&pdev->dev, "IRQ %d error %d\n", pic32_rtc_alarmno, ret);
		goto err_nortc;
	}

	clk_disable(rtc_clk);

	return 0;

 err_nortc:
	pic32_rtc_enable(pdev, 0);
	clk_disable_unprepare(rtc_clk);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id pic32_rtc_dt_ids[] = {
	{ .compatible = "microchip,pic32-rtc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_rtc_dt_ids);
#endif

static struct platform_driver pic32_rtc_driver = {
	.probe		= pic32_rtc_probe,
	.remove		= pic32_rtc_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(pic32_rtc_dt_ids),
	},
};

module_platform_driver(pic32_rtc_driver);

MODULE_DESCRIPTION("PIC32 RTC Driver");
MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
