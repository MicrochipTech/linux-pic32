/*
 * Copyright 2014 Microchip Inc.
 *
 * Licensed under the GPL-2.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <asm/mach-pic32/pbtimer.h>

struct pic32_tmr_state {
	struct iio_trigger *trig;
	struct pic32_pb_timer *timer;
	unsigned timer_num;
	bool output_enable;
	int irq;
};

static int iio_pic32_tmr_set_state(struct iio_trigger *trig, bool state)
{
	int ret;
	uint64_t period, elapsed;
	struct pic32_tmr_state *st = iio_trigger_get_drvdata(trig);

	ret = pic32_pb_timer_gettime(st->timer, &period, &elapsed);
	if (ret || (period == 0))
		return -EINVAL;

	if (state)
		pbt_enable(st->timer);
	else
		pbt_disable(st->timer);

	return 0;
}

static ssize_t iio_pic32_tmr_frequency_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct pic32_tmr_state *st = iio_trigger_get_drvdata(trig);
	unsigned int val;
	bool enabled;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	if (val == 0)
		return count;

	if (val > 100000)
		return -EINVAL;

	enabled = pbt_is_enabled(st->timer);
	if (enabled)
		pbt_disable(st->timer);

	pic32_pb_timer_settime(st->timer,
				PIC32_TIMER_MAY_RATE, val * NSEC_PER_SEC);

	if (enabled)
		pbt_enable(st->timer);

	return count;
}

static ssize_t iio_pic32_tmr_frequency_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct pic32_tmr_state *st = iio_trigger_get_drvdata(trig);
	uint64_t period;

	pic32_pb_timer_gettime(st->timer, &period, NULL);

	return sprintf(buf, "%u ns\n", (u32)period);
}

static DEVICE_ATTR(period, S_IRUGO | S_IWUSR, iio_pic32_tmr_frequency_show,
		   iio_pic32_tmr_frequency_store);

static struct attribute *iio_pic32_tmr_trigger_attrs[] = {
	&dev_attr_period.attr,
	NULL,
};

static const struct attribute_group iio_pic32_tmr_trigger_attr_group = {
	.attrs = iio_pic32_tmr_trigger_attrs,
};

static const struct attribute_group *iio_pic32_tmr_trigger_attr_groups[] = {
	&iio_pic32_tmr_trigger_attr_group,
	NULL
};

static irqreturn_t iio_pic32_tmr_trigger_isr(int irq, void *devid)
{
	struct pic32_tmr_state *st = devid;

	iio_trigger_poll(st->trig);

	return IRQ_HANDLED;
}

static const struct iio_trigger_ops iio_pic32_tmr_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = iio_pic32_tmr_set_state,
};

static int iio_pic32_tmr_trigger_probe(struct platform_device *pdev)
{
	int ret, trig_period = 4;
	struct pic32_tmr_state *st;
	struct device_node *np = pdev->dev.of_node;

	/* get platform data */


	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	st->timer = pic32_pb_timer_request_by_node(np);
	if (IS_ERR(st->timer)) {
		ret = PTR_ERR(st->timer);
		goto out;
	}

	of_property_read_u32(np, "trig-period-sec", &trig_period);

	st->irq = pic32_pb_timer_get_irq(st->timer);
	st->timer_num = st->timer->id;

	st->trig = devm_iio_trigger_alloc(&pdev->dev,
			"pic32-timer%d-trig", st->timer_num);
	if (!st->trig) {
		ret = -ENOMEM;
		goto out_put_timer;
	}

	st->trig->ops = &iio_pic32_tmr_trigger_ops;
	st->trig->dev.groups = iio_pic32_tmr_trigger_attr_groups;

	iio_trigger_set_drvdata(st->trig, st);

	ret = iio_trigger_register(st->trig);
	if (ret)
		goto out_put_timer;

	ret = devm_request_irq(&pdev->dev, st->irq, iio_pic32_tmr_trigger_isr,
			IRQF_SHARED, st->trig->name, st);
	if (ret) {
		dev_err(&pdev->dev,
			"request IRQ-%d failed", st->irq);
		goto out_unregister_trigger;
	}

	ret = pic32_pb_timer_settime(st->timer, PIC32_TIMER_MAY_RATE,
					trig_period * NSEC_PER_SEC);
	if (ret) {
		dev_err(&pdev->dev, "settimeout failed, err %d\n", ret);
		goto out_unregister_trigger;
	}

	pic32_pb_timer_start(st->timer);

	dev_info(&pdev->dev, "iio trigger PIC32 TMR%d, IRQ-%d",
		 st->timer_num, st->irq);

	platform_set_drvdata(pdev, st);

	return 0;

out_unregister_trigger:
	iio_trigger_unregister(st->trig);
out_put_timer:
	pic32_pb_timer_free(st->timer);
out:
	return ret;
}

static int iio_pic32_tmr_trigger_remove(struct platform_device *pdev)
{
	struct pic32_tmr_state *st = platform_get_drvdata(pdev);

	pic32_pb_timer_stop(st->timer);

	iio_trigger_unregister(st->trig);

	pic32_pb_timer_free(st->timer);
	return 0;
}

static const struct of_device_id of_pic32_tmr_trig_match[] = {
	{.compatible = "microchip,pic32-timer-trig", },
	{}
};
MODULE_DEVICE_TABLE(of, of_pic32_tmr_trig_match);

static struct platform_driver iio_pic32_tmr_trigger_driver = {
	.driver = {
		.name = "iio_pic32_tmr_trigger",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_pic32_tmr_trig_match),
	},
	.probe = iio_pic32_tmr_trigger_probe,
	.remove = iio_pic32_tmr_trigger_remove,
};

module_platform_driver(iio_pic32_tmr_trigger_driver);

MODULE_AUTHOR("Purna Chandra Mandal <purna.mandal@microchip.org>");
MODULE_DESCRIPTION("PIC32 timer based ADC trigger for the iio subsystem");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:iio-trig-pic32-timer");
