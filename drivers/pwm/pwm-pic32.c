/*
 * Driver for PIC32 PWM Controller
 *
 * Copyright (C) 2014 Microchip Technologies
 *		 Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * Licensed under GPLv2.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include <asm/mach-pic32/pbtimer.h>
#include <asm/mach-pic32/ocmp.h>

struct pic32_pwm_chip {
	struct pwm_chip chip;
	struct pic32_ocmp *oc;
	struct pic32_pb_timer *timer;
};

static inline struct pic32_pwm_chip *to_pic32_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct pic32_pwm_chip, chip);
}

static int pic32_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct pic32_pwm_chip *pic32_pwm = to_pic32_pwm_chip(chip);
	int ret;

	dev_vdbg(chip->dev, "dty %d, period %d\n", duty_ns, period_ns);

	/* Sanity check: already enabled and same period? */
	if (test_bit(PWMF_ENABLED, &pwm->flags) && (period_ns != pwm->period)) {
		dev_err(chip->dev, "cannot change PWM period while enabled\n");
		return -EBUSY;
	}

	/* Set period with timer */
	ret = pic32_pb_timer_settime(pic32_pwm->timer,
			PIC32_TIMER_MAY_RATE, period_ns);
	if (ret)
		dev_err(chip->dev, "set_timeout() failed, ret %d\n", ret);

	/* Set duty-cycle & PWM mode with OC */
	ret = pic32_oc_settime(pic32_pwm->oc,
			PIC32_OCM_PWM_DISABLED_FAULT, duty_ns);
	return ret;
}

static int pic32_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	/* PWM supports only NORMAL polarity */
	return (polarity == PWM_POLARITY_NORMAL) ? 0 : -EINVAL;
}

static int pic32_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pic32_pwm_chip *pic32_pwm = to_pic32_pwm_chip(chip);

	/* start oc */
	pic32_oc_start(pic32_pwm->oc);

	/* start timer */
	pic32_pb_timer_start(pic32_pwm->timer);

	dev_vdbg(chip->dev, "%s\n", __func__);
	return 0;
}

static void pic32_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct pic32_pwm_chip *pic32_pwm = to_pic32_pwm_chip(chip);

	/* stop timer */
	pic32_pb_timer_stop(pic32_pwm->timer);

	/* stop OC */
	pic32_oc_stop(pic32_pwm->oc);

	dev_vdbg(chip->dev, "%s\n", __func__);
}

static const struct pwm_ops pic32_pwm_ops = {
	.config = pic32_pwm_config,
	.set_polarity = pic32_pwm_set_polarity,
	.enable = pic32_pwm_enable,
	.disable = pic32_pwm_disable,
	.owner = THIS_MODULE,
};

static int pic32_pwm_probe(struct platform_device *pdev)
{
	struct pic32_pwm_chip *pic32_pwm;
	struct device_node *np;
	int ret;

	pic32_pwm = devm_kzalloc(&pdev->dev, sizeof(*pic32_pwm), GFP_KERNEL);
	if (!pic32_pwm)
		return -ENOMEM;

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "OF: not used\n");
		return -EINVAL;
	}

	/* get general purpose pbtimer */
	pic32_pwm->timer = pic32_pb_timer_request_by_node(np);
	if (IS_ERR(pic32_pwm->timer)) {
		dev_err(&pdev->dev, "pwm: timer not found\n");
		return PTR_ERR(pic32_pwm->timer);
	}

	/* get output-compare */
	pic32_pwm->oc = pic32_oc_request_by_node(np);
	if (IS_ERR(pic32_pwm->oc)) {
		dev_err(&pdev->dev, "pwm: ocmp not found\n");
		ret = PTR_ERR(pic32_pwm->oc);
		goto release_timer;
	}

	/* initalize PWM chip */
	pic32_pwm->chip.dev = &pdev->dev;
	pic32_pwm->chip.ops = &pic32_pwm_ops;
	pic32_pwm->chip.base = -1;
	pic32_pwm->chip.npwm = 1;
	pic32_pwm->chip.of_xlate = of_pwm_xlate_with_flags;
	pic32_pwm->chip.of_pwm_n_cells = 3;

	/* OC: set timebase */
	pic32_oc_set_time_base(pic32_pwm->oc, pic32_pwm->timer);

	/* add pwm-chip */
	ret = pwmchip_add(&pic32_pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip %d\n", ret);
		goto release_oc;
	}

	platform_set_drvdata(pdev, pic32_pwm);

	dev_info(&pdev->dev, "PWM: registered successfully\n");
	return ret;
release_oc:
	pic32_oc_free(pic32_pwm->oc);
release_timer:
	pic32_pb_timer_free(pic32_pwm->timer);
	return ret;
}

static int pic32_pwm_remove(struct platform_device *pdev)
{
	struct pic32_pwm_chip *pic32_pwm = platform_get_drvdata(pdev);

	/* release OC */
	pic32_oc_free(pic32_pwm->oc);

	/* release timer */
	pic32_pb_timer_free(pic32_pwm->timer);

	/* remove pwm */
	return pwmchip_remove(&pic32_pwm->chip);
}

static const struct of_device_id pic32_pwm_dt_ids[] = {
	{ .compatible = "microchip,pic32-pwm",},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, pic32_pwm_dt_ids);

static struct platform_driver pic32_pwm_driver = {
	.driver = {
		.name = "pic32-pwm",
		.of_match_table = of_match_ptr(pic32_pwm_dt_ids),
	},
	.probe = pic32_pwm_probe,
	.remove = pic32_pwm_remove,
};
module_platform_driver(pic32_pwm_driver);

MODULE_ALIAS("platform:pic32-pwm");
MODULE_AUTHOR("Purna Chandra Mandal <purna.mandal@microchip.com>");
MODULE_DESCRIPTION("PIC32 PWM driver");
MODULE_LICENSE("GPL v2");
