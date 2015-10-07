/*
 * Microchip PIC32 ADC Driver.
 *
 * Copyright (C) 2015 Microchip Technology, Inc.
 *
 * Author:
 *	Zane Lindstrom <zane.lindstrom@microchip.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <asm/mach-pic32/pic32.h>
#include <asm/mach-pic32/pbtimer.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define DRV_NAME "pic32-adc"

#define pic32_adc_readl(st, reg) \
	(__raw_readl(st->base + reg))
#define pic32_adc_writel(st, reg, val) \
	(__raw_writel(val, st->base + reg))

#define PIC32_MAX_NUM_CHANNELS 32

#define PIC32_ADCCON1_FSSCLKEN BIT(10)
#define PIC32_ADCCON1_ADCEN BIT(15)
#define PIC32_ADCCON1_STRGSRC(x) (x << 16)
#define PIC32_ADCCON1_SELRES(x) (x << 21)

#define PIC32_ADCCON2_ADCDIV(x) (x << 0)
#define PIC32_ADCCON2_EOSIEN BIT(13)
#define PIC32_ADCCON2_SAMC(x) (x << 16)
#define PIC32_ADCCON2_EOSRDY BIT(29)
#define PIC32_ADCCON2_BGVRRDY BIT(31)

#define PIC32_ADCCON3_INPUTSEL(x) (x << 0)
#define PIC32_ADCCON3_GSWTRG BIT(6)
#define PIC32_ADCCON3_RQCNVRT BIT(8)
#define PIC32_ADCCON3_TRGSUSP BIT(12)
#define PIC32_ADCCON3_DIGEN0 BIT(16)
#define PIC32_ADCCON3_DIGEN1 BIT(17)
#define PIC32_ADCCON3_DIGEN2 BIT(18)
#define PIC32_ADCCON3_DIGEN3 BIT(19)
#define PIC32_ADCCON3_DIGEN4 BIT(20)
#define PIC32_ADCCON3_DIGEN7 BIT(23)
#define PIC32_ADCCON3_CLKDIV(x) (x << 24)
#define PIC32_ADCCON3_ADCCLKSRC(x) (x << 30)

#define PIC32_ADCANCON_ANEN0 BIT(0)
#define PIC32_ADCANCON_ANEN1 BIT(1)
#define PIC32_ADCANCON_ANEN2 BIT(2)
#define PIC32_ADCANCON_ANEN3 BIT(3)
#define PIC32_ADCANCON_ANEN4 BIT(4)
#define PIC32_ADCANCON_ANEN7 BIT(7)
#define PIC32_ADCANCON_WKRDY0 BIT(8)
#define PIC32_ADCANCON_WKRDY1 BIT(9)
#define PIC32_ADCANCON_WKRDY2 BIT(10)
#define PIC32_ADCANCON_WKRDY3 BIT(11)
#define PIC32_ADCANCON_WKRDY4 BIT(12)
#define PIC32_ADCANCON_WKRDY7 BIT(15)
#define PIC32_ADCANCON_WKIEN0 BIT(16)
#define PIC32_ADCANCON_WKIEN1 BIT(17)
#define PIC32_ADCANCON_WKIEN2 BIT(18)
#define PIC32_ADCANCON_WKIEN3 BIT(19)
#define PIC32_ADCANCON_WKIEN4 BIT(20)
#define PIC32_ADCANCON_WKIEN7 BIT(23)
#define PIC32_ADCANCON_WKUP(x) (x << 24)

#define PIC32_ADCTIME_SAMC(x) (x << 0)
#define PIC32_ADCTIME_ADCDIV(x) (x << 16)
#define PIC32_ADCTIME_SELRES BIT(24)

#define PIC32_ADCCON1 0x000
#define PIC32_ADCCON2 0x004
#define PIC32_ADCCON3 0x008
#define PIC32_ADCTRGMODE 0x00C
#define PIC32_ADCIMCON1 0x010
#define PIC32_ADCIMCON2 0x014
#define PIC32_ADCIMCON3 0x018
#define PIC32_ADCGIRQEN1 0x020
#define PIC32_ADCGIRQEN2 0x024
#define PIC32_ADCCSS1 0x028
#define PIC32_ADCCSS2 0x02C
#define PIC32_ADCDSTAT1 0x030
#define PIC32_ADCDSTAT2 0x034
#define PIC32_ADCCMPEN1 0x038
#define PIC32_ADCCMP1 0x03C
#define PIC32_ADCCMPEN2 0x040
#define PIC32_ADCCMP2 0x044
#define PIC32_ADCCMPEN3 0x048
#define PIC32_ADCCMP3 0x04C
#define PIC32_ADCCMPEN4 0x050
#define PIC32_ADCCMP4 0x054
#define PIC32_ADCCMPEN5 0x058
#define PIC32_ADCCMP5 0x05C
#define PIC32_ADCCMPEN6 0x060
#define PIC32_ADCCMP6 0x064
#define PIC32_ADCFLTR1 0x068
#define PIC32_ADCFLTR2 0x06C
#define PIC32_ADCFLTR3 0x070
#define PIC32_ADCFLTR4 0x074
#define PIC32_ADCFLTR5 0x078
#define PIC32_ADCFLTR6 0x07C
#define PIC32_ADCTRG1 0x080
#define PIC32_ADCTRG2 0x084
#define PIC32_ADCTRG3 0x088
#define PIC32_ADCTRG4 0x08C
#define PIC32_ADCTRG5 0x090
#define PIC32_ADCTRG6 0x094
#define PIC32_ADCTRG7 0x098
#define PIC32_ADCTRG8 0x09C
#define PIC32_ADCCMPCON1 0x0A0
#define PIC32_ADCCMPCON2 0x0A4
#define PIC32_ADCCMPCON3 0x0A8
#define PIC32_ADCCMPCON4 0x0AC
#define PIC32_ADCCMPCON5 0x0B0
#define PIC32_ADCCMPCON6 0x0B4
#define PIC32_ADCBASE 0x0C0
#define PIC32_ADCTRGSNS 0x0D0
#define PIC32_ADC0TIME 0x0D4
#define PIC32_ADC1TIME 0x0D8
#define PIC32_ADC2TIME 0x0DC
#define PIC32_ADC3TIME 0x0E0
#define PIC32_ADC4TIME 0x0E4
#define PIC32_ADCEIEN1 0x0F0
#define PIC32_ADCEIEN2 0x0F4
#define PIC32_ADCEISTAT1 0x0F8
#define PIC32_ADCEISTAT2 0x0FC
#define PIC32_ADCANCON 0x100
#define PIC32_ADCSYSCFG0 0x104
#define PIC32_ADCSYSCFG1 0x108
#define PIC32_ADC0CFG 0x600
#define PIC32_ADCDATA0 0xA00

struct pic32_adc_trigger {
	const char	*name;
	u8		value;
	void		*pic32_trig_data;
	bool		is_a_timer;
};

struct pic32_adc_state {
	void __iomem			*base;
	struct iio_trigger		**trig;
	struct pic32_adc_trigger	*trigger_list;
	unsigned long			trigger_number;
	unsigned long			channel_mask;
	u8				num_channels;
	int				irq;
	struct mutex			lock;
	u8				resolution;
	wait_queue_head_t		wq_data_avail;
	bool				done;
	u16				*buffer;
	u16				last_value;
	int				chnb;
	bool				read_raw_mode;
	u32				vref_mv;
	int				timer_sampling_freq;
};

static irqreturn_t pic32_adc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *idev = pf->indio_dev;
	struct pic32_adc_state *st = iio_priv(idev);
	int bit, j = 0;

	/* Read data from each active channel */
	for_each_set_bit(bit, idev->active_scan_mask, PIC32_MAX_NUM_CHANNELS) {
		struct iio_chan_spec const *chan = idev->channels + bit;

		st->buffer[j] = pic32_adc_readl(st, (PIC32_ADCDATA0 +
							(chan->channel * 4)));
		j++;
	}

	iio_push_to_buffers_with_timestamp(idev, st->buffer, pf->timestamp);

	iio_trigger_notify_done(idev->trig);

	enable_irq(st->irq);

	return IRQ_HANDLED;
}

/* Handler for adc scan mode */
static void handle_adc_scan(int irq, struct iio_dev *idev)
{
	disable_irq_nosync(irq);
	iio_trigger_poll(idev->trig);
}

/* Handler for adc single channel read */
static void handle_adc_single_chan_read(int irq, struct iio_dev *idev)
{
	struct pic32_adc_state *st = iio_priv(idev);

	st->last_value = pic32_adc_readl(st, PIC32_ADCDATA0 + (st->chnb * 4));
	st->done = true;
	wake_up_interruptible(&st->wq_data_avail);
}

static irqreturn_t pic32_adc_interrupt(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct pic32_adc_state *st = iio_priv(idev);
	u32 status, stat1, stat2;

	/* Handle requests from read_raw */
	if (st->read_raw_mode) {
		stat1 = pic32_adc_readl(st, PIC32_ADCDSTAT1);
		status = stat1 & BIT(st->chnb);

		if (status > 0)
			handle_adc_single_chan_read(irq, idev);
	} else {
		/* Handle requests from scan mode */
		stat2 = pic32_adc_readl(st, PIC32_ADCCON2);

		if ((stat2 & PIC32_ADCCON2_EOSRDY) > 0)
			handle_adc_scan(irq, idev);
	}

	return IRQ_HANDLED;
}

static int pic32_adc_channel_init(struct iio_dev *idev)
{
	struct pic32_adc_state *st = iio_priv(idev);
	struct iio_chan_spec *chan_array, *timestamp;
	int bit = 0, idx = 0;

	idev->num_channels = st->num_channels + 1;

	chan_array = devm_kzalloc(&idev->dev,
				((idev->num_channels + 1) *
				sizeof(struct iio_chan_spec)),
				GFP_KERNEL);
	if (!chan_array)
		return -ENOMEM;

	for_each_set_bit(bit, &st->channel_mask, PIC32_MAX_NUM_CHANNELS) {
		struct iio_chan_spec *chan = chan_array + idx;

		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = bit;
		chan->scan_index = idx;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = st->resolution;
		chan->scan_type.storagebits = 16;
		chan->info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);
		chan->info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ);
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		idx++;
	}
	timestamp = chan_array + idx;

	timestamp->type = IIO_TIMESTAMP;
	timestamp->channel = -1;
	timestamp->scan_index = idx;
	timestamp->scan_type.sign = 's';
	timestamp->scan_type.realbits = 64;
	timestamp->scan_type.storagebits = 64;

	idev->channels = chan_array;
	return idev->num_channels;
}

static struct pic32_adc_trigger *
pic32_adc_get_trigger_by_name(struct iio_dev *idev,
					struct pic32_adc_trigger *triggers,
					const char *trigger_name)
{
	struct pic32_adc_state *st = iio_priv(idev);
	int i;

	for (i = 0; i < st->trigger_number; i++) {
		char *name = kasprintf(GFP_KERNEL, "%s-dev%d-%s",
					idev->name, idev->id,
					triggers[i].name);
		if (!name)
			return NULL;

		if (strcmp(trigger_name, name) == 0) {
			kfree(name);

			return &triggers[i];
		}

		kfree(name);
	}

	return NULL;
}

static int pic32_adc_start_trigger(struct iio_dev *idev,
					struct pic32_adc_trigger *active_trig)
{
	struct pic32_adc_state *st = iio_priv(idev);
	int ret;

	if (active_trig->is_a_timer) {
		ret = pic32_pb_timer_settime(active_trig->pic32_trig_data,
				PIC32_TIMER_MAY_RATE, st->timer_sampling_freq);
		if (ret) {
			dev_err(&idev->dev, "set_timeout() failed.\n");
			return -EINVAL;
		}

		pic32_pb_timer_start(active_trig->pic32_trig_data);
	}

	return 0;
}

static int pic32_adc_stop_trigger(struct iio_dev *idev,
					struct pic32_adc_trigger *active_trig)
{
	if (active_trig->is_a_timer)
		pic32_pb_timer_stop(active_trig->pic32_trig_data);

	return 0;
}

static int pic32_adc_configure_trigger(struct iio_trigger *trig, bool state)
{
	struct iio_dev *idev = iio_trigger_get_drvdata(trig);
	struct pic32_adc_state *st = iio_priv(idev);
	struct pic32_adc_trigger *active_trigger;
	int value, ret, bit = 0, reg1 = 0, reg2 = 0, reg3 = 0;

	active_trigger = pic32_adc_get_trigger_by_name(idev, st->trigger_list,
							idev->trig->name);
	if (active_trigger == NULL) {
		dev_err(&idev->dev, "Couldn't get trigger.\n");
		return -EINVAL;
	}

	value = active_trigger->value;

	if (state) {
		st->buffer = kmalloc(idev->scan_bytes, GFP_KERNEL);
		if (st->buffer == NULL)
			return -ENOMEM;

		/* Set channels to be scanned when trigger activates */
		for_each_set_bit(bit, idev->active_scan_mask,
						PIC32_MAX_NUM_CHANNELS) {
			struct iio_chan_spec const *chan = idev->channels + bit;

			reg3 |= BIT(chan->channel);
		}
		pic32_adc_writel(st, PIC32_ADCCSS1, reg3);

		/* Enable interrupt generation on end of scan */
		reg2 = pic32_adc_readl(st, PIC32_ADCCON2);
		pic32_adc_writel(st, PIC32_ADCCON2, reg2 |
							PIC32_ADCCON2_EOSIEN);

		/* Set the active trigger */
		reg1 = pic32_adc_readl(st, PIC32_ADCCON1) & 0xFFE0FFFF;
		pic32_adc_writel(st, PIC32_ADCCON1, reg1 | (value << 16));

		ret = pic32_adc_start_trigger(idev, active_trigger);
		if (ret) {
			dev_err(&idev->dev, "Could not start trigger.\n");
			return ret;
		}

	} else {
		pic32_adc_stop_trigger(idev, active_trigger);

		reg2 = pic32_adc_readl(st, PIC32_ADCCON2);
		reg2 &= ~PIC32_ADCCON2_EOSIEN;
		pic32_adc_writel(st, PIC32_ADCCON2, reg2);

		reg1 = pic32_adc_readl(st, PIC32_ADCCON1) & 0xFFE0FFFF;
		pic32_adc_writel(st, PIC32_ADCCON1, reg1);

		pic32_adc_writel(st, PIC32_ADCCSS1, 0);

		kfree(st->buffer);
	}

	return 0;
}

static const struct iio_trigger_ops pic32_adc_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &pic32_adc_configure_trigger,
};

static struct iio_trigger *pic32_adc_allocate_trigger(struct iio_dev *idev,
					struct pic32_adc_trigger *trigger)
{
	struct iio_trigger *trig;
	int ret;

	trig = iio_trigger_alloc("%s-dev%d-%s", idev->name,
				idev->id, trigger->name);
	if (trig == NULL)
		return NULL;

	trig->dev.parent = idev->dev.parent;
	iio_trigger_set_drvdata(trig, idev);
	trig->ops = &pic32_adc_trigger_ops;

	ret = iio_trigger_register(trig);
	if (ret)
		return NULL;

	return trig;
}

static int pic32_adc_trigger_init(struct iio_dev *idev)
{
	struct pic32_adc_state *st = iio_priv(idev);
	int i, ret;

	st->trig = devm_kzalloc(&idev->dev,
				st->trigger_number * sizeof(*st->trig),
				GFP_KERNEL);

	if (st->trig == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	for (i = 0; i < st->trigger_number; i++) {
		st->trig[i] = pic32_adc_allocate_trigger(idev,
					st->trigger_list + i);
		if (st->trig[i] == NULL) {
			dev_err(&idev->dev,
				"Could not allocate trigger %d\n", i);
			ret = -ENOMEM;
			goto error_trigger;
		}
	}

	return 0;

error_trigger:
	for (i--; i >= 0; i--) {
		iio_trigger_unregister(st->trig[i]);
		iio_trigger_free(st->trig[i]);
	}
error_ret:
	return ret;
}

static void pic32_adc_trigger_remove(struct iio_dev *idev)
{
	struct pic32_adc_state *st = iio_priv(idev);
	int i;

	for (i = 0; i < st->trigger_number; i++) {
		iio_trigger_unregister(st->trig[i]);
		iio_trigger_free(st->trig[i]);
	}
}

static int pic32_adc_buffer_init(struct iio_dev *idev)
{
	return iio_triggered_buffer_setup(idev, &iio_pollfunc_store_time,
					&pic32_adc_trigger_handler, NULL);
}

static void pic32_adc_buffer_remove(struct iio_dev *idev)
{
	iio_triggered_buffer_cleanup(idev);
}

static int pic32_init_hardware(struct iio_dev *idev)
{
	struct pic32_adc_state *st = iio_priv(idev);
	u32 ADCCON3_mask = 0, ADCANCON_mask = 0;
	u32 reg = 0;
	int res = 0;

	pic32_adc_writel(st, PIC32_ADCANCON, 0);
	pic32_adc_writel(st, PIC32_ADC0TIME, 0);
	pic32_adc_writel(st, PIC32_ADC1TIME, 0);
	pic32_adc_writel(st, PIC32_ADC2TIME, 0);
	pic32_adc_writel(st, PIC32_ADC3TIME, 0);
	pic32_adc_writel(st, PIC32_ADC4TIME, 0);
	pic32_adc_writel(st, PIC32_ADCTRG1, 0);
	pic32_adc_writel(st, PIC32_ADCTRG2, 0);
	pic32_adc_writel(st, PIC32_ADCTRG3, 0);

	/* Check which ADC cores are to be activated */
	if (st->channel_mask & BIT(0)) {
		ADCCON3_mask |= PIC32_ADCCON3_DIGEN0;
		ADCANCON_mask |= PIC32_ADCANCON_ANEN0;
		pic32_adc_writel(st, PIC32_ADC0TIME, PIC32_ADCTIME_SELRES |
			PIC32_ADCTIME_SAMC(15) | PIC32_ADCTIME_ADCDIV(1));
		reg |= PIC32_ADCANCON_WKRDY0;
	}
	if (st->channel_mask & BIT(1)) {
		ADCCON3_mask |= PIC32_ADCCON3_DIGEN1;
		ADCANCON_mask |= PIC32_ADCANCON_ANEN1;
		pic32_adc_writel(st, PIC32_ADC1TIME, PIC32_ADCTIME_SELRES |
			PIC32_ADCTIME_SAMC(15) | PIC32_ADCTIME_ADCDIV(1));
		reg |= PIC32_ADCANCON_WKRDY1;
	}
	if (st->channel_mask & BIT(2)) {
		ADCCON3_mask |= PIC32_ADCCON3_DIGEN2;
		ADCANCON_mask |= PIC32_ADCANCON_ANEN2;
		pic32_adc_writel(st, PIC32_ADC2TIME, PIC32_ADCTIME_SELRES |
			PIC32_ADCTIME_SAMC(15) | PIC32_ADCTIME_ADCDIV(1));
		reg |= PIC32_ADCANCON_WKRDY2;
	}
	if (st->channel_mask & BIT(3)) {
		ADCCON3_mask |= PIC32_ADCCON3_DIGEN3;
		ADCANCON_mask |= PIC32_ADCANCON_ANEN3;
		pic32_adc_writel(st, PIC32_ADC3TIME, PIC32_ADCTIME_SELRES |
			PIC32_ADCTIME_SAMC(15) | PIC32_ADCTIME_ADCDIV(1));
		reg |= PIC32_ADCANCON_WKRDY3;
	}
	if (st->channel_mask & BIT(4)) {
		ADCCON3_mask |= PIC32_ADCCON3_DIGEN4;
		ADCANCON_mask |= PIC32_ADCANCON_ANEN4;
		pic32_adc_writel(st, PIC32_ADC4TIME, PIC32_ADCTIME_SELRES |
			PIC32_ADCTIME_SAMC(15) | PIC32_ADCTIME_ADCDIV(1));
		reg |= PIC32_ADCANCON_WKRDY4;
	}
	if ((st->channel_mask & 0xFFFFFFE0) > 0) {
		ADCCON3_mask |= PIC32_ADCCON3_DIGEN7;
		ADCANCON_mask |= PIC32_ADCANCON_ANEN7;
		reg |= PIC32_ADCANCON_WKRDY7;
	}

	/* Set the ADC clock source to SYSCLK, turn on digital logic for
	 * ADC cores */
	pic32_adc_writel(st, PIC32_ADCCON3, PIC32_ADCCON3_ADCCLKSRC(1) |
			PIC32_ADCCON3_CLKDIV(0) | ADCCON3_mask);

	/* Set the shared ADC divider */
	pic32_adc_writel(st, PIC32_ADCCON2, PIC32_ADCCON2_ADCDIV(1) |
			PIC32_ADCCON2_SAMC(15));

	/* Turn on analog circuitry for ADC cores */
	pic32_adc_writel(st, PIC32_ADCANCON, PIC32_ADCANCON_WKUP(10) |
			ADCANCON_mask);

	/* Enable scan trigger on class 1 & 2 channels */
	pic32_adc_writel(st, PIC32_ADCTRG1, 0x03030303);
	pic32_adc_writel(st, PIC32_ADCTRG2, 0x03030303);
	pic32_adc_writel(st, PIC32_ADCTRG3, 0x03030303);

	if (st->resolution == 6)
		res = 0;
	else if (st->resolution == 8)
		res = 1;
	else if (st->resolution == 10)
		res = 2;
	else if (st->resolution == 12)
		res = 3;

	/* 12-bit resolution, source clock synchronous to SYSCLK, turn on
	 * ADC module */
	pic32_adc_writel(st, PIC32_ADCCON1, PIC32_ADCCON1_SELRES(res) |
			PIC32_ADCCON1_FSSCLKEN | PIC32_ADCCON1_ADCEN);

	/* Wait for ADC cores to warmup */
	if ((pic32_adc_readl(st, PIC32_ADCANCON) & reg) != reg) {
		msleep(20);
		if ((pic32_adc_readl(st, PIC32_ADCANCON) & reg) != reg)
			return -ETIMEDOUT;
	}

	return 0;
}

static int pic32_adc_write_raw(struct iio_dev *idev, struct iio_chan_spec
				const *chan, int val, int val2, long mask)
{
	struct pic32_adc_state *st = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		st->timer_sampling_freq = val;
		return 0;

	default:
		break;
	}
	return -EINVAL;
}

static int pic32_adc_read_raw(struct iio_dev *idev, struct iio_chan_spec
				const *chan, int *val, int *val2, long mask)
{
	struct pic32_adc_state *st = iio_priv(idev);
	int ret, reg = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);

		st->chnb = chan->channel;
		st->read_raw_mode = true;

		/* Enable interrupts for current channel */
		pic32_adc_writel(st, PIC32_ADCGIRQEN1, BIT(chan->channel));

		/* Start conversion of channel */
		reg = pic32_adc_readl(st, PIC32_ADCCON3);
		pic32_adc_writel(st, PIC32_ADCCON3, reg | PIC32_ADCCON3_TRGSUSP
			| PIC32_ADCCON3_INPUTSEL(chan->channel)
			| PIC32_ADCCON3_RQCNVRT);

		/* Wait for conversion to finish */
		ret = wait_event_interruptible_timeout(st->wq_data_avail,
							st->done,
							msecs_to_jiffies(1000));
		if (ret == 0)
			ret = -ETIMEDOUT;
		if (ret < 0) {
			mutex_unlock(&st->lock);
			return ret;
		}

		*val = st->last_value;

		pic32_adc_writel(st, PIC32_ADCGIRQEN1, 0);
		pic32_adc_writel(st, PIC32_ADCCON3, reg);

		st->read_raw_mode = false;
		st->chnb = 0;
		st->last_value = 0;
		st->done = false;

		mutex_unlock(&st->lock);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->timer_sampling_freq;
		return IIO_VAL_INT;

	default:
		break;
	}
	return -EINVAL;
}

static int pic32_adc_probe_dt(struct pic32_adc_state *st,
				struct platform_device *pdev)
{
	struct iio_dev *idev = iio_priv_to_dev(st);
	struct device_node *node = pdev->dev.of_node;
	struct device_node *trig_node;
	int i = 0, ret;
	u32 prop;

	if (!node)
		return -EINVAL;

	if (of_property_read_u32(node, "enabled-channel-mask", &prop)) {
		dev_err(&idev->dev,
			"Missing channels-used property in the DT.\n");
		ret = -EINVAL;
		goto error_ret;
	}
	st->channel_mask = prop;
	prop = 0;

	/* Check that at least one channel was selected  */
	if ((st->channel_mask) == 0) {
		dev_err(&idev->dev, "No channels were selected.\n");
		ret = -EINVAL;
		goto error_ret;
	}

	st->num_channels = bitmap_weight(&st->channel_mask,
							PIC32_MAX_NUM_CHANNELS);

	if (of_property_read_u32(node, "resolution", &prop)) {
		dev_err(&idev->dev, "Missing resolution property in the DT.\n");
		ret = -EINVAL;
		goto error_ret;
	}
	st->resolution = prop;
	prop = 0;

	/* Check that resolution is 6, 8, 10 or 12 bits  */
	if ((st->resolution != 6) && (st->resolution != 8) &&
	    (st->resolution != 10) && (st->resolution != 12)) {
		dev_err(&idev->dev,
				"The selected resolution is unavailable.\n");
		ret = -EINVAL;
		goto error_ret;
	}

	if (of_property_read_u32(node, "vref", &prop)) {
		dev_err(&idev->dev, "Missing vref property in the DT.\n");
		ret = -EINVAL;
		goto error_ret;
	}
	st->vref_mv = prop;
	prop = 0;

	st->trigger_number = of_get_child_count(node);
	st->trigger_list = devm_kzalloc(&idev->dev, st->trigger_number *
					sizeof(struct pic32_adc_trigger),
					GFP_KERNEL);
	if (!st->trigger_list) {
		dev_err(&idev->dev,
				"Could not allocate trigger list memory.\n");
		ret = -ENOMEM;
		goto error_ret;
	}

	for_each_child_of_node(node, trig_node) {
		struct pic32_adc_trigger *trig = st->trigger_list + i;
		const char *name;

		if (of_property_read_string(trig_node, "trigger-name", &name)) {
			dev_err(&idev->dev,
				"Missing trigger-name property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		trig->name = name;

		if (of_property_read_u32(trig_node, "trigger-value", &prop)) {
			dev_err(&idev->dev,
				"Missing trigger-value property in the DT.\n");
			ret = -EINVAL;
			goto error_ret;
		}
		trig->value = prop;

		if (of_find_property(trig_node, "microchip,timer", NULL)) {
			trig->is_a_timer = true;
			trig->pic32_trig_data =
				pic32_pb_timer_request_by_node(trig_node);
			if (IS_ERR(trig->pic32_trig_data)) {
				dev_err(&idev->dev,
						"adc: pbtimer not found\n");
				return PTR_ERR(trig->pic32_trig_data);
			}
		} else {
			trig->is_a_timer = false;
			trig->pic32_trig_data = NULL;
		}

		i++;
	}

	return 0;

error_ret:
	return ret;
}

static const struct iio_info pic32_adc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &pic32_adc_read_raw,
	.write_raw = &pic32_adc_write_raw,
};

static int pic32_adc_probe(struct platform_device *pdev)
{
	int ret;
	struct iio_dev *idev;
	struct pic32_adc_state *st;
	struct resource *res;

	idev = devm_iio_device_alloc(&pdev->dev,
					sizeof(struct pic32_adc_state));
	if (!idev)
		return -ENOMEM;

	st = iio_priv(idev);

	ret = pic32_adc_probe_dt(st, pdev);
	if (ret) {
		dev_err(&pdev->dev, "No platform data available.\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, idev);

	idev->dev.parent = &pdev->dev;
	idev->name = DRV_NAME;
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &pic32_adc_info;

	st->irq = platform_get_irq(pdev, 0);
	if (st->irq < 0) {
		dev_err(&pdev->dev, "No IRQ ID is designated\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	st->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(st->base))
		return PTR_ERR(st->base);

	/* Clear IRQs and ADC settings before initialization */
	pic32_adc_writel(st, PIC32_ADCCON1, 0);
	pic32_adc_writel(st, PIC32_ADCCON2, 0);
	pic32_adc_writel(st, PIC32_ADCCON3, 0);
	pic32_adc_writel(st, PIC32_ADCGIRQEN1, 0);
	pic32_adc_writel(st, PIC32_ADCGIRQEN2, 0);

	ret = request_irq(st->irq, pic32_adc_interrupt, 0,
				pdev->dev.driver->name, idev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to allocate IRQ.\n");
		return ret;
	}

	ret = pic32_adc_channel_init(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't initialize the channels.\n");
		goto error_free_irq;
	}

	init_waitqueue_head(&st->wq_data_avail);
	mutex_init(&st->lock);

	ret = pic32_adc_buffer_init(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't initialize the buffer.\n");
		goto error_free_irq;
	}

	ret = pic32_adc_trigger_init(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't setup the triggers.\n");
		pic32_adc_buffer_remove(idev);
		goto error_free_irq;
	}

	ret = pic32_init_hardware(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't initialize the adc hardware.\n");
		goto error_iio_device_register;
	}

	ret = iio_device_register(idev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_iio_device_register;
	}

	dev_info(&pdev->dev, "adc driver initialized.\n");

	return 0;

error_iio_device_register:
	pic32_adc_trigger_remove(idev);
	pic32_adc_buffer_remove(idev);
error_free_irq:
	free_irq(st->irq, idev);
	return ret;
}

static int pic32_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *idev = platform_get_drvdata(pdev);
	struct pic32_adc_state *st = iio_priv(idev);

	iio_device_unregister(idev);
	pic32_adc_trigger_remove(idev);
	pic32_adc_buffer_remove(idev);

	free_irq(st->irq, idev);

	return 0;
}

static const struct of_device_id pic32_adc_dt_ids[] = {
	{ .compatible = "microchip,pic32-adc",},
	{},
};
MODULE_DEVICE_TABLE(of, pic32_adc_dt_ids);

static struct platform_driver pic32_adc_driver = {
	.probe = pic32_adc_probe,
	.remove = pic32_adc_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(pic32_adc_dt_ids),
	},
};

module_platform_driver(pic32_adc_driver);

MODULE_AUTHOR("Zane Lindstrom <zane.lindstrom@microchip.com>");
MODULE_DESCRIPTION("Microchip PIC32 ADC Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
