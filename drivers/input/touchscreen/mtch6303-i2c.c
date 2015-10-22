/*
 * Microchip MTCH6303 I2C Touchscreen Driver
 *
 * Copyright (c) 2015 Microchip Technology, Inc.
 *
 * http://www.microchip.com/mtouch
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#define MTCH6303_NAME "mtch6303_i2c"

#define MAX_WIDTH 0x7fff
#define MAX_HEIGHT 0x7fff
#define POINTER_MODE_ABSOLUTE 0
#define POINTER_MODE_RELATIVE 1
#define MAX_TOUCHES 10

struct mtch6303_i2c_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct work;
	struct timer_list polling_timer;
	int pointer_mode;

	char tx_buf[100];
	char rx_buf[100];
	int rx_len;
	int firstTouchID;
};

static ssize_t pointer_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch6303_i2c_priv *priv = i2c_get_clientdata(client);

	return sprintf(buf, "%d", priv->pointer_mode);
}

static ssize_t pointer_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch6303_i2c_priv *priv = i2c_get_clientdata(client);

	sscanf(buf, "%d", &priv->pointer_mode);

	return count;
}

static ssize_t receive_buffer_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch6303_i2c_priv *priv = i2c_get_clientdata(client);
	int ret;

	ret = i2c_master_recv(client, priv->rx_buf, priv->rx_len);
	if (ret < 0) {
		dev_err(&client->dev, "error reading bytes!\n");
		return 0;
	}

	memcpy(buf, priv->rx_buf, ret);

	return ret;
}

static ssize_t receive_buffer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch6303_i2c_priv *priv = i2c_get_clientdata(client);

	return sscanf(buf,"%d", &priv->rx_len);
}

static ssize_t send_buffer_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch6303_i2c_priv *priv = i2c_get_clientdata(client);

	return sprintf(buf, "%s", priv->tx_buf);
}

static ssize_t send_buffer_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mtch6303_i2c_priv *priv = i2c_get_clientdata(client);

	char tx_buf[200];
	unsigned char tx_len;
	unsigned char address;
	int i;

	if (!buf)
		return 0;

	address = buf[0];
	tx_len = buf[1];

	memset(tx_buf, 88, sizeof(tx_buf));
	tx_buf[0] = address;

	/* re-create buffer without number of bytes element */
	for (i = 1;i < tx_len + 1;i++)
	{
		tx_buf[i] = buf[i + 1];
	}

	i2c_master_send(client, tx_buf, tx_len + 1);

	for (i = 0;i < tx_len; i++)
	{
		if (i == 0)
			sprintf(priv->tx_buf,"0x%02x",buf[i]);
		else
			sprintf(priv->tx_buf, "%s 0x%02x",
				priv->tx_buf, buf[i]);
	}

	return i + 2;
}

static DEVICE_ATTR_RW(pointer_mode);
static DEVICE_ATTR_RW(receive_buffer);
static DEVICE_ATTR_RW(send_buffer);

static struct attribute *mtch6303_attrs[] = {
	&dev_attr_pointer_mode.attr,
	&dev_attr_receive_buffer.attr,
	&dev_attr_send_buffer.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = mtch6303_attrs,
};

static void mtch6303_process_touch(struct mtch6303_i2c_priv* priv, unsigned char* buf,
				int numTouches, int slot)
{
	struct i2c_client *client = priv->client;
	int id = buf[1];
	int x = (buf[3] << 8) | buf[2];
	int y = (buf[5] << 8) | buf[4];
	int ts = buf[0] & 1;

	if (id > 15) {
		dev_err(&client->dev, "id %d is bigger than 15!\n", slot);
		return;
	}

	if ((slot & 0xff) > MAX_TOUCHES) {
		dev_err(&client->dev, "invalid slot ID %d!\n", slot);
		return;
	}

	input_mt_slot(priv->input, slot);

	if (ts) {
		input_report_abs(priv->input, ABS_MT_TRACKING_ID, id);
		input_report_abs(priv->input, ABS_MT_POSITION_X, x);
		input_report_abs(priv->input, ABS_MT_POSITION_Y, y);
	}
	else {
		input_report_abs(priv->input, ABS_MT_TRACKING_ID, -1);
	}
}

static void mtch6303_decode_packet(struct mtch6303_i2c_priv* priv,
				   unsigned char* buf, int len)
{
	struct i2c_client *client = priv->client;
	int i;
	int numTouches = 0;
	int endOfValidBytesIndex = 0;
	int firstTouchOffset = 1;

	numTouches = buf[0] & 0x0f;

	if (numTouches > MAX_TOUCHES) {
		dev_err(&client->dev, "invalid number of touches\n");
		return;
	}

	if (numTouches == 0)
		priv->firstTouchID = -1;

	if (priv->firstTouchID == -1) {
		/* the first touch ID will always be located at index 1 */
		priv->firstTouchID = buf[2];
	} else {
		for (i = 0;i < numTouches;i++) {
			if (buf[i*6+2] == priv->firstTouchID) {
				firstTouchOffset = i*6+1;
				break;
			}
		}

		/* if first touch finger is released, then reassign firstTouchID */
		if (i == numTouches && numTouches > 0) {
			priv->firstTouchID = buf[2];
			dev_dbg(&client->dev,
				"firstTouchOffset not found. Num touches %d. New first ID %d.\n",
				numTouches, priv->firstTouchID);
		}
	}

	endOfValidBytesIndex = numTouches * 6 + 1;

	for (i = 7;i <= 0x3d; i += 6) {
		/* ensure unused slots are set to touch up state by clearing touch state byte */
		if (i > endOfValidBytesIndex) {
			buf[i-6] = 0;
		}
		mtch6303_process_touch(priv, &buf[i-6],
				numTouches, ((i-1)/6) - 1);
	}

	input_report_key(priv->input, BTN_TOUCH, buf[firstTouchOffset] & 1);

	if (buf[firstTouchOffset] & 1) {
		input_report_abs(priv->input, ABS_X,
				(buf[firstTouchOffset+3]<<8) |
				buf[firstTouchOffset + 2]);
		input_report_abs(priv->input, ABS_Y,
				(buf[firstTouchOffset+5]<<8) |
				buf[firstTouchOffset + 4]);
	} else {
		priv->firstTouchID = -1;
	}

	input_sync(priv->input);
}

static void mtch6303_i2c_readdata(struct work_struct *work)
{
	struct mtch6303_i2c_priv *priv =
		container_of(work, struct mtch6303_i2c_priv, work);
	char rx_buf[150];
	size_t rx_len = 0x3d;
	int ret;
	char startAddress = 0;

	i2c_master_send(priv->client, &startAddress, 1);
	ret = i2c_master_recv(priv->client, rx_buf, rx_len);
	if (ret < 0) {
		dev_err(&priv->client->dev, "error reading touch bytes %d\n",
			ret);
		return;
	}

	mtch6303_decode_packet(priv, rx_buf, ret);
}

static irqreturn_t mtch6303_ts_interrupt(int irq, void *dev_id)
{
	struct mtch6303_i2c_priv *priv = dev_id;

	if (priv)
		schedule_work(&priv->work);

	return IRQ_HANDLED;
}

static void polling_timer_callback(unsigned long data)
{
	struct mtch6303_i2c_priv *priv = (struct mtch6303_i2c_priv *)data;

	if (priv) {
		schedule_work(&priv->work);

		mod_timer(&priv->polling_timer, jiffies + msecs_to_jiffies(20));
	}
}

static void setup_polling_timer(struct mtch6303_i2c_priv *priv)
{
	struct i2c_client *client = priv->client;
	int ret;

	setup_timer(&priv->polling_timer, polling_timer_callback,
		(unsigned long)priv);

	ret = mod_timer(&priv->polling_timer, jiffies + msecs_to_jiffies(20));
	if (ret)
		dev_err(&client->dev, "failed to start timer\n");
}

static int mtch6303_create_input_device(struct mtch6303_i2c_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	priv->input = input;

	input->name = "MTCH6303 Touchscreen";
	input->id.bustype = BUS_I2C;

	if (priv->pointer_mode == POINTER_MODE_RELATIVE) {
		dev_dbg(dev, "using relative mode\n");

		input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
		input->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT);
		input->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y) |
			BIT_MASK(REL_MISC);
	} else {
		dev_dbg(dev, "using absolute mode\n");

		input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

		input_set_abs_params(input, ABS_X, 0, MAX_WIDTH, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, MAX_HEIGHT, 0, 0);
		input_mt_init_slots(input, MAX_TOUCHES,
				INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
		input_set_abs_params(input, ABS_MT_POSITION_X, 0,
				MAX_WIDTH, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
				MAX_HEIGHT, 0, 0);
		input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(dev, "failed to register input device: %d\n", ret);
		return ret;
	}

	return 0;
}

static int mtch6303_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct mtch6303_i2c_priv *priv;
	unsigned long irqflags;
	int ret;

	dev_dbg(&client->dev, "adapter=%d, client irq: %d\n",
		client->adapter->nr, client->irq);

	/* Check if the I2C function is ok in this adaptor */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENXIO;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pointer_mode = POINTER_MODE_ABSOLUTE;
	priv->rx_len = 1;
	priv->firstTouchID = -1;

	priv->client = client;
	INIT_WORK(&priv->work, mtch6303_i2c_readdata);
	i2c_set_clientdata(client, priv);

	ret = mtch6303_create_input_device(priv);
	if (ret)
		return ret;

	if (!client->irq) {
		setup_polling_timer(priv);
	} else {

		irqflags = client->dev.of_node ? 0 : IRQF_TRIGGER_FALLING;

		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						mtch6303_ts_interrupt,
						irqflags | IRQF_ONESHOT,
						client->name, priv);

		if (ret) {
			dev_err(&client->dev, "request irq failed: %d\n", ret);
			return ret;
		}

		/* purge any pending events */
		schedule_work(&priv->work);
	}

	ret = sysfs_create_group(&client->dev.kobj, &attr_group);
	if (ret) {
		dev_err(&client->dev, "create sysfs failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int mtch6303_i2c_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &attr_group);

	return 0;
}

static const struct i2c_device_id mtch6303_i2c_id[] = {
	{ "mtch6303_i2c", 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id mtch6303_i2c_dt_ids[] = {
        { .compatible = "microchip,mtch6303_i2c", },
        { }
};
MODULE_DEVICE_TABLE(of, mtch6303_i2c_dt_ids);

/*
 * The device tree based i2c loader looks for
 * "i2c:" + second_component_of(property("compatible"))
 * and therefore we need an alias to be found.
 */
MODULE_ALIAS("i2c:mtch6303_i2c");
#endif

static struct i2c_driver mtch6303_i2c_driver = {
	.driver = {
		.name	= MTCH6303_NAME,
                .owner  = THIS_MODULE,
                .of_match_table = of_match_ptr(mtch6303_i2c_dt_ids),
	},
	.probe		= mtch6303_i2c_probe,
	.remove		= mtch6303_i2c_remove,
	.id_table	= mtch6303_i2c_id,
};

module_i2c_driver(mtch6303_i2c_driver);

MODULE_AUTHOR("Steve Grahovac <steve.grahovac@microchip.com>");
MODULE_DESCRIPTION("MTCH6303 touchscreen I2C bus driver");
MODULE_LICENSE("GPL");
