/*
 * Joshua Henderson, joshua.henderson@microchip.com
 * Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <asm/mach-pic32/pic32.h>

#define I2CCON			0x0000
#define I2CCON_SCIE		BIT(21)
#define I2CCON_ON		BIT(15)
#define I2CCON_FRZ		BIT(14)
#define I2CCON_SIDL		BIT(13)
#define I2CCON_SCLREL		BIT(12)
#define I2CCON_STRICT		BIT(11)
#define I2CCON_A10M		BIT(10)
#define I2CCON_DISSLW		BIT(9)
#define I2CCON_SMEN		BIT(8)
#define I2CCON_GCEN		BIT(7)
#define I2CCON_STREN		BIT(6)
#define I2CCON_ACKDT		BIT(5)
#define I2CCON_ACKEN		BIT(4)
#define I2CCON_RCEN		BIT(3)
#define I2CCON_PEN		BIT(2)
#define I2CCON_RSEN		BIT(1)
#define I2CCON_SEN		BIT(0)

#define I2CSTAT			0x0010
#define I2CSTAT_ACKSTAT		BIT(15)
#define I2CSTAT_TRSTAT		BIT(14)
#define I2CSTAT_BCL		BIT(10)
#define I2CSTAT_GCSTAT		BIT(9)
#define I2CSTAT_ADD10		BIT(8)
#define I2CSTAT_IWCOL		BIT(7)
#define I2CSTAT_I2COV		BIT(6)
#define I2CSTAT_DA		BIT(5)
#define I2CSTAT_P		BIT(4)
#define I2CSTAT_S		BIT(3)
#define I2CSTAT_RW		BIT(2)
#define I2CSTAT_RBF		BIT(1)
#define I2CSTAT_TBF		BIT(0)

#define I2CADD			0x0020
#define I2CMSK			0x0030
#define I2CBRG			0x0040
#define I2CTRN			0x0050
#define I2CRCV			0x0060

#define DRIVER_NAME		"pic32-i2c"

#define PIC32_I2C_SPEED_MAX	400000 /* 400kHz */
#define PIC32_I2C_SPEED_DEFAULT	100000 /* 100kHz */

#define PIC32_I2C_TIMEOUT	400

#define pic32_i2c_readreg(offset) __raw_readl(id->membase + offset)
#define pic32_i2c_writereg(val, offset)	__raw_writel(val, id->membase + offset)

enum {
	PIC32_I2C_STATE_IDLE,
	PIC32_I2C_STATE_START,
	PIC32_I2C_STATE_ADDR_DONE,
	PIC32_I2C_STATE_TX,
	PIC32_I2C_STATE_RX,
	PIC32_I2C_STATE_ACK
};

struct pic32_i2c {
	void __iomem *membase;
	struct i2c_adapter adap;
	struct i2c_msg *p_msg;
	struct completion xfer_done;
	struct device *device;
	int irq;
	unsigned int i2c_speed;
	struct clk *clk;
	unsigned long bus_rate;
	int error;
	int state;

	unsigned char *send_buf;
	unsigned int send_count;

	unsigned char *recv_buf;
	unsigned int recv_count;
};

static inline void pic32_i2c_disable(struct pic32_i2c *id)
{
	pic32_i2c_writereg(I2CCON_ON, PIC32_CLR(I2CCON));
}

static inline void pic32_i2c_enable(struct pic32_i2c *id)
{
	pic32_i2c_writereg(I2CCON_ON, PIC32_SET(I2CCON));
}

static inline int pic32_i2c_send_addr(struct pic32_i2c *id,
				struct i2c_msg *msg)
{
	u32 addr;

	addr = msg->addr << 1;

	if (msg->flags & I2C_M_RD)
		addr |= 1;

	pic32_i2c_writereg(addr, I2CTRN);

	return 0;
}

static inline void pic32_i2c_enable_receiver(struct pic32_i2c *id)
{
	id->state = PIC32_I2C_STATE_RX;
	pic32_i2c_writereg(I2CCON_RCEN, PIC32_SET(I2CCON));
}

static inline void pic32_i2c_ack(struct pic32_i2c *id, int ack)
{
	id->state = PIC32_I2C_STATE_ACK;
	if (ack)
		/* ACK */
		pic32_i2c_writereg(I2CCON_ACKDT, PIC32_CLR(I2CCON));
	else
		/* NACK */
		pic32_i2c_writereg(I2CCON_ACKDT, PIC32_SET(I2CCON));

	pic32_i2c_writereg(I2CCON_ACKEN, PIC32_SET(I2CCON));
}

static inline void pic32_i2c_tx(struct pic32_i2c *id)
{
	id->state = PIC32_I2C_STATE_TX;
	id->send_count--;
	pic32_i2c_writereg(*(id->send_buf++), I2CTRN);
}

static inline void pic32_i2c_stop(struct pic32_i2c *id)
{
	u32 timeout = 1000;

	pic32_i2c_writereg(I2CCON_PEN, PIC32_SET(I2CCON));

	while ((pic32_i2c_readreg(I2CCON) & I2CCON_PEN) && --timeout)
		cpu_relax();
}

static irqreturn_t pic32_i2c_isr(int irq, void *ptr)
{
	u32 status;
	u32 ch;
	struct pic32_i2c *id = ptr;
	int done_flag = 0;

	status = pic32_i2c_readreg(I2CSTAT);

	/* Bus collision */
	if (status & I2CSTAT_BCL) {
		pic32_i2c_writereg(I2CSTAT_BCL, PIC32_CLR(I2CSTAT));
		id->error = -EAGAIN;
		done_flag = 1;
		dev_dbg(id->device, "bus collision\n");
		goto out;
	}

	/* Write collision */
	if (status & I2CSTAT_IWCOL) {
		pic32_i2c_writereg(I2CSTAT_IWCOL, PIC32_CLR(I2CSTAT));
		id->error = -EAGAIN;
		done_flag = 1;
		dev_dbg(id->device, "write collision\n");
		goto out;
	}

	/* Receive overflow */
	if (status & I2CSTAT_I2COV) {
		pic32_i2c_writereg(I2CSTAT_I2COV, PIC32_CLR(I2CSTAT));
		id->error = -EAGAIN;
		done_flag = 1;
		dev_dbg(id->device, "recv overflow\n");
		goto out;
	}

	switch (id->state) {
	case PIC32_I2C_STATE_IDLE:
		break;

	case PIC32_I2C_STATE_START:

		id->state = PIC32_I2C_STATE_ADDR_DONE;

		/* Send slave address */
		if (!(id->p_msg->flags & I2C_M_NOSTART)) {
			pic32_i2c_send_addr(id, id->p_msg);
			goto out;
		}

		/* Fallthrough */

	case PIC32_I2C_STATE_ADDR_DONE:

		/* Transmit complete and ACK received */
		if (!(status & I2CSTAT_TRSTAT) &&
			!(status & I2CSTAT_ACKSTAT)) {

			if (!id->recv_buf)
				/* Transmit first byte */
				pic32_i2c_tx(id);
			else
				/* Enable receiver for first byte */
				pic32_i2c_enable_receiver(id);

		} else {
			id->error = -EIO;
			done_flag = 1;
			dev_dbg(id->device, "slave not responding to addr\n");
		}

		break;

	case PIC32_I2C_STATE_TX:

		/* Transmit complete and ACK received */
		if (!(status & I2CSTAT_TRSTAT) &&
			!(status & I2CSTAT_ACKSTAT)) {
			if (id->send_count) {
				/* Transmitter ready */
				if (!(status & I2CSTAT_TBF)) {
					pic32_i2c_tx(id);
				} else {
					id->error = -EIO;
					done_flag = 1;
					dev_dbg(id->device, "tx not ready %d\n",
						id->send_count);
				}
			} else {
				done_flag = 1;
			}
		} else {
			id->error = -EIO;
			done_flag = 1;
			dev_dbg(id->device, "slave not responding to tx\n");
		}

		break;

	case PIC32_I2C_STATE_RX:

		/* Received byte is available */
		if (status & I2CSTAT_RBF) {
			if (id->recv_count) {
				*(id->recv_buf)++ = pic32_i2c_readreg(I2CRCV);
				--id->recv_count;
				pic32_i2c_ack(id, id->recv_count != 0);
			} else {
				/* dummy read */
				ch = pic32_i2c_readreg(I2CRCV);
				/* NACK */
				pic32_i2c_ack(id, 0);
				dev_dbg(id->device, "unknown rx int\n");
			}
		} else {
			id->error = -EIO;
			done_flag = 1;
			dev_dbg(id->device, "slave not responding to rx\n");
		}

		break;

	case PIC32_I2C_STATE_ACK:

		/* ACK has completed */
		if (!id->recv_count)
			done_flag = 1;
		else
			pic32_i2c_enable_receiver(id);

		break;
	}

out:
	if (done_flag) {
		id->state = PIC32_I2C_STATE_IDLE;
		complete(&id->xfer_done);
	}

	return IRQ_HANDLED;
}

static int pic32_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
				int num)
{
	int ret, i;
	struct pic32_i2c *id = adap->algo_data;

	/* Retry if not idle */
	if (pic32_i2c_readreg(I2CCON) & I2CCON_ON)
		return -EAGAIN;

	pic32_i2c_enable(id);

	/* Process the msg one by one */
	for (i = 0; i < num; i++, msgs++) {

		id->state = PIC32_I2C_STATE_IDLE;
		id->error = 0;
		id->p_msg = msgs;

		/* Check for the R/W flag on each msg */
		if (msgs->flags & I2C_M_RD) {
			id->recv_buf = id->p_msg->buf;
			id->recv_count = id->p_msg->len;
		} else {
			id->recv_buf = NULL;
			id->send_buf = id->p_msg->buf;
			id->send_count = id->p_msg->len;
		}

		reinit_completion(&id->xfer_done);

		id->state = PIC32_I2C_STATE_START;

		if (i == 0) {
			/* Send start condition */
			pic32_i2c_writereg(I2CCON_SEN, PIC32_SET(I2CCON));
		} else if (!(msgs->flags & I2C_M_NOSTART)) {
			/* Send repeated condition */
			pic32_i2c_writereg(I2CCON_RSEN, PIC32_SET(I2CCON));
		}

		/* Wait for signal of completion */
		ret = wait_for_completion_timeout(&id->xfer_done,
						adap->timeout);
		if (!ret) {
			dev_dbg(id->adap.dev.parent,
				"timeout waiting on completion\n");
			ret = -ETIMEDOUT;
			goto xfer_send_stop;
		}

		if (id->error) {
			ret = id->error;
			goto xfer_send_stop;
		}
	}

	ret = num;

xfer_send_stop:

	pic32_i2c_stop(id);

	pic32_i2c_disable(id);

	return ret;
}

static u32 pic32_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_NOSTART;
}

static const struct i2c_algorithm pic32_i2c_algo = {
	.master_xfer	= pic32_i2c_master_xfer,
	.functionality	= pic32_i2c_func,
};

static int pic32_i2c_setbrg(struct pic32_i2c *id)
{
	u32 reload;

	/* I2C baud rate generator reload value calculation:
	 *   I2CxBRG = (1 / (2 * FSCK) - TPGD) * PBCLK - 2
	 *
	 * where TPGD recommended value is 0.000000104
	 *
	 * Converted to approximate calculation:
	 *   I2CxBRG = (PBCLK / (2 * FSCK)) - (PBCLK / 10000000) - 2
	 */
	reload = (id->bus_rate / (2 * id->i2c_speed)) -
		(id->bus_rate / 10000000) - 2;

	if (id->i2c_speed > PIC32_I2C_SPEED_MAX || (reload & 0xFFFF0000)) {
		pr_err("%d hz bus speed not supported\n", id->i2c_speed);
		return -EINVAL;
	}

	pic32_i2c_writereg(reload, I2CBRG);

	return 0;
}

static int pic32_i2c_get_ofdata(struct pic32_i2c *id)
{
	uint32_t speed;
	struct device *dev = id->device;
	struct device_node *node = dev->of_node;
	int ret;

	ret = of_property_read_u32(node, "clock-frequency", &speed);
	if (ret) {
		dev_warn(dev, "No I2C speed selected, using 100kHz\n");
		speed = PIC32_I2C_SPEED_DEFAULT;
	} else if (speed > PIC32_I2C_SPEED_MAX || speed == 0) {
		dev_warn(dev, "Invalid I2C speed selected, using 100kHz\n");
		speed = PIC32_I2C_SPEED_DEFAULT;
	}

	id->i2c_speed = speed;

	return 0;
}

static const struct of_device_id pic32_i2c_dt_ids[] = {
	{ .compatible = "microchip,pic32-i2c" },
	{ .compatible = "microchip,pic32-i2c" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_i2c_dt_ids);

static int pic32_i2c_probe(struct platform_device *pdev)
{
	struct resource *r_mem;
	struct pic32_i2c *id;
	int ret;
	int irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}

	id = devm_kzalloc(&pdev->dev, sizeof(*id), GFP_KERNEL);
	if (!id)
		return -ENOMEM;

	platform_set_drvdata(pdev, id);

	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	id->membase = devm_ioremap_resource(&pdev->dev, r_mem);
	if (IS_ERR(id->membase))
		return PTR_ERR(id->membase);

	id->irq = irq;
	id->adap.dev.of_node = pdev->dev.of_node;
	id->adap.algo = &pic32_i2c_algo;
	id->adap.timeout = msecs_to_jiffies(PIC32_I2C_TIMEOUT);
	id->adap.retries = 3;
	id->adap.algo_data = id;
	id->adap.dev.parent = &pdev->dev;
	id->adap.nr = pdev->id;
	id->device = &pdev->dev;
	init_completion(&id->xfer_done);
	snprintf(id->adap.name, sizeof(id->adap.name),
		 "PIC32 I2C at %08lx", (unsigned long)r_mem->start);

	id->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(id->clk)) {
		dev_err(&pdev->dev, "input clock not found.\n");
		return PTR_ERR(id->clk);
	}
	ret = clk_prepare_enable(id->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clock.\n");
		return ret;
	}
	id->bus_rate = clk_get_rate(id->clk);

	if (pdev->dev.of_node) {
		ret = pic32_i2c_get_ofdata(id);
		if (ret)
			return ret;
	}

	pic32_i2c_disable(id);

	ret = pic32_i2c_setbrg(id);
	if (ret) {
		ret = -EINVAL;
		goto out;
	}

	ret = devm_request_irq(&pdev->dev, id->irq, pic32_i2c_isr, 0,
				 DRIVER_NAME, id);
	if (ret) {
		dev_err(&pdev->dev, "cannot get irq %d\n", id->irq);
		goto out;
	}

	ret = i2c_add_numbered_adapter(&id->adap);
	if (ret < 0) {
		dev_err(&pdev->dev, "reg adap failed: %d\n", ret);
		goto out;
	}

	dev_info(&pdev->dev, "%u kHz mmio %08lx irq %d\n",
		 id->i2c_speed / 1000, (unsigned long)r_mem->start, id->irq);

out:
	return ret;
}

static int pic32_i2c_remove(struct platform_device *pdev)
{
	struct pic32_i2c *id = platform_get_drvdata(pdev);

	i2c_del_adapter(&id->adap);

	return 0;
}

static struct platform_driver pic32_i2c_drv = {
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_i2c_dt_ids),
	},
	.probe  = pic32_i2c_probe,
	.remove = pic32_i2c_remove,
};

module_platform_driver(pic32_i2c_drv);

MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_DESCRIPTION("Microchip I2C bus driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
