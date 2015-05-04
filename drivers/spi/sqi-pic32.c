/*
 * PIC32 SQI controller driver (refer spi-pic32.c)
 *
 * Copyright (c) 2014, Microchip Technology Inc.
 *      Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * Licensed under GPLv2.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/sizes.h>

/* SQI registers */
#define SQI_XIP_CONF1_REG	0x00
#define SQI_XIP_CONF2_REG	0x04
#define SQI_CONF_REG		0x08
#define SQI_CTRL_REG		0x0C
#define SQI_CLK_CTRL_REG	0x10
#define SQI_CMD_THRES_REG	0x14
#define SQI_INT_THRES_REG	0x18
#define SQI_INT_ENABLE_REG	0x1C
#define SQI_INT_STAT_REG	0x20
#define SQI_TX_DATA_REG		0x24
#define SQI_RX_DATA_REG		0x28
#define SQI_STAT1_REG		0x2C
#define SQI_STAT2_REG		0x30
#define SQI_BD_CTRL_REG		0x34
#define SQI_BD_CUR_ADDR_REG	0x38
#define SQI_BD_BASE_ADDR_REG	0x40
#define SQI_BD_STAT_REG		0x44
#define SQI_BD_POLL_CTRL_REG	0x48
#define SQI_BD_TX_DMA_STAT_REG	0x4C
#define SQI_BD_RX_DMA_STAT_REG	0x50
#define SQI_THRES_REG		0x54
#define SQI_INT_SIGEN_REG	0x58

/* SQI_CONF_REG fields */
#define SQI_MODE_BOOT	0 /* Boot mode */
#define	SQI_MODE_PIO	1 /* CPU/PIO */
#define	SQI_MODE_DMA	2 /* descriptor-based DMA */
#define	SQI_MODE_XIP	3 /* XIP */
#define	SQI_MODE	0x7
#define SQI_MODE_SHIFT	0
#define SQI_CPHA	BIT(3)
#define SQI_CPOL	BIT(4)
#define SQI_LSBF	BIT(5)
#define SQI_RXLATCH	BIT(7)
#define SQI_SERMODE	BIT(8)
#define SQI_WP_EN	BIT(9)
#define SQI_HOLD_EN	BIT(10)
#define SQI_BURST_EN	BIT(12)
#define SQI_CS_CTRL_HW	BIT(15)
#define SQI_SOFT_RESET	BIT(16)
#define SQI_LANES_SHIFT	20
#define SQI_LANE_SINGLE	0   /* Single Lane */
#define SQI_LANE_DUAL	1   /* Dual Lane */
#define SQI_LANE_QUAD	2   /* Quad Lane */
#define SQI_CSEN_SHIFT	24
#define SQI_EN_V1	BIT(31)
#define SQI_EN		BIT(23)

/* SQI_CLK_CTRL_REG fields */
#define SQI_CLK_EN		BIT(0)
#define SQI_CLK_STABLE		BIT(1)
#define SQI_CLKDIV_SHIFT	8
#define SQI_CLKDIV		0xff

/* SQI_INT_THR/CMD_THR_REG */
#define TXTHR_MASK	0x1f
#define TXTHR_SHIFT	8
#define RXTHR_MASK	0x1f
#define RXTHR_SHIFT	0

/* SQI_INT_EN/INT_STAT/INT_SIG_EN_REG */
#define TXEMPTY		BIT(0)
#define TXFULL		BIT(1)
#define TXTHR		BIT(2)
#define RXEMPTY		BIT(3)
#define RXFULL		BIT(4)
#define RXTHR		BIT(5)
#define CONFULL		BIT(6)
#define CONEMPTY	BIT(7)
#define CONTHR		BIT(8)
#define BDDONE		BIT(9)  /* current BD processing complete */
#define PKTCOMP		BIT(10) /* packet processing complete */
#define DMAERR		BIT(11) /* error */

/* SQI_BD_CTRL_REG */
#define DMA_EN		BIT(0) /* enable dma engine */
#define POLL_EN		BIT(1) /* enable poll engine */
#define BDP_START	BIT(2) /* start BD processor */

/* H/W BUFFER DESCRIPTOR */
struct hw_bd {
	u32 bd_ctrl;
	u32 bd_status;	/* reserved */
	u32 bd_addr;
	u32 bd_nextp;
};

/* BD_CTRL fields */
#define BD_BUFLEN	0x1ff
#define BD_CBD_INT_EN	BIT(16)	/* current BD is processed */
#define BD_PKT_INT_EN	BIT(17) /* All BDs of PKT processed */
#define BD_LIFM		BIT(18) /* last data of pkt */
#define BD_LAST		BIT(19) /* end of list */
#define BD_DATA_RECV	BIT(20) /* receive data */
#define BD_DDR		BIT(21) /* DDR mode */
#define BD_DUAL		BIT(22)	/* Dual SPI */
#define BD_QUAD		BIT(23) /* Quad SPI */
#define BD_LSBF		BIT(25)	/* LSB First */
#define BD_STAT_CHECK	BIT(27) /* Status poll */
#define BD_DEVSEL_SHIFT	28	/* CS */
#define BD_CS_DEASSERT	BIT(30) /* de-assert CS after current BD */
#define BD_EN		BIT(31) /* BD owned by H/W */

/* DRIVER BUFFER DESCRIPTOR */
struct sqi_desc {
	struct list_head list;
	struct hw_bd *bd;
	dma_addr_t bd_dma;
	void *buf;
	dma_addr_t buf_dma;
	struct spi_transfer *x;
	unsigned long flags;
	void *xfer_buf;	/* dest buff for rx */
	u32 xfer_len;	/* len to copy for rx */
};

/* Global constants */
#define SQI_BD_BUF_LEN_MAX	SZ_256
#define SQI_BD_BUF_DMA_THRES	SZ_32
#define SQI_BD_BUF_LEN		SQI_BD_BUF_DMA_THRES
#define SQI_BD_COUNT		(PAGE_SIZE / SQI_BD_BUF_LEN)
#define SQI_BD_COUNT_MASK	(SQI_BD_COUNT - 1)

#define SQI_VERBOSE	0

struct pic32_sqi {
	void __iomem		*regs;
	int			irq;
	struct clk		*clk;
	struct clk		*reg_clk;
	struct spi_master	*master;
	int			num_cs;
	spinlock_t		lock;
	unsigned long		irq_flags;

#define SQI_IP_V1	BIT(0)	/* SQI IP version */
	u32			flags;
	struct completion	xfer_done;

	/* Resources */
	void			*buffer;
	dma_addr_t		buffer_dma;
	struct list_head	bd_list;
	struct list_head	bd_list_used;
	struct device		*dev;

	/* Current SPI device specific */
	struct spi_device	*spi_dev;
	u32			speed_hz; /* spi-clk rate */
	u8			spi_mode;
};

static void pic32_sqi_hw_init(struct pic32_sqi *sqi);


static inline void sqi_soft_reset(struct pic32_sqi *sqi)
{
	u32 v;
	unsigned long count = 5000;

	/* assert soft-reset */
	writel(SQI_SOFT_RESET, sqi->regs + SQI_CONF_REG);

	/* wait until clear */
	for (;;) {
		v = readl(sqi->regs + SQI_CONF_REG);
		if (!(v & SQI_SOFT_RESET))
			break;

		if (--count == 0) {
			dev_err(sqi->dev, "-- soft-reset timeout --\n");
			break;
		}
	}
}

static inline void sqi_enable_spi(struct pic32_sqi *sqi)
{
	u32 v = readl(sqi->regs + SQI_CONF_REG);

	v |= (sqi->flags & SQI_IP_V1) ? SQI_EN_V1 : SQI_EN;
	writel(v, sqi->regs + SQI_CONF_REG);
}

static inline void sqi_disable_spi(struct pic32_sqi *sqi)
{
	u32 v = readl(sqi->regs + SQI_CONF_REG);

	v &= (sqi->flags & SQI_IP_V1) ? ~SQI_EN_V1 : ~SQI_EN;
	writel(v, sqi->regs + SQI_CONF_REG);
}

static inline void sqi_set_spi_mode(struct pic32_sqi *sqi, int spi_mode)
{
	u32 v;

	v = readl(sqi->regs + SQI_CONF_REG);
	v &= ~(SQI_CPOL|SQI_CPHA|SQI_LSBF);

	/* active low ? */
	if (spi_mode & SPI_CPOL)
		v |= SQI_CPOL;

	/* rx at end of tx */
	v |= SQI_CPHA;

	/* LSB first ? */
	if (spi_mode & SPI_LSB_FIRST)
		v |= SQI_LSBF;

	writel(v, sqi->regs + SQI_CONF_REG);
}

static inline void sqi_enable_clk(struct pic32_sqi *sqi)
{
	u32 v;

	/* enable clock */
	v = readl(sqi->regs + SQI_CLK_CTRL_REG);
	writel(v|SQI_CLK_EN, sqi->regs + SQI_CLK_CTRL_REG);

	/* wait for stability */
	for (;;) {
		v = readl(sqi->regs + SQI_CLK_CTRL_REG);

		if (v & SQI_CLK_STABLE)
			break;
	}
}

static inline void sqi_disable_clk(struct pic32_sqi *sqi)
{
	u32 v;

	v = readl(sqi->regs + SQI_CLK_CTRL_REG);
	v &= ~SQI_CLK_EN;
	writel(v, sqi->regs + SQI_CLK_CTRL_REG);
}

static inline void sqi_set_clk_rate(struct pic32_sqi *sqi, u32 sck)
{
	u32 v, clk_in;
	u16 div;

	v = readl(sqi->regs + SQI_CLK_CTRL_REG);
	v &= ~(SQI_CLK_STABLE|(SQI_CLKDIV << SQI_CLKDIV_SHIFT));

	/* sck = clk_in / [2 * div]
	 * ie. div = clk_in / (2 * sck)
	 */
	clk_in = clk_get_rate(sqi->clk);
	div = clk_in / (2 * sck);
	div &= SQI_CLKDIV;

	/* apply divider */
	v |= (div << SQI_CLKDIV_SHIFT);
	writel(v, sqi->regs + SQI_CLK_CTRL_REG);

	/* wait for stability, if enabled */
	for (;;) {
		v = readl(sqi->regs + SQI_CLK_CTRL_REG);
		 /* clk enabled ? */
		if (!(v & SQI_CLK_EN))
			break;

		if (v & SQI_CLK_STABLE)
			break;
	}
}

static inline void sqi_set_rx_thr(struct pic32_sqi *sqi, int fifo_lvl)
{
	u32 v = readl(sqi->regs + SQI_CMD_THRES_REG);

	v &= ~(RXTHR_MASK << RXTHR_SHIFT);
	v |= ((fifo_lvl & RXTHR_MASK) << RXTHR_SHIFT);
	writel(v, sqi->regs + SQI_CMD_THRES_REG);
}

static inline void sqi_set_rx_intr(struct pic32_sqi *sqi, int fifo_lvl)
{
	u32 v = readl(sqi->regs + SQI_INT_THRES_REG);

	v &= ~(RXTHR_MASK << RXTHR_SHIFT);
	v |= (fifo_lvl << RXTHR_SHIFT);
	writel(v, sqi->regs + SQI_INT_THRES_REG);

	v = readl(sqi->regs + SQI_CMD_THRES_REG);
	v &= ~(RXTHR_MASK << RXTHR_SHIFT);
	v |= (fifo_lvl << RXTHR_SHIFT);
	writel(v, sqi->regs + SQI_CMD_THRES_REG);
}

static inline void sqi_set_tx_thr(struct pic32_sqi *sqi, int fifo_lvl)
{
	u32 v = readl(sqi->regs + SQI_CMD_THRES_REG);

	v &= ~(TXTHR_MASK << TXTHR_SHIFT);
	v |= ((fifo_lvl & TXTHR_MASK) << TXTHR_SHIFT);
	writel(v, sqi->regs + SQI_CMD_THRES_REG);
}

static inline void sqi_set_tx_intr(struct pic32_sqi *sqi, int fifo_lvl)
{
	u32 v = readl(sqi->regs + SQI_INT_THRES_REG);

	v &= ~(TXTHR_MASK << TXTHR_SHIFT);
	v |= (fifo_lvl << TXTHR_SHIFT);
	writel(v, sqi->regs + SQI_INT_THRES_REG);
}

static inline void sqi_enable_int(struct pic32_sqi *sqi)
{
	u32 mask = DMAERR;
#if 0
	/* tx fifo */
	mask |= TXEMPTY|TXFULL|TXTHR;

	/* rx fifo */
	mask |= RXEMPTY|RXFULL;
	mask |= RXTHR;

	/* ctrl fifo */
	mask |= CONEMPTY|CONFULL|CONTHR;
#endif
	/* BD */
	mask |= BDDONE|PKTCOMP;
	writel(mask, sqi->regs + SQI_INT_ENABLE_REG);
	writel(mask, sqi->regs + SQI_INT_SIGEN_REG);
}

static inline void sqi_disable_int(struct pic32_sqi *sqi)
{
	writel(0, sqi->regs + SQI_INT_ENABLE_REG);
	writel(0, sqi->regs + SQI_INT_SIGEN_REG);
	writel(0, sqi->regs + SQI_INT_STAT_REG);
}

static inline void sqi_enable_dma(struct pic32_sqi *sqi)
{
	writel(DMA_EN|POLL_EN|BDP_START, sqi->regs + SQI_BD_CTRL_REG);
}

static inline void sqi_disable_dma(struct pic32_sqi *sqi)
{
	writel(0, sqi->regs + SQI_BD_CTRL_REG);
}

static inline void sqi_spin_lock(struct pic32_sqi *sqi)
{
	spin_lock_irqsave(&sqi->lock, sqi->irq_flags);
}

static inline void sqi_spin_unlock(struct pic32_sqi *sqi)
{
	spin_unlock_irqrestore(&sqi->lock, sqi->irq_flags);
}

static void pic32_debug_sqi(struct pic32_sqi *sqi, const char *fmt)
{
	u32 bd_current, bd_status, tx_status, rx_status;
	u32 int_status, int_enable, int_sig_enable;
	struct sqi_desc *desc;

	bd_current	= readl(sqi->regs + SQI_BD_CUR_ADDR_REG);
	bd_status	= readl(sqi->regs + SQI_BD_STAT_REG);
	tx_status	= readl(sqi->regs + SQI_BD_TX_DMA_STAT_REG);
	rx_status	= readl(sqi->regs + SQI_BD_RX_DMA_STAT_REG);
	int_status	= readl(sqi->regs + SQI_INT_STAT_REG);
	int_enable	= readl(sqi->regs + SQI_INT_ENABLE_REG);
	int_sig_enable	= readl(sqi->regs + SQI_INT_SIGEN_REG);

	pr_info("bd_cur %x: %s/ bd_status 0x%x/ tx_stat 0x%x / rx_stat 0x%x\n",
		bd_current, fmt, bd_status, tx_status, rx_status);

	pr_info("intstatus %08x, inten %08x intsig %08x\n",
		int_status, int_enable, int_sig_enable);

	list_for_each_entry(desc, &sqi->bd_list_used, list) {
		pr_info("--- BD %p: bd_addr 0x%x status 0x%x\n",
			(void *)&desc->bd, desc->bd->bd_addr,
			desc->bd->bd_ctrl);
	}

}

static irqreturn_t pic32_sqi_isr(int irq, void *dev_id)
{
	struct pic32_sqi *sqi = dev_id;
	u32 enable, status, mask = 0;

	enable = readl(sqi->regs + SQI_INT_ENABLE_REG);
	status = readl(sqi->regs + SQI_INT_STAT_REG);

	/* spurious interrupt ? */
	if (!status)
		return IRQ_NONE;

	writel(status, sqi->regs + SQI_INT_STAT_REG);
	if (status & DMAERR) {
		sqi_disable_dma(sqi);
		sqi_disable_spi(sqi);
		mask = 0xffff;
		pic32_debug_sqi(sqi, " - DMAERR - ");
		goto irq_done;
	}

	if (status & TXTHR)
		mask |= (TXTHR|TXFULL|TXEMPTY);

	if (status & RXTHR) {
		mask |= RXTHR|RXFULL|RXEMPTY;
		mask |= CONTHR|CONFULL|CONEMPTY;
	}

	if (status & BDDONE) {
		writel(BDDONE, sqi->regs + SQI_INT_STAT_REG);
		mask |= BDDONE;
		pic32_debug_sqi(sqi, " - BDDONE - ");
	}

	 /* packet processing completed ? */
	if (status & PKTCOMP) {
		/* mask all interrupts */
		mask = enable;
		/* complete trasaction */
		complete(&sqi->xfer_done);
	}

irq_done:
	/* mask interrupts, when asked */
	enable &= ~mask;
	writel(enable, sqi->regs + SQI_INT_ENABLE_REG);

	return IRQ_HANDLED;
}

static inline struct sqi_desc *sqi_desc_get(struct pic32_sqi *sqi)
{
	struct sqi_desc *b;

	if (list_empty(&sqi->bd_list)) {
		b = NULL;
		goto done;
	}

	b = list_first_entry(&sqi->bd_list, struct sqi_desc, list);
	list_del(&b->list);
	list_add_tail(&b->list, &sqi->bd_list_used);
done:
	return b;
}

static void sqi_desc_put(struct pic32_sqi *sqi, struct sqi_desc *bd)
{
	list_del(&bd->list);
	list_add(&bd->list, &sqi->bd_list);
}

static int sqi_desc_fill(struct sqi_desc *desc,
	struct spi_transfer *xfer, int remaining,
	u32 xfer_bd_ctrl, dma_addr_t dma_handle)
{
	struct hw_bd *bd;
	int offset = xfer->len - remaining;

	desc->xfer_len = min_t(u32, remaining, SQI_BD_BUF_LEN_MAX);
	desc->x = xfer;
	desc->xfer_buf = NULL;

	/* Buffer Descriptor */
	bd = desc->bd;

	/* BD CTRL: length */
	bd->bd_ctrl = xfer_bd_ctrl;
	bd->bd_ctrl |= desc->xfer_len;

	/* BD STAT */
	bd->bd_status = 0;

	/* BD BUFFER ADDRESS: zero-copy or bounce-buffer ? */
	if (!dma_handle) {
		/* use pre-allocated dma buffer for bouncing */
		if (xfer->tx_buf) {
			desc->xfer_buf = (void *)xfer->tx_buf + offset;
			memcpy(desc->buf, desc->xfer_buf, desc->xfer_len);
		} else
			desc->xfer_buf = xfer->rx_buf + offset;

		bd->bd_addr = desc->buf_dma;
	} else {
		bd->bd_addr = dma_handle + offset;
	}
#if 0
	/* BD NEXTPTR: already initialized to next BD */
	bd->bd_nextp = 0;
#endif

	return desc->xfer_len;
}

static inline dma_addr_t sqi_map_buf(struct pic32_sqi *sqi,
					const void *buf, size_t len, int tx)
{
	dma_addr_t dma_buf;
	struct device *dev = &sqi->master->dev;
	enum dma_data_direction dir = tx ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	if (is_vmalloc_addr(buf))
		dma_buf = dma_map_page(dev, vmalloc_to_page(buf),
					offset_in_page(buf), len, dir);
	else
		dma_buf = dma_map_single(dev, (void *)buf, len, dir);

	if (dma_mapping_error(dev, dma_buf))
		dma_buf = 0;

	dev_dbg(dev, "%s: buf %p, len %u, dma_buf %p, tx %d\n", __func__,
			buf, len, (void *)dma_buf, tx);
	return dma_buf;
}

static inline void sqi_unmap_buf(struct pic32_sqi *sqi, const void *buf,
					dma_addr_t dma_buf, size_t len, int tx)
{
	struct device *dev = &sqi->master->dev;
	enum dma_data_direction dir = tx ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	dev_dbg(dev, "%s: buf %p, len %u, dma_buf %p, tx %d\n", __func__,
			buf, len, (void *)dma_buf, tx);

	if (is_vmalloc_addr(buf))
		dma_unmap_page(dev, dma_buf, len, dir);
	else
		dma_unmap_single(dev, dma_buf, len, dir);

}

static int pic32_sqi_one_transfer(struct pic32_sqi *sqi,
	struct spi_message *mesg, struct spi_transfer *xfer)
{
	struct sqi_desc *desc;
	int remaining, ret;
	u32 bd_ctrl;
	u32 nbits;
	dma_addr_t dma_buf;

	dma_buf = xfer->rx_dma = xfer->tx_dma = 0;
	/* BD CTRL: length */
	bd_ctrl = 0;

	/* Device selection */
	bd_ctrl |= (mesg->spi->chip_select << BD_DEVSEL_SHIFT);

	/* Transfer/Receive mode selection */
	if (xfer->rx_buf) {
		nbits = xfer->rx_nbits;
		bd_ctrl |= BD_DATA_RECV;
		if (xfer->len >= SQI_BD_BUF_DMA_THRES) {
			dma_buf = sqi_map_buf(sqi, xfer->rx_buf, xfer->len, 0);
			xfer->rx_dma = dma_buf;
		}
		goto mapping_done;
	}

	nbits = xfer->tx_nbits;
	if (xfer->len >= SQI_BD_BUF_DMA_THRES) {
		dma_buf = sqi_map_buf(sqi, xfer->tx_buf, xfer->len, 1);
		xfer->tx_dma = dma_buf;
	}

mapping_done:
	if (nbits & SPI_NBITS_QUAD)
		bd_ctrl |= BD_QUAD;
	else if (nbits & SPI_NBITS_DUAL)
		bd_ctrl |= BD_DUAL;

	/* LSB first */
	if (mesg->spi->mode & SPI_LSB_FIRST)
		bd_ctrl |= BD_LSBF;
#if 0
	/* interrupt on this BD */
	common_bd_ctrl |= BD_CBD_INT_EN;
#endif
	/* Ownership to hw */
	bd_ctrl |= BD_EN;

	for (remaining = xfer->len; remaining;) {

		/* Alloc buffer descriptor */
		desc = sqi_desc_get(sqi);
		if (!desc)
			break;

		/* Fill descriptors */
		ret = sqi_desc_fill(desc, xfer, remaining, bd_ctrl, dma_buf);

		/* Update PTR */
		remaining -= ret;
	}

	return 0;
}

static int pic32_sqi_one_message(struct spi_master *master,
				 struct spi_message *msg)
{
	int ret = 0;
	struct hw_bd *bd;
	struct sqi_desc *desc, *ndesc;
	struct pic32_sqi *sqi;
	struct spi_transfer *xfer;
	struct spi_device *spi = msg->spi;

	dev_vdbg(&spi->dev, "new message %p submitted for %s\n",
		 msg, dev_name(&spi->dev));

	sqi = spi_master_get_devdata(master);

	msg->status = 0;
	msg->state = (void *) -2; /* running */
	msg->actual_length = 0;

#if (SQI_VERBOSE >= 2)
	/* debug msg */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		dev_vdbg(&spi->dev, "  xfer %p: len %u tx %p rx %p\n",
			 xfer, xfer->len, xfer->tx_buf, xfer->rx_buf);
		if (xfer->tx_buf)
			print_hex_dump(KERN_DEBUG, "tx_buf",
				DUMP_PREFIX_ADDRESS, 16, 1, xfer->tx_buf,
				min_t(u32, xfer->len, 16), 1);
		else
			memset(xfer->rx_buf, 0, xfer->len);
	}
#endif

	/* init completion */
	reinit_completion(&sqi->xfer_done);

	/* prepare desc-list (BDs) for transfer(s) */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		ret = pic32_sqi_one_transfer(sqi, msg, xfer);
		if (ret) {
			dev_err(&spi->dev, "xfer %p err\n", xfer);
			goto xfer_out;
		}
	}

	/* Pre-processing: prepared BD(s) are already chained
	 * (during allocation itself), only mark LAST_BD at last BD of the list.
	 */
	desc = list_last_entry(&sqi->bd_list_used, struct sqi_desc, list);
	bd = desc->bd;
	/* mark LAST */
	bd->bd_ctrl |= BD_LAST;
	/* CS deassert on last BD */
	bd->bd_ctrl |= BD_CS_DEASSERT;
	 /* interrupt on pkt-done */
	bd->bd_ctrl |= BD_LIFM|BD_PKT_INT_EN;

#if (SQI_VERBOSE >= 2)
	/* debug */
	list_for_each_entry_safe(desc, ndesc, &sqi->bd_list_used, list) {
		bd = desc->bd;
		pr_info("bd %p: bd_ctrl %08x bd_addr %08x bd_stat %08x bd_next %08x\n",
			(void *)bd, bd->bd_ctrl,
			bd->bd_addr, bd->bd_status, bd->bd_nextp);
	}
#endif

	/* interrupt lock */
	sqi_spin_lock(sqi);

	/* We can't handle spi_transfer specific "speed_hz", "bits_per_word",
	 * "delay_usecs". spi_device specific speed and mode change can be
	 * handled at best.
	 */

	/* CS switch ?*/
	if (sqi->spi_dev != spi) {
		sqi->spi_dev = spi;

		sqi_disable_spi(sqi);

		/* set spi clk */
		if (sqi->speed_hz != spi->max_speed_hz) {
			sqi->speed_hz = spi->max_speed_hz;
			sqi_set_clk_rate(sqi, spi->max_speed_hz);
			sqi_enable_clk(sqi);
		}

		/* set spi mode */
		ret = spi->mode & (SPI_MODE_3|SPI_LSB_FIRST);
		if (sqi->spi_mode != ret) {
			sqi->spi_mode = ret;
			sqi_set_spi_mode(sqi, spi->mode);
		}
	}

	/* set BD base address */
	desc = list_first_entry(&sqi->bd_list_used, struct sqi_desc, list);
	writel(desc->bd_dma, sqi->regs + SQI_BD_BASE_ADDR_REG);

	/* set tx/rx threshold */
	if (sqi->flags & SQI_IP_V1) {
		sqi_set_tx_thr(sqi, 31);
		sqi_set_rx_thr(sqi, 31);
	}

	/* enable SPI */
	sqi_enable_spi(sqi);

	/* enable interrupt */
	sqi_enable_int(sqi);

	/* enable SPI */
	sqi_enable_dma(sqi);

	sqi_spin_unlock(sqi);

	/* wait for xfer completion */
	ret = wait_for_completion_timeout(&sqi->xfer_done, 5 * HZ);

	sqi_spin_lock(sqi);
	if (ret <= 0) {
		dev_err(&sqi->master->dev, "wait timedout/interrupted\n");
		pic32_debug_sqi(sqi, " -- TIMEDOUT -- ");
		msg->status = ret = -EIO;
		goto xfer_done;
	}

	/* post-process: copy rx-data to rx_buf */
	list_for_each_entry(desc, &sqi->bd_list_used, list) {
		xfer = desc->x;

		/* Update total byte transferred */
		msg->actual_length += desc->xfer_len;

		/* skip transmit */
		if (xfer->tx_buf)
			continue;
		if (!desc->xfer_buf)
			continue;
		memcpy(desc->xfer_buf, desc->buf, desc->xfer_len);
	}

#if (SQI_VERBOSE >= 1)
	/* debug msg */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->tx_buf)
			continue;
		print_hex_dump(KERN_DEBUG, "rx_buf: ", DUMP_PREFIX_ADDRESS,
		       16, 1, xfer->rx_buf, min_t(u32, xfer->len, 256), 1);
	}
#endif
	/* update msg status */
	msg->state = NULL;
	msg->status = 0;
	ret = 0;

xfer_done:
	/* disable interrupt */
	sqi_disable_int(sqi);

	/* disable dma */
	sqi_disable_dma(sqi);

	/* disable chip */
	sqi_disable_spi(sqi);

	/* interrupt unlock */
	sqi_spin_unlock(sqi);

xfer_out:
	/* unmap dma memory, if there */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->tx_buf && xfer->tx_dma)
			sqi_unmap_buf(sqi, xfer->tx_buf,
					xfer->tx_dma, xfer->len, 1);
		else if (xfer->rx_dma)
			sqi_unmap_buf(sqi, xfer->rx_buf,
					xfer->rx_dma, xfer->len, 0);
	}

	/* release all used bds */
	list_for_each_entry_safe_reverse(desc, ndesc, &sqi->bd_list_used, list)
		sqi_desc_put(sqi, desc);

#if 0
	/* reset controller, when failed */

	/* Keeping this reset logic disabled, by default. Transfer failure
	 * could be due to various of reasons (incorrect programming sequence,
	 * dead spi client, etc). And recovery might also vary.
	 */
	if (ret) {
		dev_err(&sqi->master->dev, "reset & re-init h/w.\n");

		/* allow sometime to settle */
		udelay(10);

		/* reset */
		pic32_sqi_hw_init(sqi);
	}
#endif

	spi_finalize_current_message(spi->master);

	return ret;
}

/* This may be called twice for each spi dev */
static int pic32_sqi_setup(struct spi_device *spi)
{
	struct pic32_sqi *sqi;

	sqi = spi_master_get_devdata(spi->master);

	/* SPI master supports multiple spi-device(s) at a time.
	 * So multiple spi_setup() to different devices is allowed.
	 */
	if (spi_get_ctldata(spi)) {
		dev_err(&spi->dev, "is already associated\n");
		return -EPERM;
	}

	/* check word size */
	if (!spi->bits_per_word) {
		dev_err(&spi->dev, "No bits_per_word defined\n");
		return -EINVAL;
	}

	/* check maximum SPI clk rate */
	if (!spi->max_speed_hz) {
		dev_err(&spi->dev, "No max speed HZ parameter\n");
		return -EINVAL;
	}

	if (spi->master->max_speed_hz < spi->max_speed_hz) {
		dev_err(&spi->dev, "max speed %u HZ not valid\n",
			spi->max_speed_hz);
		return -EINVAL;
	}

	spi_set_ctldata(spi, (void *)sqi);

	dev_vdbg(&spi->master->dev,
		 "successfully registered spi-device %s\n",
		 dev_name(&spi->dev));
	return 0;
}

static void pic32_sqi_cleanup(struct spi_device *spi)
{
	struct pic32_sqi *sqi;

	sqi = spi_master_get_devdata(spi->master);

	spi_set_ctldata(spi, (void *)NULL);
}

static int sqi_desc_ring_alloc(struct pic32_sqi *sqi)
{
	int i = 0;
	int size;
	struct hw_bd *hw_bd;
	void *tmp_buf;
	dma_addr_t tmp_buf_dma;
	struct sqi_desc *desc, *d;

	/* allocate h/w descriptor and bounce-buffer */
	size = sizeof(struct hw_bd) * SQI_BD_COUNT;
	sqi->buffer = dma_zalloc_coherent(sqi->dev,
				size + (SQI_BD_COUNT * SQI_BD_BUF_LEN),
				&sqi->buffer_dma, GFP_DMA32);
	if (!sqi->buffer) {
		dev_err(sqi->dev, "error: allocating hw-bd\n");
		return -ENOMEM;
	}

	tmp_buf = sqi->buffer + size;
	tmp_buf_dma = sqi->buffer_dma + size;

	/* allocate descriptor to manage hw_bd and linked bounce_buffer */
	desc = devm_kcalloc(sqi->dev, SQI_BD_COUNT, sizeof(*d), GFP_KERNEL);
	if (!desc) {
		dma_free_coherent(sqi->dev, size, sqi->buffer, sqi->buffer_dma);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&sqi->bd_list);
	INIT_LIST_HEAD(&sqi->bd_list_used);

	hw_bd = (struct hw_bd *)sqi->buffer;
	for (i = 0, d = &desc[0]; i < SQI_BD_COUNT; i++, d++) {
		INIT_LIST_HEAD(&d->list);
		d->bd = &hw_bd[i];
		d->bd_dma = sqi->buffer_dma + ((void *)d->bd - (void *)hw_bd);
		d->buf = tmp_buf + (i * SQI_BD_BUF_LEN);
		d->buf_dma = tmp_buf_dma + (i * SQI_BD_BUF_LEN);
		list_add_tail(&d->list, &sqi->bd_list);
	}

	/* Prepare BD: link buffer-addr & chain to next BD(s) */
	for (i = 0, d = &desc[0]; i < SQI_BD_COUNT; i++, d++) {
		hw_bd[i].bd_addr = d->buf_dma;
		if (i)
			hw_bd[i-1].bd_nextp = d->bd_dma;
	}

	return 0;
}

static void sqi_desc_ring_free(struct pic32_sqi *sqi)
{
	int size;
	struct sqi_desc *b;

	/* remove DMA buffer & DMA descriptor */
	size = sizeof(struct hw_bd) * SQI_BD_COUNT;
	size += (SQI_BD_COUNT * SQI_BD_BUF_LEN);
	dma_free_coherent(sqi->dev, size, sqi->buffer, sqi->buffer_dma);

	/* remove buffer desc */
	b = list_first_entry(&sqi->bd_list, struct sqi_desc, list);
	devm_kfree(sqi->dev, b);
}

static void pic32_sqi_hw_init(struct pic32_sqi *sqi)
{
	u32 v;

	/* soft reset */
	sqi_soft_reset(sqi);

	/* disable all interrupts */
	sqi_disable_int(sqi);

	/* tx fifo interrupt threshold */
	sqi_set_tx_thr(sqi, 1);
	sqi_set_tx_intr(sqi, 1);

	/* rx fifo interrupt threshold */
	sqi_set_rx_thr(sqi, 1);
	sqi_set_rx_intr(sqi, 1);

	/* enable default interrupt */
	sqi_enable_int(sqi);

	/* default configuration */
	v = readl(sqi->regs + SQI_CONF_REG);

	/* set mode: DMA */
	v &= ~SQI_MODE;
	v |= (SQI_MODE_DMA << SQI_MODE_SHIFT);
	writel(v, sqi->regs + SQI_CONF_REG);

	/* DATAEN - SQIID0-ID3 */
	v |= (SQI_LANE_QUAD << SQI_LANES_SHIFT);

	/* burst/INCR4 enable */
	v |= SQI_BURST_EN;

	/* CSEN - all CS */
	v |= ((BIT(sqi->num_cs) - 1) << SQI_CSEN_SHIFT);

	if (sqi->flags & SQI_IP_V1) {
		/* SERMODE disabled */
		v &= ~SQI_SERMODE;

		/* CSCON - hardware */
		v |= SQI_CS_CTRL_HW;

		/* RXLATCH */
		v |= SQI_RXLATCH;
	}
	writel(v, sqi->regs + SQI_CONF_REG);

	/* write poll count */
	writel(0, sqi->regs + SQI_BD_POLL_CTRL_REG);

	/* disable module */
	sqi_disable_spi(sqi);

	sqi->speed_hz = 0;
	sqi->spi_mode = -1;
}

static int pic32_sqi_probe(struct platform_device *pdev)
{
	int ret;
	struct spi_master *master;
	struct pic32_sqi *sqi;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *r;
	u32 max_spi_hz = 50000000;

	dev_vdbg(&pdev->dev, "%s:%d\n", __func__, __LINE__);

	master = spi_alloc_master(dev, sizeof(*sqi));
	if (!master)
		return -ENOMEM;

	sqi = spi_master_get_devdata(master);
	sqi->master = master;
	sqi->dev = get_device(dev);

	/* io resource */
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no mem resource?\n");
		ret = -ENOENT;
		goto err_free_master;
	}

	sqi->regs = devm_ioremap_resource(&pdev->dev, r);
	if (!sqi->regs) {
		dev_err(&pdev->dev, "mem map failed\n");
		ret = -ENOMEM;
		goto err_free_master;
	}

	/* irq resource */
	sqi->irq = platform_get_irq(pdev, 0);
	if (sqi->irq < 0) {
		dev_err(&pdev->dev, "no irq ?\n");
		ret = sqi->irq;
		goto err_free_master;
	}

	/* clock */
	sqi->reg_clk = devm_clk_get(&pdev->dev, "reg_ck");
	if (IS_ERR(sqi->reg_clk)) {
		ret = PTR_ERR(sqi->reg_clk);
		dev_err(&pdev->dev, "no reg_clk ?\n");
		goto err_free_master;
	}
	dev_dbg(&pdev->dev, "reg-clk: %s, rate %lu\n",
		__clk_get_name(sqi->reg_clk), clk_get_rate(sqi->reg_clk));

	sqi->clk = devm_clk_get(&pdev->dev, "spi_ck");
	if (IS_ERR(sqi->clk)) {
		ret = PTR_ERR(sqi->clk);
		dev_err(&pdev->dev, "no clk ?\n");
		goto err_free_master;
	}

	clk_prepare_enable(sqi->reg_clk);
	clk_prepare_enable(sqi->clk);

	dev_dbg(&pdev->dev, "spi-clk: %s, rate %lu\n",
		__clk_get_name(sqi->clk), clk_get_rate(sqi->clk));

	sqi->num_cs = 2;
	if (np) {
		/* number of CS */
		of_property_read_u32(np, "microchip,sqi-num-cs", &sqi->num_cs);

		/* max frequency */
		of_property_read_u32(np, "max-clock-frequency", &max_spi_hz);

		/* module version */
		if (of_find_property(np, "microchip,sqi-v1", NULL)) {
			dev_info(&pdev->dev, "SQI-V1 detected\n");
			sqi->flags |= SQI_IP_V1;
		}
	}

	init_completion(&sqi->xfer_done);
	spin_lock_init(&sqi->lock);

	/* hw init */
	sqi_spin_lock(sqi);
	pic32_sqi_hw_init(sqi);
	sqi_spin_unlock(sqi);

	/* allocate buffers & descriptors */
	ret = sqi_desc_ring_alloc(sqi);
	if (ret) {
		dev_err(&pdev->dev, "request-irq %d, failed ?\n", sqi->irq);
		goto err_disable_hw;
	}

	/* install irq handlers */
	ret = devm_request_irq(dev, sqi->irq, pic32_sqi_isr,
			       0, dev_name(dev), sqi);
	if (ret < 0) {
		dev_err(&pdev->dev, "request-irq %d, failed ?\n", sqi->irq);
		goto err_free_ring;
	}

	/* register master */
	master->num_chipselect	= sqi->num_cs;
	master->max_speed_hz	= max_spi_hz;
	master->dma_alignment	= 32;
	master->dev.of_node	= of_node_get(pdev->dev.of_node);
	master->mode_bits	= SPI_MODE_3|SPI_MODE_0|SPI_TX_DUAL|
				  SPI_RX_DUAL|SPI_TX_QUAD|SPI_RX_QUAD;
	master->flags			= SPI_MASTER_HALF_DUPLEX;
	master->bits_per_word_mask	= SPI_BPW_RANGE_MASK(8, 32);
	master->transfer_one_message	= pic32_sqi_one_message;
	master->setup			= pic32_sqi_setup;
	master->cleanup			= pic32_sqi_cleanup;

	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&master->dev, "failed registering spi master\n");
		goto err_free_ring;
	}

	platform_set_drvdata(pdev, sqi);

	return 0;

err_free_ring:
	sqi_desc_ring_free(sqi);

err_disable_hw:
	/* disable hw */
	sqi_disable_spi(sqi);
	/* disable clk */
	clk_disable_unprepare(sqi->clk);
	clk_disable_unprepare(sqi->reg_clk);
err_free_master:
	spi_master_put(master);
	return ret;
}

static int pic32_sqi_remove(struct platform_device *pdev)
{
	struct pic32_sqi *sqi;

	sqi = platform_get_drvdata(pdev);

	/* disable hw */
	sqi_disable_spi(sqi);

	/* disable dma */
	sqi_disable_dma(sqi);
	sqi_disable_int(sqi);

	/* disable clk */
	sqi_disable_clk(sqi);

	clk_disable_unprepare(sqi->clk);
	clk_disable_unprepare(sqi->reg_clk);

	/* release memory */
	sqi_desc_ring_free(sqi);

	return 0;
}

static const struct of_device_id pic32_sqi_of_ids[] = {
	{.compatible = "microchip,pic32-sqi",},
	{},
};
MODULE_DEVICE_TABLE(of, pic32_sqi_of_ids);

static struct platform_driver pic32_sqi_driver = {
	.driver = {
		.name = "sqi-pic32",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_sqi_of_ids),
	},
	.probe = pic32_sqi_probe,
	.remove = pic32_sqi_remove,
};

module_platform_driver(pic32_sqi_driver);

MODULE_AUTHOR("Purna Chandra Mandal <purna.mandal@microchip.com>");
MODULE_DESCRIPTION("Microchip SPI driver for PIC32 SQI controller.");
MODULE_LICENSE("GPL v2");
