/*
 * PIC32 SPI core controller driver (refer dw_spi.c)
 *
 * Copyright (c) 2014, Microchip Technology Inc.
 *      Purna Chandra Mandal <purna.mandal@microchip.com>
 *
 * Licensed under GPLv2.
 */

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/dmaengine.h>
#include <linux/sizes.h>

/* SPI Register offsets */
#define SPIxCON		0x00
#define SPIxCON_CLR	0x04
#define SPIxCON_SET	0x08
#define SPIxSTAT	0x10
#define SPIxSTAT_CLR	0x14
#define SPIxBUF		0x20
#define SPIxBRG		0x30
#define SPIxCON2	0x40
#define SPIxCON2_CLR	0x44
#define SPIxCON2_SET	0x48

/* Bit fields in SPIxCON Register */
#define SPIxCON_RXI_SHIFT	0  /* Rx interrupt generation condition */
#define SPIxCON_TXI_SHIFT	2  /* TX interrupt generation condition */
#define SPIxCON_MSTEN		BIT(5) /* Enable SPI Master */
#define SPIxCON_CKP		BIT(6) /* active low */
#define SPIxCON_CKE		BIT(8) /* Tx on falling edge */
#define SPIxCON_SMP		BIT(9) /* Rx at middle or end of tx */
#define SPIxCON_BPW		0x03	 /* Bits per word/audio-sample */
#define SPIxCON_BPW_SHIFT	10
#define SPIxCON_SIDL		BIT(13) /* STOP on idle */
#define SPIxCON_ON		BIT(15) /* Macro enable */
#define SPIxCON_ENHBUF		BIT(16) /* Enable enhanced buffering */
#define SPIxCON_MCLKSEL		BIT(23) /* Select SPI Clock src */
#define SPIxCON_MSSEN		BIT(28) /* SPI macro will drive SS */
#define SPIxCON_FRMPOL		BIT(29) /* SPI SS polarity */
#define SPIxCON_FRMEN		BIT(31) /* Enable framing mode */

/* Bit fields in SPIxSTAT Register */
#define STAT_RF_FULL		BIT(0) /* RX fifo full */
#define STAT_TF_FULL		BIT(1) /* TX fifo full */
#define STAT_TX_EMPTY		BIT(3) /* standard buffer mode */
#define STAT_RF_EMPTY		BIT(5) /* RX Fifo empty */
#define STAT_RX_OV		BIT(6) /* err, s/w needs to clear */
#define STAT_SHIFT_REG_EMPTY	BIT(7) /* Internal shift-reg empty */
#define STAT_TX_UR		BIT(8) /* UR in Framed SPI modes */
#define STAT_BUSY		BIT(11) /* Macro is processing tx or rx */
#define STAT_FRM_ERR		BIT(12) /* Multiple Frame Sync pulse */
#define STAT_TF_LVL_MASK	0x1F
#define STAT_TF_LVL_SHIFT	16
#define STAT_RF_LVL_MASK	0x1F
#define STAT_RF_LVL_SHIFT	24

/* Bit fields in SPIxBRG Register */
#define SPIxBRG_MASK		0x1ff
#define SPIxBRG_SHIFT		0x0

/* Bit fields in SPIxCON2 Register */
#define SPI_INT_TX_UR_EN	BIT(10) /* Enable int on Tx under-run */
#define SPI_INT_RX_OV_EN	BIT(11) /* Enable int on Rx over-run */
#define SPI_INT_FRM_ERR_EN	BIT(12) /* Enable frame err int */

/* Rx-fifo state for RX interrupt generation */
#define SPI_RX_FIFO_EMTPY	0x0
#define SPI_RX_FIFO_NOT_EMPTY	0x1 /* not empty */
#define SPI_RX_FIFO_HALF_FULL	0x2 /* full by one-half or more */
#define SPI_RX_FIFO_FULL	0x3 /* completely full */

/* TX-fifo state for TX interrupt generation */
#define SPI_TX_FIFO_ALL_EMPTY	0x0 /* completely empty */
#define SPI_TX_FIFO_EMTPY	0x1 /* empty */
#define SPI_TX_FIFO_HALF_EMPTY	0x2 /* empty by half or more */
#define SPI_TX_FIFO_NOT_FULL	0x3 /* atleast one empty */

/* Transfer bits-per-word */
#define SPI_BPW_8		0x0
#define SPI_BPW_16		0x1
#define SPI_BPW_32		0x2

/* SPI clock sources */
#define SPI_CLKSRC_PBCLK	0x0
#define SPI_CLKSRC_MCLK		0x1

/* Minimum DMA transfer size */
#define SPI_DMA_LEN_MIN		SZ_64

struct pic32_spi {
	void __iomem		*regs;
	dma_addr_t		phys_base;
	int			fault_irq;
	int			rx_irq;
	int			tx_irq;
	u32			fifo_n_byte; /* FIFO depth in bytes */
	struct clk		*clk;
	spinlock_t		lock;
	struct spi_master	*master;

	/* Current SPI device specific */
	struct spi_device	*spi_dev;
	u32			speed_hz; /* spi-clk rate */
	u32			mode;

#define SPI_XFER_POLL	BIT(1) /* PIO Transfer based on polling */
#define SPI_SS_MASTER	BIT(2) /* SPI master controlled SS */
#define SPI_DMA_CAP	BIT(3) /* DMA is supported */
#define SPI_DMA_PREP	BIT(4) /* DMA channels allocated */
#define SPI_DMA_READY	BIT(5) /* buffer mapped and ready for DMA */
	u32			flags;
	u8			fifo_n_elm; /* max elements fifo can hold */
	enum dma_slave_buswidth	dma_width;

	/* Current message/transfer state */
	struct spi_message	*mesg;
	const void		*tx;
	const void		*tx_end;
	const void		*rx;
	const void		*rx_end;
	int			len;
	struct completion	xfer_done;

	/* SPI FiFo accessor */
	void (*rx_fifo)(struct pic32_spi *);
	void (*tx_fifo)(struct pic32_spi *);
};

static inline u32 spi_rx_fifo_level(struct pic32_spi *pic32s)
{
	u32 sr = readl(pic32s->regs + SPIxSTAT);

	return (sr >> STAT_RF_LVL_SHIFT) & STAT_RF_LVL_MASK;
}

static inline u32 spi_tx_fifo_level(struct pic32_spi *pic32s)
{
	u32 sr = readl(pic32s->regs + SPIxSTAT);

	return (sr >> STAT_TF_LVL_SHIFT) & STAT_TF_LVL_MASK;
}

static inline void spi_enable_chip(struct pic32_spi *pic32s)
{
	writel(SPIxCON_ON, pic32s->regs + SPIxCON_SET);
}

static inline void spi_disable_chip(struct pic32_spi *pic32s)
{
	writel(SPIxCON_ON, pic32s->regs + SPIxCON_CLR);
	cpu_relax();
}

static inline void spi_set_clk_mode(struct pic32_spi *pic32s, int mode)
{
	u32 conset = 0, conclr = 0;

	if (mode & SPI_CPOL)  /* active low */
		conset |= SPIxCON_CKP;
	else
		conclr |= SPIxCON_CKP;

	if (mode & SPI_CPHA) /* tx on rising edge of clk */
		conclr |= SPIxCON_CKE;
	else
		conset |= SPIxCON_CKE;

	/* rx at end of tx */
	conset |= SPIxCON_SMP;

	writel(conclr, pic32s->regs + SPIxCON_CLR);
	writel(conset, pic32s->regs + SPIxCON_SET);
}

static inline void spi_set_ws(struct pic32_spi *pic32s, int ws)
{
	writel(SPIxCON_BPW << SPIxCON_BPW_SHIFT, pic32s->regs + SPIxCON_CLR);
	writel(ws << SPIxCON_BPW_SHIFT, pic32s->regs + SPIxCON_SET);
}

static inline void spi_drain_rx_buf(struct pic32_spi *pic32s)
{
	u32 sr;

	/* drain rx bytes until empty */
	for (;;) {
		sr = readl(pic32s->regs + SPIxSTAT);
		if (sr & STAT_RF_EMPTY)
			break;

		(void)readl(pic32s->regs + SPIxBUF);
	}

	/* clear rx overflow */
	writel(STAT_RX_OV, pic32s->regs + SPIxSTAT_CLR);
}

static inline void spi_set_clk_rate(struct pic32_spi *pic32s, u32 sck)
{
	u16 clk_div;

	/* sck = clk_in / [2 * (clk_div + 1)]
	 * ie. clk_div = [clk_in / (2 * sck)] - 1
	 */
	clk_div = (clk_get_rate(pic32s->clk) / (2 * sck)) - 1;
	clk_div &= SPIxBRG_MASK;

	writel(clk_div, pic32s->regs + SPIxBRG);
}

static inline void spi_set_clk(struct pic32_spi *pic32s, int clk_id)
{
	switch (clk_id) {
	case SPI_CLKSRC_PBCLK:
		writel(SPIxCON_MCLKSEL, pic32s->regs + SPIxCON_CLR);
		break;

	case SPI_CLKSRC_MCLK:
		writel(SPIxCON_MCLKSEL, pic32s->regs + SPIxCON_SET);
		break;
	}
}

static inline void spi_set_ss_auto(struct pic32_spi *pic32s, u8 mst, u32 mode)
{
	u32 v;

	/* spi controller can drive CS/SS during transfer depending on fifo
	 * fill-level. SS will stay asserted as long as TX fifo has something
	 * to transfer, else will be deasserted confirming completion of
	 * the ongoing transfer.
	 */

	v = readl(pic32s->regs + SPIxCON);
	v &= ~SPIxCON_MSSEN;
	if (mst) {
		v |= SPIxCON_MSSEN;
		if (mode & SPI_CS_HIGH)
			v |= SPIxCON_FRMPOL;
		else
			v &= ~SPIxCON_FRMPOL;
	}

	writel(v, pic32s->regs + SPIxCON);
}

static inline void spi_set_err_int(struct pic32_spi *pic32s)
{
	writel(SPI_INT_TX_UR_EN|SPI_INT_RX_OV_EN|SPI_INT_FRM_ERR_EN,
		pic32s->regs + SPIxCON2_SET);
}

/* Return the max entries we can fill into tx fifo */
static inline u32 pic32_tx_max(struct pic32_spi *pic32s, int n_bytes)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (pic32s->tx_end - pic32s->tx) / n_bytes;
	tx_room = pic32s->fifo_n_elm - spi_tx_fifo_level(pic32s);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * though to use (pic32s->fifo_n_byte - rxfl - txfl) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap = ((pic32s->rx_end - pic32s->rx) -
		    (pic32s->tx_end - pic32s->tx)) / n_bytes;
	return min3(tx_left, tx_room, (u32) (pic32s->fifo_n_elm - rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 pic32_rx_max(struct pic32_spi *pic32s, int n_bytes)
{
	u32 rx_left = (pic32s->rx_end - pic32s->rx) / n_bytes;

	return min_t(u32, rx_left, spi_rx_fifo_level(pic32s));
}

#define BUILD_SPI_FIFO_RW(__name, __type, __bwl)		\
static void pic32_spi_rx_##__name(struct pic32_spi *pic32s)	\
{								\
	__type v;						\
	u32 mx = pic32_rx_max(pic32s, sizeof(__type));		\
	for (; mx; mx--) {					\
		v = read##__bwl(pic32s->regs + SPIxBUF);	\
		if (pic32s->rx_end - pic32s->len)		\
			*(__type *)(pic32s->rx) = v;		\
		pic32s->rx += sizeof(__type);			\
	}							\
}								\
								\
static void pic32_spi_tx_##__name(struct pic32_spi *pic32s)	\
{								\
	__type v;						\
	u32 mx = pic32_tx_max(pic32s, sizeof(__type));		\
	for (; mx ; mx--) {					\
		v = (__type) ~0U;				\
		if (pic32s->tx_end - pic32s->len)		\
			v = *(__type *)(pic32s->tx);		\
		write##__bwl(v, pic32s->regs + SPIxBUF);	\
		pic32s->tx += sizeof(__type);			\
	}							\
}
BUILD_SPI_FIFO_RW(byte, u8, b);
BUILD_SPI_FIFO_RW(word, u16, w);
BUILD_SPI_FIFO_RW(dword, u32, l);

static void pic32_err_stop(struct pic32_spi *pic32s, const char *msg)
{
	/* Stop the hw */
	spi_disable_chip(pic32s);

	/* Show err message and abort xfer with err */
	dev_err(&pic32s->master->dev, "%s\n", msg);
	pic32s->mesg->state = (void *)-1;
	complete(&pic32s->xfer_done);

	/* disable all interrupts */
	disable_irq_nosync(pic32s->fault_irq);
	disable_irq_nosync(pic32s->rx_irq);
	disable_irq_nosync(pic32s->tx_irq);
	dev_err(&pic32s->master->dev, "irq: disable all\n");
}

static irqreturn_t pic32_spi_fault_irq(int irq, void *dev_id)
{
	u32 status;
	struct pic32_spi *pic32s = dev_id;

	spin_lock(&pic32s->lock);

	status = readl(pic32s->regs + SPIxSTAT);

	/* Error handling */
	if (status & (STAT_RX_OV | STAT_FRM_ERR | STAT_TX_UR)) {
		writel(STAT_RX_OV, pic32s->regs + SPIxSTAT_CLR);
		writel(STAT_TX_UR, pic32s->regs + SPIxSTAT_CLR);
		pic32_err_stop(pic32s, "err_irq: fifo ov/ur-run\n");
		goto irq_handled;
	}

	if (status & STAT_FRM_ERR) {
		pic32_err_stop(pic32s, "err_irq: frame error");
		goto irq_handled;
	}

	if (!pic32s->mesg) {
		pic32_err_stop(pic32s, "err_irq: mesg error");
		goto irq_handled;
	}

irq_handled:
	spin_unlock(&pic32s->lock);

	return IRQ_HANDLED;
}

static irqreturn_t pic32_spi_rx_irq(int irq, void *dev_id)
{
	struct pic32_spi *pic32s = dev_id;

	spin_lock(&pic32s->lock);

	pic32s->rx_fifo(pic32s);

	/* rx complete? */
	if (pic32s->rx_end == pic32s->rx) {
		/* mask & disable all interrupts */
		disable_irq_nosync(pic32s->fault_irq);
		disable_irq_nosync(pic32s->rx_irq);

		/* complete current xfer */
		complete(&pic32s->xfer_done);
	}

	spin_unlock(&pic32s->lock);

	return IRQ_HANDLED;
}

static irqreturn_t pic32_spi_tx_irq(int irq, void *dev_id)
{
	struct pic32_spi *pic32s = dev_id;

	spin_lock(&pic32s->lock);

	pic32s->tx_fifo(pic32s);

	/* tx complete? mask and disable tx interrupt */
	if (pic32s->tx_end == pic32s->tx)
		disable_irq_nosync(pic32s->tx_irq);

	spin_unlock(&pic32s->lock);

	return IRQ_HANDLED;
}

static inline void pic32_spi_cs_assert(struct pic32_spi *pic32s)
{
	int cs_high;
	struct spi_device *spi_dev = pic32s->spi_dev;

	if (pic32s->flags & SPI_SS_MASTER)
		return;

	cs_high = pic32s->mode & SPI_CS_HIGH;
	gpio_set_value(spi_dev->cs_gpio, cs_high);
}

static inline void pic32_spi_cs_deassert(struct pic32_spi *pic32s)
{
	int cs_high;
	struct spi_device *spi_dev = pic32s->spi_dev;

	if (pic32s->flags & SPI_SS_MASTER)
		return;

	cs_high = pic32s->mode & SPI_CS_HIGH;
	gpio_set_value(spi_dev->cs_gpio, !cs_high);
}

static int pic32_spi_poll_transfer(struct pic32_spi *pic32s,
	unsigned long timeout)
{
	unsigned long deadline;

	deadline = timeout + jiffies;
	for (;;) {
		pic32s->tx_fifo(pic32s);
		cpu_relax();

		pic32s->rx_fifo(pic32s);
		cpu_relax();

		/* received sufficient data */
		if (pic32s->rx >= pic32s->rx_end)
			break;

		/* timedout ? */
		if (time_after_eq(jiffies, deadline))
			return -EIO;
	}

	return 0;
}

static bool pic32_spi_can_dma(struct spi_master *master,
	struct spi_device *spi, struct spi_transfer *xfer)
{
	struct pic32_spi *pic32s = spi_master_get_devdata(master);

	return (xfer->len >= SPI_DMA_LEN_MIN) && (pic32s->flags & SPI_DMA_PREP);
}

static inline bool pic32_spi_dma_is_ready(struct pic32_spi *pic32s)
{
	int ret;

	ret = (pic32s->flags & SPI_DMA_PREP) &&
		(pic32s->len >= SPI_DMA_LEN_MIN) && (pic32s->tx && pic32s->rx);

	if (ret)
		pic32s->flags |= SPI_DMA_READY;

	return ret;
}

static inline void pic32_spi_dma_unmap(struct pic32_spi *pic32s)
{
	pic32s->flags &= ~SPI_DMA_READY;
}

static void pic32_spi_dma_rx_notify(void *data)
{
	struct pic32_spi *pic32s = data;

	spin_lock(&pic32s->lock);
	complete(&pic32s->xfer_done);
	spin_unlock(&pic32s->lock);
}

static inline void pic32_spi_dma_abort(struct pic32_spi *pic32s)
{
	if (!(pic32s->flags & SPI_DMA_READY))
		return;

	if (pic32s->master->dma_rx)
		dmaengine_terminate_all(pic32s->master->dma_rx);

	if (pic32s->master->dma_tx)
		dmaengine_terminate_all(pic32s->master->dma_tx);

	dev_err(&pic32s->master->dev, "%s, aborted\n", __func__);
}

static int pic32_spi_dma_transfer(struct pic32_spi *pic32s,
	struct spi_transfer *xfer)
{
	dma_cookie_t cookie;
	struct spi_master *master = pic32s->master;
	struct dma_async_tx_descriptor *desc_rx;
	struct dma_async_tx_descriptor *desc_tx;

	if (!master->dma_rx || !master->dma_tx)
		return -ENODEV;

	desc_rx = dmaengine_prep_slave_sg(master->dma_rx,
			xfer->rx_sg.sgl, xfer->rx_sg.nents,
			DMA_FROM_DEVICE, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_rx)
		goto err_dma;

	desc_tx = dmaengine_prep_slave_sg(master->dma_tx,
			xfer->tx_sg.sgl, xfer->tx_sg.nents,
			DMA_TO_DEVICE, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc_tx)
		goto err_dma;

	dev_vdbg(&master->dev, "dma_xfer %p: len %u, tx %p(%p), rx %p(%p)\n",
		xfer, xfer->len,
		xfer->tx_buf, (void *)xfer->tx_dma,
		xfer->rx_buf, (void *)xfer->rx_dma);

	/* Put callback on the RX transfer, that should finish last */
	desc_rx->callback = pic32_spi_dma_rx_notify;
	desc_rx->callback_param = pic32s;

	cookie = dmaengine_submit(desc_rx);
	if (dma_submit_error(cookie))
		goto err_dma;

	cookie = dmaengine_submit(desc_tx);
	if (dma_submit_error(cookie))
		goto err_dma;

	dma_async_issue_pending(master->dma_rx);
	dma_async_issue_pending(master->dma_tx);

	return 0;

err_dma:
	pic32_spi_dma_abort(pic32s);
	return -ENOMEM;
}

static int pic32_spi_dma_config(struct pic32_spi *pic32s, u32 dma_width)
{
	int err;
	struct dma_slave_config cfg;
	struct spi_master *master = pic32s->master;

	cfg.device_fc		= true;
	cfg.dst_addr		= pic32s->phys_base + SPIxBUF;
	cfg.src_addr		= pic32s->phys_base + SPIxBUF;
	cfg.dst_addr_width	= dma_width;
	cfg.src_addr_width	= dma_width;
	cfg.src_maxburst	= pic32s->fifo_n_elm >> 1; /* fill one-half */
	cfg.dst_maxburst	= pic32s->fifo_n_elm >> 1; /* drain one-half */

	cfg.slave_id = pic32s->tx_irq;
	cfg.direction = DMA_MEM_TO_DEV;
	err = dmaengine_slave_config(master->dma_tx, &cfg);
	if (err) {
		dev_err(&master->dev, "configure tx dma channel failed\n");
		goto out;
	}

	cfg.slave_id = pic32s->rx_irq;
	cfg.direction = DMA_DEV_TO_MEM;
	err = dmaengine_slave_config(master->dma_rx, &cfg);
	if (err)
		dev_err(&master->dev, "configure rx dma channel failed\n");
out:
	return err;
}

static void pic32_spi_set_word_size(struct pic32_spi *pic32s, u8 bpw)
{
	u8 spi_bpw;
	enum dma_slave_buswidth busw;

	switch (bpw) {
	default:
	case 8:
		pic32s->rx_fifo = pic32_spi_rx_byte;
		pic32s->tx_fifo = pic32_spi_tx_byte;
		spi_bpw		= SPI_BPW_8;
		busw		= DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case 16:
		pic32s->rx_fifo = pic32_spi_rx_word;
		pic32s->tx_fifo = pic32_spi_tx_word;
		spi_bpw		= SPI_BPW_16;
		busw		= DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 32:
		pic32s->rx_fifo = pic32_spi_rx_dword;
		pic32s->tx_fifo = pic32_spi_tx_dword;
		spi_bpw		= SPI_BPW_32;
		busw		= DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	}
	spi_set_ws(pic32s, spi_bpw);

	/* calculate maximum elements fifo can hold */
	pic32s->fifo_n_elm = DIV_ROUND_UP(pic32s->fifo_n_byte, (bpw >> 3));

	/* re-configure dma width, if required */
	if ((pic32s->flags & SPI_DMA_PREP) && (busw != pic32s->dma_width)) {
		pic32_spi_dma_config(pic32s, busw);
		pic32s->dma_width = busw;
	}
}

static int pic32_spi_one_transfer(struct pic32_spi *pic32s,
				  struct spi_message *message,
				  struct spi_transfer *transfer)
{
	int ret = 0;
	unsigned long flags;
	struct spi_master *master = pic32s->master;

	/* set current transfer information */
	pic32s->tx = (const void *)transfer->tx_buf;
	pic32s->rx = (const void *)transfer->rx_buf;
	pic32s->tx_end = pic32s->tx + transfer->len;
	pic32s->rx_end = pic32s->rx + transfer->len;
	pic32s->len = transfer->len;

	if (transfer->speed_hz && (transfer->speed_hz != pic32s->speed_hz)) {
		spi_set_clk_rate(pic32s, transfer->speed_hz);
		pic32s->speed_hz = transfer->speed_hz;
	}

	if (transfer->bits_per_word)
		pic32_spi_set_word_size(pic32s, transfer->bits_per_word);

	spin_lock_irqsave(&pic32s->lock, flags);

	spi_enable_chip(pic32s);

	/* polling mode? */
	if (pic32s->flags & SPI_XFER_POLL) {
		ret = pic32_spi_poll_transfer(pic32s, 2 * HZ);
		spin_unlock_irqrestore(&pic32s->lock, flags);

		if (ret) {
			dev_err(&master->dev, "poll-xfer timedout\n");
			message->status = ret;
			goto err_xfer_done;
		}
		goto out_xfer_done;
	}

	reinit_completion(&pic32s->xfer_done);

	/* DMA mode ? */
	if (pic32_spi_dma_is_ready(pic32s)) {
		spin_unlock_irqrestore(&pic32s->lock, flags);

		ret = pic32_spi_dma_transfer(pic32s, transfer);
		if (ret) {
			dev_err(&master->dev, "dma xfer error\n");
			message->status = ret;
			spin_lock_irqsave(&pic32s->lock, flags);
		} else {
			goto out_wait_for_xfer;
		}
	}

	/* enable interrupt */
	enable_irq(pic32s->fault_irq);
	enable_irq(pic32s->tx_irq);
	enable_irq(pic32s->rx_irq);

	spin_unlock_irqrestore(&pic32s->lock, flags);

out_wait_for_xfer:

	/* wait for completion */
	ret = wait_for_completion_timeout(&pic32s->xfer_done, 2 * HZ);
	if (ret <= 0) {
		dev_err(&master->dev, "wait timedout/interrupted\n");
		message->status = ret = -EIO;
		pic32_spi_dma_abort(pic32s);
		goto err_xfer_done;
	}

out_xfer_done:
	/* Update total byte transferred */
	message->actual_length += transfer->len;
	ret = 0;

err_xfer_done:
	pic32_spi_dma_unmap(pic32s);
	return ret;
}

static int pic32_spi_one_message(struct spi_master *master,
				 struct spi_message *msg)
{
	int ret = 0;
	int cs_active = 0;
	struct pic32_spi *pic32s;
	struct spi_transfer *xfer;
	struct spi_device *spi = msg->spi;

	dev_vdbg(&spi->dev, "new message %p submitted for %s\n",
		 msg, dev_name(&spi->dev));

	pic32s = spi_master_get_devdata(master);

	msg->status = 0;
	msg->state = (void *) -2; /* running */
	msg->actual_length = 0;

#if 0 /* debug msg */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		dev_info(&spi->dev, "  xfer %p: len %u tx %p rx %p cschg %x\n",
			 xfer, xfer->len, xfer->tx_buf, xfer->rx_buf,
			 xfer->cs_change);
		print_hex_dump(KERN_DEBUG, "tx_buf ", DUMP_PREFIX_ADDRESS,
			       16, 1, xfer->tx_buf, min_t(u32, xfer->len, 16),
			       1);
	}
#endif

	pic32s->mesg = msg;

	spi_disable_chip(pic32s);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (!cs_active) {
			pic32_spi_cs_assert(pic32s);
			cs_active = 1;
		}

		ret = pic32_spi_one_transfer(pic32s, msg, xfer);
		if (ret) {
			dev_err(&spi->dev, "xfer %p err\n", xfer);
			goto xfer_done;
		}

		/* handle delay, if asked */
		if (xfer->delay_usecs)
			udelay(xfer->delay_usecs);

		/* handle cs-change
		 * - for terminal transfer of the list skips CS deassertion.
		 * - for others deassert CS temporarily, before starting next
		 *   transfer CS will be asserted as usual.
		 */
		if (!xfer->cs_change)
			continue;

		cs_active = 0;
		if (!list_is_last(&xfer->transfer_list, &msg->transfers)) {
			pic32_spi_cs_deassert(pic32s);
			continue;
		}
	}

	msg->state = NULL;
	msg->status = 0;

xfer_done:
	/* deassert cs */
	if (cs_active)
		pic32_spi_cs_deassert(pic32s);

	spi_disable_chip(pic32s);

#if 0 /* debug msg */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		print_hex_dump(KERN_DEBUG, "rx_buf ", DUMP_PREFIX_ADDRESS,
			       16, 1, xfer->rx_buf, min_t(u32, xfer->len, 32),
			       1);
	}
#endif

	spi_finalize_current_message(spi->master);

	return ret;
}

static void pic32_spi_cleanup(struct spi_device *spi)
{
	int cs_high;
	struct pic32_spi *pic32s;

	pic32s = spi_master_get_devdata(spi->master);

	/* diasable chip */
	spi_disable_chip(pic32s);

	/* release cs-gpio */
	if (!(pic32s->flags & SPI_SS_MASTER)) {
		cs_high = pic32s->mode & SPI_CS_HIGH;
		gpio_direction_output(spi->cs_gpio, !cs_high);
		gpio_free(spi->cs_gpio);
	}

	/* reset reference */
	pic32s->spi_dev = NULL;
	pic32s->speed_hz = 0;
}

/* This may be called multiple times by same spi dev */
static int pic32_spi_setup(struct spi_device *spi)
{
	struct pic32_spi *pic32s;
	int cs_high;
	int ret;

	pic32s = spi_master_get_devdata(spi->master);

	/* SPI master supports only one spi-device at a time.
	 * So mutiple spi_setup() to different devices is not allowed.
	 */
	if (unlikely(pic32s->spi_dev && (pic32s->spi_dev != spi))) {
		dev_err(&spi->dev, "spi-master already associated with %s\n",
			dev_name(&pic32s->spi_dev->dev));
		return -EPERM;
	}

	if (pic32s->spi_dev == spi)
		pic32_spi_cleanup(spi);

	pic32s->spi_dev = spi;

	/* set POLL mode, if invalid irq is provided */
	if ((pic32s->fault_irq <= 0) || (pic32s->rx_irq <= 0)
	    || (pic32s->tx_irq <= 0))
		pic32s->flags |= SPI_XFER_POLL;

	if (!spi->bits_per_word) {
		dev_err(&spi->dev, "No bits_per_word defined\n");
		return -EINVAL;
	}

	if (!spi->max_speed_hz) {
		dev_err(&spi->dev, "No max speed HZ parameter\n");
		return -EINVAL;
	}

	spi_disable_chip(pic32s);

	pic32_spi_set_word_size(pic32s, spi->bits_per_word);

	pic32s->speed_hz = spi->max_speed_hz;
	spi_set_clk_rate(pic32s, spi->max_speed_hz);

	/* set spi mode */
	spi_set_clk_mode(pic32s, spi->mode);
	pic32s->mode = spi->mode;

	/* configure master-ctl CS assert/deassert, if cs_gpio is invalid */
	pic32s->flags |= SPI_SS_MASTER;

	cs_high = pic32s->mode & SPI_CS_HIGH;
	if (gpio_is_valid(spi->cs_gpio)) {
		ret = gpio_request(spi->cs_gpio, dev_name(&spi->dev));
		if (!ret) {
			gpio_direction_output(spi->cs_gpio, !cs_high);
			pic32s->flags &= ~SPI_SS_MASTER;
			dev_vdbg(&spi->dev,
				"gpio-%d configured for spics (%s)\n",
				spi->cs_gpio, cs_high ? "cs_high":"cs_low");
		}
	}
	spi_set_ss_auto(pic32s, pic32s->flags & SPI_SS_MASTER, spi->mode);

	dev_vdbg(&spi->master->dev,
		 "successfully registered spi-device %s\n",
		 dev_name(&spi->dev));
	return 0;
}

static int pic32_spi_dma_prep(struct pic32_spi *pic32s, struct device *dev)
{
	int err;
	struct spi_master *master = pic32s->master;
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	master->dma_rx = dma_request_slave_channel_compat(mask, NULL, NULL,
							  dev, "spi-rx");
	if (!master->dma_rx) {
		dev_err(dev, "RX channel not found, SPI unable to use DMA\n");
		err = -EBUSY;
		goto out_err;
	}

	master->dma_tx = dma_request_slave_channel_compat(mask, NULL, NULL,
							  dev, "spi-tx");
	if (!master->dma_tx) {
		dev_err(dev, "TX channel not found, SPI unable to use DMA\n");
		err = -EBUSY;
		goto out_err;
	}

	pic32s->dma_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	err = pic32_spi_dma_config(pic32s, pic32s->dma_width);
	if (err)
		goto out_err;

	/* DMA chnls allocated & prepared */
	pic32s->flags |= SPI_DMA_PREP;

	dev_dbg(dev, "Using %s (tx) and %s (rx) for DMA transfers\n",
			dma_chan_name(master->dma_tx),
			dma_chan_name(master->dma_rx));
	return 0;

out_err:
	if (master->dma_rx)
		dma_release_channel(master->dma_rx);

	if (master->dma_tx)
		dma_release_channel(master->dma_tx);

	return err;
}

static void pic32_spi_dma_unprep(struct pic32_spi *pic32s)
{
	if (!(pic32s->flags & SPI_DMA_PREP))
		return;

	pic32s->flags &= ~SPI_DMA_PREP;
	if (pic32s->master->dma_rx)
		dma_release_channel(pic32s->master->dma_rx);

	if (pic32s->master->dma_tx)
		dma_release_channel(pic32s->master->dma_tx);
}

static void pic32_spi_hw_init(struct pic32_spi *pic32s)
{
	u32 v;

	/* disable module */
	spi_disable_chip(pic32s);

	/* drain rx buf */
	spi_drain_rx_buf(pic32s);

	v = readl(pic32s->regs + SPIxCON);

	/* enable fifo: fifo-depth is fixed to 128bit(= 16B) */
	v |= SPIxCON_ENHBUF;
	pic32s->fifo_n_byte = 16;

	/* disable framing mode */
	v &= ~SPIxCON_FRMEN;

	/* enable master mode while disabled */
	v |= SPIxCON_MSTEN;

	/* set tx fifo threshold interrupt */
	v &= ~(0x3 << SPIxCON_TXI_SHIFT);
	v |= (SPI_TX_FIFO_HALF_EMPTY << SPIxCON_TXI_SHIFT);

	/* set rx fifo threshold interrupt */
	v &= ~(0x3 << SPIxCON_RXI_SHIFT);
	v |= (SPI_RX_FIFO_NOT_EMPTY << SPIxCON_RXI_SHIFT);

	writel(v, pic32s->regs + SPIxCON);

	spi_set_err_int(pic32s);
}

static int pic32_spi_hw_probe(struct platform_device *pdev,
			      struct pic32_spi *pic32s)
{
	int ret, clk_id = SPI_CLKSRC_PBCLK;
	struct resource *mem;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem == NULL) {
		dev_err(&pdev->dev, "mem resource not found\n");
		return -ENOENT;
	}

	pic32s->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (!pic32s->regs) {
		dev_err(&pdev->dev, "mem map failed\n");
		return -ENOMEM;
	}
	pic32s->phys_base = mem->start;

	/* get irq resources: err-irq, rx-irq, tx-irq */
	pic32s->fault_irq = platform_get_irq_byname(pdev, "fault");
	if (pic32s->fault_irq < 0)
		dev_warn(&pdev->dev, "no fault-irq ?\n");

	pic32s->rx_irq = platform_get_irq_byname(pdev, "rx");
	if (pic32s->rx_irq < 0)
		dev_warn(&pdev->dev, "no rx-irq ?\n");

	pic32s->tx_irq = platform_get_irq_byname(pdev, "tx");
	if (pic32s->tx_irq < 0)
		dev_warn(&pdev->dev, "no tx-irq ?\n");

	pic32_spi_hw_init(pic32s);

	/* any one of the two clk sources is mandatory; pbxclk:0, m_clk:1 */
	pic32s->clk = devm_clk_get(&pdev->dev, "mck0");
	if (IS_ERR(pic32s->clk)) {
		pic32s->clk = devm_clk_get(&pdev->dev, "mck1");
		if (IS_ERR(pic32s->clk)) {
			ret = PTR_ERR(pic32s->clk);
			dev_err(&pdev->dev, "no clk ?\n");
			goto err_unmap_mem;
		}
		clk_id = SPI_CLKSRC_MCLK;
	}

	clk_prepare_enable(pic32s->clk);

	/* Select clk src */
	spi_set_clk(pic32s, clk_id);

	return 0;

err_unmap_mem:
	dev_err(&pdev->dev, "hw_probe failed, ret %d\n", ret);
	devm_iounmap(&pdev->dev, pic32s->regs);
	return ret;
}

static int pic32_spi_probe(struct platform_device *pdev)
{
	int ret;
	struct spi_master *master;
	struct pic32_spi *pic32s;
	struct device *dev = &pdev->dev;
	u32 max_spi_rate = 50000000;

	master = spi_alloc_master(dev, sizeof(*pic32s));
	if (!master)
		return -ENOMEM;

	pic32s = spi_master_get_devdata(master);
	pic32s->master = master;
	pic32s->flags = SPI_DMA_CAP;

	ret = pic32_spi_hw_probe(pdev, pic32s);
	if (ret) {
		dev_err(&pdev->dev, "hw probe failed.\n");
		goto err_free_master;
	}

	if (dev->of_node) {
		of_property_read_u32(dev->of_node,
				     "max-clock-frequency", &max_spi_rate);

		if (of_find_property(dev->of_node, "use-no-dma", NULL)) {
			dev_warn(dev, "DMA support not requested\n");
			pic32s->flags &= ~SPI_DMA_CAP;
		}
	}

	/* DMA support */
	if (pic32s->flags & SPI_DMA_CAP) {
		ret = pic32_spi_dma_prep(pic32s, dev);
		if (ret)
			dev_warn(dev, "DMA support kept disabled\n");
	}

	master->dev.of_node	= of_node_get(pdev->dev.of_node);
	master->mode_bits	= SPI_MODE_3|SPI_MODE_0|SPI_CS_HIGH;
	master->num_chipselect	= 1;
	master->max_speed_hz	= max_spi_rate;
	master->setup		= pic32_spi_setup;
	master->cleanup		= pic32_spi_cleanup;
	master->flags		= SPI_MASTER_MUST_TX|SPI_MASTER_MUST_RX;
	master->bits_per_word_mask	= SPI_BPW_RANGE_MASK(8, 32);
	master->transfer_one_message	= pic32_spi_one_message;
	if (pic32s->flags & SPI_DMA_PREP)
		master->can_dma	= pic32_spi_can_dma;

	init_completion(&pic32s->xfer_done);
	spin_lock_init(&pic32s->lock);

	/* install irq handlers (with irq-disabled) */
	irq_set_status_flags(pic32s->fault_irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, pic32s->fault_irq,
			       pic32_spi_fault_irq, IRQF_NO_THREAD,
			       dev_name(dev), pic32s);
	if (ret < 0) {
		dev_warn(dev, "request fault-irq %d\n", pic32s->rx_irq);
		pic32s->flags |= SPI_XFER_POLL;
		goto irq_request_done;
	}

	/* receive interrupt handler */
	irq_set_status_flags(pic32s->rx_irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, pic32s->rx_irq,
			       pic32_spi_rx_irq, IRQF_NO_THREAD, dev_name(dev),
			       pic32s);
	if (ret < 0) {
		dev_warn(dev, "request rx-irq %d\n", pic32s->rx_irq);
		pic32s->flags |= SPI_XFER_POLL;
		goto irq_request_done;
	}

	/* transmit interrupt handler */
	irq_set_status_flags(pic32s->tx_irq, IRQ_NOAUTOEN);
	ret = devm_request_irq(dev, pic32s->tx_irq,
			       pic32_spi_tx_irq, IRQF_NO_THREAD, dev_name(dev),
			       pic32s);
	if (ret < 0) {
		dev_warn(dev, "request tx-irq %d\n", pic32s->tx_irq);
		pic32s->flags |= SPI_XFER_POLL;
		goto irq_request_done;
	}

irq_request_done:
	/* register master */
	ret = devm_spi_register_master(dev, master);
	if (ret) {
		dev_err(&master->dev, "failed registering spi master\n");
		goto err_hw_remove;
	}

	platform_set_drvdata(pdev, pic32s);

	return 0;

err_hw_remove:
	/* disable hw */
	spi_disable_chip(pic32s);

	/* disable clk */
	if (!IS_ERR_OR_NULL(pic32s->clk))
		clk_disable_unprepare(pic32s->clk);
err_free_master:
	spi_master_put(master);
	return ret;
}

static int pic32_spi_remove(struct platform_device *pdev)
{
	struct pic32_spi *pic32s;

	pic32s = platform_get_drvdata(pdev);

	/* disable hw */
	spi_disable_chip(pic32s);

	/* disable clk */
	clk_disable_unprepare(pic32s->clk);

	/* unprepare dma */
	pic32_spi_dma_unprep(pic32s);

	return 0;
}

static const struct of_device_id pic32_spi_of_match[] = {
	{.compatible = "microchip,pic32-spi",},
	{},
};
MODULE_DEVICE_TABLE(of, pic32_spi_of_match);

static struct platform_driver pic32_spi_driver = {
	.driver = {
		.name = "spi-pic32",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_spi_of_match),
	},
	.probe = pic32_spi_probe,
	.remove = pic32_spi_remove,
};

module_platform_driver(pic32_spi_driver);

MODULE_AUTHOR("Purna Chandra Mandal <purna.mandal@microchip.com>");
MODULE_DESCRIPTION("Microchip SPI driver for PIC32 SPI controller.");
MODULE_LICENSE("GPL v2");
