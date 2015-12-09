/*
 * pic32_can.c - CAN network driver for Microchip pic32mzda CAN controller
 *
 * (C) 2015 by Sandeep Sheriker <SandeepSheriker.Mallikarjun@microchip.com>
 *
 * This software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2 as distributed in the 'COPYING'
 * file from the main directory of the linux kernel source.
 *
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <asm/mach-pic32/pic32.h>
#include <asm/cacheflush.h>

#define DEV_NAME	"pic32-can"
#define PIC32_VERSION	"1.0"
#define DRV_DESC "Microchip PIC32MZDA CAN Controller Driver " PIC32_VERSION

#define PIC32_CAN_MODE_CTL_REG		(0x00)
/*Module control Register bits */
#define PIC32_CAN_MODE_CTL_BUSY		BIT(11)
#define PIC32_CAN_MODE_CTL_SIDLE	BIT(13)
#define PIC32_CAN_MODE_CTL_ON		BIT(15)
#define PIC32_CAN_MODE_CTL_TMRCAP	BIT(20)
#define PIC32_CAN_MODE_CTL_ABAT		BIT(27)

#define PIC32_CAN_CFG_REG	(0x10)
/*Baud Rate Config Register Bits */
#define PIC32_CAN_CFG_WAKFIL	BIT(22)
#define PIC32_CAN_CFG_SEG2PHTS	BIT(15)
#define PIC32_CAN_CFG_SAM	BIT(14)

#define PIC32_CAN_INT_REG	(0x20)
/* Interrupt Enable bits */
#define PIC32_CAN_IRQ_IVRIE	BIT(31)
#define PIC32_CAN_IRQ_WAKIE	BIT(30)
#define PIC32_CAN_IRQ_ERRIE	BIT(29)
#define PIC32_CAN_IRQ_SERRIE	BIT(28)
#define PIC32_CAN_IRQ_RBOVIE	BIT(27)
#define PIC32_CAN_IRQ_MODIE	BIT(19)
#define PIC32_CAN_IRQ_TMRIE	BIT(18)
#define PIC32_CAN_IRQ_RBIE	BIT(17)
#define PIC32_CAN_IRQ_TBIE	BIT(16)
/* Interrupt Flag bits */
#define PIC32_CAN_IRQ_IVRIF	BIT(15)
#define PIC32_CAN_IRQ_WAKIF	BIT(14)
#define PIC32_CAN_IRQ_ERRIF	BIT(13)
#define PIC32_CAN_IRQ_SERRIF	BIT(12)
#define PIC32_CAN_IRQ_RBOVIF	BIT(11)
#define PIC32_CAN_IRQ_MODIF	BIT(3)
#define PIC32_CAN_IRQ_TMRIF	BIT(2)
#define PIC32_CAN_IRQ_RBIF	BIT(1)
#define PIC32_CAN_IRQ_TBIF	BIT(0)

#define PIC32_CAN_IVEC_REG	(0x30)
/* Interrupt Vector Register bits */
#define PIC32_CAN_INT_NOINT	0x40
#define PIC32_CAN_INT_ERRIF	0x41
#define PIC32_CAN_INT_WAKIF	0x42
#define PIC32_CAN_INT_RBOVIF	0x43
#define PIC32_CAN_INT_ASERRIF	0x44
#define PIC32_CAN_INT_BSERRIF	0x45
#define PIC32_CAN_INT_CTMRIF	0x46
#define PIC32_CAN_INT_MODIF	0x47
#define PIC32_CAN_INT_IVRIF	0x48

#define PIC32_CAN_TREC_REG	(0x40)
/* TX/RX Error Count Register bits */
#define PIC32_CAN_ERR_TXBO	BIT(21)	/* TX Error Bus OFF(TERRCNT>=256)*/
#define PIC32_CAN_ERR_TXBP	BIT(20)	/* TX Error BusPassive(TERRCNT>=128)*/
#define PIC32_CAN_ERR_RXBP	BIT(19)	/* RX Error BusPassive(RERRCNT>=128)*/
#define PIC32_CAN_ERR_TXWARN	BIT(18)	/* TX Error Warning(128>TERRCNT>=96)*/
#define PIC32_CAN_ERR_RXWARN	BIT(17)	/* RX Error Warning(128>RERRCNT>=96)*/
#define PIC32_CAN_ERR_EWARN	BIT(16)	/* TX/RX Error Warning*/

#define PIC32_CAN_FSTAT_REG	(0x50)
#define PIC32_CAN_RXOVF_REG	(0x60)
#define PIC32_CAN_TIMER_REG	(0x70)
#define PIC32_CAN_RXMN_REG(i)	((0x80) + ((i) * 0x10))
#define PIC32_CAN_FTLCTLN_REG(i) ((0xC0) + ((i) * 0x10))
#define PIC32_CAN_RXFN_REG(i)	(0x140 + ((i) * 0x10))
#define PIC32_CAN_FIFOBA_REG	(0x340)

#define PIC32_CAN_FIFOCTLN_REG(i)	(0x350 + ((i) * 0x40))
/* FIFO control Register Bits */
#define PIC32_CAN_FIFOCTL_FRST	BIT(14)
#define PIC32_CAN_FIFOCTL_UINC	BIT(13)
#define PIC32_CAN_FIFOCTL_TXEN	BIT(7)
#define PIC32_CAN_FIFOCTL_TXREQ	BIT(3)
#define PIC32_CAN_FIFOCTL_RTREN	BIT(2)
#define PIC32_CAN_FIFOCTL_TXPR	0x3
#define PIC32_CAN_FIFOCTL_FSIZE	(0x1F << 16)

#define PIC32_CAN_FIFOINTN_REG(i)	(0x360 + ((i) * 0x40))
/* FIFO Interrupt Enable Register Bits */
#define PIC32_CAN_FIFOINT_TXNFULLIE	BIT(26)
#define PIC32_CAN_FIFOINT_TXHALFIE	BIT(25)
#define PIC32_CAN_FIFOINT_TXEMPTYIE	BIT(24)
#define PIC32_CAN_FIFOINT_RXOVFLIE	BIT(19)
#define PIC32_CAN_FIFOINT_RXFULLIE	BIT(18)
#define PIC32_CAN_FIFOINT_RXHALFIE	BIT(17)
#define PIC32_CAN_FIFOINT_RXNEMPTYIE	BIT(16)
/* FIFO Interrupt FLAG Register Bits */
#define PIC32_CAN_FIFOINT_TXNFULLIF	BIT(10)
#define PIC32_CAN_FIFOINT_TXHALFIF	BIT(9)
#define PIC32_CAN_FIFOINT_TXEMPTYIF	BIT(8)
#define PIC32_CAN_FIFOINT_RXOVFLIF	BIT(3)
#define PIC32_CAN_FIFOINT_RXFULLIF	BIT(2)
#define PIC32_CAN_FIFOINT_RXHALFIF	BIT(1)
#define PIC32_CAN_FIFOINT_RXNEMPTYIF	BIT(0)

#define PIC32_CAN_FIFOUAN_REG(i)	(0x370 + ((i) * 0x40))
#define PIC32_CAN_FIFOIDXN_REG(i)	(0x380 + ((i) * 0x40))
/* End of CAN Controller Register Offset */

#define PIC32_CAN_IRQ_ALL	(PIC32_CAN_IRQ_IVRIE \
				| PIC32_CAN_IRQ_WAKIE \
				| PIC32_CAN_IRQ_ERRIE \
				| PIC32_CAN_IRQ_SERRIE \
				| PIC32_CAN_IRQ_RBOVIE \
				| PIC32_CAN_IRQ_MODIE \
				| PIC32_CAN_IRQ_TMRIE \
				| PIC32_CAN_IRQ_RBIE \
				| PIC32_CAN_IRQ_TBIE)

#define PIC32_CAN_FIFOINT_TX	(PIC32_CAN_FIFOINT_TXEMPTYIE)

#define PIC32_CAN_FIFOINT_RX	(PIC32_CAN_FIFOINT_RXOVFLIE \
				| PIC32_CAN_FIFOINT_RXNEMPTYIE)

#define PIC32_CAN_SID_MASK	0xFFE00000
#define PIC32_CAN_EID_MASK	0x0003FFFF
#define PIC32_CAN_MIDE_MASK	0x00080000

#define PIC32_MAX_FIFO		32
#define PIC32_MAX_FIFO_MSG	32
#define PIC32_MAX_FIFO_MSG_LEN	16
#define PIC32_BUF_LEN	(PIC32_MAX_FIFO * PIC32_MAX_FIFO_MSG *\
			 PIC32_MAX_FIFO_MSG_LEN)

#define PIC32_RX_FIFO_START	0
#define PIC32_RX_FIFO_END	16
#define PIC32_TX_FIFO_START	PIC32_RX_FIFO_END
#define PIC32_TX_FIFO_END	PIC32_MAX_FIFO
#define PIC32_TX_FIFO_MSG_START	0
#define PIC32_TX_FIFO_MSG_END	PIC32_MAX_FIFO_MSG

#define CAN_CTL_SUPPORT	(CAN_CTRLMODE_BERR_REPORTING \
			 | CAN_CTRLMODE_3_SAMPLES)

enum pic32_op_mode {
	NORMAL	= 0,
	DISABLE = 1,
	LOOPBACK = 2,
	LISTEN_ONLY = 3,
	CONFIG	= 4,
	LISTEN_ALL = 7,
};

static const struct can_bittiming_const pic32_bittiming_const = {
	.name = DEV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

/* tx msg sid bit fields. */
struct txmsgsid {
	unsigned sid:11;
	unsigned unused:21;
} __attribute__((__packed__));

/* txrx msg eid bit fields. */
struct txrxmsgeid {
	unsigned dlc:4;
	unsigned reserved:1;
	unsigned unused:3;
	unsigned reserved1:1;
	unsigned rtr:1;
	unsigned eid:18;
	unsigned ide:1;
	unsigned srr:1;
	unsigned unused1:2;
} __attribute__((__packed__));

/* rx msg sid bit fields. */
struct rxmsgsid {
	unsigned sid:11;
	unsigned FILHIT:5;
	unsigned CMSGTS:16;
} __attribute__((__packed__));

/* tx msg structure. */
union txbuffer {
	struct {
		struct txmsgsid msgsid;
		struct txrxmsgeid msgeid;
		u32 data0;
		u32 data1;
	};
	u32 data[4];
} __attribute__((__packed__));

/* rx msg structure. */
union rxbuffer {
	struct {
		struct rxmsgsid msgsid;
		struct txrxmsgeid msgeid;
		u32 data0;
		u32 data1;
	};
	u32 data[4];
} __attribute__((__packed__));

struct pic32_priv {
	struct can_priv can;	/* MUST be first member/field */
	u32 tx_head;
	u32 tx_msg_head;
	u32 tx_msg_tail;
	u32 tx_max;
	u32 rx_next;
	struct net_device *ndev;
	struct platform_device *pdev;
	struct clk *can_clk;
	void __iomem *reg_base;
	int transceiver_gpio;
	u32 *tx_rx_buf;
	dma_addr_t tx_rx_buf_dma;
};

static inline void pic32_can_write(struct pic32_priv *priv, u32 reg, u32 val)
{
	writel(val, priv->reg_base + reg);
}

static inline u32 pic32_can_read(struct pic32_priv *priv, u32 reg)
{
	return readl(priv->reg_base + reg);
}

static inline u32 pic32_can_get_bit(struct pic32_priv *priv, u32 reg, u32 mask)
{
	return (pic32_can_read(priv, reg) & mask) ? 1 : 0;
}

	/* bitrate values provided by CAN dev.c are not proper.
	 * calculating new values based on PIC32 CAN Controller Manual.
	 * Brp value is calculated using BitTime value 10 and PropSeg,
	 * PhaseSeg1, PhaseSeg2 values are fixed.
	 *
	 * tq = (BitTime * Bitrate)
	 *
	 * Bitrate Prescaler = (PBCLK5 / (2 * tq)) -1;
	 *
	 * BitTime = SyncSeg + PropSeg + PhaseSeg1 + PhaseSeg2
	 * i.e 10 = 1 + (3-1) + (3-1) + (3-1)
	 */

static int pic32_set_bittiming(struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bit_timing = &priv->can.bittiming;
	u32 can_btc = 0, N = 10, brpval = 0, sam = 0;
	long tq = 0;

	tq = (N * bit_timing->bitrate);
	brpval = (clk_get_rate(priv->can_clk) / (2 * tq)) - 1;

	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		sam = PIC32_CAN_CFG_SAM;

	can_btc = ((brpval & 0x3F) /* Brp */
			| ((2 & 0x7) << 8)  /* prop_seg */
			| ((2 & 0x7) << 11) /* phase_seg1 */
			| sam
			| PIC32_CAN_CFG_SEG2PHTS /* SEG2PHTS */
			| ((2 & 0x7) << 16) /* phase_seg2 */
			| PIC32_CAN_CFG_WAKFIL);  /*WAKFIL */

	/* clear Baud rate config register's old value*/
	pic32_can_write(priv,
			PIC32_CLR(PIC32_CAN_CFG_REG),
			(~0));

	/* set new values from can_btc */
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_CFG_REG),
			can_btc);
	return 0;
}

static void pic32_transceiver_switch(const struct pic32_priv *priv, int on)
{
	gpio_set_value(priv->transceiver_gpio, on);
}

static int pic32_configure_fifos(struct net_device *ndev)
{
	int i = 0;
	struct pic32_priv *priv = netdev_priv(ndev);
	u32 regval = 0;
	int timeout = 5000;

	/* Reset all FIFO's */
	for (i = 0; i < PIC32_MAX_FIFO; i++) {
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FIFOCTLN_REG(i)),
				PIC32_CAN_FIFOCTL_FRST);
		/* Wait till FIFO RESET bit gets cleared */
		while (pic32_can_get_bit(priv,
					 PIC32_CAN_FIFOCTLN_REG(i),
					 PIC32_CAN_FIFOCTL_FRST)) {
			if (--timeout <= 0) {
				netdev_err(priv->ndev, "Pic32 CAN FIFO Reset timeout\n");
				return -ETIMEDOUT;
			}
			udelay(1);
		}
	}

	/* Configuring 3 ACCEPTANCE FILTER MASK REGISTER's
	 * Register 0 is for Including SIDx, in filter comparison
	 * Register 1 is for Including EIDx, in filter comparison
	 * Register 2 is for Including MIDE, in filter comparison
	 */
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_RXMN_REG(0)),
			PIC32_CAN_SID_MASK);
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_RXMN_REG(1)),
			PIC32_CAN_EID_MASK);
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_RXMN_REG(2)),
			PIC32_CAN_MIDE_MASK);

	for (i = 0; i < PIC32_MAX_FIFO; i++)
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_RXFN_REG(i)),
				PIC32_CAN_SID_MASK);

	/* Configuring FILTER CONTROL REGISTER 0 to 3 for Rx Filters.
	 * 1. Enabled 0 to 15 Rx Filter.
	 * 2. Acceptance Mask 3 selected for 0 to 15 Rx Filter's
	 * 3. Message matching filter stored in respective 0 to 15 FIFO
	 */
	for (i = 0; i < 4; i++) {
		regval = (((0xE0 + ((i * 4) + 3)) << 24)
			| ((0xE0 + ((i * 4) + 2)) << 16)
			| ((0xE0 + ((i * 4) + 1)) << 8)
			| (0xE0 + (i * 4)));
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FTLCTLN_REG(i)),
				regval);
	}

	/* Store Message buffer Bus address */
	if (!priv->tx_rx_buf_dma) {
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FIFOBA_REG),
				priv->tx_rx_buf_dma);
	}

	/* 1. Set 0 to 15 FIFO's for Receive.
	 * 2. Set FIFO Length to 32 Messages.
	 * 3. Enable FIFO Interrupt for FIFO not empty event and overflow event
	 */
	for (i = PIC32_RX_FIFO_START; i < PIC32_RX_FIFO_END; i++) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_FIFOCTLN_REG(i)),
				PIC32_CAN_FIFOCTL_TXEN);
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FIFOCTLN_REG(i)),
				PIC32_CAN_FIFOCTL_FSIZE);
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FIFOINTN_REG(i)),
				PIC32_CAN_FIFOINT_RX);
	}

	/* 1. Set 16 to 32 FIFO's for Transmission.
	 * 2. Set FIFO Length to 32 Messages.
	 * 3. Disable Tx FIFO Interrupts & enabling in during data transfer.
	 */
	for (i = PIC32_TX_FIFO_START; i < PIC32_TX_FIFO_END; i++) {
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FIFOCTLN_REG(i)),
				PIC32_CAN_FIFOCTL_FSIZE);
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_FIFOCTLN_REG(i)),
				PIC32_CAN_FIFOCTL_TXEN |
				PIC32_CAN_FIFOCTL_TXPR |
				PIC32_CAN_FIFOCTL_RTREN);
	}
	return 0;
}

static int pic32_set_op_mode(struct net_device *ndev,
			     enum pic32_op_mode mode)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	u32 regval = 0;
	int timeout = 10000;

	switch (mode) {
	case CONFIG:
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_MODE_CTL_REG),
				(CONFIG << 24));
		do {
			regval = pic32_can_read(priv, PIC32_CAN_MODE_CTL_REG);
			regval = (regval >> 21) & 0x07;
			if (--timeout <= 0) {
				netdev_err(priv->ndev,
					   "Pic32 CAN CONFIG Mode timeout\n");
				return -ETIMEDOUT;
			}
		} while (CONFIG != regval);
		break;

	case NORMAL:
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_MODE_CTL_REG),
				(7 << 24));
		break;

	case LISTEN_ONLY:
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_MODE_CTL_REG),
				(LISTEN_ONLY << 24));
		break;

	case LOOPBACK:
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_MODE_CTL_REG),
				(LOOPBACK << 24));
		break;

	case DISABLE:
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_MODE_CTL_REG),
				(DISABLE << 24));
		do {
			regval = pic32_can_read(priv, PIC32_CAN_MODE_CTL_REG);
			regval = (regval >> 21) & 0x07;
			if (--timeout <= 0) {
				netdev_err(priv->ndev,
					   "Pic32 CAN CONFIG Mode timeout\n");
				return -ETIMEDOUT;
			}
		} while (DISABLE != regval);
		break;

	case LISTEN_ALL:
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_MODE_CTL_REG),
				(LISTEN_ALL << 24));
		break;

	default:
		netdev_err(priv->ndev, "pic32_set_op_mode: Invalid Mode\n");
		return -EINVAL;
	}
	return 0;
}

static void pic32_enable_interrupts(struct net_device *ndev, int enable)
{
	struct pic32_priv *priv = netdev_priv(ndev);

	if (enable)
		pic32_can_write(priv,
				PIC32_SET(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_ALL);
	else
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_ALL);
}

static int pic32_can_init(struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	int ret = 0;

	ret = pic32_set_op_mode(ndev, CONFIG);
	if (ret < 0)
		return ret;

	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_MODE_CTL_REG),
			PIC32_CAN_MODE_CTL_ON);

	pic32_enable_interrupts(ndev, 0);

	/*pic32_set_bittiming(ndev);*/

	ret = pic32_configure_fifos(ndev);
	if (ret < 0)
		return ret;

	pic32_enable_interrupts(ndev, 1);

	ret = pic32_set_op_mode(ndev, NORMAL);
	if (ret < 0)
		return ret;
	return ret;
}

static int pic32_can_enable(struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	int ret = 0;

	ret = pic32_set_op_mode(ndev, CONFIG);
	if (ret < 0)
		return ret;

	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_MODE_CTL_REG),
			PIC32_CAN_MODE_CTL_ON);

	ret = pic32_set_op_mode(ndev, NORMAL);
	if (ret < 0)
		return ret;

	return ret;
}

static int pic32_can_disable(struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	int timeout = 5000, ret = 0;

	ret = pic32_set_op_mode(ndev, CONFIG);
	if (ret < 0)
		return ret;

	pic32_can_write(priv,
			PIC32_CLR(PIC32_CAN_MODE_CTL_REG),
			PIC32_CAN_MODE_CTL_ON);
	while (pic32_can_get_bit(priv,
				 PIC32_CAN_MODE_CTL_REG,
				 PIC32_CAN_MODE_CTL_BUSY)) {
		if (--timeout <= 0) {
			netdev_err(priv->ndev,
				   "Pic32 CAN Busy wait timeout\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}

	ret = pic32_set_op_mode(ndev, NORMAL);
	if (ret < 0)
		return ret;

	return 0;
}

static netdev_tx_t pic32_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	struct can_frame *frame = (struct can_frame *)skb->data;
	struct net_device_stats *stats = &ndev->stats;
	u32 txfifo = priv->tx_head % priv->tx_max;
	union txbuffer *buf;
	u32 exide;
	u32 *addr;
	dma_addr_t dma_addr;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	/* Read Bus address location put TX Data */
	dma_addr = (dma_addr_t)pic32_can_read(priv,
					      PIC32_CAN_FIFOUAN_REG(txfifo));

	/* Convert Bus addr to Kernel virtual addr for kernel data handling*/
	addr = (u32 *)bus_to_virt((unsigned long int)dma_addr);
	buf = (union txbuffer *)addr;
	memset(buf, 0, sizeof(union txbuffer));

	/* Check EID or SID in frame */
	exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0;
	if (exide) {
		/* SID */
		buf->msgsid.sid = frame->can_id & CAN_SFF_MASK;
		/* MIDE */
		buf->msgeid.ide = 1;
		buf->msgeid.srr = 1;
		/* EID */
		buf->msgeid.eid = (frame->can_id & CAN_EFF_MASK) >> 11;
	} else {
		/* SID */
		buf->msgsid.sid = frame->can_id & CAN_SFF_MASK;
	}
	/* Check for RTR in frame */
	buf->msgeid.rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;

	/* DLC */
	buf->msgeid.dlc = frame->can_dlc;

	/* DATA0 convert to BigEndian */
	buf->data0 = *(__be32 *)(frame->data);
	if (frame->can_dlc > 4)
		/* DATA1 convert to BigEndian */
		buf->data1 = *(__be32 *)(frame->data + 4);

	/* Enabling TX FIFO Interrupt after writing data in to FIFO */
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_FIFOINTN_REG(txfifo)),
			PIC32_CAN_FIFOINT_TX);

	can_put_echo_skb(skb, ndev, priv->tx_msg_head);

	priv->tx_msg_head++;
	if (priv->tx_msg_head >= PIC32_TX_FIFO_MSG_END) {
		priv->tx_msg_head = PIC32_TX_FIFO_MSG_START;
		priv->tx_head++;
		if (priv->tx_head >= PIC32_TX_FIFO_END)
			priv->tx_head = PIC32_TX_FIFO_START;
	}

	stats->tx_bytes += frame->can_dlc;

	/* Flush Cache to avoid Cache problem */
	dma_sync_single_for_device(&priv->pdev->dev, dma_addr, frame->can_dlc,
				   DMA_TO_DEVICE);

	/* Increment FIFO head by a single message */
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_FIFOCTLN_REG(txfifo)),
			PIC32_CAN_FIFOCTL_UINC);

	/* send message on CAN Bus */
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_FIFOCTLN_REG(txfifo)),
			(PIC32_CAN_FIFOCTL_TXREQ));

	return NETDEV_TX_OK;
}

static int pic32_can_rx(struct  net_device *ndev, int rxfifo)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32 numpkts = 0;
	union rxbuffer *buf;

	if (!netif_running(ndev))
		return 0;

	/* Create a skb, freeing is handled by kernel */
	skb = alloc_can_skb(priv->ndev, &cf);
	if (!skb) {
		netdev_err(priv->ndev, "pic32_can_rx: alloc_can_skb() failed\n");
		return -ENOMEM;
	}

	/* Read Bus address location put TX Data */
	buf = (union rxbuffer *)pic32_can_read(priv,
					       PIC32_CAN_FIFOUAN_REG(rxfifo));

	/* Convert Bus addr to Kernel virtual addr for data handling*/
	buf = (union rxbuffer *)bus_to_virt((unsigned long int)buf);

	/* Can id Format :
	 *	0-10 SID bits
	 *	11-28 EID bits
	 *	29 error message frame bit
	 *	30 RTR Frame bit
	 *	31 EFF/SFF Bit bit to differentiate between EID/SID
	 */
	if (buf->msgeid.ide) {
		/* Extended ID */
		cf->can_id = ((buf->msgeid.eid << 11)
			      | (buf->msgsid.sid)
			      | CAN_EFF_FLAG);
	} else {
		cf->can_id = buf->msgsid.sid & CAN_SFF_MASK; /* SID */
	}

	if (buf->msgeid.rtr)
		cf->can_id |= CAN_RTR_FLAG;  /* RTR */

	cf->can_dlc = buf->msgeid.dlc;  /* DLC */
	if (cf->can_dlc > 8)
		cf->can_dlc = 8;

	/* Data0 BigEndian->LittleEndian */
	*(__le32 *)(cf->data) = cpu_to_le32(buf->data0);
	if (cf->can_dlc > 4) {
		/* Data1 BigEndian->Little Endian */
		*(__le32 *)(cf->data + 4) = cpu_to_le32(buf->data1);
	}

	/* Increment FIFO tail by a single message */
	pic32_can_write(priv,
			PIC32_SET(PIC32_CAN_FIFOCTLN_REG(rxfifo)),
			PIC32_CAN_FIFOCTL_UINC);

	/* update rx net Status Info */
	stats->rx_bytes += cf->can_dlc;
	stats->rx_packets++;
	netif_receive_skb(skb);
	numpkts++;

	return numpkts;
}

static void pic32_can_tx_complete(struct net_device *ndev, int txfifo)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	u32 regval;

	regval = pic32_can_read(priv, PIC32_CAN_FIFOINTN_REG(txfifo));

	/* Disabling TX FIFO Interrupts and enable it during transmission */
	if (regval & (PIC32_CAN_FIFOINT_TXEMPTYIF |
		      PIC32_CAN_FIFOINT_TXHALFIF |
		      PIC32_CAN_FIFOINT_TXNFULLIF))
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_FIFOINTN_REG(txfifo)),
				PIC32_CAN_FIFOINT_TX);

	/* update tx net Status Info */
	can_get_echo_skb(ndev, priv->tx_msg_tail);
	stats->tx_packets++;
	priv->tx_msg_tail++;
	if (priv->tx_msg_tail >= PIC32_TX_FIFO_MSG_END)
		priv->tx_msg_tail = PIC32_TX_FIFO_MSG_START;
}

static int pic32_can_error(struct net_device *ndev, int err_status)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	u32 regval = 0;
	int err = 0, timeout = 5000;

	/* create skb to propagate the error condition to the can stack */
	skb = alloc_can_err_skb(ndev, &cf);
	if (!skb) {
		netdev_err(priv->ndev,
			   "pic32 can alloc_can_err_skb() failed\n");
		return -ENOMEM;
	}

	if (((err_status & PIC32_CAN_INT_ASERRIF) == PIC32_CAN_INT_ASERRIF) ||
	    ((err_status & PIC32_CAN_INT_BSERRIF) == PIC32_CAN_INT_BSERRIF)) {
		/* wait till BUSY bit gets cleared */
		while (pic32_can_get_bit(priv,
					 PIC32_CAN_MODE_CTL_REG,
					 PIC32_CAN_MODE_CTL_BUSY)) {
			if (--timeout <= 0) {
				netdev_err(priv->ndev,
					   "Pic32 CAN In Busy wait timeout\n");
				return -ETIMEDOUT;
			}
			udelay(1);
		}

		/* Disable CAN Module */
		err = pic32_can_disable(ndev);
		if (err < 0)
			return err;

		/* update err net Status Info */
		++priv->can.can_stats.bus_error;
		cf->can_id |= CAN_ERR_BUSERROR | CAN_ERR_PROT;
		cf->data[1] |= CAN_ERR_CRTL_UNSPEC;
	}

	if (((err_status & PIC32_CAN_INT_ERRIF) == PIC32_CAN_INT_ERRIF)) {
		regval = pic32_can_read(priv, PIC32_CAN_TREC_REG);
		if (regval & PIC32_CAN_ERR_EWARN) {
			priv->can.state = CAN_STATE_ERROR_WARNING;
			++priv->can.can_stats.error_warning;
			cf->can_id |= CAN_ERR_CRTL;
			if (regval & PIC32_CAN_ERR_TXWARN)
				cf->data[1] |= CAN_ERR_CRTL_TX_WARNING;
			if (regval & PIC32_CAN_ERR_RXWARN)
				cf->data[1] |= CAN_ERR_CRTL_RX_WARNING;
		}

		if ((regval & PIC32_CAN_ERR_TXBP) ||
		    (regval & PIC32_CAN_ERR_RXBP)) {
			priv->can.state = CAN_STATE_ERROR_PASSIVE;
			++priv->can.can_stats.error_passive;
			cf->can_id |= CAN_ERR_CRTL;
			if (regval & PIC32_CAN_ERR_TXBP)
				cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
			if (regval & PIC32_CAN_ERR_RXBP)
				cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		}

		if (regval & PIC32_CAN_ERR_TXBO) {
			priv->can.state = CAN_STATE_BUS_OFF;
			cf->can_id |= CAN_ERR_BUSOFF;
			++priv->can.can_stats.bus_off;
			can_bus_off(ndev);
		}
	}

	/* update err net Status Info */
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);
	return 0;
}

static void pic32_rx_overflow_err(struct net_device *ndev)
{
	struct net_device_stats *stats = &ndev->stats;
	struct pic32_priv *priv = netdev_priv(ndev);
	struct sk_buff *skb;
	struct can_frame *cf;
	u32 regval = 0, count = 0;

	regval = pic32_can_read(priv, PIC32_CAN_RXOVF_REG);
	while (regval > 0) {
		if (regval & 1)
			pic32_can_write(priv, PIC32_CLR(
					PIC32_CAN_FIFOINTN_REG(count)),
					PIC32_CAN_FIFOINT_RXOVFLIF);
		regval >>= 1;
		count++;
	}

	pic32_can_write(priv,
			PIC32_CLR(PIC32_CAN_INT_REG),
			PIC32_CAN_IRQ_RBOVIF);

	/* create skb to propagate the error condition to the can stack */
	skb = alloc_can_err_skb(ndev, &cf);
	if (unlikely(!skb))
		return;

	cf->can_id |= CAN_ERR_CRTL;
	cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

	stats->rx_over_errors++;
	stats->rx_errors++;
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_receive_skb(skb);
}

static irqreturn_t pic32_can_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct pic32_priv *priv = netdev_priv(ndev);
	u32 status = 0, cintstatus = 0;

	status = pic32_can_read(priv, PIC32_CAN_IVEC_REG);
	cintstatus = pic32_can_read(priv, PIC32_CAN_INT_REG);

	if ((status & PIC32_CAN_INT_IVRIF) == PIC32_CAN_INT_IVRIF) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_IVRIF);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_MODIF) == PIC32_CAN_INT_MODIF) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_MODIF);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_CTMRIF)  ==  PIC32_CAN_INT_CTMRIF) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_INT_CTMRIF);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_BSERRIF) == PIC32_CAN_INT_BSERRIF) {
		pic32_can_error(ndev, status);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_ASERRIF)  == PIC32_CAN_INT_ASERRIF) {
		pic32_can_error(ndev, status);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_WAKIF)  == PIC32_CAN_INT_WAKIF) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_WAKIF);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_RBOVIF) == PIC32_CAN_INT_RBOVIF) {
		pic32_rx_overflow_err(ndev);
		return IRQ_HANDLED;
	}

	if ((status & PIC32_CAN_INT_ERRIF)  == PIC32_CAN_INT_ERRIF) {
		pic32_can_error(ndev, status);
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_ERRIF);
		return IRQ_HANDLED;
	}

	if ((cintstatus & PIC32_CAN_IRQ_TBIF) == PIC32_CAN_IRQ_TBIF) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_TBIF);
		pic32_can_tx_complete(ndev, (status & 0x1F));
		return IRQ_HANDLED;
	}

	if ((cintstatus & PIC32_CAN_IRQ_RBIF) == PIC32_CAN_IRQ_RBIF) {
		pic32_can_write(priv,
				PIC32_CLR(PIC32_CAN_INT_REG),
				PIC32_CAN_IRQ_RBIF);
		pic32_can_rx(ndev, (status & 0x1F));
		return IRQ_HANDLED;
	}
	return IRQ_HANDLED;
}

static int pic32_open(struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	int err = 0;

	err = request_irq(ndev->irq, pic32_can_interrupt,
			  IRQF_SHARED, ndev->name, ndev);
	if (err) {
		netdev_err(ndev, "error requesting interrupt\n");
		return err;
	}

	err = clk_prepare_enable(priv->can_clk);
	if (err) {
		netdev_err(ndev, "enable device clock failed\n");
		goto err_irq;
	}

	err = pic32_can_enable(ndev);
	if (err < 0) {
		netdev_err(ndev, "pic32 CAN Enable failed\n");
		goto err_irq;
	}

	err = pic32_can_init(ndev);
	if (err < 0) {
		netdev_err(ndev, "pic32 CAN Init failed\n");
		goto err_irq;
	}

	/* Open common can device */
	err = open_candev(ndev);
	if (err) {
		netdev_err(ndev, "open_candev() failed %d\n", err);
		goto err_can_clk;
	}

	netif_start_queue(ndev);
	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;

err_can_clk:
	clk_disable_unprepare(priv->can_clk);
err_irq:
	free_irq(ndev->irq, ndev);
	return err;
}

static int pic32_stop(struct net_device *ndev)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	int err = 0;

	err = pic32_can_disable(ndev);
	if (err < 0)
		return err;

	priv->can.state = CAN_STATE_STOPPED;
	return 0;
}

static int pic32_close(struct net_device *ndev)
{
	int err = 0;

	netif_stop_queue(ndev);
	err = pic32_stop(ndev);
	if (err < 0)
		return err;

	free_irq(ndev->irq, ndev);
	close_candev(ndev);
	return 0;
}

static int pic32_set_mode(struct net_device *ndev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		pic32_can_init(ndev);
		netif_wake_queue(ndev);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int pic32_get_berr_counter(const struct net_device *ndev,
				  struct can_berr_counter *bec)
{
	struct pic32_priv *priv = netdev_priv(ndev);
	u32 reg_ecr = pic32_can_read(priv, PIC32_CAN_TREC_REG);

	bec->rxerr = reg_ecr & 0xff;
	bec->txerr = reg_ecr >> 8;

	return 0;
}

static const struct net_device_ops pic32_netdev_ops = {
	.ndo_open	= pic32_open,
	.ndo_stop	= pic32_close,
	.ndo_start_xmit	= pic32_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static int pic32_can_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct net_device *ndev;
	struct pic32_priv *priv;
	void __iomem *addr;
	int ret = -ENODEV;
	int gpio;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr)) {
		ret = PTR_ERR(addr);
		goto err;
	}

	gpio = of_get_named_gpio(np, "transceiver-gpio", 0);
	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, gpio,
					    GPIOF_OUT_INIT_LOW,
					    "pic32can transceiver");
		if (ret < 0) {
			dev_err(&pdev->dev, "CAN Transceiver request failed.\n");
			goto err;
		}
		gpio_set_value(gpio, 1);
	}

	ndev = alloc_candev(sizeof(struct pic32_priv), PIC32_MAX_FIFO_MSG);
	if (!ndev) {
		dev_err(&pdev->dev, "alloc_candev failed\n");
		ret = -ENOMEM;
		goto err;
	}

	priv = netdev_priv(ndev);
	priv->ndev = ndev;
	priv->pdev = pdev;
	priv->reg_base = addr;
	priv->transceiver_gpio = gpio;
	priv->tx_head = PIC32_TX_FIFO_START;
	priv->tx_msg_head = PIC32_TX_FIFO_MSG_START;
	priv->tx_msg_tail = PIC32_TX_FIFO_MSG_START;
	priv->tx_max = PIC32_TX_FIFO_END;
	priv->can.bittiming_const = &pic32_bittiming_const;
	priv->can.do_set_bittiming = &pic32_set_bittiming;
	priv->can.do_set_mode = pic32_set_mode;
	priv->can.do_get_berr_counter = pic32_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTL_SUPPORT;

	ndev->irq = platform_get_irq(pdev, 0);
	ndev->flags |= IFF_ECHO;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &pic32_netdev_ops;

	/* Get CAN can_clk info */
	priv->can_clk = devm_clk_get(&pdev->dev, "can_clk");
	if (IS_ERR(priv->can_clk)) {
		dev_err(&pdev->dev, "Device clock not found.\n");
		ret = PTR_ERR(priv->can_clk);
		goto err_free;
	}

	ret = clk_prepare_enable(priv->can_clk);
	if (ret) {
		dev_err(&pdev->dev, "enable device clock failed\n");
		goto err_free;
	}

	priv->can.clock.freq = clk_get_rate(priv->can_clk) * 2;
	priv->tx_rx_buf = dmam_alloc_coherent(&pdev->dev, PIC32_BUF_LEN,
					      &priv->tx_rx_buf_dma, GFP_DMA);
	if (!priv->tx_rx_buf) {
		netdev_dbg(ndev, "Memory alloc failed...\n");
		goto err_unprepare_disable_dev;
	}

	clk_enable(priv->can_clk);

	ret = register_candev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "register_candev() failed\n");
		goto err_free_buffer;
	}

	/* Enable CAN Transceiver: set Stantby GPIO to Low */
	pic32_transceiver_switch(priv, 0);

	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%u)\n",
		 priv->reg_base, (u32)ndev->irq);

	return ret;

err_unprepare_disable_dev:
	clk_disable_unprepare(priv->can_clk);
err_free_buffer:
err_free:
	free_candev(ndev);
err:
	return ret;
}

static int pic32_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct pic32_priv *priv = netdev_priv(ndev);

	unregister_candev(ndev);
	free_candev(ndev);
	pic32_transceiver_switch(priv, 1);
	clk_disable_unprepare(priv->can_clk);
	devm_iounmap(&pdev->dev, priv->reg_base);
	return 0;
}

static const struct of_device_id pic32_can_id_table[] = {
	{ .compatible = "microchip,pic32mzda-can" },
	{}
};
MODULE_DEVICE_TABLE(of, pic32_sdhci_id_table);

static struct platform_driver pic32_can_driver = {
	.probe = pic32_can_probe,
	.remove = pic32_can_remove,
	.driver = {
		.name   = DEV_NAME,
		.of_match_table = pic32_can_id_table,
	},
};

module_platform_driver(pic32_can_driver);

MODULE_AUTHOR("Sandeep Sheriker<sandeepsheriker.mallikarjun@microchip.com>");
MODULE_VERSION(PIC32_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_ALIAS("platform:" DEV_NAME);
