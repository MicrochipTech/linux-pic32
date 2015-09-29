/*
 * Joshua Henderson, <joshua.henderson@microchip.com>
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

#ifndef _PIC32EC_ETHER_H
#define _PIC32EC_ETHER_H

/* Ethernet Controller/MAC Registers */
#define ETHCON1			0x0000
#define ETHCON2			0x0010
#define ETHTXST			0x0020
#define ETHRXST			0x0030
#define ETHHT0			0x0040
#define ETHHT1			0x0050
#define ETHPMM0			0x0060
#define ETHPMM1			0x0070
#define ETHPMCS			0x0080
#define ETHPMO			0x0090
#define ETHRXFC			0x00A0
#define ETHRXWM			0x00B0
#define ETHIEN			0x00C0
#define ETHIRQ			0x00D0
#define ETHSTAT			0x00E0
#define ETHRXOVRFLOW		0x0100
#define ETHFRMTXOK		0x0110
#define ETHSCOLFRM		0x0120
#define ETHMCOLFRM		0x0130
#define ETHFRMRXOK		0x0140
#define ETHFCSERR		0x0150
#define ETHALGNERR		0x0160
#define EMAC1CFG1		0x0200
#define EMAC1CFG2		0x0210
#define EMAC1IPGT		0x0220
#define EMAC1IPGR		0x0230
#define EMAC1CLRT		0x0240
#define EMAC1MAXF		0x0250
#define EMAC1SUPP		0x0260
#define EMAC1TEST		0x0270
#define EMAC1MCFG		0x0280
#define EMAC1MCMD		0x0290
#define EMAC1MADR		0x02A0
#define EMAC1MWTD		0x02B0
#define EMAC1MRDD		0x02C0
#define EMAC1MIND		0x02D0
#define EMAC1SA0		0x0300
#define EMAC1SA1		0x0310
#define EMAC1SA2		0x0320

/* EMAC1MCMD_READ Offsets/Sizes */
#define EMAC1MCMD_READ		0

/* EMAC1MADR Offset/Sizes */
#define EMAC1MADR_PHYADDR	8
#define EMAC1MADR_REGADDR	0

/* ETHSTAT Offsets/Sizes */
#define ETHSTAT_BUFCNT		16
#define ETHSTAT_BUFCNT_SIZE	8
#define ETHSTAT_ETHBUSY		7
#define ETHSTAT_TXBUSY		6
#define ETHSTAT_RXBUSY		5

/* ETHCON1 Offsets/Sizes */
#define ETHCON1_ON		15
#define ETHCON1_SIDL		13
#define ETHCON1_TXRTS		9
#define ETHCON1_RXEN		8
#define ETHCON1_AUTOFC		7
#define ETHCON1_MANFC		4
#define ETHCON1_BUFCDEC		0

/* ETHCON2 Offsets/Sizes */
#define ETHCON2_RXBUFSZ		4
#define ETHCON2_RXBUFSZ_SIZE	7

/* ETHRXFC Offsets/Sizes */
#define ETHRXFC_HTEN		15
#define ETHRXFC_MPEN		14
#define ETHRXFC_NOTPM		12
#define ETHRXFC_PMMODE		8
#define ETHRXFC_PMMODE_SIZE	4
#define ETHRXFC_CRCERREN	7
#define ETHRXFC_CRCOKEN		6
#define ETHRXFC_RUNTERREN	5
#define ETHRXFC_RUNTEN		4
#define ETHRXFC_UCEN		3
#define ETHRXFC_NOTMEEN		2
#define ETHRXFC_MCEN		1
#define ETHRXFC_BCEN		0

/* ETHIEN Offsets/Sizes */
#define ETHIEN_TXBUSEIE		14
#define ETHIEN_RXBUSEIE		13
#define ETHIEN_EWMARKIE		9
#define ETHIEN_FWMARKIE		8
#define ETHIEN_RXDONEIE		7
#define ETHIEN_PKTPENDIE	6
#define ETHIEN_RXACTIE		5
#define ETHIEN_TXDONEIE		3
#define ETHIEN_TXABORTIE	2
#define ETHIEN_RXBUFNAIE	1
#define ETHIEN_RXOVFLWIE	0

/* ETHIRQ Offsets/Sizes */
#define ETHIRQ_TXBUSE		14
#define ETHIRQ_RXBUSE		13
#define ETHIRQ_EWMARK		9
#define ETHIRQ_FWMARK		8
#define ETHIRQ_RXDONE		7
#define ETHIRQ_PKTPEND		6
#define ETHIRQ_RXACT		5
#define ETHIRQ_TXDONE		3
#define ETHIRQ_TXABORT		2
#define ETHIRQ_RXBUFNA		1
#define ETHIRQ_RXOVFLW		0

/* EMAC1CFG1 Offsets/Sizes */
#define EMAC1CFG1_SOFTRESET	15
#define EMAC1CFG1_SIMRESET	14
#define EMAC1CFG1_RESETRMCS	11
#define EMAC1CFG1_RESETRFUN	10
#define EMAC1CFG1_RESETTMCS	9
#define EMAC1CFG1_RESETTFUN	8
#define EMAC1CFG1_LOOPBACK	4
#define EMAC1CFG1_TXPAUSE	3
#define EMAC1CFG1_RXPAUSE	2
#define EMAC1CFG1_PASSALL	1
#define EMAC1CFG1_RXENABLE	0

/* EMAC1CFG2 Offsets/Sizes */
#define EMAC1CFG2_EXCESSDFR	14
#define EMAC1CFG2_BPNOBKOFF	13
#define EMAC1CFG2_NOBKOFF	12
#define EMAC1CFG2_LONGPRE	9
#define EMAC1CFG2_PUREPRE	8
#define EMAC1CFG2_AUTOPAD	7
#define EMAC1CFG2_VLANPAD	6
#define EMAC1CFG2_PADENABLE	5
#define EMAC1CFG2_CRCENABLE	4
#define EMAC1CFG2_DELAYCRC	3
#define EMAC1CFG2_HUGEFRM	2
#define EMAC1CFG2_LENGTHCK	1
#define EMAC1CFG2_FULLDPLX	0

/* EMAC1SUPP Offsets/Sizes */
#define EMAC1SUPP_RESETRMII	11
#define EMAC1SUPP_SPEEDRMII	8

/* EMAC1MCFG Offsets/Sizes */
#define EMAC1MCFG_RESETMGMT	15
#define EMAC1MCFG_CLKSEL	2
#define EMAC1MCFG_NOPRE		1
#define EMAC1MCFG_SCANINC	0

#define CLKSEL_DIV4		1
#define CLKSEL_DIV6		2
#define CLKSEL_DIV8		3
#define CLKSEL_DIV10		4
#define CLKSEL_DIV14		5
#define CLKSEL_DIV20		6
#define CLKSEL_DIV28		7
#define CLKSEL_DIV40		8
#define CLKSEL_DIV48		9
#define CLKSEL_DIV50		10

/* EMAC1MIND Offsets/Sizes */
#define EMAC1MIND_LINKFAIL	3
#define EMAC1MIND_NOTVALID	2
#define EMAC1MIND_SCAN		1
#define EMAC1MIND_MIIMBUSY	0

#define MAC_RX_INT_FLAGS (MAC_BIT(ETHIRQ_RXDONE) | \
				MAC_BIT(ETHIRQ_PKTPEND))

#define MAC_TX_INT_FLAGS (MAC_BIT(ETHIRQ_TXDONE) | \
				MAC_BIT(ETHIRQ_TXBUSE) | \
				MAC_BIT(ETHIRQ_TXABORT))

#define MAC_RX_EN_FLAGS (MAC_BIT(ETHIEN_RXDONEIE) | \
				MAC_BIT(ETHIEN_RXBUFNAIE) | \
				MAC_BIT(ETHIEN_RXOVFLWIE) | \
				MAC_BIT(ETHIEN_RXBUSEIE))

#define MAC_TX_EN_FLAGS (MAC_BIT(ETHIEN_TXBUSEIE) | \
				MAC_BIT(ETHIEN_TXDONEIE) | \
				MAC_BIT(ETHIEN_TXABORTIE))

/* PHY register offsets */
#define PHY_EXT_PAGE_ACCESS	0x1f

#define EC_QUIRK_USE_SRAM	BIT(1)

#define MAC_BIT(offset) (1 << offset)
#define MAC_BF(name, value)				\
	(((value) & ((1 << name##_SIZE) - 1)) << name)
#define MAC_GF(name, value)				\
	(((value) >> name) & ((1 << name##_SIZE) - 1))

#define mac_readl(port, reg)			\
	__raw_readl((port)->regs + reg)

#define mac_writel(port, reg, value)			\
	__raw_writel((value), (port)->regs + reg)

struct pic32ec_stats {
	u32 rx_overruns;
	u32 tx_ok;
	u32 tx_single_cols;
	u32 tx_multiple_cols;
	u32 rx_ok;
	u32 rx_fcs_errors;
	u32 rx_align_errors;
};

/**
 * struct pic32ec_dma_desc - Hardware DMA descriptor
 * @ctrl: Descriptor control bits
 * @addr: DMA address of data buffer
 * @stat0: TX/RX status bits
 * @stat1: TX/RX status bits
 * @next: DMA address of next descriptor
 */
struct pic32ec_dma_desc {
	u32 ctrl;
	u32 addr;
	u32 stat0;
	u32 stat1;
	u32 next;
} __packed;

/* pic32ec_dma_desc::ctrl Offsets/Sizes */
#define EOWN 7
#define NPV 8
#define BCOUNT 16
#define BCOUNT_SIZE 11
#define EOP 30
#define SOP 31

/* pic32ec_dma_desc::stat Offsets/Sizes */
#define RSV_RECEIVED_BYTE_COUNT 0
#define RSV_RECEIVED_BYTE_COUNT_SIZE 16

/**
 * struct pic32ec_tx_skb - data about an skb which is being transmitted
 * @skb: skb currently being transmitted
 * @mapping: DMA address of the skb's data buffer
 */
struct pic32ec_tx_skb {
	struct sk_buff *skb;
	dma_addr_t mapping;
	void *data;
	size_t len;
};

struct pic32ec {
	void __iomem *regs;

	unsigned int rx_tail;
	struct pic32ec_dma_desc *rx_ring;
	struct sk_buff **rx_skbuff;
	void *rx_buffers;
	size_t rx_buffer_size;

	unsigned int tx_head, tx_tail;
	struct pic32ec_dma_desc *tx_ring;
	struct pic32ec_tx_skb *tx_skb;

	spinlock_t lock;	/* interrupt lock */
	struct platform_device *pdev;
	struct clk *pclk;
	struct net_device *dev;
	struct napi_struct napi;
	struct work_struct tx_error_task;
	struct net_device_stats stats;

	struct pic32ec_stats hw_stats;

	dma_addr_t rx_ring_dma;
	dma_addr_t tx_ring_dma;
	dma_addr_t rx_buffers_dma;

	struct mii_bus *mii_bus;
	struct phy_device *phy_dev;
	unsigned int link;
	unsigned int speed;
	unsigned int duplex;
	u32 quirks;

	phy_interface_t phy_interface;
};

#endif
