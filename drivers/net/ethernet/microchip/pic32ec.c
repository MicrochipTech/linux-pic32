/*
 * PIC32 Ethernet controller driver
 *
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
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/circ_buf.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/pic32ec.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_gpio.h>
#include <asm/mach-pic32/pic32.h>

#include "pic32ec.h"

#define MAC_RX_BUFFER_SIZE	256	/* must be a multiple of 16 */
#define RX_RING_SIZE		256	/* must be power of 2 */
#define RX_RING_BYTES		(sizeof(struct pic32ec_dma_desc) * RX_RING_SIZE)

#define TX_RING_SIZE		128	/* must be power of 2 */
#define TX_RING_BYTES		(sizeof(struct pic32ec_dma_desc) * TX_RING_SIZE)

/* level of occupied TX descriptors under which we wake up TX process */
#define MAC_TX_WAKEUP_THRESH	(3 * TX_RING_SIZE / 4)

/* Graceful stop timeouts in us. We should allow up to
 * 1 frame time (10 Mbits/s, full-duplex, ignoring collisions)
 */
#define MAC_HALT_TIMEOUT	1230

#define DRIVER_NAME "pic32ec"

#define PIC32_SEL(set, reg)	(set ? PIC32_SET(reg) : PIC32_CLR(reg))

static u32 pic32ec_mdc_clk_div(struct pic32ec *bp);
static void pic32ec_reset_hw(struct pic32ec *bp);
static void pic32ec_set_rx_mode(struct net_device *dev);

/* Ring buffer accessors */
static inline unsigned int pic32ec_tx_ring_wrap(unsigned int index)
{
	return index & (TX_RING_SIZE - 1);
}

static inline struct pic32ec_dma_desc *pic32ec_tx_desc(struct pic32ec *bp,
						       unsigned int index)
{
	return &bp->tx_ring[pic32ec_tx_ring_wrap(index)];
}

static inline struct pic32ec_tx_skb *pic32ec_tx_skb(struct pic32ec *bp,
						    unsigned int index)
{
	return &bp->tx_skb[pic32ec_tx_ring_wrap(index)];
}

static inline dma_addr_t pic32ec_tx_dma(struct pic32ec *bp, unsigned int index)
{
	dma_addr_t offset;

	offset = pic32ec_tx_ring_wrap(index) * sizeof(struct pic32ec_dma_desc);

	return bp->tx_ring_dma + offset;
}

static inline unsigned int pic32ec_rx_ring_wrap(unsigned int index)
{
	return index & (RX_RING_SIZE - 1);
}

static inline struct pic32ec_dma_desc *pic32ec_rx_desc(struct pic32ec *bp,
						       unsigned int index)
{
	return &bp->rx_ring[pic32ec_rx_ring_wrap(index)];
}

static inline void *pic32ec_rx_buffer(struct pic32ec *bp, unsigned int index)
{
	return bp->rx_buffers +
	    bp->rx_buffer_size * pic32ec_rx_ring_wrap(index);
}

static void pic32ec_set_hwaddr(struct pic32ec *bp)
{
	u16 val;

	val = cpu_to_le16(*((u16 *)(bp->dev->dev_addr)));
	mac_writel(bp, EMAC1SA0, val);
	val = cpu_to_le16(*((u16 *)(bp->dev->dev_addr + 2)));
	mac_writel(bp, EMAC1SA1, val);
	val = cpu_to_le16(*((u16 *)(bp->dev->dev_addr + 4)));
	mac_writel(bp, EMAC1SA2, val);
}

static void pic32ec_get_hwaddr(struct pic32ec *bp)
{
	struct pic32ec_platform_data *pdata;
	u8 addr[6];

	pdata = dev_get_platdata(&bp->pdev->dev);

	if (pdata && pdata->rev_eth_addr) {
		addr[5] = (mac_readl(bp, EMAC1SA0) & 0x00ff);
		addr[4] = (mac_readl(bp, EMAC1SA0) & 0xff00) >> 8;
		addr[3] = (mac_readl(bp, EMAC1SA1) & 0x00ff);
		addr[2] = (mac_readl(bp, EMAC1SA1) & 0xff00) >> 8;
		addr[1] = (mac_readl(bp, EMAC1SA2) & 0x00ff);
		addr[0] = (mac_readl(bp, EMAC1SA2) & 0xff00) >> 8;
	} else {
		addr[0] = (mac_readl(bp, EMAC1SA0) & 0x00ff);
		addr[1] = (mac_readl(bp, EMAC1SA0) & 0xff00) >> 8;
		addr[2] = (mac_readl(bp, EMAC1SA1) & 0x00ff);
		addr[3] = (mac_readl(bp, EMAC1SA1) & 0xff00) >> 8;
		addr[4] = (mac_readl(bp, EMAC1SA2) & 0x00ff);
		addr[5] = (mac_readl(bp, EMAC1SA2) & 0xff00) >> 8;
	}

	if (is_valid_ether_addr(addr)) {
		memcpy(bp->dev->dev_addr, addr, sizeof(addr));
		return;
	}

	netdev_info(bp->dev, "invalid hw address, using random\n");
	eth_hw_addr_random(bp->dev);
}

static int pic32ec_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct pic32ec *bp = bus->priv;
	int value;

	/* Set the address of the PHY, and the register number. */
	mac_writel(bp, EMAC1MADR, (mii_id << EMAC1MADR_PHYADDR) |
		   (regnum & PHY_EXT_PAGE_ACCESS));

	mac_writel(bp, EMAC1MCMD, MAC_BIT(EMAC1MCMD_READ));

	/* Must wait for pipeline before checking busy flag */
	udelay(1);

	while ((mac_readl(bp, EMAC1MIND) & MAC_BIT(EMAC1MIND_MIIMBUSY)))
		cpu_relax();

	mac_writel(bp, PIC32_CLR(EMAC1MCMD), MAC_BIT(EMAC1MCMD_READ));

	value = mac_readl(bp, EMAC1MRDD);

	return value;
}

static int pic32ec_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			      u16 value)
{
	struct pic32ec *bp = bus->priv;

	/* Set the address of the PHY, and the register number. */
	mac_writel(bp, EMAC1MADR, (mii_id << EMAC1MADR_PHYADDR) |
		   (regnum & PHY_EXT_PAGE_ACCESS));

	mac_writel(bp, EMAC1MWTD, value);

	/* Must wait for pipeline before checking busy flag */
	udelay(1);

	while ((mac_readl(bp, EMAC1MIND) & MAC_BIT(EMAC1MIND_MIIMBUSY)))
		cpu_relax();

	return 0;
}

static int pic32ec_mdio_reset(struct mii_bus *bus)
{
	struct pic32ec *bp = bus->priv;

	mac_writel(bp, PIC32_SET(EMAC1MCFG), MAC_BIT(EMAC1MCFG_RESETMGMT));
	mac_writel(bp, PIC32_CLR(EMAC1MCFG), MAC_BIT(EMAC1MCFG_RESETMGMT));

	/* Wait for the operation to finish */
	while ((mac_readl(bp, EMAC1MIND) & MAC_BIT(EMAC1MIND_MIIMBUSY)))
		cpu_relax();

	mac_writel(bp, PIC32_CLR(EMAC1MCFG), MAC_BIT(EMAC1MCFG_RESETMGMT));

	/* Wait for the operation to finish */
	while ((mac_readl(bp, EMAC1MIND) & MAC_BIT(EMAC1MIND_MIIMBUSY)))
		cpu_relax();

	/* Set the MII Management Clock (MDC) - no faster than 2.5 MHz */
	mac_writel(bp, EMAC1MCFG, pic32ec_mdc_clk_div(bp) << EMAC1MCFG_CLKSEL);

	/* Wait for the operation to finish */
	while ((mac_readl(bp, EMAC1MIND) & MAC_BIT(EMAC1MIND_MIIMBUSY)))
		cpu_relax();

	return 0;
}

static void pic32ec_set_tx_speed(int speed, struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);

	switch (speed) {
	case SPEED_10:
		mac_writel(bp, PIC32_CLR(EMAC1SUPP),
			   MAC_BIT(EMAC1SUPP_SPEEDRMII));
		break;
	case SPEED_100:
		mac_writel(bp, PIC32_SET(EMAC1SUPP),
			   MAC_BIT(EMAC1SUPP_SPEEDRMII));
		break;
	default:
		netdev_warn(dev, "unsupported speed: %d\n", speed);
		break;
	}
}

static void pic32ec_handle_link_change(struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;
	unsigned long flags;

	int status_change = 0;

	spin_lock_irqsave(&bp->lock, flags);

	if (phydev->link) {
		if ((bp->speed != phydev->speed) ||
		    (bp->duplex != phydev->duplex)) {
			mac_writel(bp,
				   PIC32_SEL(phydev->duplex == DUPLEX_FULL,
					     EMAC1CFG2),
				   MAC_BIT(EMAC1CFG2_FULLDPLX));

			pic32ec_set_tx_speed(phydev->speed, dev);

			if (phydev->duplex == DUPLEX_FULL)
				mac_writel(bp, EMAC1IPGT, 0x15);
			else
				mac_writel(bp, EMAC1IPGT, 0x12);

			bp->speed = phydev->speed;
			bp->duplex = phydev->duplex;
			status_change = 1;
		}
	}

	if (phydev->link != bp->link) {
		if (!phydev->link) {
			bp->speed = 0;
			bp->duplex = -1;
		}
		bp->link = phydev->link;

		status_change = 1;
	}

	spin_unlock_irqrestore(&bp->lock, flags);

	if (status_change) {
		if (phydev->link) {
			netif_carrier_on(dev);
			netdev_info(dev, "link up (%d/%s)\n",
				    phydev->speed,
				    phydev->duplex == DUPLEX_FULL ?
				    "Full" : "Half");
		} else {
			netif_carrier_off(dev);
			netdev_info(dev, "link down\n");
		}
	}
}

static int pic32ec_mii_probe(struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);
	struct phy_device *phydev;
	int ret;

	phydev = phy_find_first(bp->mii_bus);
	if (!phydev) {
		netdev_err(dev, "no PHY found\n");
		return -ENXIO;
	}

	netdev_vdbg(bp->dev, "PHY: addr %d, phy_id 0x%08X\n",
		    phydev->addr, phydev->phy_id);

	/* attach the mac to the phy */
	ret = phy_connect_direct(dev, phydev, &pic32ec_handle_link_change,
				 bp->phy_interface);
	if (ret) {
		netdev_err(dev, "could not attach to PHY\n");
		return ret;
	}

	/* mask with MAC supported features */
	phydev->supported &= PHY_BASIC_FEATURES;

	phydev->advertising = phydev->supported;

	bp->link = 0;
	bp->speed = 0;
	bp->duplex = -1;
	bp->phy_dev = phydev;

	return 0;
}

#define MHZ(x)		((x) * 1000UL * 1000UL)

static u32 pic32ec_mdc_clk_div(struct pic32ec *bp)
{
	u32 config;
	unsigned long pclk_hz;

	/* Find a div that puts the clock no higher than 2.5MHz */
	pclk_hz = clk_get_rate(bp->pclk);

	if (pclk_hz <= MHZ(10))
		config = CLKSEL_DIV4;
	else if (pclk_hz <= MHZ(15))
		config = CLKSEL_DIV6;
	else if (pclk_hz <= MHZ(20))
		config = CLKSEL_DIV8;
	else if (pclk_hz <= MHZ(25))
		config = CLKSEL_DIV10;
	else if (pclk_hz <= MHZ(35))
		config = CLKSEL_DIV14;
	else if (pclk_hz <= MHZ(50))
		config = CLKSEL_DIV20;
	else if (pclk_hz <= MHZ(70))
		config = CLKSEL_DIV28;
	else if (pclk_hz <= MHZ(100))
		config = CLKSEL_DIV40;
	else if (pclk_hz <= MHZ(120))
		config = CLKSEL_DIV48;
	else
		config = CLKSEL_DIV50;

	return config;
}

static int pic32ec_mii_init(struct pic32ec *bp)
{
	struct pic32ec_platform_data *pdata;
	struct device_node *np;
	int err = -ENXIO, i;

	pic32ec_reset_hw(bp);

	/* Turn on the MAC */
	mac_writel(bp, EMAC1CFG1, MAC_BIT(EMAC1CFG1_TXPAUSE) |
		   MAC_BIT(EMAC1CFG1_RXPAUSE) | MAC_BIT(EMAC1CFG1_RXENABLE));

	bp->mii_bus = mdiobus_alloc();
	if (!bp->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	bp->mii_bus->name = "MAC_mii_bus";
	bp->mii_bus->read = &pic32ec_mdio_read;
	bp->mii_bus->write = &pic32ec_mdio_write;
	bp->mii_bus->reset = &pic32ec_mdio_reset;
	snprintf(bp->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 bp->pdev->name, bp->pdev->id);
	bp->mii_bus->priv = bp;
	bp->mii_bus->parent = &bp->dev->dev;
	pdata = dev_get_platdata(&bp->pdev->dev);

	bp->mii_bus->irq = kzalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!bp->mii_bus->irq) {
		err = -ENOMEM;
		goto err_out_free_mdiobus;
	}

	dev_set_drvdata(&bp->dev->dev, bp->mii_bus);

	np = bp->pdev->dev.of_node;
	if (np) {
		/* try dt phy registration */
		err = of_mdiobus_register(bp->mii_bus, np);

		/* fallback to standard phy registration if no phy were
		 * found during dt phy registration
		 */
		if (!err && !phy_find_first(bp->mii_bus)) {
			for (i = 0; i < PHY_MAX_ADDR; i++) {
				struct phy_device *phydev;

				phydev = mdiobus_scan(bp->mii_bus, i);
				if (IS_ERR(phydev)) {
					err = PTR_ERR(phydev);
					break;
				}
			}

			if (err)
				goto err_out_unregister_bus;
		}
	} else {
		for (i = 0; i < PHY_MAX_ADDR; i++)
			bp->mii_bus->irq[i] = PHY_POLL;

		if (pdata)
			bp->mii_bus->phy_mask = pdata->phy_mask;

		err = mdiobus_register(bp->mii_bus);
	}

	if (err)
		goto err_out_free_mdio_irq;

	err = pic32ec_mii_probe(bp->dev);
	if (err)
		goto err_out_unregister_bus;

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(bp->mii_bus);
err_out_free_mdio_irq:
	kfree(bp->mii_bus->irq);
err_out_free_mdiobus:
	mdiobus_free(bp->mii_bus);
err_out:
	return err;
}

static void pic32ec_update_stats(struct pic32ec *bp)
{
	u32 __iomem *reg = bp->regs + ETHRXOVRFLOW;
	u32 *p = &bp->hw_stats.rx_overruns;
	u32 *end = &bp->hw_stats.rx_align_errors + 1;

	for (; p < end; p++, reg += 4)
		*p += __raw_readl(reg);
}

static int pic32ec_halt_tx(struct pic32ec *bp)
{
	unsigned long halt_time, timeout;
	u32 status;

	mac_writel(bp, PIC32_CLR(ETHCON1), MAC_BIT(ETHCON1_TXRTS));

	timeout = jiffies + usecs_to_jiffies(MAC_HALT_TIMEOUT);
	do {
		halt_time = jiffies;
		status = mac_readl(bp, ETHSTAT);
		if (!(status & MAC_BIT(ETHSTAT_TXBUSY)))
			return 0;

		usleep_range(10, 250);
	} while (time_before(halt_time, timeout));

	return -ETIMEDOUT;
}

static void pic32ec_tx_unmap(struct pic32ec *bp, struct pic32ec_tx_skb *tx_skb)
{
	if (bp->quirks & EC_QUIRK_USE_SRAM) {
		if (tx_skb->mapping) {
			dma_free_coherent(&bp->pdev->dev, tx_skb->len,
					  tx_skb->data, tx_skb->mapping);
			tx_skb->mapping = 0;
		}
	} else {
		if (tx_skb->mapping) {
			dma_unmap_single(&bp->pdev->dev, tx_skb->mapping,
					 tx_skb->len, DMA_TO_DEVICE);
			tx_skb->mapping = 0;
		}
	}

	if (tx_skb->skb) {
		dev_kfree_skb_any(tx_skb->skb);
		tx_skb->skb = NULL;
	}
}

static void pic32ec_tx_error_task(struct work_struct *work)
{
	struct pic32ec *bp = container_of(work, struct pic32ec,
					  tx_error_task);
	struct pic32ec_tx_skb *tx_skb;
	struct pic32ec_dma_desc *desc;
	struct sk_buff *skb;
	unsigned int tail;
	unsigned long flags;

	spin_lock_irqsave(&bp->lock, flags);

	netdev_err(bp->dev, "pic32ec_tx_error_task: t = %u, h = %u\n",
		   bp->tx_tail, bp->tx_head);

	/* Make sure nobody is trying to queue up new packets */
	netif_stop_queue(bp->dev);

	/* Stop transmission now
	 * (in case we have just queued new packets)
	 */
	if (pic32ec_halt_tx(bp))
		netdev_err(bp->dev, "BUG: halt tx timed out\n");

	/* Treat frames in TX queue including the ones that caused the error.
	 * Free transmit buffers in upper layer.
	 */
	for (tail = bp->tx_tail; tail != bp->tx_head; tail++) {
		u32 ctrl;

		desc = pic32ec_tx_desc(bp, tail);
		ctrl = desc->ctrl;
		tx_skb = pic32ec_tx_skb(bp, tail);
		skb = tx_skb->skb;

		if (!(ctrl & MAC_BIT(EOWN))) {
			if (skb) {
				netdev_vdbg(bp->dev,
					    "txerr skb %u (data %p) TX complete\n",
					    pic32ec_tx_ring_wrap(tail),
					    skb->data);
				bp->stats.tx_packets++;
				bp->stats.tx_bytes += skb->len;
			}
		} else {
			desc->stat0 = 0;
			desc->stat1 = 0;
			desc->addr = 0;
			desc->ctrl = MAC_BIT(NPV);
		}

		pic32ec_tx_unmap(bp, tx_skb);
	}

	/* Reinitialize the TX desc queue */
	mac_writel(bp, ETHTXST, bp->tx_ring_dma);

	/* Make TX ring reflect state of hardware */
	bp->tx_head = 0;
	bp->tx_tail = 0;

	/* Housework before enabling TX IRQ */
	mac_writel(bp, PIC32_CLR(ETHIRQ), -1);
	mac_writel(bp, PIC32_SET(ETHIEN), MAC_TX_EN_FLAGS);

	/* Now we are ready to start transmission again */
	netif_wake_queue(bp->dev);

	spin_unlock_irqrestore(&bp->lock, flags);
}

static void pic32ec_tx_interrupt(struct pic32ec *bp)
{
	unsigned int tail;
	unsigned int head;

	head = bp->tx_head;
	for (tail = bp->tx_tail; tail != head; tail++) {
		struct pic32ec_tx_skb *tx_skb;
		struct sk_buff *skb;
		struct pic32ec_dma_desc *desc;
		u32 ctrl;

		desc = pic32ec_tx_desc(bp, tail);

		ctrl = desc->ctrl;

		if (ctrl & MAC_BIT(EOWN))
			break;

		tx_skb = pic32ec_tx_skb(bp, tail);
		skb = tx_skb->skb;

		if (skb) {
			netdev_vdbg(bp->dev, "skb %u (data %p) TX complete\n",
				    pic32ec_tx_ring_wrap(tail), skb->data);
			bp->stats.tx_packets++;
			bp->stats.tx_bytes += skb->len;
		}

		pic32ec_tx_unmap(bp, tx_skb);

		desc->stat0 = 0;
		desc->stat1 = 0;
		desc->addr = 0;
		desc->ctrl = MAC_BIT(NPV);
	}

	bp->tx_tail = tail;
	if (netif_queue_stopped(bp->dev) &&
	    (CIRC_CNT(bp->tx_head, bp->tx_tail, TX_RING_SIZE) <=
	     MAC_TX_WAKEUP_THRESH))
		netif_wake_queue(bp->dev);
}

/* Mark DMA descriptors from begin up to and not including end as unused */
static void discard_partial_frame(struct pic32ec *bp, unsigned int begin,
				  unsigned int end)
{
	unsigned int frag;
	struct pic32ec_dma_desc *desc;
	u32 ctrl;

	netdev_dbg(bp->dev,
		   "discard_partial_frame frags %u - %u\n", begin, end);

	for (frag = begin; frag != end; frag++) {
		desc = pic32ec_rx_desc(bp, frag);
		desc->stat0 = 0;
		desc->stat1 = 0;
		ctrl = MAC_BIT(EOWN) | MAC_BIT(NPV);

		desc->ctrl = ctrl;

		mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_BUFCDEC));
	}
}

static int pic32ec_rx_frame(struct pic32ec *bp, unsigned int first_frag,
			    unsigned int last_frag)
{
	unsigned int len;
	unsigned int frag;
	unsigned int offset;
	struct sk_buff *skb;
	struct pic32ec_dma_desc *desc;

	desc = pic32ec_rx_desc(bp, first_frag);
	len = MAC_GF(RSV_RECEIVED_BYTE_COUNT, desc->stat1);

	netdev_vdbg(bp->dev, "pic32ec_rx_frame frags %u - %u (len %u)\n",
		    pic32ec_rx_ring_wrap(first_frag),
		    pic32ec_rx_ring_wrap(last_frag), len);

	skb = netdev_alloc_skb(bp->dev, len + NET_IP_ALIGN);
	if (!skb) {
		bp->stats.rx_dropped++;
		discard_partial_frame(bp, first_frag, last_frag + 1);
		return 1;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	offset = 0;
	skb_checksum_none_assert(skb);
	skb_put(skb, len);

	for (frag = first_frag;; frag++) {
		unsigned int frag_len;

		desc = pic32ec_rx_desc(bp, frag);
		frag_len = MAC_GF(BCOUNT, desc->ctrl);

		if (offset + frag_len > len) {
			BUG_ON(frag != last_frag);
			frag_len = len - offset;
		}

		skb_copy_to_linear_data_offset(skb, offset,
					       pic32ec_rx_buffer(bp, frag),
					       frag_len);

#if defined(VERBOSE_DEBUG)
		print_hex_dump(KERN_DEBUG, "data: ", DUMP_PREFIX_OFFSET, 16, 1,
			       pic32ec_rx_buffer(bp, frag), frag_len, true);
#endif

		offset += frag_len;

		desc = pic32ec_rx_desc(bp, frag);
		desc->stat0 = 0;
		desc->stat1 = 0;
		desc->ctrl = MAC_BIT(EOWN) | MAC_BIT(NPV);

		mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_BUFCDEC));

		if (frag == last_frag)
			break;
	}

	skb->protocol = eth_type_trans(skb, bp->dev);

	skb_checksum_none_assert(skb);
	if (bp->dev->features & NETIF_F_RXCSUM &&
	    !(bp->dev->flags & IFF_PROMISC))
		skb->ip_summed = CHECKSUM_UNNECESSARY;

	bp->stats.rx_packets++;
	bp->stats.rx_bytes += skb->len;
	netdev_vdbg(bp->dev, "received skb of length %u, csum: %08x\n",
		    skb->len, skb->csum);
	netif_receive_skb(skb);

	return 0;
}

static int pic32ec_rx(struct pic32ec *bp, int budget)
{
	int received = 0;
	unsigned int tail;
	int first_frag = -1;

	for (tail = bp->rx_tail; budget > 0; tail++) {
		struct pic32ec_dma_desc *desc = pic32ec_rx_desc(bp, tail);
		u32 ctrl;

		ctrl = desc->ctrl;

		if (ctrl & MAC_BIT(EOWN))
			break;

		if (ctrl & MAC_BIT(SOP)) {
			if (first_frag != -1)
				discard_partial_frame(bp, first_frag, tail);
			first_frag = tail;
		}

		if (ctrl & MAC_BIT(EOP)) {
			int dropped;

			BUG_ON(first_frag == -1);
			dropped = pic32ec_rx_frame(bp, first_frag, tail);
			first_frag = -1;
			if (!dropped) {
				received++;
				budget--;
			}
		}
	}

	if (first_frag != -1)
		bp->rx_tail = first_frag;
	else
		bp->rx_tail = tail;

	return received;
}

static int pic32ec_poll(struct napi_struct *napi, int budget)
{
	struct pic32ec *bp = container_of(napi, struct pic32ec, napi);
	int work_done;
	u32 status;

	netdev_vdbg(bp->dev, "poll: budget = %d\n", budget);

	work_done = pic32ec_rx(bp, budget);
	if (work_done < budget) {
		napi_complete(napi);

		/* Packets received while interrupts were disabled */
		status = mac_readl(bp, ETHIRQ);
		if (status & MAC_RX_INT_FLAGS) {
			mac_writel(bp, PIC32_CLR(ETHIRQ), MAC_RX_INT_FLAGS);
			napi_reschedule(napi);
		} else {
			mac_writel(bp, PIC32_SET(ETHIEN), MAC_RX_EN_FLAGS);
		}
	}

	return work_done;
}

static irqreturn_t pic32ec_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct pic32ec *bp = netdev_priv(dev);
	u32 status;

	status = mac_readl(bp, ETHIRQ);

	if (unlikely(!status))
		return IRQ_NONE;

	netdev_vdbg(bp->dev, "isr = 0x%08lx\n", (unsigned long)status);

	spin_lock(&bp->lock);

	if (status & MAC_BIT(ETHIRQ_RXBUSE)) {
		netdev_warn(bp->dev, "rx bus error\n");
		mac_writel(bp, PIC32_CLR(ETHIRQ), MAC_BIT(ETHIRQ_RXBUSE));
	}

	if (status & MAC_BIT(ETHIRQ_RXBUFNA)) {
		netdev_warn(bp->dev, "rx buf na\n");
		mac_writel(bp, PIC32_CLR(ETHIRQ), MAC_BIT(ETHIRQ_RXBUFNA));
	}

	if (status & MAC_BIT(ETHIRQ_RXOVFLW)) {
		netdev_warn(bp->dev, "rx overflow\n");
		mac_writel(bp, PIC32_CLR(ETHIRQ), MAC_BIT(ETHIRQ_RXOVFLW));
	}

	mac_writel(bp, PIC32_CLR(ETHIRQ), MAC_TX_INT_FLAGS);

	/* close possible race with dev_close */
	if (unlikely(!netif_running(dev))) {
		mac_writel(bp, PIC32_CLR(ETHIEN), -1);
	} else {
		if (status & MAC_RX_INT_FLAGS) {
			/* There's no point taking any more RX interrupts
			 * until we have processed the buffers. The
			 * scheduling call may fail if the poll routine
			 * is already scheduled, so disable interrupts
			 * now.
			 */
			mac_writel(bp, PIC32_CLR(ETHIEN),
				   MAC_BIT(ETHIEN_RXDONEIE) |
				   MAC_BIT(ETHIEN_PKTPENDIE));
			mac_writel(bp, PIC32_CLR(ETHIRQ), MAC_RX_INT_FLAGS);

			napi_schedule(&bp->napi);
		}

		if (unlikely(status & (MAC_BIT(ETHIRQ_TXABORT) |
				       MAC_BIT(ETHIRQ_TXBUSE)))) {
			mac_writel(bp, PIC32_CLR(ETHIEN), MAC_TX_EN_FLAGS);
			schedule_work(&bp->tx_error_task);
		} else {
			if (status & MAC_BIT(ETHIRQ_TXDONE))
				pic32ec_tx_interrupt(bp);
		}
	}

	spin_unlock(&bp->lock);

	return IRQ_HANDLED;
}

static int pic32ec_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);
	dma_addr_t mapping;
	unsigned int len, entry;
	struct pic32ec_dma_desc *desc;
	struct pic32ec_tx_skb *tx_skb;
	u32 ctrl;
	unsigned long flags;
	void *data;

	netdev_dbg(bp->dev,
		   "start_xmit: len %u head %p data %p tail %p end %p\n",
		   skb->len, skb->head, skb->data,
		   skb_tail_pointer(skb), skb_end_pointer(skb));

#if defined(VERBOSE_DEBUG)
	print_hex_dump(KERN_DEBUG, "data: ", DUMP_PREFIX_OFFSET, 16, 1,
		       skb->data, 16, true);
#endif

	len = skb->len;
	spin_lock_irqsave(&bp->lock, flags);

	/* This is a hard error, log it. */
	if (CIRC_SPACE(bp->tx_head, bp->tx_tail, TX_RING_SIZE) < 1) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&bp->lock, flags);
		netdev_err(bp->dev, "BUG! Tx Ring full when queue awake!\n");
		netdev_dbg(bp->dev, "tx_head = %u, tx_tail = %u\n",
			   bp->tx_head, bp->tx_tail);
		return NETDEV_TX_BUSY;
	}

	entry = pic32ec_tx_ring_wrap(bp->tx_head);
	netdev_vdbg(bp->dev, "Allocated ring entry %u\n", entry);

	if (bp->quirks & EC_QUIRK_USE_SRAM) {
		data = dmam_alloc_coherent(&bp->pdev->dev, len,
					   &mapping, GFP_KERNEL);
		if (!data) {
			dev_kfree_skb_any(skb);
			goto unlock;
		}

		skb_copy_from_linear_data(skb, data, len);
	} else {
		data = skb->data;
		mapping = dma_map_single(&bp->pdev->dev, data,
					 len, DMA_TO_DEVICE);
		if (dma_mapping_error(&bp->pdev->dev, mapping)) {
			dev_kfree_skb_any(skb);
			goto unlock;
		}
	}

	bp->tx_head++;
	tx_skb = &bp->tx_skb[entry];
	tx_skb->skb = skb;
	tx_skb->mapping = mapping;
	tx_skb->data = data;
	tx_skb->len = len;

	netdev_vdbg(bp->dev, "Mapped skb data %p to DMA addr %08lx\n",
		    data, (unsigned long)mapping);

	ctrl = MAC_BF(BCOUNT, len);
	ctrl |= MAC_BIT(EOWN);
	ctrl |= MAC_BIT(NPV);
	ctrl |= MAC_BIT(SOP);
	ctrl |= MAC_BIT(EOP);

	desc = &bp->tx_ring[entry];

	desc->addr = mapping;
	desc->stat0 = 0;
	desc->stat1 = 0;
	desc->ctrl = ctrl;

	skb_tx_timestamp(skb);

	/* ship it */
	mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_TXRTS));

	if (CIRC_SPACE(bp->tx_head, bp->tx_tail, TX_RING_SIZE) < 1) {
		netdev_err(bp->dev, "TX queue full\n");
		netif_stop_queue(dev);
	}

unlock:
	spin_unlock_irqrestore(&bp->lock, flags);

	return NETDEV_TX_OK;
}

static void pic32ec_init_rx_buffer_size(struct pic32ec *bp)
{
	bp->rx_buffer_size = MAC_RX_BUFFER_SIZE;

	netdev_dbg(bp->dev, "mtu [%u] rx_buffer_size [%Zu]\n",
		   bp->dev->mtu, bp->rx_buffer_size);
}

static void pic32ec_free_rx_buffers(struct pic32ec *bp)
{
	if (!bp->rx_buffers)
		return;

	dmam_free_coherent(&bp->pdev->dev,
			   RX_RING_SIZE * bp->rx_buffer_size,
			   bp->rx_buffers, bp->rx_buffers_dma);
	bp->rx_buffers = NULL;
}

static void pic32ec_free_consistent(struct pic32ec *bp)
{
	kfree(bp->tx_skb);
	bp->tx_skb = NULL;

	pic32ec_free_rx_buffers(bp);
	if (bp->rx_ring) {
		dmam_free_coherent(&bp->pdev->dev, RX_RING_BYTES,
				   bp->rx_ring, bp->rx_ring_dma);
		bp->rx_ring = NULL;
	}
	if (bp->tx_ring) {
		dmam_free_coherent(&bp->pdev->dev, TX_RING_BYTES,
				   bp->tx_ring, bp->tx_ring_dma);
		bp->tx_ring = NULL;
	}
}

static int pic32ec_alloc_rx_buffers(struct pic32ec *bp)
{
	int size;

	size = RX_RING_SIZE * bp->rx_buffer_size;
	bp->rx_buffers = dmam_alloc_coherent(&bp->pdev->dev, size,
					     &bp->rx_buffers_dma, GFP_KERNEL);
	if (!bp->rx_buffers)
		return -ENOMEM;

	netdev_dbg(bp->dev,
		   "Alloc RX buffers of %d bytes at %08lx (mapped %p)\n",
		   size, (unsigned long)bp->rx_buffers_dma, bp->rx_buffers);

	return 0;
}

static int pic32ec_alloc_rings(struct pic32ec *bp)
{
	int size;

	size = TX_RING_SIZE * sizeof(struct pic32ec_tx_skb);
	bp->tx_skb = kmalloc(size, GFP_KERNEL);
	if (!bp->tx_skb)
		goto out_err;

	size = RX_RING_BYTES;
	bp->rx_ring = dmam_alloc_coherent(&bp->pdev->dev, size,
					  &bp->rx_ring_dma, GFP_KERNEL);
	if (!bp->rx_ring)
		goto out_err;

	netdev_dbg(bp->dev,
		   "Allocated RX ring of %d bytes at %08lx (mapped %p)\n",
		   size, (unsigned long)bp->rx_ring_dma, bp->rx_ring);

	size = TX_RING_BYTES;
	bp->tx_ring = dmam_alloc_coherent(&bp->pdev->dev, size,
					  &bp->tx_ring_dma, GFP_KERNEL);
	if (!bp->tx_ring)
		goto out_err;
	netdev_dbg(bp->dev,
		   "Allocated TX ring of %d bytes at %08lx (mapped %p)\n",
		   size, (unsigned long)bp->tx_ring_dma, bp->tx_ring);

	if (pic32ec_alloc_rx_buffers(bp))
		goto out_err;

	return 0;

out_err:
	pic32ec_free_consistent(bp);
	return -ENOMEM;
}

static void pic32ec_init_rings(struct pic32ec *bp)
{
	int i;
	dma_addr_t addr;

	addr = bp->rx_buffers_dma;
	for (i = 0; i < RX_RING_SIZE; i++) {
		struct pic32ec_dma_desc *desc = &bp->rx_ring[i];

		desc->ctrl = MAC_BIT(EOWN) | MAC_BIT(NPV);
		desc->addr = addr;
		desc->stat0 = 0;
		desc->stat1 = 0;
		desc->next = bp->rx_ring_dma + ((i + 1) * sizeof(*desc));
		addr += bp->rx_buffer_size;
	}
	bp->rx_ring[RX_RING_SIZE - 1].next = bp->rx_ring_dma;

	for (i = 0; i < TX_RING_SIZE; i++) {
		struct pic32ec_dma_desc *desc = &bp->tx_ring[i];

		desc->ctrl = MAC_BIT(NPV);
		desc->addr = 0;
		desc->stat0 = 0;
		desc->stat1 = 0;
		desc->next = bp->tx_ring_dma +
		    ((i + 1) * sizeof(struct pic32ec_dma_desc));
	}
	bp->tx_ring[TX_RING_SIZE - 1].next = bp->tx_ring_dma;

	bp->rx_tail = 0;
	bp->tx_head = 0;
	bp->tx_tail = 0;
}

static void pic32ec_reset_mac(struct pic32ec *bp)
{
	/* Reset the MAC */
	mac_writel(bp, PIC32_SET(EMAC1CFG1), MAC_BIT(EMAC1CFG1_SOFTRESET));
	mac_writel(bp, PIC32_CLR(EMAC1CFG1), MAC_BIT(EMAC1CFG1_SOFTRESET));

	if (bp->phy_interface == PHY_INTERFACE_MODE_RMII) {
		/* Reset RMII module */
		mac_writel(bp, PIC32_SET(EMAC1SUPP),
			   MAC_BIT(EMAC1SUPP_RESETRMII));
		mac_writel(bp, PIC32_CLR(EMAC1SUPP),
			   MAC_BIT(EMAC1SUPP_RESETRMII));
	}

	/* MIIM block reset */
	mac_writel(bp, PIC32_SET(EMAC1MCFG), MAC_BIT(EMAC1MCFG_RESETMGMT));
	mac_writel(bp, PIC32_CLR(EMAC1MCFG), MAC_BIT(EMAC1MCFG_RESETMGMT));

	/* Set MII management clock divider */
	mac_writel(bp, EMAC1MCFG, pic32ec_mdc_clk_div(bp) << EMAC1MCFG_CLKSEL);
}

static void pic32ec_reset_hw(struct pic32ec *bp)
{
	/* Disable RX and TX and halt any existing transmission. */
	mac_writel(bp, PIC32_CLR(ETHCON1), MAC_BIT(ETHCON1_ON) |
		   MAC_BIT(ETHCON1_TXRTS) | MAC_BIT(ETHCON1_RXEN));

	/* Wait until not busy */
	while (mac_readl(bp, ETHSTAT) & ETHSTAT_ETHBUSY)
		cpu_relax();

	/* Turn ethernet controller ON */
	mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_ON));

	/* Decrement buffcnt to zero */
	while (MAC_GF(ETHSTAT_BUFCNT, mac_readl(bp, ETHSTAT)))
		mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_BUFCDEC));

	/* Clear any interrupt flags */
	mac_writel(bp, PIC32_CLR(ETHIRQ), -1);

	/* Clear TX/RX start addresses */
	mac_writel(bp, PIC32_CLR(ETHTXST), -1);
	mac_writel(bp, PIC32_CLR(ETHRXST), -1);

	pic32ec_reset_mac(bp);
}

static void pic32ec_init_hw(struct pic32ec *bp)
{
	pic32ec_reset_hw(bp);
	pic32ec_set_hwaddr(bp);

	pic32ec_set_rx_mode(bp->dev);

	mac_writel(bp, EMAC1CFG2, MAC_BIT(EMAC1CFG2_EXCESSDFR) |
		   MAC_BIT(EMAC1CFG2_AUTOPAD) |
		   MAC_BIT(EMAC1CFG2_PADENABLE) |
		   MAC_BIT(EMAC1CFG2_CRCENABLE) |
		   MAC_BIT(EMAC1CFG2_NOBKOFF) | MAC_BIT(EMAC1CFG2_LENGTHCK));

	mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_AUTOFC));

	/* automatic flow control to kick in when we only have enough
	 * space for 2 packets and release at half that
	 */

#define WATERMARK (((MAC_RX_BUFFER_SIZE * RX_RING_SIZE) - 0xC00) / RX_RING_SIZE)

	mac_writel(bp, ETHRXWM, WATERMARK << 16 | (WATERMARK / 2));

	/* back to back intergap at half duplex */
	mac_writel(bp, EMAC1IPGT, 0x12);

	/* non back to back intergap */
	mac_writel(bp, EMAC1IPGR, 0xc12);

	/* recommended collision window retry limit */
	mac_writel(bp, EMAC1CLRT, 0x370f);

	bp->speed = SPEED_10;
	bp->duplex = DUPLEX_HALF;

	/* Initialize TX and RX buffers */
	mac_writel(bp, ETHCON2,
		   MAC_BF(ETHCON2_RXBUFSZ, MAC_RX_BUFFER_SIZE / 16));
	mac_writel(bp, ETHRXST, bp->rx_ring_dma);
	mac_writel(bp, ETHTXST, bp->tx_ring_dma);

	/* Enable TX and RX */
	mac_writel(bp, PIC32_SET(ETHCON1), MAC_BIT(ETHCON1_ON) |
		   MAC_BIT(ETHCON1_RXEN));

	mac_writel(bp, EMAC1CFG1, MAC_BIT(EMAC1CFG1_TXPAUSE) |
		   MAC_BIT(EMAC1CFG1_RXPAUSE) | MAC_BIT(EMAC1CFG1_RXENABLE));

	mac_writel(bp, PIC32_CLR(ETHIRQ), -1);

	/* Enable interrupts */
	mac_writel(bp, ETHIEN, MAC_RX_EN_FLAGS | MAC_TX_EN_FLAGS);
}

static void pic32ec_set_rx_mode(struct net_device *dev)
{
	u32 config;
	struct pic32ec *bp = netdev_priv(dev);

	/* Filters are applied by priority. The first one that matches makes
	 * all lower priority filters have no effect.
	 *
	 * ETHRXFC_CRCERREN - Accept invalid CRC
	 * ETHRXFC_RUNTERREN - Accept runt packets
	 * ETHRXFC_CRCOKEN - Reject invalid CRC
	 * ETHRXFC_RUNTEN - Reject runt packets
	 * ETHRXFC_UCEN - Accept unicast to us
	 * ETHRXFC_NOTMEEN - Accept not me unicast
	 * ETHRXFC_MCEN - Accept multicast
	 * ETHRXFC_BCEN - Accept broadcast
	 */

	if (bp->dev->flags & IFF_PROMISC) {
		config = MAC_BIT(ETHRXFC_UCEN);
		config |= MAC_BIT(ETHRXFC_NOTMEEN);
		config |= MAC_BIT(ETHRXFC_MCEN);
		config |= MAC_BIT(ETHRXFC_BCEN);
	} else {
		config = MAC_BIT(ETHRXFC_CRCOKEN);
		config |= MAC_BIT(ETHRXFC_NOTMEEN);
		config |= MAC_BIT(ETHRXFC_RUNTEN);
		config |= MAC_BIT(ETHRXFC_UCEN);

		if (dev->flags & IFF_ALLMULTI || !netdev_mc_empty(dev)) {
			/* Enable all multicast mode */
			config |= MAC_BIT(ETHRXFC_MCEN);
		}
	}

	if (bp->dev->flags & IFF_BROADCAST)
		config |= MAC_BIT(ETHRXFC_BCEN);

	mac_writel(bp, ETHRXFC, config);
}

static int pic32ec_open(struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);
	int err;

	/* carrier starts down */
	netif_carrier_off(dev);

	/* if the phy is not yet registered, retry later */
	if (!bp->phy_dev)
		return -EAGAIN;

	/* RX buffers initialization */
	pic32ec_init_rx_buffer_size(bp);

	err = pic32ec_alloc_rings(bp);
	if (err) {
		netdev_err(dev, "Unable to allocate DMA memory (error %d)\n",
			   err);
		return err;
	}

	napi_enable(&bp->napi);

	pic32ec_init_rings(bp);
	pic32ec_init_hw(bp);

	/* schedule a link state check */
	phy_start(bp->phy_dev);

	netif_start_queue(dev);

	return 0;
}

static int pic32ec_close(struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);
	unsigned long flags;

	netif_stop_queue(dev);
	napi_disable(&bp->napi);

	if (bp->phy_dev)
		phy_stop(bp->phy_dev);

	spin_lock_irqsave(&bp->lock, flags);
	pic32ec_reset_hw(bp);
	netif_carrier_off(dev);
	spin_unlock_irqrestore(&bp->lock, flags);

	pic32ec_free_consistent(bp);

	return 0;
}

static struct net_device_stats *pic32ec_get_stats(struct net_device *dev)
{
	struct pic32ec *bp = netdev_priv(dev);
	struct net_device_stats *nstat = &bp->stats;
	struct pic32ec_stats *hwstat = &bp->hw_stats;

	/* read stats from hardware */
	pic32ec_update_stats(bp);

	/* Convert HW stats into netdevice stats */
	nstat->rx_errors = (hwstat->rx_fcs_errors +
			    hwstat->rx_align_errors + hwstat->rx_overruns);
	nstat->tx_errors = 0;
	nstat->collisions = (hwstat->tx_single_cols + hwstat->tx_multiple_cols);
	nstat->rx_length_errors = 0;

	nstat->rx_over_errors = hwstat->rx_overruns;
	nstat->rx_crc_errors = hwstat->rx_fcs_errors;
	nstat->rx_frame_errors = hwstat->rx_align_errors;
	nstat->rx_fifo_errors = hwstat->rx_overruns;
	nstat->tx_aborted_errors = 0;
	nstat->tx_carrier_errors = 0;
	nstat->tx_fifo_errors = 0;

	return nstat;
}

static int pic32ec_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct pic32ec *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_gset(phydev, cmd);
}

static int pic32ec_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct pic32ec *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_sset(phydev, cmd);
}

static int pic32ec_get_regs_len(struct net_device *netdev)
{
	return 9 * sizeof(u32);
}

static void pic32ec_get_regs(struct net_device *dev,
			     struct ethtool_regs *regs, void *p)
{
	struct pic32ec *bp = netdev_priv(dev);
	unsigned int tail, head;
	u32 *regs_buff = p;

	regs->version = 0;

	tail = pic32ec_tx_ring_wrap(bp->tx_tail);
	head = pic32ec_tx_ring_wrap(bp->tx_head);

	regs_buff[0] = mac_readl(bp, ETHCON1);
	regs_buff[1] = mac_readl(bp, ETHCON2);
	regs_buff[2] = mac_readl(bp, ETHRXFC);
	regs_buff[3] = mac_readl(bp, EMAC1CFG1);
	regs_buff[4] = mac_readl(bp, EMAC1CFG2);

	regs_buff[5] = tail;
	regs_buff[6] = head;
	regs_buff[7] = pic32ec_tx_dma(bp, tail);
	regs_buff[8] = pic32ec_tx_dma(bp, head);
}

static const struct ethtool_ops pic32ec_ethtool_ops = {
	.get_settings		= pic32ec_get_settings,
	.set_settings		= pic32ec_set_settings,
	.get_regs_len		= pic32ec_get_regs_len,
	.get_regs		= pic32ec_get_regs,
	.get_link		= ethtool_op_get_link,
	.get_ts_info		= ethtool_op_get_ts_info,
};

static int pic32ec_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct pic32ec *bp = netdev_priv(dev);
	struct phy_device *phydev = bp->phy_dev;

	if (!netif_running(dev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	return phy_mii_ioctl(phydev, rq, cmd);
}

static const struct net_device_ops pic32ec_netdev_ops = {
	.ndo_open		= pic32ec_open,
	.ndo_stop		= pic32ec_close,
	.ndo_start_xmit		= pic32ec_start_xmit,
	.ndo_set_rx_mode	= pic32ec_set_rx_mode,
	.ndo_get_stats		= pic32ec_get_stats,
	.ndo_do_ioctl		= pic32ec_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_set_mac_address	= eth_mac_addr,
};

static struct platform_device_id pic32ec_devtype[] = {
	{
		.name = "pic32-ec",
		.driver_data = EC_QUIRK_USE_SRAM,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(platform, ec_devtype);

#if defined(CONFIG_OF)
static const struct of_device_id pic32ec_dt_ids[] = {
	{ .compatible = "microchip,pic32-ec", .data = &pic32ec_devtype[0], },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, pic32ec_dt_ids);
#endif

static int __init pic32ec_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pic32ec_platform_data *pdata;
	struct resource *regs, *mem;
	struct net_device *dev;
	struct pic32ec *bp;
	struct phy_device *phydev;
	const struct of_device_id *of_id;
	int err = -ENXIO;
	const char *mac;
	int gpio;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "no mmio resource defined\n");
		goto err_out;
	}

	dev = alloc_etherdev(sizeof(*bp));
	if (!dev) {
		err = -ENOMEM;
		goto err_out;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->features |= NETIF_F_RXCSUM;	/* RX checksum offload */

	bp = netdev_priv(dev);
	bp->pdev = pdev;
	bp->dev = dev;

	of_id = of_match_device(pic32ec_dt_ids, &pdev->dev);
	if (of_id)
		pdev->id_entry = of_id->data;
	bp->quirks = pdev->id_entry->driver_data;

	if (bp->quirks & EC_QUIRK_USE_SRAM) {
		mem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mem");
		if (!mem) {
			dev_err(&pdev->dev, "no coherent mem resource ?\n");
			err = -ENOMEM;
			goto err_out;
		}

		err = dma_declare_coherent_memory(&pdev->dev,
						  (dma_addr_t)mem->start,
						  (dma_addr_t)mem->start,
						  resource_size(mem),
						  DMA_MEMORY_MAP |
						  DMA_MEMORY_EXCLUSIVE);
		if (!err) {
			dev_err(&pdev->dev, "Failed declare_coherent_memory\n"
				"for ethernet controller device\n");
			goto err_out;
		}
	}

	spin_lock_init(&bp->lock);
	INIT_WORK(&bp->tx_error_task, pic32ec_tx_error_task);

	bp->pclk = devm_clk_get(&pdev->dev, "eth_clk");
	if (IS_ERR(bp->pclk)) {
		err = PTR_ERR(bp->pclk);
		dev_err(&pdev->dev, "failed to get eth_clk (%u)\n", err);
		goto err_out_free_dev;
	}

	err = clk_prepare_enable(bp->pclk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable eth_clk (%u)\n", err);
		goto err_out_free_dev;
	}

	bp->regs = devm_ioremap(&pdev->dev, regs->start, resource_size(regs));
	if (!bp->regs) {
		dev_err(&pdev->dev, "failed to map registers, aborting.\n");
		err = -ENOMEM;
		goto err_out_disable_clocks;
	}

	dev->irq = platform_get_irq(pdev, 0);
	err = devm_request_irq(&pdev->dev, dev->irq, pic32ec_interrupt, 0,
			       dev->name, dev);
	if (err) {
		dev_err(&pdev->dev, "Unable to request IRQ %d (error %d)\n",
			dev->irq, err);
		goto err_out_disable_clocks;
	}

	dev->netdev_ops = &pic32ec_netdev_ops;

	netif_napi_add(dev, &bp->napi, pic32ec_poll, 64);

	dev->ethtool_ops = &pic32ec_ethtool_ops;
	dev->base_addr = regs->start;

	mac = of_get_mac_address(np);
	if (mac)
		ether_addr_copy(bp->dev->dev_addr, mac);
	else
		pic32ec_get_hwaddr(bp);

	err = of_get_phy_mode(np);
	if (err < 0) {
		pdata = dev_get_platdata(&pdev->dev);
		if (pdata && pdata->is_rmii)
			bp->phy_interface = PHY_INTERFACE_MODE_RMII;
		else
			bp->phy_interface = PHY_INTERFACE_MODE_MII;
	} else {
		bp->phy_interface = err;
	}

	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "Cannot register net device, aborting.\n");
		goto err_out_disable_clocks;
	}

	gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (gpio_is_valid(gpio)) {
		err = devm_gpio_request_one(&pdev->dev, gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "pic32ec reset");
		if (err < 0)
			goto err_out_unregister_netdev;

		gpio_set_value(gpio, 0);
		usleep_range(1250, 1500);
		gpio_set_value(gpio, 1);
		usleep_range(1250, 1500);

		dev_warn(&pdev->dev, "reset-gpio asserted.\n");
	}

	err = pic32ec_mii_init(bp);
	if (err)
		goto err_out_unregister_netdev;

	platform_set_drvdata(pdev, dev);

	netif_carrier_off(dev);

	netdev_info(dev, "device at 0x%08lx irq %d (%pM)\n",
		    dev->base_addr, dev->irq, dev->dev_addr);

	phydev = bp->phy_dev;
	netdev_info(dev,
		    "attached PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		    phydev->drv->name, dev_name(&phydev->dev), phydev->irq);

	return 0;

err_out_unregister_netdev:
	unregister_netdev(dev);
err_out_disable_clocks:
	clk_disable_unprepare(bp->pclk);
err_out_free_dev:
	free_netdev(dev);
err_out:
	return err;
}

static int __exit pic32ec_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct pic32ec *bp;

	dev = platform_get_drvdata(pdev);

	if (dev) {
		bp = netdev_priv(dev);
		if (bp->phy_dev)
			phy_disconnect(bp->phy_dev);
		mdiobus_unregister(bp->mii_bus);
		kfree(bp->mii_bus->irq);
		mdiobus_free(bp->mii_bus);
		unregister_netdev(dev);
		clk_disable_unprepare(bp->pclk);
		free_netdev(dev);
	}

	return 0;
}

static struct platform_driver pic32ec_driver = {
	.remove = __exit_p(pic32ec_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pic32ec_dt_ids),
	},
};

module_platform_driver_probe(pic32ec_driver, pic32ec_probe);

MODULE_DESCRIPTION("Microchip PIC32 Ethernet Controller Driver");
MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
