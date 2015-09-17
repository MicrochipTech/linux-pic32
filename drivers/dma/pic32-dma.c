/*
 * PIC32 DMA controller driver
 *
 * Joshua Henderson, <joshua.henderson@microchip.com>
 * Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_dma.h>

#include <asm/mach-pic32/pic32.h>

#include "virt-dma.h"

#define DRV_NAME "pic32-dma"

#define PIC32_DMA_MAX_CHANS	8

#define PIC32_DMACON		0x00
#define PIC32_DMACON_ON		BIT(15)
#define PIC32_DMACON_SUSPEND	BIT(12)
#define PIC32_DMACON_DMABUSY	BIT(11)
#define PIC32_DMASTAT		0x10
#define PIC32_DMASTAT_RDWR	BIT(31)
#define PIC32_DMAADDR		0x20
#define PIC32_DCHCON		0x00
#define PIC32_DCHCON_CHBUSY	BIT(15)
#define PIC32_DCHCON_CHEN	BIT(7)
#define PIC32_DCHCON_CHCHN	BIT(5)
#define PIC32_DCHCON_CHAEN	BIT(4)
#define PIC32_DCHCON_CHEDET	BIT(2)
#define PIC32_DCHECON		0x10
#define PIC32_DCHECON_CHSIRQ_MASK (0xFF << 8)
#define PIC32_DCHECON_CFORCE	BIT(7)
#define PIC32_DCHECON_CABORT	BIT(6)
#define PIC32_DCHECON_SIRQEN	BIT(4)
#define PIC32_DCHINT		0x20
#define PIC32_DCHINT_CHSDIE	BIT(23)
#define PIC32_DCHINT_CHSHIE	BIT(22)
#define PIC32_DCHINT_CHDDIE	BIT(21)
#define PIC32_DCHINT_CHDHIE	BIT(20)
#define PIC32_DCHINT_CHBCIE	BIT(19)
#define PIC32_DCHINT_CHCCIE	BIT(18)
#define PIC32_DCHINT_CHTAIE	BIT(17)
#define PIC32_DCHINT_CHERIE	BIT(16)
#define PIC32_DCHINT_CHSDIF	BIT(7)
#define PIC32_DCHINT_CHSHIF	BIT(6)
#define PIC32_DCHINT_CHDDIF	BIT(5)
#define PIC32_DCHINT_CHDHIF	BIT(4)
#define PIC32_DCHINT_CHBCIF	BIT(3)
#define PIC32_DCHINT_CHCCIF	BIT(2)
#define PIC32_DCHINT_CHTAIF	BIT(1)
#define PIC32_DCHINT_CHERIF	BIT(0)
#define PIC32_DCHSSA		0x30
#define PIC32_DCHDSA		0x40
#define PIC32_DCHSSIZ		0x50
#define PIC32_DCHDSIZ		0x60
#define PIC32_DCHSPTR		0x70
#define PIC32_DCHDPTR		0x80
#define PIC32_DCHCSIZ		0x90
#define PIC32_DCHDAT		0xB0

#define PIC32_IRQ_INT_MASK	(PIC32_DCHINT_CHBCIE |	\
				PIC32_DCHINT_CHTAIE |	\
				PIC32_DCHINT_CHERIE)

#define PIC32_IRQ_FLAG_MASK	(PIC32_DCHINT_CHSDIF |	\
				PIC32_DCHINT_CHSHIF |	\
				PIC32_DCHINT_CHDDIF |	\
				PIC32_DCHINT_CHDHIF |	\
				PIC32_DCHINT_CHBCIF |	\
				PIC32_DCHINT_CHCCIF |	\
				PIC32_DCHINT_CHTAIF |	\
				PIC32_DCHINT_CHERIF)

#define PIC32_MAX_TRANSFER_SIZE	65535

struct pic32_dma_sg {
	dma_addr_t addr;
	dma_addr_t addr_dest; /* Only used by DMA_MEM_TO_MEM */
	unsigned int len;
};

struct pic32_desc {
	struct virt_dma_desc vdesc;
	enum dma_transfer_direction dir;
	uint32_t num_sgs;
	struct pic32_dma_sg sg[];
};

struct pic32_chan {
	struct virt_dma_chan vchan;
	int id;
	void __iomem *chan_base;
	int irq;
	struct pic32_desc *desc;
	unsigned int next_sg;
	bool cyclic;
	bool paused;
	struct dma_slave_config cfg;
};

struct pic32_dma_dev {
	struct dma_device ddev;
	void __iomem *base;
	struct pic32_chan chan[PIC32_DMA_MAX_CHANS];
};

static inline struct pic32_dma_dev *to_pic32_dma_dev(struct dma_device *d)
{
	return container_of(d, struct pic32_dma_dev, ddev);
}

static inline struct pic32_chan *to_pic32_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct pic32_chan, vchan.chan);
}

static inline struct pic32_desc *to_pic32_dma_desc(
	struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct pic32_desc, vdesc);
}

static inline u32 pic32_chan_readl(struct pic32_chan *chan, unsigned int reg)
{
	return __raw_readl(chan->chan_base + reg);
}

static inline void pic32_chan_writel(struct pic32_chan *chan,
				unsigned int reg, uint32_t val)
{
	__raw_writel(val, chan->chan_base + reg);
}

static void pic32_dma_desc_free(struct virt_dma_desc *vdesc)
{
	struct pic32_desc *desc = container_of(vdesc, struct pic32_desc, vdesc);

	kfree(desc);
}

static struct pic32_desc *pic32_dma_alloc_desc(unsigned int num_sgs)
{
	return kzalloc(sizeof(struct pic32_desc) +
		sizeof(struct pic32_dma_sg) * num_sgs, GFP_ATOMIC);
}

static int pic32_dma_start_desc(struct pic32_chan *chan)
{
	struct virt_dma_desc *vdesc;
	struct pic32_dma_sg *sg;
	u32 conf;

	if (!chan->desc) {
		vdesc = vchan_next_desc(&chan->vchan);
		if (!vdesc)
			return 0;
		chan->desc = to_pic32_dma_desc(vdesc);
		chan->next_sg = 0;
	} else if (chan->next_sg == chan->desc->num_sgs)
		chan->next_sg = 0;

	sg = &chan->desc->sg[chan->next_sg];

	switch (chan->desc->dir) {
	case DMA_MEM_TO_DEV:
		pic32_chan_writel(chan, PIC32_DCHSSA, sg->addr);
		pic32_chan_writel(chan, PIC32_DCHDSA, chan->cfg.dst_addr);

		/* Set src length */
		pic32_chan_writel(chan, PIC32_DCHSSIZ, sg->len);

		/* dst length and cell size set to the same value */
		pic32_chan_writel(chan, PIC32_DCHDSIZ,
				chan->cfg.dst_addr_width);
		pic32_chan_writel(chan, PIC32_DCHCSIZ,
				chan->cfg.dst_addr_width);

		break;
	case DMA_DEV_TO_MEM:
		pic32_chan_writel(chan, PIC32_DCHSSA, chan->cfg.src_addr);
		pic32_chan_writel(chan, PIC32_DCHDSA, sg->addr);

		/* Set dst length */
		pic32_chan_writel(chan, PIC32_DCHDSIZ, sg->len);

		/* src length and cell size set to the same value */
		pic32_chan_writel(chan, PIC32_DCHSSIZ,
				chan->cfg.src_addr_width);
		pic32_chan_writel(chan, PIC32_DCHCSIZ,
				chan->cfg.src_addr_width);

		break;
	case DMA_MEM_TO_MEM:
		pic32_chan_writel(chan, PIC32_DCHSSA, sg->addr);
		pic32_chan_writel(chan, PIC32_DCHDSA, sg->addr_dest);

		/* Set src/dst/cell length */
		pic32_chan_writel(chan, PIC32_DCHSSIZ, sg->len);
		pic32_chan_writel(chan, PIC32_DCHDSIZ, sg->len);
		pic32_chan_writel(chan, PIC32_DCHCSIZ, sg->len);
		break;
	default:
		dev_err(chan->vchan.chan.device->dev,
			"%s channel %d: invalid case %d\n",
			__func__, chan->id, chan->desc->dir);
		return -EINVAL;
	}

	chan->next_sg++;

	/* Set and enable start transfer IRQ */
	if (chan->cfg.direction != DMA_MEM_TO_MEM) {
		conf = (chan->cfg.slave_id << 8) | PIC32_DCHECON_SIRQEN;
		pic32_chan_writel(chan, PIC32_DCHECON, conf);

		/* Enable interrupts */
		pic32_chan_writel(chan, PIC32_DCHINT, PIC32_IRQ_INT_MASK);

		/* Enable the channel */
		pic32_chan_writel(chan, PIC32_SET(PIC32_DCHCON),
				PIC32_DCHCON_CHEN);
	} else {
		pic32_chan_writel(chan, PIC32_DCHECON, 0);

		/* Enable interrupts */
		pic32_chan_writel(chan, PIC32_DCHINT, PIC32_IRQ_INT_MASK);

		/* Enable the channel */
		pic32_chan_writel(chan, PIC32_SET(PIC32_DCHCON),
				PIC32_DCHCON_CHEN);

		/* Force start the transfer if not in cyclic mode. */
		pic32_chan_writel(chan, PIC32_DCHECON, PIC32_DCHECON_CFORCE);
	}

	return 0;
}

static irqreturn_t pic32_dma_isr(int irq, void *data)
{
	struct pic32_chan *chan = data;
	u32 status;

	status = pic32_chan_readl(chan, PIC32_DCHINT) & PIC32_IRQ_FLAG_MASK;

	pic32_chan_writel(chan, PIC32_CLR(PIC32_DCHINT), PIC32_IRQ_FLAG_MASK);

	spin_lock(&chan->vchan.lock);

	if (unlikely(status & PIC32_DCHINT_CHTAIF))
		dev_err(chan->vchan.chan.device->dev, "DMA transfer aborted\n");

	if (unlikely(status & PIC32_DCHINT_CHERIF))
		dev_err(chan->vchan.chan.device->dev, "DMA address error\n");

	if (likely(status & PIC32_DCHINT_CHBCIF)) {
		dev_vdbg(chan->vchan.chan.device->dev,
			"DMA block transfer complete\n");

		if (chan->desc) {
			if (chan->cyclic) {
				vchan_cyclic_callback(&chan->desc->vdesc);
			} else if (chan->next_sg == chan->desc->num_sgs) {
				list_del(&chan->desc->vdesc.node);
				vchan_cookie_complete(&chan->desc->vdesc);
				chan->desc = NULL;
			}
		}
	}

	pic32_dma_start_desc(chan);

	spin_unlock(&chan->vchan.lock);

	return IRQ_HANDLED;
}

static int pic32_dma_alloc_chan_resources(struct dma_chan *dchan)
{
	return 0;
}

static void pic32_dma_free_chan_resources(struct dma_chan *dchan)
{
	vchan_free_chan_resources(to_virt_chan(dchan));
}

static size_t pic32_dma_desc_residue(struct pic32_chan *chan,
	struct pic32_desc *desc, unsigned int next_sg)
{
	unsigned int residue;
	unsigned int i;

	residue = 0;

	for (i = next_sg; i < desc->num_sgs; i++)
		residue += desc->sg[i].len;

	if (next_sg != 0) {
		if (desc->dir == DMA_DEV_TO_MEM)
			residue += (desc->sg[next_sg-1].len -
				pic32_chan_readl(chan, PIC32_DCHDPTR));
		else
			residue += (desc->sg[next_sg-1].len -
				pic32_chan_readl(chan, PIC32_DCHSPTR));
	}

	return residue;
}

static enum dma_status pic32_dma_tx_status(struct dma_chan *dchan,
	dma_cookie_t cookie, struct dma_tx_state *state)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	struct virt_dma_desc *vdesc;
	enum dma_status status;
	unsigned long flags;

	status = dma_cookie_status(dchan, cookie, state);
	if (status == DMA_COMPLETE || !state)
		return status;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	vdesc = vchan_find_desc(&chan->vchan, cookie);
	if (cookie == chan->desc->vdesc.tx.cookie) {
		state->residue = pic32_dma_desc_residue(chan, chan->desc,
				chan->next_sg);
	} else if (vdesc) {
		state->residue = pic32_dma_desc_residue(chan,
				to_pic32_dma_desc(vdesc), 0);
	} else {
		state->residue = 0;
	}
	spin_unlock_irqrestore(&chan->vchan.lock, flags);

	return status;
}

static void pic32_dma_issue_pending(struct dma_chan *dchan)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->vchan.lock, flags);
	if (vchan_issue_pending(&chan->vchan) && !chan->desc)
		pic32_dma_start_desc(chan);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
}

static struct dma_async_tx_descriptor *pic32_prep_dma_memcpy(
	struct dma_chan *dchan, dma_addr_t dest,
	dma_addr_t src, size_t len, unsigned long flags)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	struct pic32_desc *desc;
	unsigned int num_periods, i;
	size_t block;

	dev_dbg(dchan->device->dev,
		"%s channel %d: src=%#llx dest=%#llx len=%zu\n",
		__func__, chan->id, (u64)src, (u64)dest, len);

	if (unlikely(!len)) {
		dev_warn(dchan->device->dev,
			"pic32_prep_dma_memcpy: length is zero!\n");
		return NULL;
	}

	num_periods = (len / PIC32_MAX_TRANSFER_SIZE);
	if (len % PIC32_MAX_TRANSFER_SIZE)
		num_periods++;

	desc = pic32_dma_alloc_desc(num_periods);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		block = (len > PIC32_MAX_TRANSFER_SIZE) ?
			PIC32_MAX_TRANSFER_SIZE : len;
		desc->sg[i].addr = src;
		desc->sg[i].addr_dest = dest;
		desc->sg[i].len = block;
		src += block;
		dest += block;
		len -= block;
	}

	desc->num_sgs = num_periods;
	desc->dir = DMA_MEM_TO_MEM;
	chan->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *pic32_dma_prep_slave_sg(
	struct dma_chan *dchan, struct scatterlist *sgl,
	unsigned int sg_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	struct pic32_desc *desc;
	struct scatterlist *sg;
	unsigned int i;

	dev_dbg(dchan->device->dev,
		"%s channel %d: sg_len=%zu\n",
		__func__, chan->id, sg_len);

	desc = pic32_dma_alloc_desc(sg_len);
	if (!desc)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		desc->sg[i].addr = sg_dma_address(sg);
		desc->sg[i].len = sg_dma_len(sg);
	}

	desc->num_sgs = sg_len;
	desc->dir = direction;
	chan->cyclic = false;

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *pic32_dma_prep_dma_cyclic(
	struct dma_chan *dchan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	struct pic32_desc *desc;
	unsigned int num_periods, i;
	dma_addr_t buf = buf_addr;

	if (!is_slave_direction(direction)) {
		dev_err(dchan->device->dev, "channel %d: bad direction\n",
			chan->id);
		return NULL;
	}

	if (period_len > PIC32_MAX_TRANSFER_SIZE) {
		dev_err(dchan->device->dev,
			"channel %d: max period size exceeded: %d > %d\n",
			chan->id, period_len, PIC32_MAX_TRANSFER_SIZE);
		return NULL;
	}

	if ((buf_len % period_len) != 0) {
		dev_err(dchan->device->dev,
			"channel %d: buf_len not multiple of period len\n",
			chan->id);
		return NULL;
	}

	num_periods = buf_len / period_len;

	desc = pic32_dma_alloc_desc(num_periods);
	if (!desc)
		return NULL;

	for (i = 0; i < num_periods; i++) {
		desc->sg[i].addr = buf;
		desc->sg[i].len = period_len;
		buf += period_len;
	}

	desc->num_sgs = num_periods;
	desc->dir = direction;
	chan->cyclic = true;

	dev_dbg(dchan->device->dev,
		"%s channel %d: buf_addr=%#llx buf_len=%zu period_len=%zu\n",
		__func__, chan->id, (u64)buf_addr, buf_len, period_len);

	return vchan_tx_prep(&chan->vchan, &desc->vdesc, flags);
}

static int pic32_dma_slave_config(struct dma_chan *dchan,
		struct dma_slave_config *config)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);

	memcpy(&chan->cfg, config, sizeof(chan->cfg));

	return 0;
}

static int pic32_dma_abort(struct pic32_chan *chan)
{
	unsigned long cs;
	long int timeout = 10000;

	dev_dbg(chan->vchan.chan.device->dev, "%s channel %d\n",
		__func__, chan->id);

	if (!chan->paused) {

		/* Disable interrupts */
		pic32_chan_writel(chan, PIC32_CLR(PIC32_DCHINT), -1);

		/* Stop the channel */
		pic32_chan_writel(chan, PIC32_CLR(PIC32_DCHCON),
				PIC32_DCHCON_CHEN);

		/* Wait for any current transfer to complete */
		cs = pic32_chan_readl(chan, PIC32_DCHCON);
		while ((cs & PIC32_DCHCON_CHBUSY) && --timeout) {
			cpu_relax();
			cs = pic32_chan_readl(chan, PIC32_DCHCON);
		}

		if (!timeout) {
			dev_err(chan->vchan.chan.device->dev,
				"%s failed to stop channel %d\n",
				__func__, chan->id);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int pic32_dma_terminate_all(struct dma_chan *dchan)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	unsigned long flags;
	int ret;
	LIST_HEAD(head);

	dev_dbg(chan->vchan.chan.device->dev, "%s\n", __func__);

	spin_lock_irqsave(&chan->vchan.lock, flags);
	ret = pic32_dma_abort(chan);
	if (ret) {
		spin_unlock_irqrestore(&chan->vchan.lock, flags);
		return ret;
	}
	chan->desc = NULL;

	if (chan->cyclic) {
		chan->cyclic = false;
		chan->paused = false;
	}

	vchan_get_all_descriptors(&chan->vchan, &head);
	spin_unlock_irqrestore(&chan->vchan.lock, flags);
	vchan_dma_desc_free_list(&chan->vchan, &head);

	return 0;
}

static int pic32_dma_pause(struct dma_chan *dchan)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);
	int ret = 0;

	/* Pause/Resume only allowed with cyclic mode */
	if (!chan->cyclic)
		return -EINVAL;

	if (!chan->paused) {
		ret = pic32_dma_abort(chan);
		if (!ret)
			chan->paused = true;
	}

	return ret;
}

static int pic32_dma_resume(struct dma_chan *dchan)
{
	struct pic32_chan *chan = to_pic32_dma_chan(dchan);

	/* Pause/Resume only allowed with cyclic mode */
	if (!chan->cyclic)
		return -EINVAL;

	if (chan->paused) {
		pic32_chan_writel(chan, PIC32_SET(PIC32_DCHCON),
				PIC32_DCHCON_CHEN);
		chan->paused = false;
	}

	return 0;
}

#define PIC32_DMA_CHAN(n)		((n) * 0xC0)
#define PIC32_DMA_CHANIO(base, n)	((base) + 0x60 + PIC32_DMA_CHAN(n))

static int pic32_dma_chan_init(struct pic32_dma_dev *od,
			struct pic32_chan *chan, int chan_id, int irq)
{
	int ret;

	chan->id = chan_id;
	chan->vchan.desc_free = pic32_dma_desc_free;

	chan->chan_base = PIC32_DMA_CHANIO(od->base, chan_id);
	chan->irq = irq;

	pic32_chan_writel(chan, PIC32_CLR(PIC32_DCHCON), -1);

	ret = devm_request_irq(od->ddev.dev, chan->irq,
			pic32_dma_isr, 0, DRV_NAME, chan);
	if (ret) {
		dev_warn(od->ddev.dev,
			"error requesting DMA channel %d IRQ %d\n",
			chan->id, chan->irq);
		goto err;
	}

	dev_dbg(od->ddev.dev, "Init PIC32 DMA channel %d\n", chan_id);

	return 0;

err:
	return ret;
}

static void pic32_dma_free(struct pic32_dma_dev *od)
{
	struct pic32_chan *chan, *next;

	list_for_each_entry_safe(chan, next, &od->ddev.channels,
				vchan.chan.device_node) {
		free_irq(chan->irq, chan);
		list_del(&chan->vchan.chan.device_node);
		tasklet_kill(&chan->vchan.task);
	}
}

static struct dma_chan *
pic32_dma_xlate_of(struct of_phandle_args *dma_spec, struct of_dma *ofdma)
{
	struct dma_device *dev = ofdma->of_dma_data;
	struct dma_chan *dchan, *match = NULL;
	struct pic32_chan *chan;

	if (!dev || dma_spec->args_count != 1)
		return NULL;

	list_for_each_entry(dchan, &dev->channels, device_node) {
		chan = to_pic32_dma_chan(dchan);
		if (chan->irq == dma_spec->args[0]) {
			match = dchan;
			break;
		}
	}

	if (!match)
		return NULL;

	pr_debug("chan: %s (%s)\n", dma_chan_name(match),
		match->client_count ? "busy" : "free");

	return dma_get_slave_channel(match);
}

#define PIC32_DMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |	\
				BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |	\
				BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

static const struct of_device_id pic32_dma_dt_ids[] = {
	{ .compatible = "microchip,pic32-dma" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pic32_dma_dt_ids);

static int pic32_dma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct pic32_chan *chan;
	struct pic32_dma_dev *dmadev;
	struct dma_device *dd;
	unsigned int i;
	struct resource *res;
	int ret;
	int irq;

	dmadev = devm_kzalloc(&pdev->dev, sizeof(*dmadev), GFP_KERNEL);
	if (!dmadev)
		return -EINVAL;

	dd = &dmadev->ddev;

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dma_set_max_seg_size(&pdev->dev, PIC32_MAX_TRANSFER_SIZE);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dmadev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dmadev->base))
		return PTR_ERR(dmadev->base);

	dma_cap_set(DMA_SLAVE, dd->cap_mask);
	dma_cap_set(DMA_CYCLIC, dd->cap_mask);
	dma_cap_set(DMA_MEMCPY, dd->cap_mask);
	dma_cap_set(DMA_PRIVATE, dd->cap_mask);
	dd->device_alloc_chan_resources = pic32_dma_alloc_chan_resources;
	dd->device_free_chan_resources = pic32_dma_free_chan_resources;
	dd->device_tx_status = pic32_dma_tx_status;
	dd->device_issue_pending = pic32_dma_issue_pending;
	dd->device_prep_slave_sg = pic32_dma_prep_slave_sg;
	dd->device_prep_dma_cyclic = pic32_dma_prep_dma_cyclic;
	dd->device_prep_dma_memcpy = pic32_prep_dma_memcpy;
	dd->device_config = pic32_dma_slave_config;
	dd->device_terminate_all = pic32_dma_terminate_all;
	dd->device_pause = pic32_dma_pause;
	dd->device_resume = pic32_dma_resume;
	dd->src_addr_widths = PIC32_DMA_BUSWIDTHS;
	dd->dst_addr_widths = PIC32_DMA_BUSWIDTHS;
	dd->directions = BIT(DMA_MEM_TO_MEM) |
		BIT(DMA_DEV_TO_MEM) |
		BIT(DMA_MEM_TO_DEV);
	dd->residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;


	dd->chancnt = 0;
	dd->dev = &pdev->dev;

	INIT_LIST_HEAD(&dd->channels);

	for (i = 0; i < PIC32_DMA_MAX_CHANS; i++) {
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			break;

		chan = &dmadev->chan[i];
		ret = pic32_dma_chan_init(dmadev, chan, i, irq);
		if (ret)
			goto err;

		dd->chancnt++;

		vchan_init(&chan->vchan, dd);
	}

	/* enable DMA engine */
	__raw_writel(PIC32_DMACON_ON, PIC32_SET(dmadev->base + PIC32_DMACON));

	ret = dma_async_device_register(dd);
	if (ret)
		return ret;

	if (np) {
		ret = of_dma_controller_register(np, pic32_dma_xlate_of, dd);
		if (ret)
			dev_err(&pdev->dev,
				"could not register of_dma_controller\n");
	}

	platform_set_drvdata(pdev, dmadev);

	return 0;

err:
	return ret;
}

static int pic32_dma_remove(struct platform_device *pdev)
{
	struct pic32_dma_dev *od = platform_get_drvdata(pdev);

	pic32_dma_free(od);
	dma_async_device_unregister(&od->ddev);
	return 0;
}

static struct platform_driver pic32_dma_driver = {
	.probe	= pic32_dma_probe,
	.remove	= pic32_dma_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pic32_dma_dt_ids),
	},
};

module_platform_driver(pic32_dma_driver);

MODULE_DESCRIPTION("Microchip PIC32 DMA controller driver");
MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
