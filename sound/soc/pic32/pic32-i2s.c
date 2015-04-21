/*
 *  Joshua Henderson <joshua.henderson@microchip.com>
 *  Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>

#include "pic32-i2s.h"

#define DRIVER_NAME "pic32-i2s"

struct pic32_i2s {
	struct resource *mem;
	void __iomem *base;
	dma_addr_t phys_base;

	struct clk *clk_i2s;
	struct clk *clk_refclk;

	struct snd_dmaengine_dai_dma_data playback_dma_data;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
};

static inline uint32_t pic32_i2s_read(const struct pic32_i2s *i2s,
	unsigned int reg)
{
	return readl(i2s->base + reg);
}

static inline void pic32_i2s_write(const struct pic32_i2s *i2s,
	unsigned int reg, uint32_t value)
{
	writel(value, i2s->base + reg);
}

static int pic32_i2s_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (dai->active)
		return 0;

	pic32_i2s_write(i2s, PIC32_CLR(SPISTAT), -1);

	clk_prepare_enable(i2s->clk_refclk);

	return 0;
}

static void pic32_i2s_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (dai->active)
		return;

	pic32_i2s_write(i2s, PIC32_CLR(SPICON), SPICON_ON);

	clk_disable_unprepare(i2s->clk_refclk);
}

static int pic32_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *dai)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 trash;

	dev_dbg(dai->dev, "%s cmd:%d\n", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		trash = pic32_i2s_read(i2s, SPIBUF);
		pic32_i2s_write(i2s, SPIBUF, 0);
		pic32_i2s_write(i2s, PIC32_CLR(SPISTAT), -1);
		pic32_i2s_write(i2s, PIC32_SET(SPICON), SPICON_ON);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pic32_i2s_write(i2s, PIC32_CLR(SPICON), SPICON_ON);
		pic32_i2s_write(i2s, PIC32_CLR(SPISTAT), -1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int pic32_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 conf;
	u32 conf2;

	dev_dbg(dai->dev, "%s format:%d\n", __func__, fmt);

	conf = pic32_i2s_read(i2s, SPICON);
	conf &= ~(SPICON_MSTEN | SPICON_CKP | SPICON_FRMPOL |
		SPICON_SPIFE | SPICON_FRMSYNC);

	conf2 = pic32_i2s_read(i2s, SPICON2);
	conf2 &= ~SPICON2_AUDMOD_MASK;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		conf |= SPICON_MSTEN;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		conf |= SPICON_MSTEN;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		conf |= SPICON_FRMSYNC;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		conf2 |= SPICON2_AUDMOD_PCM;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		conf2 |= SPICON2_AUDMOD_RJ;
		conf |= SPICON_FRMPOL;
		conf |= SPICON_SPIFE;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		conf2 |= SPICON2_AUDMOD_LJ;
		conf |= SPICON_FRMPOL;
		conf |= SPICON_SPIFE;
		break;
	case SND_SOC_DAIFMT_I2S:
		conf2 |= SPICON2_AUDMOD_I2S;
		conf |= SPICON_CKP;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}

	pic32_i2s_write(i2s, SPICON, conf);
	pic32_i2s_write(i2s, SPICON2, conf2);

	return 0;
}

/*
 * brg = (clock_frequency / (2 * baudrate)) - 1
 */
static u32 pic32_calc_brg(u32 clock_frequency, u32 baudrate)
{
	u32 brg;
	u32 baudhigh;
	u32 baudlow;
	u32 errorhigh;
	u32 errorlow;

	brg = (((clock_frequency / baudrate) / 2) - 1);

	baudhigh = clock_frequency / (2 * (brg + 1));
	baudlow = clock_frequency / (2 * (brg + 2));
	errorhigh = baudhigh - baudrate;
	errorlow = baudrate - baudlow;

	if (errorhigh > errorlow)
		brg++;

	return brg;
}

static int pic32_i2s_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 frame_size;
	u32 conf;
	u32 width;

	dev_dbg(dai->dev, "%s format:0x%x channels:%d\n", __func__,
		params_format(params), params_channels(params));

	conf = pic32_i2s_read(i2s, SPICON);
	conf &= ~SPICON_MODE_MASK;

	/* handle sample format */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		conf |= SPICON_MODE_16B_16;
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		frame_size = 32;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		conf |= SPICON_MODE_24B_32;
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		frame_size = 64;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		conf |= SPICON_MODE_32B_32;
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		frame_size = 64;
		break;
	default:
		dev_warn(dai->dev, "PCM format not supported\n");
		return -EINVAL;
	}

	i2s->playback_dma_data.addr_width = width;
	i2s->playback_dma_data.maxburst = 16 / width / 2;
	i2s->capture_dma_data.addr_width = width;
	i2s->capture_dma_data.maxburst = 16 / width / 2;

	pic32_i2s_write(i2s, SPICON, conf);

	/* handle channel params */
	switch (params_channels(params)) {
	case 1:
		pic32_i2s_write(i2s, PIC32_SET(SPICON2), SPICON2_AUDMONO);
		break;
	case 2:
		pic32_i2s_write(i2s, PIC32_CLR(SPICON2), SPICON2_AUDMONO);
		break;
	default:
		dev_warn(dai->dev, "channel not supported\n");
		return -EINVAL;
	}

	pic32_i2s_write(i2s, SPIBRG,
			pic32_calc_brg(clk_get_rate(i2s->clk_i2s),
				params_rate(params) * frame_size));

	return 0;
}

static int pic32_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	switch (clk_id) {
	case PIC32_I2S_REFCLK:

		if (dir == SND_SOC_CLOCK_OUT) {
			dev_dbg(dai->dev, "setting refclk rate %d\n", freq);

			clk_set_rate(i2s->clk_refclk, freq);
			if (freq != clk_get_rate(i2s->clk_refclk)) {
				dev_dbg(dai->dev,
					"failed to set refclk actual freq %ld to intended freq %d\n",
					clk_get_rate(i2s->clk_refclk), freq);
			}
		}
		break;
	}

	return 0;
}

static void pic32_i2s_init_pcm_config(struct pic32_i2s *i2s)
{
	struct snd_dmaengine_dai_dma_data *dma_data;

	/* Playback */
	dma_data = &i2s->playback_dma_data;
	dma_data->addr = i2s->phys_base + SPIBUF;

	/* Capture */
	dma_data = &i2s->capture_dma_data;
	dma_data->addr = i2s->phys_base + SPIBUF;
}

static int pic32_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct pic32_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	u32 conf, conf2;

	pic32_i2s_init_pcm_config(i2s);
	snd_soc_dai_init_dma_data(dai, &i2s->playback_dma_data,
		&i2s->capture_dma_data);

	/* disable peripheral */
	pic32_i2s_write(i2s, PIC32_CLR(SPICON), SPICON_ON);

	/* clear status register */
	pic32_i2s_write(i2s, PIC32_CLR(SPISTAT), -1);

	conf = SPICON_FRMEN |
		SPICON_FRMCNT_2BIT |
		SPICON_ENHBUF |
		SPICON_STXISEL_HALF_EMPTY |
		SPICON_SRXISEL_NOT_EMPTY;

	pic32_i2s_write(i2s, SPICON, conf);

	conf2 = SPICON2_AUDEN |
		SPICON2_IGNROV |
		SPICON2_IGNTUR;

	pic32_i2s_write(i2s, SPICON2, conf2);

	return 0;
}

static int pic32_i2s_dai_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static const struct snd_soc_dai_ops pic32_i2s_dai_ops = {
	.startup = pic32_i2s_startup,
	.shutdown = pic32_i2s_shutdown,
	.trigger = pic32_i2s_trigger,
	.hw_params = pic32_i2s_hw_params,
	.set_fmt = pic32_i2s_set_fmt,
	.set_sysclk = pic32_i2s_set_sysclk,
};

#define PIC32_I2S_RATES (SNDRV_PCM_RATE_16000 |		\
				SNDRV_PCM_RATE_22050 |	\
				SNDRV_PCM_RATE_32000 |	\
				SNDRV_PCM_RATE_44100 |	\
				SNDRV_PCM_RATE_48000 |	\
				SNDRV_PCM_RATE_96000)

#define PIC32_I2S_FMTS (SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver pic32_i2s_dai = {
	.probe = pic32_i2s_dai_probe,
	.remove = pic32_i2s_dai_remove,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = PIC32_I2S_RATES,
		.formats = PIC32_I2S_FMTS,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = PIC32_I2S_RATES,
		.formats = PIC32_I2S_FMTS,
	},
	.symmetric_rates = 1,
	.symmetric_channels = 1,
	.symmetric_samplebits = 1,
	.ops = &pic32_i2s_dai_ops,
};

static const struct snd_soc_component_driver pic32_i2s_component = {
	.name		= DRIVER_NAME,
};

static int pic32_i2s_dev_probe(struct platform_device *pdev)
{
	struct pic32_i2s *i2s;
	struct resource *mem;
	int ret;
	int irq_rx, irq_tx;

	irq_rx = platform_get_irq(pdev, 0);
	if (irq_rx < 0) {
		dev_err(&pdev->dev, "rx irq not found\n");
		return irq_rx;
	}

	irq_tx = platform_get_irq(pdev, 1);
	if (irq_tx < 0) {
		dev_err(&pdev->dev, "tx irq not found\n");
		return irq_tx;
	}

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->capture_dma_data.slave_id = irq_rx;
	i2s->playback_dma_data.slave_id = irq_tx;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(i2s->base))
		return PTR_ERR(i2s->base);

	i2s->phys_base = mem->start;
	i2s->clk_i2s = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2s->clk_i2s)) {
		dev_err(&pdev->dev, "i2s clock not found.\n");
		return PTR_ERR(i2s->clk_i2s);
	}

	i2s->clk_refclk = devm_clk_get(&pdev->dev, "mck1");
	if (IS_ERR(i2s->clk_refclk)) {
		dev_err(&pdev->dev, "i2s refclk not found.\n");
		return PTR_ERR(i2s->clk_refclk);
	}

	platform_set_drvdata(pdev, i2s);

	ret = devm_snd_soc_register_component(&pdev->dev,
		&pic32_i2s_component, &pic32_i2s_dai, 1);
	if (ret)
		return ret;

	return devm_snd_dmaengine_pcm_register(&pdev->dev, NULL,
					SND_DMAENGINE_PCM_FLAG_COMPAT);
}

static const struct of_device_id pic32_i2s_of_match[] = {
	{.compatible = "microchip,pic32-i2s", },
	{ },
};
MODULE_DEVICE_TABLE(of, pic32_i2s_of_match);

static struct platform_driver pic32_i2s_driver = {
	.probe = pic32_i2s_dev_probe,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = pic32_i2s_of_match,
	},
};

module_platform_driver(pic32_i2s_driver);

MODULE_DESCRIPTION("PIC32 SoC I2S driver");
MODULE_AUTHOR("Joshua Henderson, <joshua.henderson@microchip.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
