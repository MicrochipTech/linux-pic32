/*
 *  ASoC Driver for PIC32 connected to AK4953A
 *
 *  Joshua Henderson, <joshua.henderson@microchip.com>
 *  Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "pic32-i2s.h"

static int snd_pic32_ak4953a_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int bfs, rfs, ret;
	unsigned long rclk;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U24:
	case SNDRV_PCM_FORMAT_S24:
		bfs = 48;
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		if (bfs == 48)
			rfs = 384;
		else
			rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		if (bfs == 48)
			rfs = 768;
		else
			rfs = 512;
		break;
	default:
		return -EINVAL;
	}

	rclk = params_rate(params) * rfs;

	/* Set the Codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* Set the AP DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PIC32_I2S_REFCLK, rclk,
				SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, rclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops snd_pic32_ak4953a_ops = {
	.hw_params = snd_pic32_ak4953a_hw_params,
};

static int ak4953a_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link snd_pic32_ak4953a_dai[] = {
	{
		.name = "AK4953A AIF1",
		.stream_name = "Playback",
		.cpu_dai_name = "pic32-i2s.0",
		.codec_dai_name = "ak4953a-AIF1",
		.platform_name = "pic32-i2s.0",
		.codec_name = "ak4953a.0-0012",
		.init = ak4953a_init,
		.ops = &snd_pic32_ak4953a_ops,
	},
	{
		.name = "AK4953A AIF2",
		.stream_name = "Capture",
		.cpu_dai_name = "pic32-i2s.1",
		.codec_dai_name = "ak4953a-AIF1",
		.platform_name = "pic32-i2s.1",
		.codec_name = "ak4953a.0-0012",
		.init = ak4953a_init,
		.ops = &snd_pic32_ak4953a_ops,
	},
};

static struct snd_soc_card snd_pic32_ak4953a = {
	.name = "snd-pic32-ak4953a",
	.dai_link = snd_pic32_ak4953a_dai,
	.num_links = ARRAY_SIZE(snd_pic32_ak4953a_dai),
};

static void snd_pic32_ak4953a_of_node_put(void)
{
	int i;

	for (i = 0; i < 2; i++) {
		if (snd_pic32_ak4953a_dai[i].cpu_of_node)
			of_node_put((struct device_node *)
				snd_pic32_ak4953a_dai[i].cpu_of_node);
		if (snd_pic32_ak4953a_dai[i].codec_of_node)
			of_node_put((struct device_node *)
				snd_pic32_ak4953a_dai[i].codec_of_node);
	}
}

static int snd_pic32_ak4953a_of_probe(struct platform_device *pdev,
			   struct device_node *np)
{
	struct device_node *codec_np, *msp_np[2];
	int i;

	msp_np[0] = of_parse_phandle(np, "microchip,cpu-dai", 0);
	msp_np[1] = of_parse_phandle(np, "microchip,cpu-dai", 1);
	codec_np  = of_parse_phandle(np, "microchip,audio-codec", 0);

	if (!(msp_np[0] && msp_np[1] && codec_np)) {
		dev_err(&pdev->dev, "Handle missing or invalid\n");
		snd_pic32_ak4953a_of_node_put();
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		snd_pic32_ak4953a_dai[i].cpu_of_node = msp_np[i];
		snd_pic32_ak4953a_dai[i].cpu_dai_name = NULL;
		snd_pic32_ak4953a_dai[i].platform_of_node = msp_np[i];
		snd_pic32_ak4953a_dai[i].platform_name = NULL;
		snd_pic32_ak4953a_dai[i].codec_of_node = codec_np;
		snd_pic32_ak4953a_dai[i].codec_name = NULL;
	}

	return 0;
}

static int snd_pic32_ak4953a_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (np) {
		ret =  snd_pic32_ak4953a_of_probe(pdev, np);
		if (ret)
			return ret;
	}

	snd_pic32_ak4953a.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_pic32_ak4953a);
	if (ret) {
		dev_err(&pdev->dev,
				"snd_soc_register_card() failed: %d\n", ret);
	}

	return ret;
}

static int snd_pic32_ak4953a_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_pic32_ak4953a);
}

static const struct of_device_id pic32_ak4953a_of_match[] = {
	{.compatible = "microchip,snd-pic32-ak4953a", },
	{ },
};
MODULE_DEVICE_TABLE(of, pic32_ak4953a_of_match);

static struct platform_driver snd_pic32_ak4953a_driver = {
	.driver = {
		.name   = "snd-pic32-ak4953a",
		.owner  = THIS_MODULE,
		.of_match_table = pic32_ak4953a_of_match,
	},
	.probe	  = snd_pic32_ak4953a_probe,
	.remove	 = snd_pic32_ak4953a_remove,
};

module_platform_driver(snd_pic32_ak4953a_driver);

MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_DESCRIPTION("ASoC Driver for Microchip PIC32 with AK4953A");
MODULE_LICENSE("GPL");
