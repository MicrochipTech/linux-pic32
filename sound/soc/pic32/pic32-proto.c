/*
 * ASoC driver for PROTO AudioCODEC (with a WM8731)
 *
 * Author:      Florian Meier, <koalo@koalo.de>
 *	      Copyright 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "../codecs/wm8731.h"

static const unsigned int wm8731_rates_12288000[] = {
	8000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list wm8731_constraints_12288000 = {
	.list = wm8731_rates_12288000,
	.count = ARRAY_SIZE(wm8731_rates_12288000),
};

static int snd_pic32_proto_startup(struct snd_pcm_substream *substream)
{
	/* Setup constraints, because there is a 12.288 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&wm8731_constraints_12288000);
	return 0;
}

static int snd_pic32_proto_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int sysclk = 12288000; /* This is fixed on this board */

	/* Set proto sysclk */
	int ret = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK_XTAL,
		sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(codec->dev,
			"Failed to set WM8731 SYSCLK: %d\n", ret);
		return ret;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops snd_pic32_proto_ops = {
	.startup = snd_pic32_proto_startup,
	.hw_params = snd_pic32_proto_hw_params,
};

static struct snd_soc_dai_link snd_pic32_proto_dai[] = {
	{
		.name		= "WM8731",
		.stream_name	= "WM8731 PCM",
		.cpu_dai_name	= "pic32-i2s.0",
		.codec_dai_name	= "wm8731-hifi",
		.platform_name	= "pic32-i2s.0",
		.codec_name	= "wm8731.0-001a",
		.dai_fmt	= SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBM_CFM,
		.ops		= &snd_pic32_proto_ops,
	},
};

/* audio machine driver */
static struct snd_soc_card snd_pic32_proto = {
	.name		= "snd-pic32-proto",
	.owner		= THIS_MODULE,
	.probe		= NULL,
	.dai_link	= snd_pic32_proto_dai,
	.num_links	= ARRAY_SIZE(snd_pic32_proto_dai),
};

static void snd_pic32_proto_of_node_put(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(snd_pic32_proto_dai); i++) {
		if (snd_pic32_proto_dai[i].cpu_of_node)
			of_node_put((struct device_node *)
				snd_pic32_proto_dai[i].cpu_of_node);
		if (snd_pic32_proto_dai[i].codec_of_node)
			of_node_put((struct device_node *)
				snd_pic32_proto_dai[i].codec_of_node);
	}
}

static int snd_pic32_proto_of_probe(struct platform_device *pdev,
			   struct device_node *np)
{
	struct device_node *codec_np, *msp_np[2];
	int i;

	msp_np[0] = of_parse_phandle(np, "microchip,cpu-dai", 0);
	msp_np[1] = of_parse_phandle(np, "microchip,cpu-dai", 1);
	codec_np  = of_parse_phandle(np, "microchip,audio-codec", 0);

	if (!(msp_np[0] && msp_np[1] && codec_np)) {
		dev_err(&pdev->dev, "Handle missing or invalid\n");
		snd_pic32_proto_of_node_put();
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(snd_pic32_proto_dai); i++) {
		snd_pic32_proto_dai[i].cpu_of_node = msp_np[i];
		snd_pic32_proto_dai[i].cpu_dai_name = NULL;
		snd_pic32_proto_dai[i].platform_of_node = msp_np[i];
		snd_pic32_proto_dai[i].platform_name = NULL;
		snd_pic32_proto_dai[i].codec_of_node = codec_np;
		snd_pic32_proto_dai[i].codec_name = NULL;
	}

	return 0;
}

static int snd_pic32_proto_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;

	if (np) {
		ret =  snd_pic32_proto_of_probe(pdev, np);
		if (ret)
			return ret;
	}

	snd_pic32_proto.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_pic32_proto);
	if (ret) {
		dev_err(&pdev->dev,
				"snd_soc_register_card() failed: %d\n", ret);
	}

	return ret;
}

static int snd_pic32_proto_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_pic32_proto);
}

static const struct of_device_id pic32_proto_of_match[] = {
	{.compatible = "microchip,snd-pic32-proto", },
	{ },
};
MODULE_DEVICE_TABLE(of, pic32_proto_of_match);

static struct platform_driver snd_pic32_proto_driver = {
	.driver = {
		.name   = "snd-pic32-proto",
		.owner  = THIS_MODULE,
		.of_match_table = pic32_proto_of_match,
	},
	.probe	  = snd_pic32_proto_probe,
	.remove	 = snd_pic32_proto_remove,
};

module_platform_driver(snd_pic32_proto_driver);

MODULE_AUTHOR("Joshua Henderson <joshua.henderson@microchip.com>");
MODULE_DESCRIPTION("ASoC Driver for PIC32 connected to PROTO board (WM8731)");
MODULE_LICENSE("GPL");
