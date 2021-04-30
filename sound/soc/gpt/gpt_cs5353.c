/*
 * gpt_cs5353.c - gpt machine ASoC driver for boards using cs5353 codec.
 * machine driver
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010-2012 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../codecs/wm8753.h"

#include "gpt_asoc_utils.h"

#define DRV_NAME "gpt-snd-cs5353"

struct gpt_cs5353 {
	struct gpt_asoc_utils_data util_data;
};

static int gpt_cs5353_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = rtd->card;
	struct gpt_cs5353 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	default:
		mclk = 12288000;
		break;
	}

	err = gpt_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

	return 0;
}

static struct snd_soc_ops gpt_cs5353_ops = {
	.hw_params = gpt_cs5353_hw_params,
};

static const struct snd_soc_dapm_widget gpt_cs5353_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static struct snd_soc_dai_link gpt_cs5353_dai = {
	.name = "cs5353",
	.stream_name = "cs5353 PCM",
	.platform_name = "f0014000.i2s",
	.codec_dai_name = "cs5353-hifi",
	.ops = &gpt_cs5353_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
};

static struct snd_soc_card snd_soc_gpt_cs5353 = {
	.name = "gpt-cs5353",
	.owner = THIS_MODULE,
	.dai_link = &gpt_cs5353_dai,
	.num_links = 1,

	.dapm_widgets = gpt_cs5353_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(gpt_cs5353_dapm_widgets),
	.fully_routed = true,
};

static int gpt_cs5353_machine_driver_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_gpt_cs5353;
	struct gpt_cs5353 *machine;
	int ret;

	machine = devm_kzalloc(&pdev->dev, sizeof(struct gpt_cs5353),
			       GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate gpt_cs5353 struct\n");
		return -ENOMEM;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret)
		goto err;
/*
	ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
	if (ret)
		goto err;
*/
	gpt_cs5353_dai.codec_of_node = of_parse_phandle(np,
			"audio-codec", 0);
	if (!gpt_cs5353_dai.codec_of_node) {
		dev_err(&pdev->dev,
			"Property 'audio-codec' missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

	gpt_cs5353_dai.cpu_of_node = of_parse_phandle(np,
			"i2s-controller", 0);
	if (!gpt_cs5353_dai.cpu_of_node) {
		dev_err(&pdev->dev,
			"Property 'i2s-controller' missing or invalid\n");
		ret = -EINVAL;
		goto err;
	}

/*	gpt_cs5353_dai.platform_of_node = gpt_cs5353_dai.cpu_of_node;*/

	ret = gpt_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret)
		goto err;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_fini_utils;
	}

	return 0;

err_fini_utils:
	gpt_asoc_utils_fini(&machine->util_data);
err:
	return ret;
}

static int gpt_cs5353_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct gpt_cs5353 *machine = snd_soc_card_get_drvdata(card);

	snd_soc_unregister_card(card);

	gpt_asoc_utils_fini(&machine->util_data);

	return 0;
}

static const struct of_device_id gpt_cs5353_of_match[] = {
	{ .compatible = "cs,sound", },
	{},
};

static struct platform_driver gpt_cs5353_machine_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = gpt_cs5353_of_match,
	},
	.probe = gpt_cs5353_machine_driver_probe,
	.remove = gpt_cs5353_driver_remove,
};
module_platform_driver(gpt_cs5353_machine_driver);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("gpt+cs5353 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, gpt_cs5353_of_match);
