/*
 * tegra20_i2s.c - Tegra20 I2S driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010,2012 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Scott Peterson <speterson@nvidia.com>
 *
 * Copyright (C) 2010 Google, Inc.
 * Iliyan Malchev <malchev@google.com>
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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>

#include <asm/mach-chip2/apiu.h>
#include "gpt_i2s.h"
#include "gpt-pcm.h"

#define DRV_NAME "gpt-i2s"


#ifdef	CONFIG_APIU
#define i2s_readl(addr)		apiu_readl(addr)
#define i2s_writel(val,addr)	apiu_writel(val,addr)
#endif


//#define AF_PRINT
#ifdef  AF_PRINT
#define  SP_PRINTK(format,arg...) do { \
                                          printk(KERN_INFO format,##arg);\
                                  }while(0)
#else
#define  SP_PRINTK(format,arg...) do { \
                                          }while(0)
#endif



static int gpt_i2s_set_fmt(struct snd_soc_dai *dai,
				unsigned int fmt)
{
//	struct gpt_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	default:
		return -EINVAL;
	}


	return 0;
}

static int gpt_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	//struct device *dev = dai->dev;
	//struct gpt_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	int sample_size;
	//int i2sclock, srate, ret, bitcnt;
	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_size = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sample_size = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sample_size = 32;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void gpt_i2s_start_playback(struct gpt_i2s *i2s)
{
	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);
}

static void  gpt_i2s_stop_playback(struct gpt_i2s *i2s)
{
	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);
}

static void gpt_i2s_start_capture(struct gpt_i2s *i2s)
{
	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);
}

static void gpt_i2s_stop_capture(struct gpt_i2s *i2s)
{
	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);
}

static int gpt_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct gpt_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			gpt_i2s_start_playback(i2s);
		else
			gpt_i2s_start_capture(i2s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			gpt_i2s_stop_playback(i2s);
		else
			gpt_i2s_stop_capture(i2s);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int gpt_i2s_probe(struct snd_soc_dai *dai)
{
	//struct gpt_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	SP_PRINTK("file: %s,func: %s,line: %d \n",__FILE__,__func__,__LINE__);

	return 0;
}

static const struct snd_soc_dai_ops gpt_i2s_dai_ops = {
	.set_fmt	= gpt_i2s_set_fmt,
	.hw_params	= gpt_i2s_hw_params,
	.trigger	= gpt_i2s_trigger,
};

static const struct snd_soc_dai_driver gpt_i2s_dai_template = {
	.probe = gpt_i2s_probe,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &gpt_i2s_dai_ops,
	.symmetric_rates = 1,
};

static const struct snd_soc_component_driver gpt_i2s_component = {
	.name		= DRV_NAME,
};

static bool tegra20_i2s_wr_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	default:
		return false;
	}
}

static bool tegra20_i2s_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	default:
		return false;
	}
}

static bool tegra20_i2s_precious_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	default:
		return false;
	}
}

static const struct regmap_config tegra20_i2s_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.writeable_reg = tegra20_i2s_wr_rd_reg,
	.readable_reg = tegra20_i2s_wr_rd_reg,
	.volatile_reg = tegra20_i2s_volatile_reg,
	.precious_reg = tegra20_i2s_precious_reg,
	.cache_type = REGCACHE_FLAT,
};
static void i2s_init(struct gpt_i2s* i2s){
	unsigned int value;

	SP_PRINTK("file:%s, func: %s,line=%d,i2s_regs = %p\n",__FILE__,__func__,__LINE__,i2s->base + APIU_UNIT_UCR);

	i2s_writel(I2S_EN, i2s->base + APIU_UNIT_UCR);
	udelay(10);
	value = I2S_CHSIZE_16 | I2S_FRMSIZE_32 |(1 << I2S_MASTER_sft) | I2S_UMASK_STD | I2S_UMASK_BST;

	SP_PRINTK("	value =%x\n",value);
	i2s_writel(value, i2s->base + APIU_UNIT_UCR);
	udelay(2);
	i2s_writel(I2S_EN | value, i2s->base + APIU_UNIT_UCR);
	udelay(2);
}

static int gpt_i2s_platform_probe(struct platform_device *pdev)
{
	struct gpt_i2s *i2s;
	struct resource *mem;
	void __iomem *regs;

	int ret;
	SP_PRINTK("---------------------gpt_i2s_platform_probe--------------\n");
	i2s = devm_kzalloc(&pdev->dev, sizeof(struct gpt_i2s), GFP_KERNEL);
	if (!i2s) {
		dev_err(&pdev->dev, "Can't allocate gpt_i2s\n");
		ret = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, i2s);

	i2s->dai = gpt_i2s_dai_template;
	i2s->dai.name = dev_name(&pdev->dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No memory resource\n");
		ret = -ENODEV;
		goto err_clk_put;
	}

	regs = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_clk_put;
	}

	i2s->base = regs;
	SP_PRINTK("i2s->base = %p\n",i2s->base);
	i2s_init(i2s);
	ret = snd_soc_register_component(&pdev->dev, &gpt_i2s_component,
					 &i2s->dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAI: %d\n", ret);
		ret = -ENOMEM;
		goto err_suspend;
	}

	ret = gpt_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_component;
	}
	SP_PRINTK("-------------gpt-i2s probe end----------------------\n");
	return 0;

err_unregister_component:
	snd_soc_unregister_component(&pdev->dev);
err_suspend:
err_clk_put:
	clk_put(i2s->clk_i2s);
err:
	return ret;
}

static int gpt_i2s_platform_remove(struct platform_device *pdev)
{
	struct gpt_i2s *i2s = dev_get_drvdata(&pdev->dev);

	//tegra_pcm_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);

	clk_put(i2s->clk_i2s);

	return 0;
}

static const struct of_device_id gpt_i2s_of_match[] = {
	{ .compatible = "gpt,gpt-i2s0", },
	{},
};

static struct platform_driver gpt_i2s_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gpt_i2s_of_match,
	},
	.probe = gpt_i2s_platform_probe,
	.remove = gpt_i2s_platform_remove,
};
module_platform_driver(gpt_i2s_driver);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("gpt I2S ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, gpt_i2s_of_match);
