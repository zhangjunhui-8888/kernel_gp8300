/*
 * GPT i2s core driver
 *
 * Copyright (C) 2020 by GPT
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation.
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
#include <sound/soc.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/fs.h>

#include "i2s-gpt-core.h"

#define DRV_NAME "i2s-gpt"

//#define AF_PRINT
#ifdef  AF_PRINT
#define  SP_PRINTK(format,arg...) do { \
	printk(KERN_INFO format,##arg);\
}while(0)
#else
#define  SP_PRINTK(format,arg...) do { \
}while(0)
#endif

struct gpt_i2s {
	struct device *dev;
	void __iomem*base;
	const char  *name;

	/*
	 *sclk_ref=Refclk/1/2/4/8  
	 *LRCK=sclk_ref/Frame size
	 */
	struct clk *sclk;
	unsigned int sclk_freq;

	u8   mode;
	u8   align_mode;
	u16  sample_size;
	u16  frame_size;

	u8 bus_num;

	int rx_irq;
	int tx_irq;
};
struct gpt_i2s *i2s_gpt;

/*
 * i2s mode, align mode, frame size, sample size setup
 */
int i2s_gpt_mode_setup(u8 mode, u8 align_mode, u16 sample_size, 
		u16 frame_size)
{
	u32 value = 0;

	i2s_gpt->mode = mode;
	i2s_gpt->align_mode = align_mode;
	i2s_gpt->sample_size = sample_size;
	i2s_gpt->frame_size = frame_size;

	value = readl(i2s_gpt->base + I2S_UCR);

	switch (i2s_gpt->mode) {
		case 0:
			value |= I2S_MASTER;
			break;
		case 1:
			value &= 0x7fffffff;
			break;
		default:
			printk("i2s mode is Invalid argument!\n");
			return -EINVAL;	
	}

	switch (i2s_gpt->align_mode) {
		case 0:
			value &= 0xfffffffc;
			break;
		case 1:
			break;
			value |= MODE_I2S_RJ;
		case 2:
			value = (value & 0xfffffffd) | MODE_I2S_LJ;
			break;
		default:
			printk("i2s align mode is Invalid argument!\n");
			return -EINVAL;	
	}

	switch (i2s_gpt->sample_size){
		case 16:
			value &= 0xfffffff3;
			break;
		case 20:
			value = (value & 0xfffffff7) | I2S_CHSIZE_20;
			break;
		case 24:
			value = (value & 0xfffffffb) | I2S_CHSIZE_24;
			break;
		case 32:
			value |= I2S_CHSIZE_32;
			break;
		default:
			printk("i2s sample_size is Invalid argument!\n");
			return -EINVAL;	
	}

	switch (i2s_gpt->frame_size) {
		case 32:
			value &= 0xffffff8f;
			break;
		case 48:
			value = (value & 0xffffff9f) | I2S_FRMSIZE_48;	
			break;
		case 64:
			value = (value & 0xffffffaf) | I2S_FRMSIZE_64;
			break;
		case 128:
			value = (value & 0xffffffbf) | I2S_FRMSIZE_128;
			break;
		case 256:
			value = (value | 0xffffffcf) | I2S_FRMSIZE_256;
			break;
		case 384:
			value |= 0x70;
			break;
		default:
			printk("i2s frame_size is Invalid argument!\n");
			return -EINVAL;	
	}

	writel(value, i2s_gpt->base + I2S_UCR);
	return 0;
}
EXPORT_SYMBOL_GPL(i2s_gpt_mode_setup);

/*
 *GPT i2s mode enable or disable
 */
void i2s_gpt_enable(bool enable)
{
	u32 status = 0;

	if(enable) {
		status = readl(i2s_gpt->base + I2S_UCR);
		writel(status | I2S_EN, i2s_gpt->base + I2S_UCR);

	} else {
		status = readl(i2s_gpt->base + I2S_UCR);
		writel(status & 0xffffff7f, i2s_gpt->base + I2S_UCR);
	}
}
EXPORT_SYMBOL_GPL(i2s_gpt_enable);

/*
 *GPT i2s playback function
 */
int i2s_gpt_playback(u32 L_data, u32 R_data)
{
	u32 status = 0;
#if 0
	//INT
	status = readl(i2s_gpt->base + I2S_UCR);
	writel(status | I2S_UMASK_INT, i2s_gpt->base + I2S_UCR);
#endif

	while(1) {
		status = readl(i2s_gpt->base + I2S_USFR);

		if((status & I2S_RAW_TXL_LOW) && (status & I2S_RAW_TXR_LOW) !=0) {
			writel(L_data, i2s_gpt->base + I2S_TXL);
			writel(R_data, i2s_gpt->base + I2S_TXR);
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(i2s_gpt_playback);

/*
 *GPT i2s capture function
 */
uint64_t i2s_gpt_capture(void)
{
	u32 status = 0;
	u32 L_data = 0; 
	u32 R_data = 0;
	u64 data = 0;

	status = readl(i2s_gpt->base + I2S_USFR);

	while(1) {
		if((status & I2S_RAW_RXL_DATA_AVAILABLE) && (status & I2S_RAW_RXR_DATA_AVAILABLE) !=0) {
			L_data = readl(i2s_gpt->base + I2S_RXL);
			R_data = readl(i2s_gpt->base + I2S_RXR);
			break;
		}
	}

	data = R_data + L_data * (2 ^ 32);
	return data;
}
EXPORT_SYMBOL_GPL(i2s_gpt_capture);

#if 0
static irqreturn_t gpt_i2s_rx_irq(int irq, void *dev_id)
{
	u32 value;

	struct gpt_i2s *i2s = dev_id;

	value = readl(i2s->base + I2S_USFR);

	return IRQ_HANDLED;
}

static irqreturn_t gpt_i2s_tx_irq(int irq, void *dev_id)
{
	u32 value;

	struct gpt_i2s *i2s = dev_id;

	value = readl(i2s_gpt->base + I2S_USFR);

	return IRQ_HANDLED;
}
#endif

/*
 *GPT i2s moudle init
 */
static void i2s_gpt_init(struct gpt_i2s* i2s)
{
	u32 value = 0;

	/*
	 *enable i2s; master; Left Justified; Sample size:16; Frame size:32; LRCK=SCLK_REF/Frame size=48Khz;
	 *cs4334(DA) only support 48KHZ
	 */
	value = I2S_MASTER | MODE_I2S | I2S_CHSIZE_16 | I2S_FRMSIZE_32 | I2S_UMASK_BST | I2S_UMASK_INT;  //master
	//value = MODE_I2S | I2S_CHSIZE_16 | I2S_FRMSIZE_384 | I2S_UMASK_BST | I2S_UMASK_INT;			 //slave
	writel(value, i2s->base + I2S_UCR);
	udelay(2);
}

/*
 *GPT i2s SCLK setup
 */
int i2s_gpt_sclk_setup(unsigned int sclk_freq)
{
	i2s_gpt->sclk_freq = sclk_freq;

	uint32_t *extsrc = ioremap(0xf0000010,0x40);
	if (extsrc == NULL) {
		printk("%s:%d-->get memory/io resource filed\n", __func__, __LINE__);
		return -ENXIO;
	}

	switch (i2s_gpt->sclk_freq) {
		case 12288000:
			//Sclk_ref = Refclk = 12.288Mhz      25:24=00
			writel((readl(extsrc) & 0xFCFFFFFF), extsrc);
			break;
		case 6144000:
			//Sclk_ref = Refclk/2 = 6144Khz      25:24=01
			writel((readl(extsrc) & 0xFDFFFFFF) | 0x1000000, extsrc);
			break;
		case 3072000:
			//Sclk_ref = Refclk/4 = 3072Khz      25:24=10
			writel((readl(extsrc) & 0xFEFFFFFF) | 0x2000000, extsrc);
			break;
		case 1536000:
			//Sclk_ref = Refclk/8 = 1536Khz      25:24=11
			writel(readl(extsrc) | 0x3000000, extsrc);
			break;
		default:
			printk("i2s sclk freq is Invalid argument!\n");
			return -EINVAL;	
	}

	iounmap(extsrc);

	return 0;
}
EXPORT_SYMBOL_GPL(i2s_gpt_sclk_setup);

static int gpt_i2s_probe(struct platform_device *pdev)
{
	struct resource *mem;
	int ret;
	struct device_node *node = pdev->dev.of_node;

	i2s_gpt = devm_kzalloc(&pdev->dev, sizeof(struct gpt_i2s), GFP_KERNEL);
	if (!i2s_gpt) {
		dev_err(&pdev->dev, "Can't allocate for gpt_i2s\n");
		ret = -ENOMEM;
		return ret;
	}

	i2s_gpt->mode = 0;
	i2s_gpt->align_mode = 0;
	i2s_gpt->sample_size = 16;
	i2s_gpt->frame_size = 32;

	i2s_gpt->sclk_freq = 1536000;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s_gpt->base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(i2s_gpt->base)) {
		ret = PTR_ERR(i2s_gpt->base);
		printk("fail to ioremap i2s base!\n");
		return ret;
	}

	i2s_gpt->dev = &pdev->dev;
	i2s_gpt->name = pdev->name;
	platform_set_drvdata(pdev, i2s_gpt);

#if 0
	i2s_gpt->sclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2s_gpt->sclk)) {
		dev_err(&pdev->dev, "Can't retrieve i2s clock\n");
		ret = PTR_ERR(i2s_gpt->sclk);
		return ret;
	}

	ret = clk_prepare_enable(i2s_gpt->sclk);
	if (ret)
		return ret;
	i2s_gpt->sclk_freq = clk_get_rate(i2s_gpt->sclk);
	printk("sclk = %ld-------------------\n", i2s_gpt->sclk_freq);
#endif

	i2s_gpt_init(i2s_gpt);

	//sclk_ref set; if not,Sclk_ref = Refclk = 12.288Mhz
	i2s_gpt_sclk_setup(i2s_gpt->sclk_freq);
	i2s_gpt_mode_setup(i2s_gpt->mode, i2s_gpt->align_mode, i2s_gpt->sample_size, 
			i2s_gpt->frame_size);


	i2s_gpt->bus_num = of_alias_get_id(node, "i2s");

#if 0
	//request irq
	i2s_gpt->rx_irq = platform_get_irq(pdev, 0);
	if (i2s_gpt->rx_irq < 0) {
		dev_err(&pdev->dev,"no i2s IRQ specified!!");
		ret = i2s_gpt->rx_irq;
		return ret;
	}

	i2s_gpt->tx_irq = platform_get_irq(pdev, 1);
	if (i2s_gpt->tx_irq < 0) {
		dev_err(&pdev->dev,"no i2s IRQ specified!!");
		ret = i2s_gpt->tx_irq;
		return ret;
	}

	ret = devm_request_irq(&pdev->dev, i2s_gpt->rx_irq, gpt_i2s_rx_irq, 0,
			dev_name(&pdev->dev), i2s_gpt);
	if (ret)
		return ret;

	ret = devm_request_irq(&pdev->dev, i2s_gpt->tx_irq, gpt_i2s_tx_irq, 0,
			dev_name(&pdev->dev), i2s_gpt);
	if (ret)
		return ret;
#endif
	return 0;
}

static int gpt_i2s_remove(struct platform_device *pdev)
{
	struct gpt_i2s *i2s = dev_get_drvdata(&pdev->dev);

	iounmap((void __iomem*)i2s->base);

	return 0;
}

static const struct of_device_id gpt_i2s_of_match[] = {
	{ .compatible = "gpt,gpt-i2s", },
	{},
};

static struct platform_driver gpt_i2s_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gpt_i2s_of_match,
	},
	.probe = gpt_i2s_probe,
	.remove = gpt_i2s_remove,
};
module_platform_driver(gpt_i2s_driver);

MODULE_DESCRIPTION("gpt I2S  driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, gpt_i2s_of_match);
