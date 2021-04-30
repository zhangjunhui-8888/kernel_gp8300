/*
 * Arasan Secure Digital Host Controller Interface.
 * Copyright (C) 2011 - 2012 Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2012 Wind River Systems, Inco
 * Copyright (C) 2013 Pengutronix e.K.
 * Copyright (C) 2013 Xilinx Inc.
 *
 * Based on sdhci-of-esdhc.c
 *
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 * Copyright (c) 2009 MontaVista Software, Inc.
 *
 * Authors: Xiaobo Xie <X.Xie@freescale.com>
 *	    Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/module.h>
#include "sdhci-pltfm.h"

#define SDHCI_ARASAN_CLK_CTRL_OFFSET	0x2c

#define CLK_CTRL_TIMEOUT_SHIFT		16
#define CLK_CTRL_TIMEOUT_MASK		(0xf << CLK_CTRL_TIMEOUT_SHIFT)
#define CLK_CTRL_TIMEOUT_MIN_EXP	13

#define EXTCR_BASE			0xf0000100
#define EXTCR_SD_M			((1 << 26))
#define EXTCR_SD_EN			((1 << 10) | EXTCR_SD_M)

#define	CORE_CTRL_REG			0x1000
#define CLK_A_EN			(0x01 << 8)
#define CLK_B_EN			(0x02 << 8)
/**
 * struct sdhci_arasan_data
 * @clk_ahb:	Pointer to the AHB clock
 */
struct sdhci_arasan_data {
	struct clk	*clk_ahb;
};

static unsigned int sdhci_arasan_get_timeout_clock(struct sdhci_host *host)
{
	u32 div;
	unsigned long freq;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	div = readl(host->ioaddr + SDHCI_ARASAN_CLK_CTRL_OFFSET);
	div = (div & CLK_CTRL_TIMEOUT_MASK) >> CLK_CTRL_TIMEOUT_SHIFT;

	freq = clk_get_rate(pltfm_host->clk);
	freq /= 1 << (CLK_CTRL_TIMEOUT_MIN_EXP + div);

	return freq;
}

static struct sdhci_ops sdhci_arasan_ops = {
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.get_timeout_clock = sdhci_arasan_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_arasan_pdata = {
	.ops = &sdhci_arasan_ops,
};

#ifdef CONFIG_PM_SLEEP
/**
 * sdhci_arasan_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Put the device in a low power state.
 */
static int sdhci_arasan_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable(pltfm_host->clk);
	clk_disable(sdhci_arasan->clk_ahb);

	return 0;
}

/**
 * sdhci_arasan_resume - Resume method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Resume operation after suspend
 */
static int sdhci_arasan_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;
	int ret;

	ret = clk_enable(sdhci_arasan->clk_ahb);
	if (ret) {
		dev_err(dev, "Cannot enable AHB clock.\n");
		return ret;
	}

	ret = clk_enable(pltfm_host->clk);
	if (ret) {
		dev_err(dev, "Cannot enable SD clock.\n");
		clk_disable(sdhci_arasan->clk_ahb);
		return ret;
	}

	return sdhci_resume_host(host);
}
#endif /* ! CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_arasan_dev_pm_ops, sdhci_arasan_suspend,
			 sdhci_arasan_resume);

#ifdef CONFIG_PLAT_GPT_CHIP2_POLARIS_EVK

#define APIU_GPIO1_DIR	0xF0008400
#define APIU_GPIO1_DATA	0xF0008000

void gpt_sdhci_gpio_init(void)
{
	int dir_base, data_base, val = 0;
	
	dir_base = ioremap_nocache(APIU_GPIO1_DIR, 0x100);
	data_base = ioremap_nocache(APIU_GPIO1_DATA, 0x100);
 
	val = readl(dir_base);
	writel(val & 0xfffffffc, dir_base);	//set gpio0(sd)/gpio1(emmc) output 
	val = readl(data_base);
	writel(val | 0x01, data_base);		//set gpio0 output high 
	iounmap(dir_base);
	iounmap(data_base);
}

#endif


static int sdhci_arasan_probe(struct platform_device *pdev)
{
	int ret;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_arasan_data *sdhci_arasan;
	unsigned int *extcr, val;

	extcr = ioremap(EXTCR_BASE, 0x100);
	writel(readl(extcr) | EXTCR_SD_EN, extcr);
	iounmap(extcr);

	sdhci_arasan = devm_kzalloc(&pdev->dev, sizeof(*sdhci_arasan),
			GFP_KERNEL);
	if (!sdhci_arasan)
		return -ENOMEM;

	host = sdhci_pltfm_init(pdev, &sdhci_arasan_pdata, 0);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		dev_err(&pdev->dev, "platform init failed (%u)\n", ret);
		goto clk_disable_all;
	}

	host->quirks |= SDHCI_QUIRK_BROKEN_DMA;
	//host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;
	host->quirks |= SDHCI_QUIRK2_PRESET_VALUE_BROKEN;
	host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;
	host->quirks |= SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;

	sdhci_get_of_property(pdev);
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = sdhci_arasan;

	mmc_of_parse(host->mmc);

	val = sdhci_readl(host, CORE_CTRL_REG); 
#if defined(CONFIG_PLAT_GPT_CHIP2_POLARIS_EVK)
	val |= CLK_A_EN;
	//gpt_sdhci_gpio_init();
#else
	val |= CLK_B_EN;
#endif
	sdhci_writel(host, val, CORE_CTRL_REG); 

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "platform register failed (%u)\n", ret);
		goto err_pltfm_free;
	}

	printk("gpt %s\n", __func__);

	return 0;

err_pltfm_free:
	sdhci_pltfm_free(pdev);
clk_disable_all:

	return ret;
}

static int sdhci_arasan_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(sdhci_arasan->clk_ahb);

	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id sdhci_arasan_of_match[] = {
	{ .compatible = "gpt,arasan" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_arasan_of_match);

static struct platform_driver sdhci_arasan_driver = {
	.driver = {
		.name = "sdhci-arasan",
		.of_match_table = sdhci_arasan_of_match,
		.pm = &sdhci_arasan_dev_pm_ops,
	},
	.probe = sdhci_arasan_probe,
	.remove = sdhci_arasan_remove,
};

module_platform_driver(sdhci_arasan_driver);
