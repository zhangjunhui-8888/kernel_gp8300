/*
 * GPT Secure Digital Host Controller Interface.

 *
 * Based on sdhci-of-esdhc.c

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include "sdhci-pltfm.h"
#include "sdhci.h"

#define SDHCI_GPT_CLK_CTRL_OFFSET	0x2c

#define CLK_CTRL_TIMEOUT_SHIFT		16
#define CLK_CTRL_TIMEOUT_MASK		(0xf << CLK_CTRL_TIMEOUT_SHIFT)
#define CLK_CTRL_TIMEOUT_MIN_EXP	13

#define CORE_CTRL_REG			0x1000
#define CLK_0_EN			(0x01 << 8)
#define CLK_1_EN			(0x02 << 8)

struct gpio_desc *gpio0_signal;
struct gpio_desc *gpio0_power;

struct sdhci_gpt_data {
	unsigned int slot;
};

static unsigned int sdhci_gpt_get_timeout_clock(struct sdhci_host *host)
{
	u32 div;
	unsigned long freq;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	div = readl(host->ioaddr + SDHCI_GPT_CLK_CTRL_OFFSET);
	div = (div & CLK_CTRL_TIMEOUT_MASK) >> CLK_CTRL_TIMEOUT_SHIFT;

	freq = clk_get_rate(pltfm_host->clk);
	freq /= 1 << (CLK_CTRL_TIMEOUT_MIN_EXP + div);

	return freq;
}

static struct sdhci_ops sdhci_gpt_ops = {
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.get_timeout_clock = sdhci_gpt_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_gpt_pdata = {
          .ops = &sdhci_gpt_ops,
          .quirks = SDHCI_QUIRK_BROKEN_DMA | SDHCI_QUIRK_CARD_DETECTION |
		    SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK | SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,

          .quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN | SDHCI_QUIRK2_CARD_ON_NEEDS_BUS_ON,
  };

#ifdef CONFIG_PM_SLEEP
/**
 * sdhci_gpt_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Put the device in a low power state.
 */
static int sdhci_gpt_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable(pltfm_host->clk);

	return 0;
}

/**
 * sdhci_gpt_resume - Resume method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Resume operation after suspend
 */
static int sdhci_gpt_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int ret;

	ret = clk_enable(pltfm_host->clk);
	if (ret) {
		dev_err(dev, "Cannot enable SD clock.\n");
		return ret;
	}

	return sdhci_resume_host(host);
}
#endif /* ! CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_gpt_dev_pm_ops, sdhci_gpt_suspend,
			 sdhci_gpt_resume);

void sdhci_signal_voltage_switch(struct gpio_desc *gpio, unsigned char voltage)
{
	switch (voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
       		gpiod_direction_output(gpio, 1);   	  
		pr_debug("sdhci-gpt output 3.3v voltage\n");	 
		break;
	case MMC_SIGNAL_VOLTAGE_180:
       		gpiod_direction_output(gpio, 0);   	  
		pr_debug("sdhci-gpt output 1.8v voltage\n");	 
		break;
	default:
		break;
	}

}

void sdhci_power_voltage_switch(struct gpio_desc *gpio, unsigned char power_mode)
{
	switch (power_mode) {
	case MMC_POWER_ON:
       		gpiod_direction_output(gpio, 1);   	  
		break;
	case MMC_POWER_OFF:
       		gpiod_direction_output(gpio, 0);   	  
		break;
	default:
		break;
	}

}

static int sdhci_gpio_parse(struct device_node *devnod) 
{
	int gpio_power = 0;

#ifndef CONFIG_MMC_SDHCI_OF_GPT_EMMC
	int gpio_signal = 0;

       	/* set gpio0_0(signal) output for sd card*/ 
	gpio_signal = of_get_named_gpio_flags(devnod, "signal-gpios", 0, NULL);
	if (gpio_signal < 0) {
		if (gpio_signal != -EPROBE_DEFER)
			pr_err("%s: Can't get 'signal-gpios' DT property\n",
			       __func__);
		return -1;
	}

	gpio0_signal = gpio_to_desc(gpio_signal);
	pr_debug("sdhci-gpt gpio_signal:%d\n", gpio_signal);
#endif

       	/* set gpio0_1(power) output for sd card; set gpio0_1(reset) output for emmc*/
	gpio_power = of_get_named_gpio_flags(devnod, "power-gpios", 0, NULL);
	if (gpio_power < 0) {
		if (gpio_power != -EPROBE_DEFER)
			pr_err("%s: Can't get 'power-gpios' DT property\n",
			       __func__);
		return -1;
	}

	gpio0_power = gpio_to_desc(gpio_power);
	pr_debug("sdhci-gpt gpio_power:%d\n", gpio_power);

	return 0;
}

static int sdhci_gpt_probe(struct platform_device *pdev)
{
	int ret;
	struct clk *clk_xin;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_gpt_data *pdata;
	unsigned long val;
	struct device_node *np = pdev->dev.of_node;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata),
			GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	ret = sdhci_gpio_parse(np);
	if (ret) {
		dev_err(&pdev->dev, "can not find SD device.\n");
		return ret;
	}

	clk_xin = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk_xin)) {
		dev_err(&pdev->dev, "clock not found.\n");
		return PTR_ERR(clk_xin);
	}

	ret = clk_prepare_enable(clk_xin);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SD clock.\n");
		return ret;
	}

	host = sdhci_pltfm_init(pdev, &sdhci_gpt_pdata, 0);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		dev_err(&pdev->dev, "platform init failed (%u)\n", ret);
		goto clk_disable;
	}

	ret = mmc_of_parse(host->mmc);
	if(ret) {
		goto clk_disable;
	}

	sdhci_get_of_property(pdev);
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = pdata;
	pltfm_host->clk = clk_xin;

        if (of_property_read_u32(np, "slot",
                                &pdata->slot)) {
                dev_info(&pdev->dev, "slot property not found, "
                                "assuming slot 0 is available\n");
                pdata->slot = 0;
        }
	val = sdhci_readl(host, CORE_CTRL_REG);
	if(pdata->slot == 0)
		val |= CLK_0_EN;
	else
		val |= CLK_1_EN;

	sdhci_writel(host, val, CORE_CTRL_REG);

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "platform register failed (%u)\n", ret);
		goto err_pltfm_free;
	}

	return 0;

err_pltfm_free:
	sdhci_pltfm_free(pdev);
clk_disable:
	clk_disable_unprepare(clk_xin);

	return ret;
}

static int sdhci_gpt_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	clk_disable_unprepare(pltfm_host->clk);

	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id sdhci_gpt_of_match[] = {
	{ .compatible = "gpt,sdhci-8.9a" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_gpt_of_match);

static struct platform_driver sdhci_gpt_driver = {
	.driver = {
		.name = "sdhci-gpt",
		.of_match_table = sdhci_gpt_of_match,
		.pm = &sdhci_gpt_dev_pm_ops,
	},
	.probe = sdhci_gpt_probe,
	.remove = sdhci_gpt_remove,
};

module_platform_driver(sdhci_gpt_driver);

MODULE_DESCRIPTION("Driver for the GPT SDHCI Controller");
MODULE_LICENSE("GPL");
