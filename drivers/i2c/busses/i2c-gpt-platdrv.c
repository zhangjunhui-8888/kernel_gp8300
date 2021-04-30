/*
 * GPT  I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
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
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/platform_data/i2c-gpt.h>
#include "i2c-gpt-core.h"
#include <asm/mach-chip2/apiu.h>


static struct i2c_algorithm i2c_gpt_algo = {
	.master_xfer	= i2c_gpt_xfer,
	.functionality	= i2c_gpt_func,
};
static u32 i2c_gpt_get_clk_rate_khz(struct gpt_i2c_dev *dev)
{
	return clk_get_rate(dev->clk)/1000;
}

static void enable_i2c_ctrl(void __iomem *addr)
{
	writel(0x1fff5503, (void *)(addr + APIU_UCR));
}

static void gpt_inithw(struct gpt_i2c_dev *dev)
{

	apiu_unit_init(dev->base,0);

}

static int gpt_i2c_parse_dt(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *pinctrl;
	const char *pctrl_state;
	struct pinctrl_state *states;
	struct device_node *np = pdev->dev.of_node;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	pctrl_state = devm_kzalloc(&pdev->dev, sizeof(pctrl_state), GFP_KERNEL);
	if (!pctrl_state) {
		dev_err(&pdev->dev, "Cannot allocate pctrl_state\n");
		return -ENOMEM;
	}

	states = devm_kzalloc(&pdev->dev, sizeof(states), GFP_KERNEL);
	if (!states) {
		dev_err(&pdev->dev, "Cannot allocate states\n");
		return -ENOMEM;
	}

	ret = of_property_read_string_index(np, "pinctrl-names", 0,
							&pctrl_state);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot parse pinctrl-names %d\n", ret);
		return ret;
	}

	states = pinctrl_lookup_state(pinctrl, pctrl_state);
	if (IS_ERR(states)) {
		dev_err(&pdev->dev, "Lookup state failed\n");
		return IS_ERR(states);
	}

	ret = pinctrl_select_state(pinctrl, states);
	if (ret < 0) {
		dev_err(&pdev->dev, "Select state failed\n");
		return ret;
	}

	return 0;
}

static int gpt_i2c_probe(struct platform_device *pdev)
{
	struct gpt_i2c_dev *dev;
	struct i2c_adapter *adap;
	struct resource *mem;
	struct gpt_i2c_platform_data *pdata;
	int irq, r;
	u32 clk_freq, ht = 0;

	if (of_alias_get_id(pdev->dev.of_node, "i2c") != 0)
		gpt_i2c_parse_dt(pdev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq; /* -ENXIO */
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(struct gpt_i2c_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_ioremap_resource(&pdev->dev, mem);

	gpt_inithw(dev);

	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);

	enable_i2c_ctrl(dev->base);
	init_completion(&dev->cmd_complete);
	mutex_init(&dev->lock);
	dev->dev = &pdev->dev;
	dev->irq = irq;
	platform_set_drvdata(pdev, dev);

	/* fast mode by default because of legacy reasons */

	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node, "i2c_scl_freq",&clk_freq);
		/* Only standard mode at 100kHz and fast mode at 400kHz
		 * are supported.
		 */
	    dev->scl_freq = clk_freq;	
        if (clk_freq != 100 && clk_freq != 400) {
			dev_err(&pdev->dev, "Only 100kHz and 400kHz supported");
			return -EINVAL;
		}
	} else {
		pdata = dev_get_platdata(&pdev->dev);
		if (pdata)
			clk_freq = pdata->i2c_scl_freq;
	}

	dev->functionality =
		I2C_FUNC_I2C |
		I2C_FUNC_10BIT_ADDR |
		I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK;
	if (clk_freq == 100)
		dev->master_cfg =  GPT_IC_CON_MASTER | GPT_IC_CON_SLAVE_DISABLE |
			GPT_IC_CON_RESTART_EN | GPT_IC_CON_SPEED_STD;
	else
		dev->master_cfg =  GPT_IC_CON_MASTER | GPT_IC_CON_SLAVE_DISABLE |
			GPT_IC_CON_RESTART_EN | GPT_IC_CON_SPEED_FAST;

	dev->clk = devm_clk_get(&pdev->dev, NULL);
	dev->get_clk_rate_khz = i2c_gpt_get_clk_rate_khz;
	if (IS_ERR(dev->clk))
		return PTR_ERR(dev->clk);
	clk_prepare_enable(dev->clk);

	if (!dev->sda_hold_time && ht) {
		u32 ic_clk = dev->get_clk_rate_khz(dev);

		dev->sda_hold_time = div_u64((u64)ic_clk * ht + 500000,
					     1000000);
	}

	if (!dev->tx_fifo_depth) {
		u32 param1 = i2c_gpt_read_comp_param(dev);

		dev->tx_fifo_depth = ((param1 >> 16) & 0xff) + 1;
		dev->rx_fifo_depth = ((param1 >> 8)  & 0xff) + 1;
		dev->adapter.nr = pdev->id;
	}
	r = i2c_gpt_init(dev);
	if (r)
		return r;

	i2c_gpt_disable_int(dev);
	r = devm_request_irq(&pdev->dev, dev->irq, i2c_gpt_isr, IRQF_SHARED | IRQF_NO_THREAD,
			pdev->name, dev);
	if (r) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);
		return r;
	}

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	strlcpy(adap->name, "gpt I2C adapter",
			sizeof(adap->name));
	adap->algo = &i2c_gpt_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		return r;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, 1000);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int gpt_i2c_remove(struct platform_device *pdev)
{
	struct gpt_i2c_dev *dev = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);

	i2c_del_adapter(&dev->adapter);

	i2c_gpt_disable(dev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gpt_i2c_of_match[] = {
	{ .compatible = "gpt,i2c0.0", },
	{ .compatible = "gpt,i2c0.1", },
	{ .compatible = "gpt,i2c0.2", },
//	{ .compatible = "gpt,i2c0.3", },
	{},
};
MODULE_DEVICE_TABLE(of, gpt_i2c_of_match);
#endif

#ifdef CONFIG_PM
static int gpt_i2c_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpt_i2c_dev *i_dev = platform_get_drvdata(pdev);

	i2c_gpt_disable(i_dev);
	clk_disable_unprepare(i_dev->clk);

	return 0;
}

static int gpt_i2c_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpt_i2c_dev *i_dev = platform_get_drvdata(pdev);

	clk_prepare_enable(i_dev->clk);
	i2c_gpt_init(i_dev);

	return 0;
}
#endif


/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_gpt");

static struct platform_driver gpt_i2c_driver = {
	.probe = gpt_i2c_probe,
	.remove = gpt_i2c_remove,
	.driver		= {
		.name	= "i2c_gpt",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpt_i2c_of_match),
	},
};

static int __init gpt_i2c_init_driver(void)
{
	return platform_driver_register(&gpt_i2c_driver);
}
subsys_initcall(gpt_i2c_init_driver);

static void __exit gpt_i2c_exit_driver(void)
{
	platform_driver_unregister(&gpt_i2c_driver);
}
module_exit(gpt_i2c_exit_driver);

MODULE_AUTHOR("GPT Inc.");
MODULE_DESCRIPTION("GPT I2C bus adapter");
MODULE_LICENSE("GPL");
