/*
 * GP8300 SoCs pinctrl driver.
 *
 * Copyright (C) 2014 
 *
 *
 * Copyright (C) 2014 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-gp8300.h"

static int gpt_io_cnf_reg(void)
{
	uint32_t *extsrc = ioremap(0xf0000200, 0x40);
	if (extsrc == NULL) {
		printk("%s:%d--> get memory/io resource failed\n",
					__func__, __LINE__);
		return -ENXIO;
	}
	printk("GPT io_cnf address(%x) remap to %p\n", 0xf0000200, extsrc);

	writel(readl(extsrc) | PINMUX_DEFAULT_CONFIG,extsrc);

	printk("GPT io_cnf address(%x) remap to %p\n", 0xf0000200, extsrc);
	iounmap(extsrc);
	return 0;
}

static int gp8300_pinctrl_probe(struct platform_device *pdev)
{
	printk("start gpt8300_pinctrl_init\n");
	gpt_io_cnf_reg();
	return 0;

}

#define PINCTRL_DRIVER_NAME	"gpt_pinctrl"
static struct of_device_id gp8300_pinctrl_match[] = {
	{ .name= PINCTRL_DRIVER_NAME,},
	{},
};
MODULE_DEVICE_TABLE(of, gp8300_pinctrl_match);

static struct platform_driver gp8300_pinctrl_driver = {
	.probe	= gp8300_pinctrl_probe,
	.driver	= {
		.name		= PINCTRL_DRIVER_NAME, 
		.owner		= THIS_MODULE,
		.of_match_table	= gp8300_pinctrl_match,
	},
};
module_platform_driver(gp8300_pinctrl_driver);

MODULE_AUTHOR("");
MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com");
MODULE_DESCRIPTION("gp8300 pinctrl driver");
MODULE_LICENSE("GPL");
