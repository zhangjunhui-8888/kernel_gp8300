/*
 * GPT Chip2 clock driver on simulator and FPGA
 *
 * Copyright (C) 2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
 
#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <dt-bindings/clock/gpt,chip2dummy-clock.h>

#include "clk.h"

static const struct clk_div_table gpt_common_table[] = {
	{ .val = 7,  .div = 1  },
	{ .val = 15, .div = 1  },	
#ifdef CONFIG_COMMON_GPT_CHIP2_POLARIS_100MHz
	{ .val = 0,  .div = 1  },	/*polaris 100MHz*/
#else
	{ .val = 0,  .div = 2  },	/*default value*/
#endif
	{ .val = 8,  .div = 3  },	
	{ .val = 1,  .div = 4  },
	{ .val = 9,  .div = 6  },	
	{ .val = 2,  .div = 8  },
	{ .val = 10, .div = 12 },	
	{ .val = 3,  .div = 16 },
	{ .val = 11, .div = 24 },	
	{ .val = 4,  .div = 32 },
	{ .val = 12, .div = 48 },
	{ .val = 13, .div = 48 },
	{ .val = 14, .div = 48 },	
	{ .val = 5,  .div = 64 },
	{ .val = 6,  .div = 64 },
	{ } /* sentinel */
};

static const struct clk_div_table gpt_apb_table[] = {
	{ .val = 0,  .div = 4  },	/*default value*/
	{ .val = 1,  .div = 8  },
	{ .val = 2,  .div = 16 },
	{ .val = 3,  .div = 32 },
	{ .val = 4,  .div = 64 },
	{ .val = 5,  .div = 64 },
	{ .val = 6,  .div = 64 },
	{ .val = 7,  .div = 64 },	
	{ } /* sentinel */
};

/* fixed rate clocks */
static struct gpt_fixed_rate_clock chip2dummy_fixed_rate_clks[] __initdata = {
	{ GPT_CHIP2DUMMY_PLL,  "gptpll",  NULL, CLK_IS_ROOT, 100000000, },
};

/* divider rate clocks */
static struct gpt_divider_clock chip2dummy_div_clks[] __initdata = {
	{ GPT_CHIP2DUMMY_CLK0, "gptcpu0", "gptpll", 0, 0x00, 0, 4, 0, gpt_common_table, "cpu0"},
	{ GPT_CHIP2DUMMY_CLK1, "gptcpu1", "gptpll", 0, 0x04, 0, 4, 0, gpt_common_table, "cpu1"},
	{ GPT_CHIP2DUMMY_L2C,  "gptl2c",  "gptpll", 0, 0x10, 4, 4, 0, gpt_common_table, "l2cache"},	
	{ GPT_CHIP2DUMMY_APB,  "gptapb",  "gptl2c", 0, 0x10, 0, 3, 0, gpt_apb_table, "apb"},
};

static void __init chip2dummy_clk_init(struct device_node *np)
{
	struct gpt_clock_data *clk_data;

	clk_data = gpt_clk_init(np, CHIP2DUMMY_NR_CLKS);
	if (!clk_data)
		return;

	gpt_clk_register_fixed_rate(chip2dummy_fixed_rate_clks,
				     ARRAY_SIZE(chip2dummy_fixed_rate_clks),
				     clk_data);
	gpt_clk_register_divider(chip2dummy_div_clks, ARRAY_SIZE(chip2dummy_div_clks),
				  clk_data);
}
CLK_OF_DECLARE(chip2dummy_clk, "gpt,sysctrl", chip2dummy_clk_init);
