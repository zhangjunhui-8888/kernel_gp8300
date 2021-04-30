/*
 * GPT Chip2 clock driver for chase project
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

#include <dt-bindings/clock/gpt,chip2fpga-clock.h>

#include "clk.h"
/* clock parent list */
static const char *cpll_p[] __initdata = { "osc", };
static const char *dpll_p[] __initdata = { "osc", };

static const struct clk_div_table gpt_cpu_table[] = {
	{ .val = 7,  .div = 1  },
	{ .val = 15, .div = 1  },	
	{ .val = 0,  .div = 2  },	/*default value*/
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
static struct gpt_fixed_rate_clock chip2chase_fixed_rate_clks[] __initdata = {
	{ GPT_CHIP2CHASE_OSC,  "osc",  NULL, CLK_IS_ROOT, 12000000, },
};

/* fixed pll clocks */
static struct gpt_pll_clock chip2chase_pll_clks[] __initdata = {
	{ GPT_CHIP2CHASE_CPLL,  "gptcpll",  cpll_p, ARRAY_SIZE(cpll_p) , 0},
	{ GPT_CHIP2CHASE_DPLL,  "gptdpll",  dpll_p, ARRAY_SIZE(dpll_p) , 0},	
};

/* divider rate clocks */
static struct gpt_divider_clock chip2chase_div_clks[] __initdata = {
	{ GPT_CHIP2CHASE_CLK0, "gptcpu0",  "gptcpll", 0, 0x00, 0, 4, CLK_DIVIDER_HIWORD_MASK, gpt_cpu_table, "cpu0"},
	{ GPT_CHIP2CHASE_CLK1, "gptcpu1",  "gptcpll", 0, 0x04, 0, 4, CLK_DIVIDER_HIWORD_MASK, gpt_cpu_table, "cpu1"},
	{ GPT_CHIP2CHASE_CLK2, "gptcpu2",  "gptcpll", 0, 0x04, 0, 4, CLK_DIVIDER_HIWORD_MASK, gpt_cpu_table, "cpu2"},
	{ GPT_CHIP2CHASE_CLK3, "gptcpu3",  "gptcpll", 0, 0x04, 0, 4, CLK_DIVIDER_HIWORD_MASK, gpt_cpu_table, "cpu3"},			
	{ GPT_CHIP2CHASE_APB,  "gptapb",   "gptcpll", 0, 0x10, 0, 3, CLK_DIVIDER_HIWORD_MASK, gpt_apb_table, "apb"},
	/* l2cache has same freq  table */
	{ GPT_CHIP2CHASE_L2C,  "gptl2c",   "gptcpll", 0, 0x10, 0, 3, CLK_DIVIDER_HIWORD_MASK, gpt_cpu_table, "l2cache"},
};

/*		      |---CPU0
 *		      |---CPU1
 *	    |-- CPLL--|---CPU2
 *    	    |         |---CPU3
 * OSC------|	      |---VIDMEM
 *	    |	      |---APB--|---apb peripheral clocks
 *	    |
 *	    |-- DPLL--|---EXT--|---external peripheral clocks
 *	    |
 *	    |-- PCIE
 */

static void __init chip2chase_clk_init(struct device_node *np)
{
	struct gpt_clock_data *clk_data;

	clk_data = gpt_clk_init(np, CHIP2CHASE_NR_CLKS);
	if (!clk_data)
		return;

	gpt_clk_register_fixed_rate(chip2chase_fixed_rate_clks,
				     ARRAY_SIZE(chip2chase_fixed_rate_clks),
				     clk_data);
	gpt_clk_register_divider(chip2chase_div_clks, ARRAY_SIZE(chip2chase_div_clks),
				  clk_data);
}
CLK_OF_DECLARE(chip2chase_clk, "gpt,sysctrl", chip2chase_clk_init);
