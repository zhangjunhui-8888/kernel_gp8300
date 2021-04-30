/*
 * GPT Chip2 clock driver for polaris project
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

#include <dt-bindings/clock/gpt,chip2polaris-clock.h>

#include "clk.h"

/* clock parent list */
static const char *cpll_p[] __initdata = { "osc", };
static const char *dpll_p[] __initdata = { "osc", };

static const struct clk_div_table gpt_common_div_table[] = {
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

static const struct clk_div_table gpt_apiu_div_table[] = {
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

static const struct clk_div_table gpt_i2s_div_table[] = {
	{ .val = 0,  .div = 1 },	/*default value*/
	{ .val = 1,  .div = 2 },
	{ .val = 2,  .div = 4 },
	{ .val = 3,  .div = 8 },
	{ } /* sentinel */
};

/*	The Polaris's clock mapping
 *		      |---CPU0
 *		      |---CPU1
 *	    	      |---CPU2
 *    	              |---CPU3
 *	    |-- CPLL->|---VIDMEM
 *	    |	      |---GSNN
 *OSC-----> |         |---SD
 *	    |         |---VCAP->|--VOUT
 *	    |         |---SPINOR
 *	    |	      |---L2C->|--APB->|---apb peripheral clocks
 *	    |
 *	    |-- DPLL->|---DDR
 *	    |
 *	    |-- PCIE
 */

/* fixed rate clocks */
static struct gpt_fixed_rate_clock chip2polaris_fixed_rate_clks[] __initdata = {
	{ GPT_CHIP2POLARIS_OSC,  "osc",  NULL, CLK_IS_ROOT, 12000000, },
};

/* fixed pll clocks */
static struct gpt_pll_clock chip2polaris_pll_clks[] __initdata = {
	{ GPT_CHIP2POLARIS_CPLL,  "gptcpll",  cpll_p, ARRAY_SIZE(cpll_p) , 0x34, 0},
	{ GPT_CHIP2POLARIS_DPLL,  "gptdpll",  dpll_p, ARRAY_SIZE(dpll_p) , 0x38, 0},
};

/* divider rate clocks */
static struct gpt_divider_clock chip2polaris_div_clks[] __initdata = {
	{ GPT_CHIP2POLARIS_CPU0_DIV,  "gptcpu0",  "gptcpll", 0, 0x00, 0, 4, 0, gpt_common_div_table, "cpu0"},
	{ GPT_CHIP2POLARIS_CPU1_DIV,  "gptcpu1",  "gptcpll", 0, 0x04, 0, 4, 0, gpt_common_div_table, "cpu1"},
	{ GPT_CHIP2POLARIS_CPU2_DIV,  "gptcpu2",  "gptcpll", 0, 0x08, 0, 4, 0, gpt_common_div_table, "cpu2"},
	{ GPT_CHIP2POLARIS_CPU3_DIV,  "gptcpu3",  "gptcpll", 0, 0x0c, 0, 4, 0, gpt_common_div_table, "cpu3"},	
	{ GPT_CHIP2POLARIS_GSNN_DIV,  "gptgsnn",  "gptcpll", 0, 0x10, 8, 4, 0, gpt_common_div_table, "gsnn"},
	{ GPT_CHIP2POLARIS_VIDMEM_DIV,"gptvidmem","gptcpll", 0, 0x10, 12,4, 0, gpt_common_div_table, "vidmem"},	
	{ GPT_CHIP2POLARIS_APB_DIV,   "gptapb",   "gptcpll", 0, 0x10, 16,4, 0, gpt_common_div_table, "apb"},
	{ GPT_CHIP2POLARIS_SD_DIV,    "gptsd",    "gptcpll", 0, 0x10, 20,4, 0, gpt_common_div_table, "sd"},
	{ GPT_CHIP2POLARIS_I2S_DIV,   "gpti2s",   "gptcpll", 0, 0x10, 24,2, 0, gpt_i2s_div_table, "i2s"},
	{ GPT_CHIP2POLARIS_L2C_DIV,   "gptl2c",   "gptcpll", 0, 0x10, 4, 4, 0, gpt_common_div_table, "l2cache"},
	{ GPT_CHIP2POLARIS_APIU_DIV,  "gptapiu",  "gptl2c",  0, 0x10, 0, 3, 0, gpt_apiu_div_table,   "apiu"},

};

/* gate clocks */
static struct gpt_gate_clock chip2polaris_gate_clks[] __initdata = {
	{ GPT_CHIP2POLARIS_CPU0_GATE,  "gptcpu0_gate",  "gptcpu0",  0, 0x00,  5,  0, "cpu0_gate"},
	{ GPT_CHIP2POLARIS_CPU1_GATE,  "gptcpu1_gate",  "gptcpu1",  0, 0x04,  5,  0, "cpu1_gate"},
	{ GPT_CHIP2POLARIS_CPU2_GATE,  "gptcpu2_gate",  "gptcpu2",  0, 0x08,  5,  0, "cpu2_gate"},
	{ GPT_CHIP2POLARIS_CPU3_GATE,  "gptcpu3_gate",  "gptcpu3",  0, 0x0c,  5,  0, "cpu3_gate"},
	{ GPT_CHIP2POLARIS_GSNN0_GATE, "gptgsnn0_gate", "gptgsnn0", 0, 0x100, 1,  0, "gsnn0_gate"},
	{ GPT_CHIP2POLARIS_GSNN1_GATE, "gptgsnn1_gate", "gptgsnn1", 0, 0x100, 3,  0, "gsnn1_gate"},
	{ GPT_CHIP2POLARIS_DDR_GATE,   "gptddr_gate",   "gptddr",   0, 0x100, 14, 0, "ddr_gate"},
};

static void __init chip2polaris_clk_init(struct device_node *np)
{
	struct gpt_clock_data *clk_data;

	clk_data = gpt_clk_init(np, CHIP2POLARIS_NR_CLKS);
	if (!clk_data)
		return;
	gpt_clk_register_fixed_rate(chip2polaris_fixed_rate_clks,
				     ARRAY_SIZE(chip2polaris_fixed_rate_clks),
				     clk_data);
	gpt_clk_register_pll(chip2polaris_pll_clks,
				     ARRAY_SIZE(chip2polaris_pll_clks),
				     clk_data);
	gpt_clk_register_divider(chip2polaris_div_clks,
				     ARRAY_SIZE(chip2polaris_div_clks),
				     clk_data);
	gpt_clk_register_gate(chip2polaris_gate_clks,
				     ARRAY_SIZE(chip2polaris_gate_clks),
				     clk_data);
}
CLK_OF_DECLARE(chip2polaris_clk, "gpt,sysctrl", chip2polaris_clk_init);
