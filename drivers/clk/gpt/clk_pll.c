/*
 * GPT chip2 PLL driver
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
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

 /*
 * The output frequency formula for the pll is:
 * clkout = ((M/N)*fin)/P
 *
 *	M		SIC		SIP
 *    16  ~ 50	      5'b0_1111	      5'b0_1001
 *    51  ~ 85	      5'b1_0011	      5'b0_1101
 *    86  ~ 116	      5'b1_0111	      5'b1_0001
 *    117 ~ 160	      5'b1_1001	      5'b1_0011
 *    161 ~ 120	      5'b1_1011	      5'b1_0101
 *    211 ~ 256	      5'b1_1011	      5'b1_0111
 */

#define PDIV_MASK	0x3f
#define MDIV_MASK	0xff
#define NDIV_MASK	0x1f
#define SIP_MASK	0x1f
#define SIC_MASK	0x1f
#define BYPASS_MASK	0x1

#define PDIV_SHIFT	0
#define MDIV_SHIFT	6
#define NDIV_SHIFT	14
#define BYPASS_SHIFT	19
#define SIP_SHIFT	20
#define SIC_SHIFT	25

struct gpt_clk_pll {
	struct clk_hw hw;
	void __iomem *reg;
};

#define to_gpt_clk_pll(_hw) container_of(_hw, struct gpt_clk_pll, hw)

static unsigned long gpt_pll_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	struct gpt_clk_pll *pll = to_gpt_clk_pll(hw);
	u64 m, n, p, rate = parent_rate / 1000000;
	u32 pllcon;

	pllcon = readl(pll->reg);
	if ((pllcon >> BYPASS_SHIFT) & BYPASS_MASK) {
		pr_warn("%s: pll %s is bypassed\n", __func__,
			__clk_get_name(hw->clk));
		return parent_rate;
	}
	
	p = ((pllcon >> PDIV_SHIFT) & PDIV_MASK) + 1;
	m = ((pllcon >> MDIV_SHIFT) & MDIV_MASK) + 1;
	n = ((pllcon >> NDIV_SHIFT) & NDIV_MASK) + 1;
	
	rate = ((m/n)*parent_rate)/p;
	
	return (unsigned long)rate;
}

static const struct clk_ops gpt_pll_ops = {
	.recalc_rate = gpt_pll_recalc_rate,
};

struct clk * gpt_pll_register(struct device *dev, const char *name,
		     const char **parent_name, u8 num_parents,
		     void __iomem *reg,unsigned long flags)
{
	struct clk_init_data init;
	struct gpt_clk_pll *pll;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->reg = reg;
	pll->hw.init = &init;
	init.name = name;
	init.ops = &gpt_pll_ops;
	init.parent_names = parent_name;
	init.num_parents = num_parents;
	init.flags = flags;

	return clk_register(dev, &pll->hw);
}
