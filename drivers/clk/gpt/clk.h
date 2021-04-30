/*
 * GPT clock driver,refer to gptlicon
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

#ifndef	__GPT_CLK_H
#define	__GPT_CLK_H

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/spinlock.h>

struct gpt_clock_data {
	struct clk_onecell_data	clk_data;
	void __iomem		*base;
};

struct gpt_pll_clock {
	unsigned int		id;
	char			*name;
	const char		**parent_name;
	u8			num_parents;
	unsigned long		offset;
	unsigned long		flags;
};

struct gpt_fixed_rate_clock {
	unsigned int		id;
	char			*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		fixed_rate;
};

struct gpt_fixed_factor_clock {
	unsigned int		id;
	char			*name;
	const char		*parent_name;
	unsigned long		mult;
	unsigned long		div;
	unsigned long		flags;
};

struct gpt_mux_clock {
	unsigned int		id;
	const char		*name;
	const char		**parent_names;
	u8			num_parents;
	unsigned long		flags;
	unsigned long		offset;
	u8			shift;
	u8			width;
	u8			mux_flags;
	u32			*table;
	const char		*alias;
};

struct gpt_divider_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		offset;
	u8			shift;
	u8			width;
	u8			div_flags;
	const struct clk_div_table	*table;
	const char		*alias;
};

struct gpt_gate_clock {
	unsigned int		id;
	const char		*name;
	const char		*parent_name;
	unsigned long		flags;
	unsigned long		offset;
	u8			bit_idx;
	u8			gate_flags;
	const char		*alias;
};

struct gpt_composite_clock {
	int			id;

	const char		*name;
	const char		**parent_names;
	const char		*parent;
	unsigned long		flags;
	u8			num_parents;

	u32			mux_reg;
	signed char		mux_shift;
	signed char		mux_width;

	u32			divider_reg;
	signed char		divider_shift;
	signed char		divider_width;

	u32 			gate_reg;
	signed char		gate_shift;
};

struct gpt_clock_data __init *gpt_clk_init(struct device_node *, int);
void __init gpt_clk_register_pll(struct gpt_pll_clock *clks,
					int nums, struct gpt_clock_data *data);
void __init gpt_clk_register_fixed_rate(struct gpt_fixed_rate_clock *,
					int, struct gpt_clock_data *);
void __init gpt_clk_register_fixed_factor(struct gpt_fixed_factor_clock *,
					int, struct gpt_clock_data *);
void __init gpt_clk_register_mux(struct gpt_mux_clock *, int,
				struct gpt_clock_data *);
void __init gpt_clk_register_divider(struct gpt_divider_clock *,
				int, struct gpt_clock_data *);
void __init gpt_clk_register_gate(struct gpt_gate_clock *,
					int, struct gpt_clock_data *);
struct clk * gpt_pll_register(struct device *dev, const char *name,
		     const char **parent_name, u8 num_parents, 
		     void __iomem *reg,unsigned long flags);
					
#endif	/* __GPT_CLK_H */
