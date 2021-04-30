/*
 * GPT clock driver, refer to hisilicon
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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include "clk.h"

static DEFINE_SPINLOCK(gpt_clk_lock);

struct gpt_clock_data __init *gpt_clk_init(struct device_node *np,
					     int nr_clks)
{
	struct gpt_clock_data *clk_data;
	struct clk **clk_table;
	void __iomem *base;

	if (np) {
		base = of_iomap(np, 0);
		if (!base) {
			pr_err("failed to map gpt clock registers\n");
			goto err;
		}
	} else {
		pr_err("failed to find gpt clock node in DTS\n");
		goto err;
	}

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data) {
		pr_err("%s: could not allocate clock data\n", __func__);
		goto err;
	}
	clk_data->base = base;

	clk_table = kzalloc(sizeof(struct clk *) * nr_clks, GFP_KERNEL);
	if (!clk_table) {
		pr_err("%s: could not allocate clock lookup table\n", __func__);
		goto err_data;
	}
	clk_data->clk_data.clks = clk_table;
	clk_data->clk_data.clk_num = nr_clks;
	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_data->clk_data);
	return clk_data;
err_data:
	kfree(clk_data);
err:
	return NULL;
}

void __init gpt_clk_register_pll(struct gpt_pll_clock *clks,
					 int nums, struct gpt_clock_data *data)
{
	struct clk *clk;
	int i;
	void __iomem *base = data->base;

	for (i = 0; i < nums; i++) {
		clk = gpt_pll_register(NULL, clks[i].name,
		     			clks[i].parent_name,
		     			clks[i].num_parents,
		     			base + clks[i].offset,
		     			clks[i].flags);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}
		data->clk_data.clks[clks[i].id] = clk;
	}
}

void __init gpt_clk_register_fixed_rate(struct gpt_fixed_rate_clock *clks,
					 int nums, struct gpt_clock_data *data)
{
	struct clk *clk;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_fixed_rate(NULL, clks[i].name,
					      clks[i].parent_name,
					      clks[i].flags,
					      clks[i].fixed_rate);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}
		data->clk_data.clks[clks[i].id] = clk;
	}
}

void __init gpt_clk_register_fixed_factor(struct gpt_fixed_factor_clock *clks,
					   int nums,
					   struct gpt_clock_data *data)
{
	struct clk *clk;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_fixed_factor(NULL, clks[i].name,
						clks[i].parent_name,
						clks[i].flags, clks[i].mult,
						clks[i].div);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}
		data->clk_data.clks[clks[i].id] = clk;
	}
}

void __init gpt_clk_register_mux(struct gpt_mux_clock *clks,
				  int nums, struct gpt_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		u32 mask = BIT(clks[i].width) - 1;

		clk = clk_register_mux_table(NULL, clks[i].name,
					clks[i].parent_names,
					clks[i].num_parents, clks[i].flags,
					base + clks[i].offset, clks[i].shift,
					mask, clks[i].mux_flags,
					clks[i].table, &gpt_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}
}

void __init gpt_clk_register_divider(struct gpt_divider_clock *clks,
				      int nums, struct gpt_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_divider_table(NULL, clks[i].name,
						 clks[i].parent_name,
						 clks[i].flags,
						 base + clks[i].offset,
						 clks[i].shift, clks[i].width,
						 clks[i].div_flags,
						 clks[i].table,
						 &gpt_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}
}

void __init gpt_clk_register_gate(struct gpt_gate_clock *clks,
				       int nums, struct gpt_clock_data *data)
{
	struct clk *clk;
	void __iomem *base = data->base;
	int i;

	for (i = 0; i < nums; i++) {
		clk = clk_register_gate(NULL, clks[i].name,
						clks[i].parent_name,
						clks[i].flags,
						base + clks[i].offset,
						clks[i].bit_idx,
						clks[i].gate_flags,
						&gpt_clk_lock);
		if (IS_ERR(clk)) {
			pr_err("%s: failed to register clock %s\n",
			       __func__, clks[i].name);
			continue;
		}

		if (clks[i].alias)
			clk_register_clkdev(clk, clks[i].alias, NULL);

		data->clk_data.clks[clks[i].id] = clk;
	}
}


struct clk *gpt_clk_register_composite(const struct gpt_composite_clock *gc,
		void __iomem *base)
{
	struct clk *clk;
	struct clk_mux *mux = NULL;
	struct clk_gate *gate = NULL;
	struct clk_divider *div = NULL;
	struct clk_hw *mux_hw = NULL, *gate_hw = NULL, *div_hw = NULL;
	const struct clk_ops *mux_ops = NULL, *gate_ops = NULL, *div_ops = NULL;
	const char **parent_names;
	const char *parent;
	int num_parents;
	int ret;

	if (gc->mux_shift >= 0) {
		mux = kzalloc(sizeof(*mux), GFP_KERNEL);
		if (!mux)
			return ERR_PTR(-ENOMEM);

		mux->reg = base + gc->mux_reg;
		mux->mask = BIT(gc->mux_width) - 1;
		mux->shift = gc->mux_shift;
		mux->lock = &gpt_clk_lock;

		mux_hw = &mux->hw;
		mux_ops = &clk_mux_ops;

		parent_names = gc->parent_names;
		num_parents = gc->num_parents;
	} else {
		parent = gc->parent;
		parent_names = &parent;
		num_parents = 1;
	}

	if (gc->gate_shift >= 0) {
		gate = kzalloc(sizeof(*gate), GFP_KERNEL);
		if (!gate) {
			ret = -ENOMEM;
			goto err_out;
		}

		gate->reg = base + gc->gate_reg;
		gate->bit_idx = gc->gate_shift;
		gate->flags = CLK_GATE_SET_TO_DISABLE;
		gate->lock = &gpt_clk_lock;

		gate_hw = &gate->hw;
		gate_ops = &clk_gate_ops;
	}

	if (gc->divider_shift >= 0) {
		div = kzalloc(sizeof(*div), GFP_KERNEL);
		if (!div) {
			ret = -ENOMEM;
			goto err_out;
		}

		div->reg = base + gc->divider_reg;
		div->shift = gc->divider_shift;
		div->width = gc->divider_width;
		div->lock = &gpt_clk_lock;

		div_hw = &div->hw;
		div_ops = &clk_divider_ops;
	}

	clk = clk_register_composite(NULL, gc->name, parent_names, num_parents,
		mux_hw, mux_ops,
		div_hw, div_ops,
		gate_hw, gate_ops,
		gc->flags);

	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto err_out;
	}

	return clk;
err_out:
	kfree(div);
	kfree(gate);
	kfree(mux);

	return ERR_PTR(ret);
}

void gpt_clk_register_composites(const struct gpt_composite_clock *gcs,
		int num, void __iomem *base, struct gpt_clock_data *data)
{
	struct clk *clk;
	int i;

	for (i = 0; i < num; i++) {
		const struct gpt_composite_clock *gc = &gcs[i];

		if (data && !IS_ERR_OR_NULL(data->clk_data.clks[gc->id]))
			continue;

		clk = gpt_clk_register_composite(gc, base);

		if (IS_ERR(clk)) {
			pr_err("Failed to register clk %s: %ld\n",
					gc->name, PTR_ERR(clk));
			continue;
		}

		data->clk_data.clks[gc->id] = clk;
	}
}
