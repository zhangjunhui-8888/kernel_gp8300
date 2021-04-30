/*
 * tegra_asoc_utils.h - Definitions for Tegra DAS driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010,2012 - NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __GPT_ASOC_UTILS_H__
#define __GPT_ASOC_UTILS_H__

struct clk;
struct device;

enum tegra_asoc_utils_soc {
	GPT_ASOC_UTILS_SOC_GPT20,
	GPT_ASOC_UTILS_SOC_GPT30,
	GPT_ASOC_UTILS_SOC_GPT114,
	GPT_ASOC_UTILS_SOC_GPT124,
};

struct gpt_asoc_utils_data {
	struct device *dev;
	enum tegra_asoc_utils_soc soc;
	struct clk *clk_pll_a;
	struct clk *clk_pll_a_out0;
	struct clk *clk_cdev1;
	int set_baseclock;
	int set_mclk;
};

int gpt_asoc_utils_set_rate(struct gpt_asoc_utils_data *data, int srate,
			      int mclk);
int gpt_asoc_utils_set_ac97_rate(struct gpt_asoc_utils_data *data);
int gpt_asoc_utils_init(struct gpt_asoc_utils_data *data,
			  struct device *dev);
void gpt_asoc_utils_fini(struct gpt_asoc_utils_data *data);

#endif
