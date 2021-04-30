/*
 * gpt_i2s.h - Definitions for gpt I2S driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010,2012 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * Copyright (c) 2009-2010, NVIDIA Corporation.
 * Scott Peterson <speterson@nvidia.com>
 *
 * Copyright (C) 2010 Google, Inc.
 * Iliyan Malchev <malchev@google.com>
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

#ifndef __GPT_I2S_H__
#define __GPT_I2S_H__


struct gpt_i2s {
	struct snd_soc_dai_driver dai;
	void __iomem*base;
	struct clk *clk_i2s;
	struct regmap *regmap;
};



#ifdef CONFIG_GPT_IRQ_ENABLE
int I2S_RX_IRQ[] = {
	I2S1_HW_IRQN_RX,
#ifndef CONFIG_TARGET_POLARIS_FPGA
	I2S2_HW_IRQN_RX
#endif
};

int I2S_TX_IRQ[] = {
	I2S1_HW_IRQN_TX,
#ifndef CONFIG_TARGET_POLARIS_FPGA
	I2S2_HW_IRQN_TX
#endif
};
#endif


//------------------gpt-i2s-reg defs end--------------------------//

#endif
