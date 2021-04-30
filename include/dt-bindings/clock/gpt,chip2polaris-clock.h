/*
 * GPT Chip2 clock driver
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

#ifndef __DTS_CHIP2CHASE_CLOCK_H
#define __DTS_CHIP2CHASE_CLOCK_H

#define CHIP2CHASE_NONE_CLOCK		0

/* fixed clocks */
#define GPT_CHIP2POLARIS_OSC		1
/* pll clocks */
#define GPT_CHIP2POLARIS_CPLL		2
#define GPT_CHIP2POLARIS_DPLL		3
/* divider clocks */
#define GPT_CHIP2POLARIS_CPU0_DIV	4
#define GPT_CHIP2POLARIS_CPU1_DIV	5
#define GPT_CHIP2POLARIS_CPU2_DIV	6
#define GPT_CHIP2POLARIS_CPU3_DIV	7
#define GPT_CHIP2POLARIS_GSNN_DIV	8
#define GPT_CHIP2POLARIS_VIDMEM_DIV	9
#define GPT_CHIP2POLARIS_APB_DIV	10
#define GPT_CHIP2POLARIS_SD_DIV		11
#define GPT_CHIP2POLARIS_I2S_DIV	12
#define GPT_CHIP2POLARIS_L2C_DIV	13
#define GPT_CHIP2POLARIS_APIU_DIV	14
/* gate clocks */
#define GPT_CHIP2POLARIS_CPU0_GATE	15
#define GPT_CHIP2POLARIS_CPU1_GATE	16
#define GPT_CHIP2POLARIS_CPU2_GATE	17
#define GPT_CHIP2POLARIS_CPU3_GATE	18
#define GPT_CHIP2POLARIS_GSNN0_GATE	19
#define GPT_CHIP2POLARIS_GSNN1_GATE	20
#define GPT_CHIP2POLARIS_DDR_GATE	21

#define CHIP2POLARIS_NR_CLKS		22

#endif	/* __DTS_CHIP2CHASE_CLOCK_H */
