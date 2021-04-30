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

#ifndef __DTS_CHIP2DUMMY_CLOCK_H
#define __DTS_CHIP2DUMMY_CLOCK_H

#define CHIP2DUMMY_NONE_CLOCK	0

/* fixed clocks */
#define GPT_CHIP2DUMMY_PLL	1

/* divider clocks */
#define GPT_CHIP2DUMMY_CLK0	2
#define GPT_CHIP2DUMMY_CLK1	3
#define GPT_CHIP2DUMMY_L2C	4
#define GPT_CHIP2DUMMY_APB	5

#define CHIP2DUMMY_NR_CLKS	6

#endif	/* __DTS_CHIP2DUMMY_CLOCK_H */
