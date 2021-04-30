/*
 * arch/gpt/kernel/time.c
 *
 * GPT tick timer support.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/clk-provider.h>
#include <linux/time.h>
#include <linux/delay.h>

unsigned long gpt_get_cpu_clk(void)
{
	struct clk*cpu;
	char cpu_name[8];

	sprintf(cpu_name, "cpu%d", smp_processor_id());
	cpu = clk_get(NULL, cpu_name);
	if (IS_ERR(cpu)) {
		panic("Unable to get CPU clock\n");
	}else{
		return clk_get_rate(cpu);
	}
}

/* Called from init/main.c */
void __init time_init(void)
{
	unsigned long rate;

	of_clk_init(NULL);
	clocksource_of_init();
	tick_setup_hrtimer_broadcast();

	rate = gpt_get_cpu_clk() >> 3;
	lpj_fine = rate / HZ;
}
