/*
 * GPT timer API
 * Copyright (C) 2018, General Processor Techologies Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __ASM_GPT_TIME_H
#define __ASM_GPT_TIME_H

unsigned long gpt_get_cpu_clk(void);

extern void gpt_timer_set(unsigned long count);
extern void gpt_timer_set_next(unsigned long delta);

#ifdef CONFIG_SMP
extern void synchronise_count_master(int cpu);
extern void synchronise_count_slave(int cpu);
#endif

#endif /* __ASM_GPT_TIME_H */
