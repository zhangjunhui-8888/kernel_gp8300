/*
 * arch/gpt/include/asm/timex.h
 *
 * GPT access kernel's cycle counter. Use 64 bit time base register.
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

#ifndef __ASM_GPT_TIMEX_H
#define __ASM_GPT_TIMEX_H

typedef unsigned long long cycles_t;

static inline cycles_t get_cycles(void)
{
        return  sprget_gpt(TIM);
}

#define ARCH_HAS_READ_CURRENT_TIMER

#endif /* __ASM_GPT_TIMEX_H */
