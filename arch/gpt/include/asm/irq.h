/*
 * arch/gpt/include/asm/irq.h
 *
 * GPT irq definitions.
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

#ifndef __ASM_GPT_IRQ_H__
#define __ASM_GPT_IRQ_H__

#define NR_IRQS         64

#include <asm-generic/irq.h>

#define NO_IRQ          (-1)

typedef void (*irq_handler)(struct pt_regs *, unsigned int);
extern irq_handler handle_arch_irq[IX_XPEN_MAX];
extern void set_handle_irq(irq_handler, unsigned int);

#endif /* __ASM_GPT_IRQ_H__ */
