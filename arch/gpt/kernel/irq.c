/*
 * arch/gpt/kernel/irq.c
 *
 * GPT interrupt handling.
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

#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ftrace.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/seq_file.h>
#include <linux/export.h>
#include <linux/irqflags.h>

#include <asm/spr.h>

unsigned long irq_err_count;
irq_handler handle_arch_irq[IX_XPEN_MAX];

int arch_show_interrupts(struct seq_file *p, int prec)
{
#ifdef CONFIG_SMP
	show_ipi_list(p, prec);
#endif
	seq_printf(p, "%*s: %10lu\n", prec, "Err", irq_err_count);
	return 0;
}

/* Called from init/main.c */
void __init init_IRQ(void)
{
	unsigned i;
	irqchip_init();
	for (i = 0; i < IX_XPEN_MAX; i++)
	  if (!handle_arch_irq[i]) {
		panic("No interrupt controller found.");
	  }
}


/* Called from drivers/irqchip/irq-gpt-pic.c */
void __init set_handle_irq(irq_handler handler, unsigned int irq)
{
	handle_arch_irq[irq] = handler;
}
