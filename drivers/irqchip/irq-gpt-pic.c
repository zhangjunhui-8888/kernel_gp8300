/*
 * arch/drivers/irqchip/irq-gpt-pic.c
 *
 * GPT interrupt subsystem interface to kernel through irqchip abstraction.
 *
 * Copyright (C) 2015, Optimum Semiconductor Technologies
 *  Enrique Barria <ebarria@optimumsemi.com>
 *  Nick Wu <fwu@hxgpt.com>
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
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/ftrace.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include "irqchip.h"

/* gpt PIC implementation */

struct gpt_pic_dev {
	struct irq_chip chip;
	struct irq_domain *root_domain;
	unsigned long flags;
};

/* Disable (mask) interrupt */
static void gpt_pic_mask(struct irq_data *data)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	if (hwirq == IX_TIMER_COUNT0)
	  sprset_gpt(TCD0C, sprget_gpt(TCD0C) & ~0x2);
}

/* Enable (unmask) interrupt */
static void gpt_pic_unmask(struct irq_data *data)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	if (hwirq == IX_TIMER_COUNT0)
	  sprset_gpt(TCD0C, sprget_gpt(TCD0C) | 0x2);
}

static void gpt_pic_ack(struct irq_data *data)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	/* Clear operation has no effect on external irqs. */
	if (hwirq < IX_SGI_0)
	  sprset_gpt(XCLR, 1UL << hwirq);
}

static void gpt_pic_mask_ack(struct irq_data *data)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(data);
	if (hwirq == IX_TIMER_COUNT0)
	  sprset_gpt(TCD0C, sprget_gpt(TCD0C) & ~0x2);
	/* Clear operation has no effect on external irqs. */
	if(hwirq < IX_SGI_0)
		sprset_gpt(XCLR, 1UL << hwirq);
}

static struct gpt_pic_dev gpt_pic = {
	.chip = {
		.name = "GPT-PIC",
		//.irq_unmask = gpt_pic_unmask,
		//.irq_mask = gpt_pic_mask,
		//.irq_ack = gpt_pic_ack,
		//.irq_mask_ack = gpt_pic_mask_ack,
	},
};

/* Main entry point to service hardware interrupt */
static void __irq_entry gpt_pic_handle_irq(struct pt_regs *regs, unsigned int irq)
{
	struct gpt_pic_dev *pic = &gpt_pic;
	
	handle_domain_irq(pic->root_domain, irq, regs);
}

static int gpt_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	struct gpt_pic_dev *pic = d->host_data;

	irq_set_percpu_devid(irq);
	irq_set_chip_and_handler(irq, &pic->chip, handle_percpu_devid_irq);
	irq_set_status_flags(irq, IRQ_NOAUTOEN);

	return 0;
}

static const struct irq_domain_ops gpt_irq_domain_ops = {
	.xlate = NULL,  // Keep ISA interrupt number unchanged.
	.map = gpt_map,
};

/*
 * This sets up the IRQ domain for the PIC built in to the GPT
 * CPU.  This is the "root" domain as these are the interrupts
 * that directly trigger an exception in the CPU.
 */
static int __init gpt_pic_init(struct device_node *node, struct device_node *parent)
{
	struct gpt_pic_dev *pic = &gpt_pic;
	unsigned int i;
	
	pic->root_domain = irq_domain_add_linear(node, NR_IRQS, &gpt_irq_domain_ops, pic);
        if (!pic->root_domain) {
                pr_err("gpt_pic: Unable to create IRQ domain\n");
                return -ENODEV;
        }

	for (i = 0; i < IX_XPEN_MAX; i++)
	  set_handle_irq(gpt_pic_handle_irq, i);

	return 0;
}

IRQCHIP_DECLARE(gpt_pic, "gpt,gpt-pic", gpt_pic_init);
