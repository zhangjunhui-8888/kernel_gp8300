/*
 * arch/drivers/irqchip/irq-gpt-mpic.c
 *
 * GPT Multi-Processor Interrupt Controller Driver.
 *
 * Copyright (C) 2017, Optimum Semiconductor Technologies
 * Copyright (C) 2017, HXGPT Inc.
 *  Nick Wu <fwu@hxgpt.com>
 * Copyright (C) 2019, HXGPT Inc.
 *  scxie <scxie@hxgpt.com>
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
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqchip/gpt-mpic.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 3, 0)
#include <linux/irqchip.h>
#else
#include "irqchip.h"
#endif

struct gpt_mpic {
	void __iomem *base;
	struct irq_domain *domain;
	unsigned long core[GPT_MPIC_NR_IRQS];
	unsigned long type[GPT_MPIC_NR_IRQS]; /* 0: irq; 1: fiq */
};

static struct gpt_mpic gpt_mpic;

#ifdef CONFIG_SMP
void __iomem *gpt_mpic_sgi_clear = NULL;
void __iomem *gpt_mpic_sgi_set = NULL;
#endif

static DEFINE_RAW_SPINLOCK(irq_controller_lock);

static inline u32 mpic_readreg(void __iomem *base, loff_t offset)
{
        return readl(base + offset);
}

static inline u64 mpic_read_dreg(void __iomem *base, loff_t offset)
{
	u64 data, tmp;
	void *addr = base + offset + 4;
	asm volatile (
	"fence\n"
	"lduw	%1, %2, -4\n"
	"shli	%1, %1, 32\n"
	"ldw	%0, %2\n"
	"or	%0, %0, %1\n"
	:"=r"(data), "=r"(tmp), "+a"(addr)
	::"memory");
	return data;
}

static inline void mpic_writereg(void __iomem *base, loff_t offset, u32 data)
{
        writel(data, base + offset);
}

static inline void raw_mpic_enable(void *base, int hwirq, unsigned long cpu)
{
	u32 val, offset;

	offset = hwirq >> 5 << 2;
	offset += GPT_MPIC_IENL(cpu);
		
	hwirq &= 0x1F;
	val = mpic_readreg(base, offset) | (1 << hwirq);
	mpic_writereg(base, offset, val);
}

static inline void raw_mpic_disable(void *base, int hwirq, unsigned long cpu)
{
	u32 val, offset;

	offset = hwirq >> 5 << 2;
	offset += GPT_MPIC_IENL(cpu);
		
	hwirq &= 0x1F;
	val = mpic_readreg(base, offset) & ~(1 << hwirq);
	mpic_writereg(base, offset, val);
}

static inline void raw_mpic_set_type(void *base, int hwirq, unsigned long cpu, unsigned long type)
{
	u32 val, offset;

	offset = hwirq >> 5 << 2;
	offset += GPT_MPIC_ITYPEL(cpu);
	val = mpic_readreg(base, offset);

	hwirq &= 0x1F;
	val = type ? val | (1 << hwirq) : val & ~(1 << hwirq);
	mpic_writereg(base, offset, val);
}

static inline u64 raw_mpic_get_state(void *base, unsigned long cpu, unsigned long type)
{
	u64 offset;
	offset = GPT_MPIC_MFIQL(cpu);
	offset += !type ? GPT_MPIC_MIRQL(0) - GPT_MPIC_MFIQL(0) : 0;
	return mpic_read_dreg(base, offset);
}

static inline unsigned int raw_mpic_get_hwirq(void *base, unsigned long cpu)
{
	u32 offset;
	offset = GPT_MPIC_VADDR(cpu);
	return mpic_readreg(base, offset);	
}

#ifdef CONFIG_SMP
static int gpt_mpic_set_affinity(struct irq_data *d,
				 const struct cpumask *mask_val, bool force)
{
	unsigned long old_core, new_core;
	unsigned long type;

	struct gpt_mpic *mpic = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	/* Select a single core from the affinity mask which is online */
	if (!force) {
		new_core = cpumask_any_and(mask_val, cpu_online_mask);
	}else {
		new_core = cpumask_first(mask_val);
	}

	// TODO: If set affinity is called inside of spin lock protection,
	// irq_controller_lock should be removed.
	raw_spin_lock(&irq_controller_lock);

	type = mpic->type[hwirq];
	old_core = mpic->core[hwirq];
	mpic->core[hwirq] = new_core;

	raw_spin_unlock(&irq_controller_lock);

	raw_mpic_set_type(mpic->base, hwirq, new_core, type);
	raw_mpic_disable(mpic->base, hwirq, old_core);
	raw_mpic_enable(mpic->base, hwirq, new_core);

	return 0;
}

static inline u8 gpt_mpic_get_sgi_stat(void *base, u32 cpu)
{
	u32 offset;
	offset = GPT_MPIC_SGI_STAT(cpu);	
	return mpic_readreg(base, offset);
}

static inline void gpt_mpic_clr_sgi_stat(void *base, u32 cpu, u8 val)
{
	u32 offset;
	offset = GPT_MPIC_SGI_STAT(cpu);
	mpic_writereg(base, offset, val);
}

static inline void gpt_mpic_set_sgi(void *base, u32 sgi_id, u32 src_cpu, u32 dst_cpumap, u16 param)
{
	u32 offset, val;
	val = (sgi_id << GPT_MPIC_SGI_SET_OFF_ID) |
	      (dst_cpumap << GPT_MPIC_SGI_SET_OFF_CPU) | 
	      param;
	offset = GPT_MPIC_SGI_SET(src_cpu);
	mpic_writereg(base, offset, val);
}

static inline u32 gpt_mpic_get_sgi_param(void *base, u32 cpu, u32 sgi)
{
	u32 offset = GPT_MPIC_SGI_PARAM(cpu, sgi);
	return mpic_readreg(base, offset);
}

static void gpt_mpic_raise_softirq(const struct cpumask *mask, unsigned int ipi_msg)
{	
	u32 dst_cpu, dst_cpumap = 0;
	u32 src_cpu = smp_processor_id();
	struct gpt_mpic *mpic = &gpt_mpic;
	u64 stat;
	unsigned long flags;
	unsigned int msg;

	// To avoid the use of spin lock, use core respective channel.
	local_irq_save(flags);
	for_each_cpu(dst_cpu, mask) {
	  stat = gpt_mpic_get_sgi_stat(mpic->base, dst_cpu);
	  msg = 1 << ipi_msg;
	  if ((stat & (1 << (src_cpu + 1))))
	    msg |= gpt_mpic_get_sgi_param(mpic->base, dst_cpu, src_cpu + 1);
	  gpt_mpic_set_sgi(mpic->base, src_cpu + 1, src_cpu, 1UL << dst_cpu, msg);
	}
	local_irq_restore(flags);
}

/* Main entry point to service ipi interrupt */
static void gpt_mpic_handle_sgi(struct pt_regs *regs, unsigned int irq)
{
	u64 stat, sgi;
	struct gpt_mpic *mpic = &gpt_mpic;
	unsigned int cpu = smp_processor_id();
	unsigned long ipi_msg[4] = {0};
	unsigned int msg, i, param;

	struct pt_regs *old_regs;	

	WARN_ON(!irqs_disabled());

	while ((stat = gpt_mpic_get_sgi_stat(mpic->base, cpu)) & 0xFE) {
	  sgi = __ffs(stat & 0xFE);
	  param=gpt_mpic_get_sgi_param(mpic->base, cpu, sgi);
	  BUG_ON(!param);
	  ipi_msg[sgi-1] |= param;
	  gpt_mpic_clr_sgi_stat(mpic->base, cpu, 1 << sgi);
	  BUG_ON (sgi == 0 || ipi_msg[sgi-1]==0);
	}
	for(i=0; i < NR_CPUS; i++){
		while (ipi_msg[i] != 0) {
		  old_regs = set_irq_regs(regs);
		  msg = __ffs(ipi_msg[i]);
		  handle_IPI(msg);
		  ipi_msg[i] ^= 1 << msg;
		  set_irq_regs(old_regs);
		}
	}
}

#endif

/* Disable (mask) interrupt */
static void gpt_mpic_disable_or_mask(struct irq_data *data)
{
	struct gpt_mpic *mpic = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);

	raw_mpic_disable(mpic->base, hwirq, mpic->core[hwirq]);
}

/* Enable (unmask) interrupt */
static void gpt_mpic_enable_or_unmask(struct irq_data *data)
{
	struct gpt_mpic *mpic = irq_data_get_irq_chip_data(data);
	irq_hw_number_t hwirq = irqd_to_hwirq(data);

	raw_mpic_enable(mpic->base, hwirq, mpic->core[hwirq]);
}

static void gpt_mpic_eoi(struct irq_data *data)
{
}

static struct irq_chip mpic_chip = {
	.name = "GPT-MPIC",
	.irq_mask = gpt_mpic_disable_or_mask,
	.irq_unmask = gpt_mpic_enable_or_unmask,
	.irq_eoi = gpt_mpic_eoi,
#ifdef CONFIG_SMP
	.irq_set_affinity = gpt_mpic_set_affinity,
#endif
};

/* Main entry point to service hardware interrupt */
static void gpt_mpic_handle_irq(struct pt_regs *regs, unsigned int irq)
{
	u64 irqstate;
	struct gpt_mpic *mpic = &gpt_mpic;
	unsigned int cpu = smp_processor_id();
	unsigned int rawirq;

	while ((irqstate = raw_mpic_get_state(mpic->base, cpu, 0)))
	{
	  rawirq = mpic_readreg(mpic->base, GPT_MPIC_VADDR(cpu));
	  handle_domain_irq(mpic->domain, rawirq, regs);
	}
}

static void gpt_mpic_handle_fiq(struct pt_regs *regs, unsigned int irq)
{
	u64 irqstate;
	struct gpt_mpic *mpic = &gpt_mpic;
	unsigned int cpu = smp_processor_id();
	unsigned int rawirq;

	while ((irqstate = raw_mpic_get_state(mpic->base, cpu, 1)))
	{
	  rawirq = __ffs(irqstate);
	  handle_domain_irq(mpic->domain, rawirq, regs);
	}
}

static int gpt_mpic_irq_domain_xlate(struct irq_domain *d,
				     struct device_node *controller,
				     const u32 *intspec, unsigned int intsize,
				     unsigned long *out_hwirq, unsigned int *out_type)
{
	int ret = 0;
	unsigned long type;
	unsigned long core;
	unsigned long irq;

	struct gpt_mpic *mpic = d->host_data;

	if (d->of_node != controller || intsize !=3) {
		return -EINVAL;
	}
	/* Check the correct irq cell */
	/* We can support 3 cells: cpu core num, type(irq/fiq), irqno */
	core  = intspec[0];
	type  = !!intspec[1];
	irq   = intspec[2];

	if (core >= num_possible_cpus() || irq >= GPT_MPIC_NR_IRQS) {
		return -EINVAL;
	}

	*out_hwirq = irq;
	*out_type  = IRQ_TYPE_NONE;

	mpic->core[irq] = core;
	mpic->type[irq] = type;

	raw_mpic_set_type(mpic->base, irq, core, type);

	pr_debug("mpic set irq: cpu: %lu, type: %lu, irq: %lu\n", core, type, irq);
	return ret;
}

static int gpt_mpic_irq_domain_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &mpic_chip, handle_fasteoi_irq);
	irq_set_status_flags(irq, IRQ_TYPE_NONE);
	irq_set_chip_data(irq, d->host_data);
	return 0;
}

static const struct irq_domain_ops gpt_mpic_irq_domain_ops = {
	.xlate = gpt_mpic_irq_domain_xlate,
	.map = gpt_mpic_irq_domain_map,
};

static inline void gpt_mpic_percpu_init(void) {
	struct gpt_mpic *mpic = &gpt_mpic;
	unsigned int cpu = smp_processor_id();

	/*
	 * Disable all external interrupts until they are
	 * explicity requested.
	 */
	writel(0, mpic->base + GPT_MPIC_IENL(cpu));
	writel(0, mpic->base + GPT_MPIC_IENH(cpu));
	writel(0, mpic->base + GPT_MPIC_ITYPEL(cpu));
	writel(0, mpic->base + GPT_MPIC_ITYPEH(cpu));
#ifdef CONFIG_SMP
	gpt_mpic_clr_sgi_stat(mpic->base, cpu, 0xFF);
#endif
}

#ifdef CONFIG_SMP
static int gpt_pic_secondary_init(struct notifier_block *nfb,
			      unsigned long action, void *hcpu)
{
	if (action == CPU_STARTING || action == CPU_STARTING_FROZEN)
	  gpt_mpic_percpu_init();
	
		
	return NOTIFY_OK;
}
static struct notifier_block gpt_pic_cpu_notifier = {
	.notifier_call = gpt_pic_secondary_init,
	.priority = 100,
};
#endif

/*
 * This sets up the IRQ domain for the MPIC built in to the GPT
 * CPU.  This is the "root" domain as these are the interrupts
 * that directly trigger an exception in the CPU.
 */
static int __init gpt_mpic_init(struct device_node *node, struct device_node *parent)
{
	struct gpt_mpic *mpic;
	unsigned int irq;
	int i, nrirqs;

	if (WARN_ON(!node)) {
		return -ENODEV;
	}

	mpic = &gpt_mpic;

	mpic->base = of_iomap(node, 0);
	if (!mpic->base) {
	  pr_err("%s: Unable to map memory\n", node->full_name);
	  return -ENOMEM;
	}


	/* The purpose of VADDR_ISRC is to save the physical address of irq handler,
	 * but it is not expected in kernel, instead we save irq number.
	 */
	for (irq = 0; irq < GPT_MPIC_NR_IRQS; irq++) {
	  writel(irq, mpic->base + GPT_MPIC_VADDR_ISRC(irq));
	}

	gpt_mpic_percpu_init();

	pr_debug("%s: num_irq=%d\n", node->full_name, GPT_MPIC_NR_IRQS);

	mpic->domain = irq_domain_add_linear(node, GPT_MPIC_NR_IRQS,
					     &gpt_mpic_irq_domain_ops, mpic);
	if (WARN_ON(!mpic->domain)) {
		goto error_iounmap;
	}

	irq_set_default_host(mpic->domain);

	/*
	 * Check if this interrupt controller is a chained interrupt controller
	 * and if so then set up the handler and enable it.
	 */
	nrirqs = of_irq_count(node);
	  // This is the root MPIC. It will connected to FIQ and IRQ, and also
	  // supply SGIs. "node->full_name" chained MPIC not implemented, parent is "parent->full_name"
	  for (i = 0; i < nrirqs; i++) {
	    irq = irq_of_parse_and_map(node, i);
	    if (irq < IX_SGI_0 || irq >= IX_XPEN_MAX) {
	      pr_err("%s: unable to connet to the external interrupt %d\n", node->full_name, irq);
	      goto errot_domain;
	    }
	    irq_set_handler_data(irq, mpic);
	    if (i == 0) {
	      pr_info("%s: root MPIC FIQ connected to %d\n", node->full_name, irq);
	      set_handle_irq(gpt_mpic_handle_fiq, irq);
	    } else if (i == 1) {
	      pr_info("%s: root MPIC IRQ connected to %d\n", node->full_name, irq);
	      set_handle_irq(gpt_mpic_handle_irq, irq);
#ifdef CONFIG_SMP
	    } else {
	      pr_info("%s: root MPIC SGI%d connected to %d\n", node->full_name, i - 2, irq);
	      set_handle_irq(gpt_mpic_handle_sgi, irq);
#endif
	    }
	  }
#if 0
	} else {
	// TODO: Implement chained MPICs.
	  goto errot_domain;
	}
#endif
	
#ifdef CONFIG_SMP
	gpt_mpic_sgi_clear = mpic->base + GPT_MPIC_SGI_STAT(0);
	gpt_mpic_sgi_set = mpic->base + GPT_MPIC_SGI_SET(0);
	register_cpu_notifier(&gpt_pic_cpu_notifier);
	set_smp_cross_call(gpt_mpic_raise_softirq);
#endif	
	return 0;
	
errot_domain:
	irq_domain_remove(mpic->domain);
error_iounmap:
	iounmap(mpic->base);

	return -ENODEV;
}

IRQCHIP_DECLARE(gpt_mpic, "gpt,gpt-mpic", gpt_mpic_init);
