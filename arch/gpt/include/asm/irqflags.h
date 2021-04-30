/*
 * arch/gpt/include/asm/irqflags.h
 *
 * GPT irq low level procedures.
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

#ifndef ___ASM_GPT_IRQFLAGS_H
#define ___ASM_GPT_IRQFLAGS_H

#include <machine/gpt_mach.h>

#ifndef __ASSEMBLY__
#define _ONE 1UL
#else
#define _ONE 1
#endif

#define IX_MC_EXCEPTIONS ((_ONE << IX_INSN_TLB) | (_ONE << IX_DATA_TLB) | (_ONE << IX_SGI_0) | (_ONE << 0) | (_ONE << 63) | (_ONE << IX_DBG_STEP) | (_ONE << IX_DBG_BRANCH) | (_ONE << IX_DBG_INSN) | (_ONE << IX_DBG_DATA))
#define IX_LEV1_EXCEPTIONS (-_ONE & ~((_ONE << IX_INSN_TLB) | (_ONE << IX_DATA_TLB) | (_ONE << 0) | (_ONE << 63) | (_ONE << IX_SGI_0) | (_ONE << IX_DBG_STEP)))
#define DEFAULT_XEN (((_ONE << IX_TIMER_COUNT0) | (_ONE << IX_MPINT_FIQ) | (_ONE << IX_MPINT_IRQ) | (_ONE << IX_GPTDMA) | (_ONE << IX_SGI_1) | (_ONE << IX_SGI_2) | ( _ONE << IX_SGI_3) | (_ONE << IX_SGI_4)) & (~(_ONE << IX_SGI_0)))

#define TRAP_NUMS               64

#define TRAP_MASK		((_ONE << TRAPM_OFFSE_interrupt_0) |	\
                                 (_ONE << TRAPM_OFFSE_interrupt_x) |	\
                                 (TRAP_NUMS - 1))

#define TIMER0_ENABLE_IRQ       (_ONE << IX_TIMER_COUNT0)

#define XEN_EXTERNAL_IRQ_MASK   (~((_ONE << (IX_MAX + 1)) - 1))

#ifndef __ASSEMBLY__
#include <asm/spr.h>

static inline unsigned long arch_local_save_flags(void)
{
        return sprget_gpt(XEN);
}
/* NOTE:
 *	we use below code to do local_irq_save(),
 *	a. rsetspr	\rr, PSC
 *	b. ori		\rr, \rr, (1<<PSC_OFFSE_interrupt_x)
 *	c. sprsetr	\rr, PSC
 * 	A interrupt may happen after stpe (a), 
 *	and a context switch may happend in the interrupt context,
 *	the PROCID may be changed, so we disable interrupt by CPUC first.
 */
static inline unsigned long arch_local_irq_save(void)
{
	unsigned long flags;
	flags = sprget_gpt(XEN);
	sprset_gpt(XEN, 0);
	return flags;
}

static inline void arch_local_irq_restore(unsigned long flags)
{
	sprset_gpt(XEN, flags);
}

static inline int arch_irqs_disabled(void)
{
	return sprget_gpt(XEN) == 0;
}

static inline int arch_irqs_disabled_flags(unsigned long flags)
{
        return flags == 0;
}

static inline void arch_local_irq_enable(void)
{
	sprset_gpt(XEN, DEFAULT_XEN);
}

static inline void arch_local_irq_disable(void)
{
	sprset_gpt(XEN, 0);
}
#else
#include <asm/asm-defs.h>
#include <asm/ptrace.h>

.macro enable_exception rr
rsetspr		\rr, PSC
ori		\rr, \rr, (1<<PSC_OFFSE_interrupt_1) | (1<< PSC_OFFSE_interrupt_x)
sprsetr		\rr, PSC
.endm

.macro disable_exception rr
rsetspr		\rr, PSC
andi		\rr, \rr, ~((1<<PSC_OFFSE_interrupt_1) | (1<< PSC_OFFSE_interrupt_x))
sprsetr		\rr, PSC
.endm

.macro enable_irq rr1
rsetli		\rr1, DEFAULT_XEN
sprsetr		\rr1, XEN
.endm

.macro disable_irq rr1
rseti		\rr1, 0
sprsetr		\rr1, XEN
.endm
#endif /* __ASSEMBLY__ */
#endif /* ___ASM_GPT_IRQFLAGS_H */
