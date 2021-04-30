/*
 * gpt Linux
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * gpt implementation:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef __ASM_GPT_PTRACE_H
#define __ASM_GPT_PTRACE_H


#include <uapi/asm/ptrace.h>

/* CPU run mode
 * NOTE: PRIV and MEM bits should be same
 * bit63  0- hypervisor 1- kernel(reserved for kvm)
 * bit62  0- N/A        1- user
 * bit61  0- hyper mem  1- kernel memory permission
 * bit60  0- N/A        1- user memory permission
 */
#define         PSC_MODE_MC     0x0000000000000001              // PSC_MC_BIT
#define         PSC_MODE_PRIV   0x0300000000000005              // (PSC_IT_BIT|PSC_DT_BIT|PSC_MC_BIT|PSC_EL1_BIT)
#define         PSC_MODE_KERNEL 0x030000000000000d              // (PSC_IT_BIT|PSC_DT_BIT|PSC_MC_BIT|PSC_EL1_BIT|PSC_INT_BIT)
#define         PSC_MODE_USER   0x530000000000000d              // (PSC_IT_BIT|PSC_DT_BIT|PSC_MC_BIT|PSC_EL1_BIT|PSC_INT_BIT|PSC_USER_BIT)

#ifndef CONFIG_GPT_OUT_OF_ORDER_OFF
#define         CPUC_DEFAULT    (-1)
#else
/* Bit-24 must be zero to disable OO */
#define         CPUC_DEFAULT    (0xFFFFFFFFFEFFFFFF)
#endif

#ifndef __ASSEMBLY__
/*
 * The struct pt_regs describes how the registers are laid out on the kernel stack
 * during a syscall or other kernel entry.
 */
struct pt_regs {        
	union {
		struct user_pt_regs user_regs;	
		struct{
			u64 r_regs[16];
			u64 a_regs[16];
			u64 t_regs[8];	/* Now only $t0/1/4/7 are used. */
			u64 sp;
			u64 pc;
			u64 sr;
			u64 ca;		/* It is used to save carry_flag */
		};
	};
        /* Misc registers to help low level processing */
	u64 ixi;
	u64 xen;
        u64 r8_orig;			/* original return value of system call */
        u64 syscallno;			/* original $r7 */	        
};

#define instruction_pointer(regs)       ((regs)->pc)
#define user_mode(regs)                 (((regs)->sr & (1UL << PSC_OFFSE_mem_priv)) != 0)
#define user_stack_pointer(regs)        ((unsigned long)(regs)->sp)
#define interrupts_enabled(regs)        ((regs)->xen != 0)

static inline unsigned long regs_return_value(struct pt_regs *regs)
{
    return regs->r_regs[8];
}

#ifdef	CONFIG_HAVE_HW_BREAKPOINT
#define arch_has_single_step()	(1)
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ASM_GPT_PTRACE_H */
