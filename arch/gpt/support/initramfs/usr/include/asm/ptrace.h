/*
 * Copyright (C) 2017 GPT Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_PTRACE_H
#define __ASM_PTRACE_H

#include <linux/types.h>
#include <asm/hwcap.h>

/* CPU run mode 
 * NOTE: PRIV and MEM bits should be same
 * bit63  0- hypervisor 1- kernel(reserved for kvm)
 * bit62  0- N/A	1- user
 * bit61  0- hyper mem  1- kernel memory permission
 * bit60  0- N/A        1- user memory permission
 */
#define		PSC_MODE_MC	0x0000000000000001		// PSC_MC_BIT
#define		PSC_MODE_PRIV	0x0300000000000005		// (PSC_IT_BIT|PSC_DT_BIT|PSC_MC_BIT|PSC_EL1_BIT)
#define		PSC_MODE_KERNEL	0x030000000000000d		// (PSC_IT_BIT|PSC_DT_BIT|PSC_MC_BIT|PSC_EL1_BIT|PSC_INT_BIT)
#define		PSC_MODE_USER	0x530000000000000d		// (PSC_IT_BIT|PSC_DT_BIT|PSC_MC_BIT|PSC_EL1_BIT|PSC_INT_BIT|PSC_USER_BIT)

#define		CPUC_DEFAULT	(-1)

#ifndef __ASSEMBLY__

/*
 * User structures for general purpose, floating point and debug registers.
 */
struct user_pt_regs {
	__u64 r_regs[16];
	__u64 a_regs[16];
        __u64 t_regs[8];  /* Now only $t0/1/4/7 are used. */
	__u64 sp;
	__u64 pc;
	__u64 sr;
        __u64 ca;         /* It is used to save carry_flag*/
};

struct user_fpu_state {
	__u64 f_regs[16];
	__u64 fpsr;
	__u64 fpcr;
};

struct user_vpu_state {
        __u16 n_regs[8];
        __u64 m_regs[32];
        __u64 v_regs[1024];
	__u64 vfsr;
	__u64 vfcr;        
};

struct user_hwdebug_state {
	__u32		dbg_info;
	__u32		pad;
	struct {
		__u64	ctrl;    /*IMxE and DMxE. x=0/1.  */
		__u64	mask;    /*IMxM and DMxM. x=0/1.  */
                __u64	addr;    /*IMxC and DMxC. x=0/1.  */
	} im_regs[2], dm_regs[2]; 
};

#endif /* __ASSEMBLY__ */

#endif /* __ASM_PTRACE_H */
