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
