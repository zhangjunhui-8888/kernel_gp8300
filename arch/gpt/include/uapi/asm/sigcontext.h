/*
 * Copyright (C) 2016 GPT Ltd.
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
#ifndef __ASM_SIGCONTEXT_H
#define __ASM_SIGCONTEXT_H

#include <linux/types.h>

/*
 * Signal context structure - contains all info to do with the state
 * before the signal handler was invoked.
 */
struct sigcontext {
	__u64 fault_address;
	__u64 r_regs[16];
	__u64 t_regs[3];	/* Now only $t0/1/4/7 are used. */
        __u64 ra;	
	__u64 pc;
        __u64 ca;         	/* It is used to save carry_flag*/
	__u64 sp;
	__u64 a_regs[15];
	/* 16K reserved for FPU/VPU state and future expansion */
	__u8 __reserved[1024 *16] __attribute__((__aligned__(8)));
};

/*
 * Header to be used at the beginning of structures extending the user
 * context. Such structures must be placed after the rt_sigframe on the stack
 * and be 8-byte aligned. The last structure must be a dummy one with the
 * magic and size set to 0.
 */
struct _gpt_ctx {
	__u32 magic;
	__u32 size;
};

#define FPU_MAGIC	0x46508001

struct fpu_context {
	struct _gpt_ctx head;
	__u64 fpsr;
	__u64 fpcr;
	__u64 f_regs[16];
};

#define VPU_MAGIC	0x45535201

/* VPU has the following registers.
   8 1KB-byte vector registers, $v0~$v7  --- Total 1024*8*8 bits (1024 __u64)
   4 512-bit mask registers,    $m0~$m3  --- Total 512*4 bits    (32 __u64)
   8 16-bit length registers,   $n0~$n7. */ 
struct vpu_context {
	struct _gpt_ctx head;
	__u64 vfsr;
	__u64 vfcr;
        __u16 n_regs[8];
        __u64 m_regs[32];
        __u64 v_regs[1024];
};

#endif /* __ASM_SIGCONTEXT_H */
