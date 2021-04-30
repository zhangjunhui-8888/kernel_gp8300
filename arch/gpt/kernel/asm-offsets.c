/*
 * arch/gpt/kernel/asm-offsets.c
 *
 * This program is used to generate definitions needed by
 * assembly language modules. Kbuild generates
 * include/generated/asm-offsets.h from this source.
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

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/thread_info.h>
#include <linux/kbuild.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/processor.h>
#include <asm/vdso_datapage.h>

int main(void)
{
        /* offsets into the task_struct in ~/include/linux/sched.h */
        DEFINE(TASK_STATE, offsetof(struct task_struct, state));
        DEFINE(TASK_FLAGS, offsetof(struct task_struct, flags));
        DEFINE(TASK_PTRACE, offsetof(struct task_struct, ptrace));
        DEFINE(TASK_THREAD, offsetof(struct task_struct, thread));
        DEFINE(TASK_MM, offsetof(struct task_struct, mm));
        DEFINE(TASK_ACTIVE_MM, offsetof(struct task_struct, active_mm));
        DEFINE(TASK_STACK, offsetof(struct task_struct, stack));
	DEFINE(TASK_THREAD_CPU_CONTEXT,	offsetof(struct task_struct, thread.cpu_context));

        /* offsets into thread_info in ~/arch/gpt/include/asm/thread_info.h */
        DEFINE(TI_TASK, offsetof(struct thread_info, task));
        DEFINE(TI_FLAGS, offsetof(struct thread_info, flags));
        DEFINE(TI_PREEMPT, offsetof(struct thread_info, preempt_count));

        /* offsets to pt_regs data structure in ~/arch/gpt/include/asm/ptrace.h */
        DEFINE(PT_regs_size, sizeof(struct pt_regs));
        DEFINE(PT_sp, offsetof(struct pt_regs, sp));
        DEFINE(PT_pc, offsetof(struct pt_regs, pc));
        DEFINE(PT_sr, offsetof(struct pt_regs, sr));
        DEFINE(PT_ca, offsetof(struct pt_regs, ca));        
        DEFINE(PT_xen, offsetof(struct pt_regs, xen));        
        DEFINE(PT_a0, offsetof(struct pt_regs, a_regs[0]));
        DEFINE(PT_a1, offsetof(struct pt_regs, a_regs[1]));
        DEFINE(PT_a8, offsetof(struct pt_regs, a_regs[8]));
        DEFINE(PT_a10, offsetof(struct pt_regs, a_regs[10]));        
        DEFINE(PT_a15, offsetof(struct pt_regs, a_regs[15]));
        DEFINE(PT_r8, offsetof(struct pt_regs, r_regs[8]));
        DEFINE(PT_r9, offsetof(struct pt_regs, r_regs[9]));        
        DEFINE(PT_r15, offsetof(struct pt_regs, r_regs[15]));        
        DEFINE(PT_t0, offsetof(struct pt_regs, t_regs[0]));
        DEFINE(PT_t1, offsetof(struct pt_regs, t_regs[1]));
        DEFINE(PT_t4, offsetof(struct pt_regs, t_regs[4]));
        DEFINE(PT_t7, offsetof(struct pt_regs, t_regs[7]));
        DEFINE(PT_r8_orig, offsetof(struct pt_regs, r8_orig));
        DEFINE(PT_syscallno, offsetof(struct pt_regs, syscallno));

        /* offsets into vfp_state in ~/arch/gpt/include/asm/vfp.h:46 */
        DEFINE(VFP_n0,offsetof(struct vfp_state,n_regs[0]));
        DEFINE(VFP_m0,offsetof(struct vfp_state,m_regs[0]));
        DEFINE(VFP_v0,offsetof(struct vfp_state,v_regs[0]));
        DEFINE(VFP_f0,offsetof(struct vfp_state,f_regs[0]));

        DEFINE(NUM_USER_SEGMENTS, TASK_SIZE >> 28);
	
	DEFINE(VDSO_CS_CYCLE_LAST,	offsetof(struct vdso_data, cs_cycle_last));
	DEFINE(VDSO_XTIME_CLK_SEC,	offsetof(struct vdso_data, xtime_clock_sec));
	DEFINE(VDSO_XTIME_CLK_NSEC,	offsetof(struct vdso_data, xtime_clock_nsec));
	DEFINE(VDSO_XTIME_CRS_SEC,	offsetof(struct vdso_data, xtime_coarse_sec));
	DEFINE(VDSO_XTIME_CRS_NSEC,	offsetof(struct vdso_data, xtime_coarse_nsec));
	DEFINE(VDSO_WTM_CLK_SEC,	offsetof(struct vdso_data, wtm_clock_sec));
	DEFINE(VDSO_WTM_CLK_NSEC,	offsetof(struct vdso_data, wtm_clock_nsec));
	DEFINE(VDSO_TB_SEQ_COUNT,	offsetof(struct vdso_data, tb_seq_count));
	DEFINE(VDSO_CS_MULT,		offsetof(struct vdso_data, cs_mult));
	DEFINE(VDSO_CS_SHIFT,		offsetof(struct vdso_data, cs_shift));
	DEFINE(VDSO_TZ_MINWEST,	offsetof(struct vdso_data, tz_minuteswest));
	DEFINE(VDSO_TZ_DSTTIME,	offsetof(struct vdso_data, tz_dsttime));
	DEFINE(VDSO_USE_SYSCALL,	offsetof(struct vdso_data, use_syscall));

        DEFINE(NR_CPUS, NR_CPUS);

        return 0;
}
