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

#ifndef _ASM_GPT_THREAD_INFO_H
#define _ASM_GPT_THREAD_INFO_H

#ifdef __KERNEL__

#ifndef __ASSEMBLY__
#include <asm/types.h>
#include <asm/processor.h>
#endif


/* THREAD_SIZE is the size of the task_struct/kernel_stack combo.
 * normally, the stack is found by doing something like p + THREAD_SIZE
 * in or32, a page is 8192 bytes, which seems like a sane size
 */

#define THREAD_SIZE_ORDER 0
#define THREAD_SIZE       (PAGE_SIZE << THREAD_SIZE_ORDER)
#define THREAD_START_SP         (THREAD_SIZE - 16)

/*
 * low level task data that entry.S needs immediate access to
 * - this struct should fit entirely inside of one cache line
 * - this struct shares the supervisor stack pages
 * - if the contents of this structure are changed, the assembly constants
 *   must also be changed
 */
#ifndef __ASSEMBLY__

typedef unsigned long mm_segment_t;

struct thread_info {
	volatile unsigned long		flags;		/* low level flags */
	mm_segment_t		addr_limit;	/* address limit */
	struct task_struct	*task;		/* main task structure */
	struct exec_domain	*exec_domain;	/* execution domain */
	struct restart_block	restart_block;
	int			preempt_count;	/* 0 => preemptable, <0 => bug */
	int			cpu;		/* cpu */
};

#define INIT_THREAD_INFO(tsk)						\
{									\
	.task		= &tsk,						\
	.exec_domain	= &default_exec_domain,				\
	.flags		= 0,						\
	.preempt_count	= INIT_PREEMPT_COUNT,				\
	.addr_limit	= KERNEL_DS,					\
	.restart_block	= {						\
		.fn	= do_no_restart_syscall,			\
	},								\
}

#define init_thread_info        (init_thread_union.thread_info)

/* Stack pointer for current thread in $asp */
register unsigned long current_stack_pointer asm("$asp");

static inline struct thread_info *current_thread_info(void) __attribute_const__;

static inline struct thread_info *current_thread_info(void)
{
	return (struct thread_info *)
		(current_stack_pointer & ~(THREAD_SIZE - 1));
}

#define get_thread_info(ti) get_task_struct((ti)->task)
#define put_thread_info(ti) put_task_struct((ti)->task)

#define thread_saved_pc(tsk)    \
        ((unsigned long)(tsk->thread.cpu_context.pc))
#define thread_saved_sp(tsk)    \
        ((unsigned long)(tsk->thread.cpu_context.sp))
#define thread_saved_fp(tsk)    \
        ((unsigned long)(tsk->thread.cpu_context.a1))

#endif /* !__ASSEMBLY__ */

/*
 * thread information flags
 *   these are process state flags that various assembly files may need to
 *   access
 *   - pending work-to-be-done flags are in LSW
 *   - other flags in MSW
 */
#define TIF_SYSCALL_TRACE       0       /* syscall trace active */
#define TIF_NOTIFY_RESUME       1       /* resumption notification requested */
#define TIF_SIGPENDING          2       /* signal pending */
#define TIF_NEED_RESCHED        3       /* rescheduling necessary */
#define TIF_FOREIGN_FPUSTATE	4	/* FPU state is not current task's */
#define TIF_FOREIGN_VPUSTATE	5	/* VPU state is not current task's */
#define TIF_SINGLESTEP  	6       /* restore singlestep on return to user mode */

#define TIF_SYSCALL_TRACEPOINT  8       /* for ftrace syscall instrumentation */
#define TIF_RESTORE_SIGMASK     9
#define TIF_FAULT_TYPE		12	/* tlb fault type */
#define TIF_POLLING_NRFLAG      16
#define TIF_MEMDIE              17
#define TIF_SWITCH_MM		18	/* deferred switch_mm */

#define _TIF_SYSCALL_TRACE      (1<<TIF_SYSCALL_TRACE)
#define _TIF_NOTIFY_RESUME      (1<<TIF_NOTIFY_RESUME)
#define _TIF_SIGPENDING         (1<<TIF_SIGPENDING)
#define _TIF_NEED_RESCHED       (1<<TIF_NEED_RESCHED)
#define _TIF_SINGLESTEP         (1<<TIF_SINGLESTEP)
#define _TIF_POLLING_NRFLAG     (1<<TIF_POLLING_NRFLAG)
#define _TIF_FOREIGN_FPUSTATE	(1<<TIF_FOREIGN_FPUSTATE)
#define _TIF_FOREIGN_VPUSTATE	(1<<TIF_FOREIGN_VPUSTATE)

/* Work to do when returning from interrupt/exception */
/* For OpenRISC, this is anything in the LSW other than syscall trace */
#define _TIF_WORK_MASK          (0xff & ~(_TIF_SYSCALL_TRACE|_TIF_SINGLESTEP))
#define _TIF_ALLWORK_MASK       0x0000FFFF      /* work to do on any return to u-space */

#endif /* __KERNEL__ */

#endif /* _ASM_GPT_THREAD_INFO_H */
