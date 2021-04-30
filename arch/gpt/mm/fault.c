/*
 * gpt fault.c
 *
 * Linux architectural port borrowing liberally from similar works of
 * others.  All original copyrights apply as per the original source
 * declaration.
 *
 * Modifications for the gpt architecture:
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/ftrace.h>
#include <linux/syscalls.h>

#include <asm/uaccess.h>
#include <asm/siginfo.h>
#include <asm/signal.h>
#include <asm/tlbhw.h>
#include <asm/tlb.h>
#include <asm/insn.h>
#include <asm/irq.h>

static inline void gpt_notify_die_helper(const char *str, struct pt_regs *regs,
					 int signo, int errno, int code, unsigned long addr, int err)
{
	struct siginfo info;
	info.si_signo = signo;
	info.si_errno = errno;
	info.si_code  = code;
	info.si_addr  = (void __user *)addr;
	gpt_notify_die(str, regs, &info, err);
}

void show_pte(struct mm_struct *mm, unsigned long addr)
{
	pgd_t *pgd;

	if (!mm)
		mm = &init_mm;

	pr_alert("pgd = %p\n", mm->pgd);
	pgd = pgd_offset(mm, addr);
	pr_alert("[%08lx] *pgd=%016lx", addr, pgd_val(*pgd));

	do {
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;

		if (pgd_none(*pgd) || pgd_bad(*pgd))
			break;

		pud = pud_offset(pgd, addr);
		printk(", *pud=%016lx", pud_val(*pud));
		if (pud_none(*pud) || pud_bad(*pud))
			break;

		pmd = pmd_offset(pud, addr);
		printk(", *pmd=%016lx", pmd_val(*pmd));
		if (pmd_none(*pmd) || pmd_bad(*pmd))
			break;

		pte = pte_offset_map(pmd, addr);
		printk(", *pte=%016lx", pte_val(*pte));
		pte_unmap(pte);
	} while(0);

	printk("\n");
}

static void __do_user_fault(struct task_struct *tsk, unsigned long addr,
			    unsigned int sig, int code, struct pt_regs *regs)
{
	struct siginfo si;

	if (show_unhandled_signals && unhandled_signal(tsk, sig) &&
	    printk_ratelimit()) {
		pr_info("%s[%d]: unhandled (%d) at 0x%08lx\n",
			tsk->comm, task_pid_nr(tsk), sig, addr);
		show_regs(regs);
		dump_stack();
	}

	tsk->thread.fault_address = addr;
	tsk->thread.fault_code = code;
	si.si_signo = sig;
	si.si_errno = 0;
	si.si_code = code;
	si.si_addr = (void __user *)addr;
	force_sig_info(sig, &si, tsk);
}

static void __do_kernel_fault(struct mm_struct *mm, unsigned long addr,
			      unsigned int ixno, struct pt_regs *regs)
{
 	if (fixup_exception(regs))
 		return;
	
	bust_spinlocks(1);
	pr_alert("Unable to handle kernel %s at virtual address %016lx\n",
		 (addr < PAGE_SIZE) ? "NULL pointer dereference" :
		 "paging request", addr);

	show_pte(mm, addr);
	die("Oops", regs, ixno);
	bust_spinlocks(0);
	do_exit(SIGKILL);
}

#define VM_FAULT_BADMAP		0x010000
#define VM_FAULT_BADACCESS	0x020000
static int __do_page_fault(struct mm_struct *mm, unsigned long addr,
			   unsigned int mm_flags, unsigned long vm_flags,
			   struct task_struct *tsk, unsigned long ixno)
{
	struct vm_area_struct *vma;
	int fault;

	vma = find_vma(mm, addr);
	fault = VM_FAULT_BADMAP;
	if (unlikely(!vma))
		goto out;
	if (unlikely(vma->vm_start > addr))
		goto check_stack;

	/*
	 * Ok, we have a good vm_area for this memory access, so we can handle
	 * it.
	 */
good_area:
	/*
	 * Check that the permissions on the VMA allow for the fault which
	 * occurred. If we encountered a write or exec fault, we must have
	 * appropriate permissions, otherwise we allow any permission.
	 */
	if (!(vma->vm_flags & vm_flags)) {
		fault = VM_FAULT_BADACCESS;
		goto out;
	}
		

	return handle_mm_fault(mm, vma, addr & PAGE_MASK, mm_flags);

check_stack:
	if (vma->vm_flags & VM_GROWSDOWN && !expand_stack(vma, addr))
		goto good_area;
out:
	return fault;
}
asmlinkage int do_page_fault(struct pt_regs *regs, unsigned long addr, unsigned long ixno)
{
	struct task_struct *tsk;
	struct mm_struct *mm;
	int fault, sig, code;
	unsigned long vm_flags = VM_READ | VM_WRITE | VM_EXEC;
	unsigned int mm_flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_KILLABLE;

	tsk = current;
	mm  = tsk->mm;

	/* Enable interrupts if they were enabled in the parent context. */
	if (interrupts_enabled(regs))
		local_irq_enable();

	/*
	 * If we're in an interrupt or have no user context, we must not take
	 * the fault.
	 */
	if (in_atomic() || !mm)
		goto no_context;

	if (user_mode(regs))
		mm_flags |= FAULT_FLAG_USER;

	if (ixno == IX_INSN_NOEXEC) {
		vm_flags = VM_EXEC;
	} else if ((ixno == IX_DATA_NOWRITE) && (user_mode(regs) || __addr_ok(addr))) {
		vm_flags = VM_WRITE;
		mm_flags |= FAULT_FLAG_WRITE;
	}

	/*
	 * As per x86, we may deadlock here. However, since the kernel only
	 * validly references user space from well defined areas of the code,
	 * we can bug out early if this is from code which shouldn't.
	 */
	if (!down_read_trylock(&mm->mmap_sem)) {
		if (!user_mode(regs) && !search_exception_tables(regs->pc))
			goto no_context;
retry:
		down_read(&mm->mmap_sem);

	} else {
		/*
		 * The above down_read_trylock() might have succeeded in which
		 * case, we'll have missed the might_sleep() from down_read().
		 */
		might_sleep();
#ifdef CONFIG_DEBUG_VM
		if (!user_mode(regs) && !search_exception_tables(regs->pc))
			goto no_context;
#endif
	}

	fault = __do_page_fault(mm, addr, mm_flags, vm_flags, tsk, ixno);

	/*
	 * If we need to retry but a fatal signal is pending, handle the
	 * signal first. We do not need to release the mmap_sem because it
	 * would already be released in __lock_page_or_retry in mm/filemap.c.
	 */
	if ((fault & VM_FAULT_RETRY) && fatal_signal_pending(current))
		return 0;

	/*
	 * Major/minor page fault accounting is only done on the initial
	 * attempt. If we go through a retry, it is extremely likely that the
	 * page will be found in page cache at that point.
	 */

	if (mm_flags & FAULT_FLAG_ALLOW_RETRY) {
		if (fault & VM_FAULT_MAJOR) {
			tsk->maj_flt++;
		} else {
			tsk->min_flt++;
		}
		if (fault & VM_FAULT_RETRY) {
			/*
			 * Clear FAULT_FLAG_ALLOW_RETRY to avoid any risk of
			 * starvation.
			 */
			mm_flags &= ~FAULT_FLAG_ALLOW_RETRY;
			mm_flags |= FAULT_FLAG_TRIED;
			goto retry;
		}
	}

	up_read(&mm->mmap_sem);

	/*
	 * Handle the "normal" case first - VM_FAULT_MAJOR / VM_FAULT_MINOR
	 */
	if (likely(!(fault & (VM_FAULT_ERROR | VM_FAULT_BADMAP |
			      VM_FAULT_BADACCESS))))
		return 0;

	/*
	 * If we are in kernel mode at this point, we have no context to
	 * handle this fault with.
	 */
	if (!user_mode(regs))
		goto no_context;

	if (fault & VM_FAULT_OOM) {
		/*
		 * We ran out of memory, call the OOM killer, and return to
		 * userspace (which will retry the fault, or kill us if we got
		 * oom-killed).
		 */
		pagefault_out_of_memory();
		return 0;
	}

	if (fault & VM_FAULT_SIGBUS) {
		/*
		 * We had some memory, but were unable to successfully fix up
		 * this page fault.
		 */
		sig = SIGBUS;
		code = BUS_ADRERR;
	} else {
		/*
		 * Something tried to access memory that isn't in our memory
		 * map.
		 */
		sig = SIGSEGV;
		code = fault == VM_FAULT_BADACCESS ?
			SEGV_ACCERR : SEGV_MAPERR;
	}

	__do_user_fault(tsk, addr, sig, code, regs);
	return 0;

no_context:
	__do_kernel_fault(mm, addr, ixno, regs);
	return 0;
}

static int do_data_noread(struct pt_regs *regs, unsigned long user)
{
	unsigned long address = get_ixa_va(regs, user);
	do_page_fault(regs, address, IX_DATA_NOREAD);
	
	return 0;
}

static int do_data_nowrite(struct pt_regs *regs, unsigned long user)
{
	unsigned long address = get_ixa_va(regs, user);
	do_page_fault(regs, address, IX_DATA_NOWRITE);
	
	return 0;
}

static int do_insn_noexec(struct pt_regs *regs, unsigned long user)
{
	unsigned long address = instruction_pointer(regs);
	do_page_fault(regs, address, IX_INSN_NOEXEC);
	
	return 0;
}

SYSCALL_DEFINE0(do_abort)
{
	struct pt_regs *regs = current_pt_regs();
        unsigned long fault_pc = instruction_pointer(regs);
 
        __do_user_fault(current, fault_pc, SIGABRT, BUS_ADRERR, regs);

	return 0;
}

/*
 * This abort handler always returns "fault".
 */
static int do_bad(struct pt_regs *regs, unsigned long user)
{
	return 1;
}

static int do_bad_insn(struct pt_regs *regs, unsigned long user)
{
        unsigned long fault_pc = instruction_pointer(regs);

	if (user)
        	__do_user_fault(current, fault_pc, SIGILL, ILL_ILLOPC, regs);
	else
		__do_kernel_fault(current->mm, fault_pc, IX_INSN_BAD, regs);

	return 0;	
}

static int do_divide(struct pt_regs *regs, unsigned long user)
{
        unsigned long fault_pc = instruction_pointer(regs);

        if(fault_pc<TASK_SIZE)
                __do_user_fault(current, fault_pc, SIGFPE, FPE_INTDIV, regs);
        else
                __do_kernel_fault(current->mm, fault_pc, IX_INT_DIVIDE, regs);

        return 0;
}

static int do_insn_perm(struct pt_regs *regs, unsigned long user)
{
        unsigned long fault_pc = instruction_pointer(regs);

        if (user)
                __do_user_fault(current, fault_pc, SIGSEGV, SEGV_ACCERR, regs);
        else
                __do_kernel_fault(current->mm, fault_pc, IX_INSN_PERM, regs);

        return 0;
}

static struct fault_info {
	int	(*fn)(struct pt_regs *regs, unsigned long ixi);
	int	sig;
	int	code;
	const char *name;
} fault_info[] = {
	{ do_bad,		SIGBUS,  0,		"machine check"			}, /*  0 */
	{ do_bad,		SIGBUS,  0,		"timer base"			}, /*  1 */
	{ do_bad,		SIGBUS,  0,		"timer watchdog"		}, /*  2 */
	{ do_bad,		SIGBUS,  0,		"timer 0 countdown"		}, /*  3 */
	{ do_bad,		SIGBUS,  0,		"timer 1 countdown"		}, /*  4 */
	{ do_bad,		SIGSEGV, SEGV_MAPERR,  "Vector unit unavailable"	}, /*  5 */
	{ do_bad,		SIGSEGV, SEGV_MAPERR,	"FPU unavailable"		}, /*  6 */
	{ do_bad_insn,		SIGILL,  ILL_ILLOPC,	"bad instruction"		}, /*  7 */
	{ do_bad_insn,		SIGILL,  ILL_ILLOPC,	"bad instruction field"		}, /*  8 */
	{ do_divide,		SIGSEGV, SEGV_ACCERR,	"integer divide by zero"	}, /*  9 */
	{ do_insn_perm,		SIGSEGV, SEGV_ACCERR,	"instruction permission"	}, /* 10 */
	{ do_bad,		SIGSEGV, SEGV_ACCERR,	"FPU denorm"			}, /* 11 */
	{ do_bad,		SIGSEGV, SEGV_ACCERR,	"instruction alignment"		}, /* 12 */
	{ do_bad,		SIGSEGV, SEGV_ACCERR,	"instruction TLB miss"		}, /* 13 */
	{ do_insn_noexec,	SIGSEGV, SEGV_ACCERR,	"instruction non executable"	}, /* 14 */
	{ do_bad,		SIGBUS,  BUS_ADRALN,	"data alignment"		}, /* 15 */
	{ do_bad,		SIGSEGV, SEGV_ACCERR,	"data TLB miss"			}, /* 16 */
	{ do_data_noread,	SIGSEGV, SEGV_ACCERR,	"data non read"			}, /* 17 */
	{ do_data_nowrite,	SIGSEGV, SEGV_ACCERR,	"data non write"		}, /* 18 */
	{ do_bad,		SIGFPE,  FPE_FLTINV,	"FPU instruction invalid"	}, /* 19 */
	{ do_bad,		SIGFPE,  FPE_INTDIV,	"FPU divide by zero"		}, /* 20 */
	{ do_bad,		SIGFPE,  FPE_FLTOVF,	"FPU overflow fault"		}, /* 21 */
	{ do_bad,		SIGFPE,  FPE_FLTUND,	"FPU underflow fault"		}, /* 22 */
	{ do_bad,		SIGFPE,  FPE_FLTRES,	"FPU inexact"			}, /* 23 */
	{ do_bad,		SIGBUS,  0,		"debug single step"		}, /* 24 */
	{ do_bad,		SIGBUS,  0,		"debug branch"			}, /* 25 */
	{ do_bad,		SIGBUS,  0,		"debug instruction match"	}, /* 26 */
	{ do_bad,		SIGBUS,  0,		"debug data match"		}, /* 27 */
	{ do_bad,		SIGFPE,  FPE_FLTINV,	"VPU invalid instruction"	}, /* 28 */
	{ do_bad,		SIGFPE,  FPE_INTDIV,	"VPU divide by zero"		}, /* 29 */
	{ do_bad,		SIGFPE,  FPE_FLTOVF,	"VPU overflow"			}, /* 30 */
	{ do_bad,		SIGFPE,  FPE_FLTUND,	"VPU underflow"			}, /* 31 */
	{ do_bad,		SIGFPE,  FPE_FLTRES,	"VPU inexact"			}, /* 32 */
	{ do_bad,		SIGFPE,  FPE_FLTSUB,	"VPU length exceeds maximum"	}, /* 33 */
	{ do_bad,		SIGFPE,  0,		"VPU operation not supported"	}, /* 34 */
	{ do_bad,		SIGBUS,  0,		"software interrupt"		}, /* 35 */
	{ do_bad,		SIGBUS,  0,		"unknown 36"			}, /* 36 */
	{ do_bad,		SIGBUS,  0,		"unknown 37"			}, /* 37 */
	{ do_bad,		SIGBUS,  0,		"SGI 0"				}, /* 38 */
	{ do_bad,		SIGBUS,  0,		"SGI 1"				}, /* 39 */
	{ do_bad,		SIGBUS,  0,		"SGI 2"				}, /* 40 */
	{ do_bad,		SIGBUS,  0,		"SGI 3"				}, /* 41 */
	{ do_bad,		SIGBUS,  0,		"SGI 4"				}, /* 42 */
	{ do_bad,		SIGBUS,  0,		"SGI 5"				}, /* 43 */
	{ do_bad,		SIGBUS,  0,		"SGI 6"				}, /* 44 */
	{ do_bad,		SIGBUS,  0,		"SGI 7"				}, /* 45 */
	{ do_bad,		SIGBUS,  0,		"MPIC FIQ 46"			}, /* 46 */
	{ do_bad,		SIGBUS,  0,		"MPIC IRQ 47"			}, /* 47 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 48"		}, /* 48 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 49"		}, /* 49 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 50"		}, /* 50 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 51"		}, /* 51 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 52"		}, /* 52 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 53"		}, /* 53 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 54"		}, /* 54 */
	{ do_bad,		SIGBUS,  0,		"Direct interrupt 55"		}, /* 55 */
	{ do_bad,		SIGBUS,  0,		"unknown 56"			}, /* 56 */
	{ do_bad,		SIGBUS,  0,		"unknown 57"			}, /* 57 */
	{ do_bad,		SIGBUS,  0,		"unknown 58"			}, /* 58 */
	{ do_bad,		SIGBUS,  0,		"unknown 59"			}, /* 59 */
	{ do_bad,		SIGBUS,  0,		"unknown 60"			}, /* 60 */
	{ do_bad,		SIGBUS,  0,		"unknown 61"			}, /* 61 */
	{ do_bad,		SIGBUS,  0,		"unknown 62"			}, /* 62 */
	{ do_bad,		SIGBUS,  0,		"unknown 63"			}, /* 63 */
};

void __init
hook_fault_code(int nr, int (*fn)(struct pt_regs *regs, unsigned long user),
		int sig, int code, const char *name)
{
	if (nr < 0 || nr >= ARRAY_SIZE(fault_info))
		BUG();

	fault_info[nr].fn   = fn;
	fault_info[nr].sig  = sig;
	fault_info[nr].code = code;
	fault_info[nr].name = name;
}

asmlinkage void do_hyper_exception(struct pt_regs *regs, unsigned long ixno, unsigned long user)
{
	const struct fault_info *info;
	unsigned long fault_pc = instruction_pointer(regs);
	info = fault_info + ixno;

	if (!info->fn(regs, user)) {
		return;
	}

	pr_alert("Unhandled exception: %s (0x%08lx) at 0x%016lx\n", info->name, ixno, fault_pc);

	gpt_notify_die_helper(info->name, regs, info->sig, 0, info->code, fault_pc, ixno);
}

asmlinkage void brk_handler(struct pt_regs *regs)
{
	siginfo_t info;

	regs->pc -= 4;
	if (user_mode(regs)) {
		info = (siginfo_t) {
			.si_signo = SIGTRAP,
				.si_errno = 0,
				.si_code  = TRAP_BRKPT,
				.si_addr  = (void __user *)instruction_pointer(regs),
		};
		force_sig_info(SIGTRAP, &info, current);
	} else {
		pr_warning("Unexpected kernel BRK exception\n");
	}
}

