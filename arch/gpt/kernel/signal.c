/*
 * gpt signal.c
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

#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/kernel.h>
#include <linux/signal.h>
#include <linux/personality.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/ptrace.h>
#include <linux/unistd.h>
#include <linux/stddef.h>
#include <linux/tracehook.h>

#include <asm/signal.h>
#include <asm/processor.h>
#include <asm/syscall.h>
#include <asm/ucontext.h>
#include <asm/uaccess.h>
#include <asm/vdso.h>
#include <asm/vfp.h>

struct rt_sigframe {
	struct siginfo info;
	struct ucontext uc;
	u64 fp;
	u64 ra;
};

/*
 * translate the signal
 */
static inline int map_sig(int sig)
{
	struct thread_info *thread = current_thread_info();
	if (sig < 32 && thread->exec_domain && thread->exec_domain->signal_invmap)
		sig = thread->exec_domain->signal_invmap[sig];
	return sig;
}

static int save_fpu_context(struct fpu_context __user *ctx)
{
	int err;
	struct vfp_state *vfp = &current->thread.vfp_state;
	
	fpu_save_current_state(vfp);

	/* copy the FP and status/control registers */
	err = __copy_to_user(&ctx->f_regs, vfp->f_regs, sizeof(vfp->f_regs));
	err |= __put_user(vfp->fpsr, &ctx->fpsr);
	err |= __put_user(vfp->fpcr, &ctx->fpcr);

	/* copy the magic/size information */
	err |= __put_user(FPU_MAGIC, &ctx->head.magic);
	err |= __put_user(sizeof(struct fpu_context), &ctx->head.size);		

	return err ? -EFAULT : 0;
}

static int restore_fpu_context(struct fpu_context __user *ctx)
{
	int err=0;
	__u32 magic, size;	
	struct vfp_state *vfp = &current->thread.vfp_state;
		
	/* check the magic/size information */
	err |= __get_user(magic, &ctx->head.magic);
	err |= __get_user(size, &ctx->head.size);
	if (err)
		return -EFAULT;	
	if (magic != FPU_MAGIC || size != sizeof(struct fpu_context))
		return -EINVAL;	
	
	/* copy the FP and status/control registers */
	err |= __copy_from_user(vfp->f_regs, ctx->f_regs,
			       sizeof(vfp->f_regs));
	err |= __get_user(vfp->fpsr, &ctx->fpsr);
	err |= __get_user(vfp->fpcr, &ctx->fpcr);

	if(!err){
		fpu_update_current_state(vfp);
	}
	return err;	
}


static int save_vpu_context(struct vpu_context __user *ctx)
{
	int err;
	struct vfp_state *vfp = &current->thread.vfp_state;
	
	vpu_save_current_state(vfp);

	/* copy the VP and status/control registers */
	err = __copy_to_user(&ctx->n_regs, vfp->n_regs, sizeof(vfp->n_regs));
	err = __copy_to_user(&ctx->m_regs, vfp->m_regs, sizeof(vfp->m_regs));		
	err = __copy_to_user(&ctx->v_regs, vfp->v_regs, sizeof(vfp->v_regs));	
	err |= __put_user(vfp->vfsr, &ctx->vfsr);
	err |= __put_user(vfp->vfcr, &ctx->vfcr);

	/* copy the magic/size information */
	err |= __put_user(VPU_MAGIC, &ctx->head.magic);
	err |= __put_user(sizeof(struct vpu_context), &ctx->head.size);		

	return err ? -EFAULT : 0;
}

static int restore_vpu_context(struct vpu_context __user *ctx)
{	
	int err=0;
	__u32 magic, size;	
	struct vfp_state *vfp = &current->thread.vfp_state;
		
	/* check the magic/size information */
	err |= __get_user(magic, &ctx->head.magic);
	err |= __get_user(size, &ctx->head.size);
	if (err)
		return -EFAULT;	
	if (magic != VPU_MAGIC || size != sizeof(struct vpu_context))
		return -EINVAL;	
	
	/* copy the VP and status/control registers */
	err |= __copy_from_user(vfp->n_regs, ctx->n_regs,
			       sizeof(vfp->n_regs));
	err |= __copy_from_user(vfp->m_regs, ctx->m_regs,
			       sizeof(vfp->m_regs));
	err |= __copy_from_user(vfp->v_regs, ctx->v_regs,
			       sizeof(vfp->v_regs));			       
	err |= __get_user(vfp->vfsr, &ctx->vfsr);
	err |= __get_user(vfp->vfcr, &ctx->vfcr);

	if(!err){
		vpu_update_current_state(vfp);
	}
	return err;	
}


static int restore_sigcontext(struct pt_regs *regs,
			      struct rt_sigframe __user * sf)
{
	struct fpu_context *fpu_ctx;
	struct vpu_context *vpu_ctx;
	int err = 0, i;
	void *aux = sf->uc.uc_mcontext.__reserved;	
	
	/* Always make any pending restarted system calls return -EINTR */
	current_thread_info()->restart_block.fn = do_no_restart_syscall;

	/* set up the stack frame for unwinding */
	err |= __copy_from_user(&regs->r_regs, &sf->uc.uc_mcontext.r_regs, sizeof(regs->r_regs));
	for (i = 1; i <= 10; i++)
	  err |= __get_user(regs->a_regs[i], &sf->uc.uc_mcontext.a_regs[i - 1]);
	err |= __get_user(regs->t_regs[0], &sf->uc.uc_mcontext.t_regs[0]);
	err |= __get_user(regs->t_regs[1], &sf->uc.uc_mcontext.t_regs[1]);
	err |= __get_user(regs->t_regs[4], &sf->uc.uc_mcontext.t_regs[2]);
	err |= __get_user(regs->t_regs[7], &sf->uc.uc_mcontext.ra);
 	err |= __get_user(regs->sp, &sf->uc.uc_mcontext.sp);
	err |= __get_user(regs->pc, &sf->uc.uc_mcontext.pc);
	err |= __get_user(regs->ca, &sf->uc.uc_mcontext.ca);
	
	if(err==0){
		fpu_ctx = container_of(aux, struct fpu_context, head);		
		err |= restore_fpu_context(fpu_ctx);
		aux += sizeof(*fpu_ctx);
		
		vpu_ctx = container_of(aux, struct vpu_context, head);		
		err |= restore_vpu_context(vpu_ctx);
		aux += sizeof(*vpu_ctx);
	}
				 			
	return err;
}

static void __user *get_sigframe(struct ksignal *ksig,struct pt_regs *regs,int framesize)
{
	unsigned long sp, sp_top;
	void __user *frame;

	sp_top = sigsp(regs->sp, ksig);
	sp = ((sp_top - framesize) & ~15UL);	//16-byte alignment
	frame = (void __user *)(sp);

	/*
	 * Check that we can actually write to the signal frame.
	 */
	if (!access_ok(VERIFY_WRITE, frame, framesize))
		frame = NULL;	
	return frame;
}

asmlinkage long _sys_rt_sigreturn(struct pt_regs *regs)
{
	struct rt_sigframe *frame = (struct rt_sigframe __user *)regs->sp;		
	sigset_t set;
	/*
	 * Since we stacked the signal on a dword boundary,
	 * then frame should be dword aligned here.  If it's
	 * not, then the user is trying to mess with us.
	 */
	if (((long)frame) & 0xF)
		goto badframe;

	if (!access_ok(VERIFY_READ, frame, sizeof(*frame)))
		goto badframe;	
	if (__copy_from_user(&set, &frame->uc.uc_sigmask, sizeof(set)))
		goto badframe;

	set_current_blocked(&set);

	if (restore_sigcontext(regs, frame))
		goto badframe;

	if (restore_altstack(&frame->uc.uc_stack))
		goto badframe;

	return regs->r_regs[8];

badframe:
	if (show_unhandled_signals)
		pr_info("%s[%d]: bad frame in %s: pc=%08llx sp=%08llx\n",
		        current->comm, task_pid_nr(current), __func__,
		        regs->pc, regs->sp);
	force_sig(SIGSEGV, current);
	return 0;
}

static int setup_return(struct pt_regs *regs,  struct ksignal *ksig,
			 struct rt_sigframe __user *frame)			 
{	
	int err=0;
	__sigrestore_t sigtramp;
	regs->r_regs[8] =(unsigned long)map_sig(ksig->sig); 			/*arg1*/
	regs->sp = (unsigned long)frame;
	regs->a_regs[1] = regs->sp + offsetof(struct rt_sigframe, fp);		/* Chaining the frame. */
	regs->pc = (unsigned long)ksig->ka.sa.sa_handler;		
	if (ksig->ka.sa.sa_flags & SA_SIGINFO) {
		err |= copy_siginfo_to_user(&frame->info, &ksig->info);		
		regs->r_regs[9] = (unsigned long)&frame->info;		/*arg2*/
		regs->r_regs[10] = (unsigned long)&frame->uc;			/*arg3*/
	}
	sigtramp = VDSO_SYMBOL(current->mm->context.vdso_base, sigtramp);
	regs->t_regs[7] = (unsigned long)sigtramp;

	return 0;
}

static int setup_sigframe(struct rt_sigframe __user *sf,
			  struct pt_regs *regs, sigset_t *set)
{
	struct fpu_context *fpu_ctx;
	struct vpu_context *vpu_ctx;

	int err=0, i;
	void *aux = sf->uc.uc_mcontext.__reserved;	
	struct _gpt_ctx*end;
	/* set up the stack frame for unwinding */
	err |= __put_user(regs->t_regs[7], &sf->ra);
	err |= __put_user(regs->a_regs[1], &sf->fp);

	err |= __copy_to_user(&sf->uc.uc_mcontext.r_regs, &regs->r_regs, sizeof(regs->r_regs));
	for (i = 1; i <= 10; i++)
	  err |= __put_user(regs->a_regs[i], &sf->uc.uc_mcontext.a_regs[i - 1]);
	err |= __put_user(regs->t_regs[0], &sf->uc.uc_mcontext.t_regs[0]);
	err |= __put_user(regs->t_regs[1], &sf->uc.uc_mcontext.t_regs[1]);
	err |= __put_user(regs->t_regs[4], &sf->uc.uc_mcontext.t_regs[2]);
	err |= __put_user(regs->t_regs[7], &sf->uc.uc_mcontext.ra);
	err |= __put_user(regs->sp, &sf->uc.uc_mcontext.sp);
	err |= __put_user(regs->pc, &sf->uc.uc_mcontext.pc);
	err |= __put_user(regs->ca, &sf->uc.uc_mcontext.ca);
	err |= __put_user(current->thread.fault_address, &sf->uc.uc_mcontext.fault_address);

	err |= __copy_to_user(&sf->uc.uc_sigmask, set, sizeof(*set));
	if (err == 0) {
		fpu_ctx = container_of(aux, struct fpu_context, head);
		err |= save_fpu_context(fpu_ctx);
		aux += sizeof(*fpu_ctx);

		vpu_ctx = container_of(aux, struct vpu_context, head);
		err |= save_vpu_context(vpu_ctx);		
		aux += sizeof(*vpu_ctx);
	}

	/* fault information, if valid */
	if (current->thread.fault_code) {
		/*TODO: glibc should support this*/
	}
	/* set the "end" magic */
	end = aux;
	err |= __put_user(0, &end->magic);
	err |= __put_user(0, &end->size);
		
	return err;
}
static int setup_rt_frame(struct ksignal *ksig, sigset_t *set,
			  struct pt_regs *regs)
{
	struct rt_sigframe __user *frame;
	int err = 0;
	frame = get_sigframe(ksig, regs,sizeof(*frame));
	if (!frame)
		return 1;
	err |= __put_user(0, &frame->uc.uc_flags);
	err |= __put_user(NULL, &frame->uc.uc_link);
	err |= __save_altstack(&frame->uc.uc_stack, regs->sp);
	err |= setup_sigframe(frame, regs, set);
	if (err == 0) {
		err=setup_return(regs, ksig, frame);
	}

	return err;
}

static void handle_signal(struct ksignal *ksig, struct pt_regs *regs)
{
	int ret = 0;
	sigset_t *oldset = sigmask_to_save();
	ret = setup_rt_frame(ksig, oldset, regs);		
	signal_setup_done(ret, ksig, 0);
}

/*
 * Note that 'init' is a special process: it doesn't get signals it doesn't
 * want to handle. Thus you cannot kill init even with a SIGKILL even by
 * mistake.
 *
 * Also note that the regs structure given here as an argument, is the latest
 * pushed pt_regs. It may or may not be the same as the first pushed registers
 * when the initial usermode->kernelmode transition took place. Therefore
 * we can use user_mode(regs) to see if we came directly from kernel or user
 * mode below.
 */

void do_signal(struct pt_regs *regs)
{	
	unsigned long continue_addr = 0, restart_addr = 0;
	int retval = 0;
	int syscall = (int)regs->syscallno;
	struct ksignal ksig;
			
	/* If we come from a system call... */
	if (syscall >= 0) {
		/* Checking for system call restarting... */
		continue_addr = regs->pc;
		restart_addr = continue_addr - 4;
		retval = regs->r_regs[8];
		/*
		 * Avoid additional syscall restarting via ret_to_user.
		 */		
		regs->syscallno = ~0UL;
		
		switch (syscall_get_error(current, regs)) {
			case -ERESTARTNOHAND:
			case -ERESTARTSYS:
			case -ERESTARTNOINTR:
			case -ERESTART_RESTARTBLOCK:
				regs->r_regs[8] = regs->r8_orig;
				regs->pc = restart_addr;
				break;
		}
	}	
	/* Get the signal to deliver. */
	if (get_signal(&ksig)) {	
		if (regs->pc == restart_addr) {
			if (retval == -ERESTARTNOHAND ||
			    retval == -ERESTART_RESTARTBLOCK
			    || (retval == -ERESTARTSYS
			        && !(ksig.ka.sa.sa_flags & SA_RESTART))) {
				/* No automatic restart */
				regs->r_regs[8] = -EINTR;
				regs->pc = continue_addr;
			}
		}	
		
		handle_signal(&ksig, regs);
		return;
	}
	if (syscall >= 0 && regs->pc == restart_addr) {
	  if (retval == -ERESTART_RESTARTBLOCK)
	    regs->r_regs[8] = __NR_restart_syscall;
	}

	/* No signal to deliver, we just put the saved sigmask back */
	restore_saved_sigmask();
}

/*
 * notification of userspace execution resumption
 * - triggered by current->work.notify_resume
 */
asmlinkage void do_notify_resume(struct pt_regs *regs, u32 thread_info_flags)
{
        struct vfp_state *st = &current->thread.vfp_state;
#ifndef CONFIG_HAVE_HW_BREAKPOINT
	/* Pending single-step? */
	if (thread_info_flags & _TIF_SINGLESTEP) {
                BUG(); /* No HW single-step now */
        }
#endif
        /* deal with pending signal delivery */
        if (thread_info_flags & _TIF_SIGPENDING)
                do_signal(regs);

        if (thread_info_flags & _TIF_NOTIFY_RESUME) {
                clear_thread_flag(TIF_NOTIFY_RESUME);
                tracehook_notify_resume(regs);
        }

        if (thread_info_flags & _TIF_FOREIGN_FPUSTATE){
		fpu_restore_current_state(st);
	}
        if (thread_info_flags & _TIF_FOREIGN_VPUSTATE){
		vpu_restore_current_state(st);
	}
}

