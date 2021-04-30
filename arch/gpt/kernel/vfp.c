/*
 * VFP context switching and fault handling
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

#include <linux/cpu_pm.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/signal.h>

#include <asm/spr.h>
#include <asm/insn.h>
#include <asm/vfp.h>
#include <asm/bug.h>
/*
 *
 */

static DEFINE_PER_CPU(struct vfp_state *, fpu_last_state);
static DEFINE_PER_CPU(struct vfp_state *, vpu_last_state);

/*declared in kernel/entry-vfp.S*/
extern void fpu_save_state(struct vfp_state*state);
extern void fpu_load_state(struct vfp_state*state);
extern void vpu_save_state(struct vfp_state*state);
extern void vpu_load_state(struct vfp_state*state);

extern unsigned long vpu_get_fcr(void);
extern unsigned long vpu_get_fsr(void);	
extern unsigned long fpu_get_fcr(void);
extern unsigned long fpu_get_fsr(void);
	
extern unsigned long fpu_set_fcr(unsigned long val);
extern unsigned long vpu_set_fcr(unsigned long val);

void _fpu_save_state(struct vfp_state*state);
void _fpu_load_state(struct vfp_state*state);
void _vpu_save_state(struct vfp_state*state);
void _vpu_load_state(struct vfp_state*state);
	
static inline void close_vfp(int bit)
{
	unsigned long flags, flags2;
	local_irq_save(flags2);
	asm volatile (
	"taddpci	$t0, 1f\n"
	"rsetspr	%0, %2\n"
	"cand		%0, %1, %0\n"
	".align 4\n"
	"sprsetr	%0, %3\n"
	"rsett		%0, $t0\n"
	"sprsetr	%0, %4\n"
	"retfi_alt\n"
	"1:\n"
	"rsetspr	%0, %5\n"
	"cand		%0, %1, %0\n"
	"sprsetr	%0, %5\n"
	:"=&r"(flags)
	:"r"(1UL << bit), "i"(PSC), "i"(XSA), "i"(XRA), "i"(PSCH)
	:"memory", "$t0", "PSC", "XSA", "XRA", "PSCH");
	local_irq_restore(flags2);
}
static inline void open_vfp(int bit)
{
	unsigned long flags, flags2;
	local_irq_save(flags2);
	asm volatile (
	"taddpci	$t0, 1f\n"
	"rsetspr	%0, %2\n"
	"or		%0, %1, %0\n"
	".align 4\n"
	"sprsetr	%0, %3\n"
	"rsett		%0, $t0\n"
	"sprsetr	%0, %4\n"
	"retfi_alt\n"
	"1:\n"
	"rsetspr	%0, %5\n"
	"or		%0, %1, %0\n"
	"sprsetr	%0, %5\n"
	:"=&r"(flags)
	:"r"(1UL << bit), "i"(PSC), "i"(XSA), "i"(XRA), "i"(PSCH)
	:"memory", "$t0", "PSC", "XSA", "XRA", "PSCH");
	local_irq_restore(flags2);
}
static inline u64 vfp_is_open(int bit)
{
	return (get_psc()&(1UL << bit));
}
#define fpu_is_open()	vfp_is_open(PSC_OFFSE_unit_fpu)
#define vpu_is_open()	vfp_is_open(PSC_OFFSE_unit_vec)

#define open_fpu()	open_vfp(PSC_OFFSE_unit_fpu)
#define open_vpu()	open_vfp(PSC_OFFSE_unit_vec)	
//#define close_fpu()	close_vfp(PSC_OFFSE_unit_fpu)
//#define close_vpu()	close_vfp(PSC_OFFSE_unit_vec)
#define close_vpu()  do{}while(0);
#define close_fpu()  do{}while(0);

static inline void thread_open_vfp(struct task_struct *t, int bit_off)
{
	struct pt_regs*regs=task_pt_regs(t);
	regs->sr = regs->sr | 1UL << bit_off;
	open_vfp(bit_off);
}
static inline void thread_close_vfp(struct task_struct *t, int bit_off)
{
	return ;
}

#define thread_open_fpu(t)	thread_open_vfp(t, PSC_OFFSE_unit_fpu)
#define thread_open_vpu(t)	thread_open_vfp(t, PSC_OFFSE_unit_vec)
	
static int do_fpu_unavail(struct pt_regs *regs, unsigned long user)
{
	struct vfp_state *st = &current->thread.vfp_state;
	preempt_disable();
	thread_open_fpu(current);
	_fpu_load_state(st);
	
	this_cpu_write(fpu_last_state, st);	
	st->fcpu = smp_processor_id();
	clear_thread_flag(TIF_FOREIGN_FPUSTATE);
	preempt_enable();
	
	return 0;
}

static int do_vpu_unavail(struct pt_regs *regs, unsigned long user)
{
	struct vfp_state *st = &current->thread.vfp_state;	
	preempt_disable();
	thread_open_vpu(current);
	_vpu_load_state(st);
	
	this_cpu_write(vpu_last_state, st);	
	st->vcpu = smp_processor_id();
	clear_thread_flag(TIF_FOREIGN_VPUSTATE);
	preempt_enable();

	return 0;
}


/*
 * Raise a SIGFPE for the current process.
 * NOT enabled yet.
 */


static inline int __do_vfp_exc(struct pt_regs *regs, unsigned long ixno)
{
	siginfo_t info;
	unsigned int si_code = 0;
	switch(ixno)
	{
		case IX_FP_INVALID:
		case IX_VFP_INVALID:
			si_code = FPE_FLTINV;
		break;
		case IX_FP_DIVBYZERO:
		case IX_VFP_DIVBYZERO:
			si_code = FPE_FLTDIV;
		break;
		case IX_FP_OVERFLOW:
		case IX_VFP_OVERFLOW:
			si_code = FPE_FLTOVF;
		break;	
		case IX_FP_UNDERFLOW:
		case IX_VFP_UNDERFLOW:
			si_code = FPE_FLTUND;
		break;
		case IX_FP_INEXACT:
		case IX_VFP_INEXACT:
			si_code = FPE_FLTRES;
		break;
		case IX_VEC_LENGTH:
			si_code = FPE_FLTSUB;
		break;
		case IX_VEC_TYPE:
			si_code = 0;
		break;
		default:
			BUG();
		return 0;
	}
		
	memset(&info, 0, sizeof(info));
	info.si_signo = SIGFPE;
	info.si_code = si_code;
	info.si_addr = (void __user *)instruction_pointer(regs);

	send_sig_info(SIGFPE, &info, current);

	return 0;
}
#define DO_VFP_EXC(IXNO) \
static int do_vfp_exc_##IXNO (struct pt_regs *regs, unsigned long user)	\
{									\
	if (user)							\
	  return __do_vfp_exc(regs, IXNO);				\
	bust_spinlocks(1);						\
	pr_alert("Unable to handle kernel "				\
		 "VFP insn at virtual address %016llx\n", regs->pc);	\
	die("Oops", regs, IXNO);					\
	bust_spinlocks(0);						\
	do_exit(SIGKILL);						\
	return 1;							\
}

DO_VFP_EXC(IX_FP_INVALID)
DO_VFP_EXC(IX_FP_DIVBYZERO)
DO_VFP_EXC(IX_FP_OVERFLOW)
DO_VFP_EXC(IX_FP_UNDERFLOW)
DO_VFP_EXC(IX_FP_INEXACT)
DO_VFP_EXC(IX_VFP_INVALID)
DO_VFP_EXC(IX_VFP_DIVBYZERO)
DO_VFP_EXC(IX_VFP_OVERFLOW)
DO_VFP_EXC(IX_VFP_UNDERFLOW)
DO_VFP_EXC(IX_VFP_INEXACT)
DO_VFP_EXC(IX_VEC_LENGTH)
DO_VFP_EXC(IX_VEC_TYPE)

#undef DO_VFP_EXC
#define DO_VFP_EXC(IXNO) do_vfp_exc_##IXNO

void _fpu_save_state(struct vfp_state*state)
{
	u64 fpu_st=fpu_is_open();
	if(!fpu_st){
		open_fpu();
	}
	fpu_save_state(state); 
}
void _fpu_load_state(struct vfp_state*state)
{
	u64 fpu_st=fpu_is_open();
	if(!fpu_st){
		open_fpu();
	}
	fpu_load_state(state);		
}

void _vpu_save_state(struct vfp_state*state)
{
	u64 vpu_st=vpu_is_open();
	if(!vpu_st){
		open_vpu();
	}
	vpu_save_state(state);
}
void _vpu_load_state(struct vfp_state*state)
{
	u64 vpu_st=vpu_is_open();
	if(!vpu_st){
		open_vpu();		
	}
	vpu_load_state(state);	
}


void vfp_thread_switch(struct task_struct *next)
{
	/*
	 * Save the current VFP state to memory, but only if whatever is in
	 * the registers is in fact the most recent userland VFP state of
	 * 'current'.
	 */
	struct vfp_state *old_st = &current->thread.vfp_state;
	if (current->mm){
		if(!test_thread_flag(TIF_FOREIGN_FPUSTATE))
			_fpu_save_state(old_st);
		if(!test_thread_flag(TIF_FOREIGN_VPUSTATE))
			_vpu_save_state(old_st);
	}

	if (next->mm) {
		struct vfp_state *st = &next->thread.vfp_state;

		if (__this_cpu_read(fpu_last_state) == st
		    && st->fcpu == smp_processor_id()){
			clear_ti_thread_flag(task_thread_info(next),
					     TIF_FOREIGN_FPUSTATE);
		}
		else{
			set_ti_thread_flag(task_thread_info(next),
				   TIF_FOREIGN_FPUSTATE);
		}

		if (__this_cpu_read(vpu_last_state) == st
		    && st->vcpu == smp_processor_id())
			clear_ti_thread_flag(task_thread_info(next),
					     TIF_FOREIGN_VPUSTATE);
		else
			set_ti_thread_flag(task_thread_info(next),
				   TIF_FOREIGN_VPUSTATE);
	}
}

void vfp_flush_thread(void)
{
	struct vfp_state *st = &current->thread.vfp_state;
	memset(st, 0, sizeof(struct vfp_state));
	
	set_thread_flag(TIF_FOREIGN_FPUSTATE);
	set_thread_flag(TIF_FOREIGN_VPUSTATE);
	
}

void fpu_save_current_state(struct vfp_state *st)
{
	preempt_disable();
	if(!test_thread_flag(TIF_FOREIGN_FPUSTATE))
		_fpu_save_state(st);		
	preempt_enable();
}

void vpu_save_current_state(struct vfp_state *st)
{
	preempt_disable();
	if(!test_thread_flag(TIF_FOREIGN_VPUSTATE))
		_vpu_save_state(st);				
	preempt_enable();		
}

void fpu_restore_current_state(struct vfp_state *st)
{
	preempt_disable();	
	if (test_and_clear_thread_flag(TIF_FOREIGN_FPUSTATE)) {
		_fpu_load_state(st);
		this_cpu_write(fpu_last_state, st);
		st->fcpu = smp_processor_id();
	}
	preempt_enable();
}
void vpu_restore_current_state(struct vfp_state *st)
{
	preempt_disable();
	if (test_and_clear_thread_flag(TIF_FOREIGN_VPUSTATE)) {
		_vpu_load_state(st);
		this_cpu_write(vpu_last_state, st);
		st->vcpu = smp_processor_id();
	}
	preempt_enable();
}
/*
 * Load an updated userland VFP state for 'current' from memory and set the
 * flag that indicates that the VFP register contents are the most recent
 * VFP state of 'current'
 */
void fpu_update_current_state(struct vfp_state *st)
{
	preempt_disable();
	_fpu_load_state(st);
	if (test_and_clear_thread_flag(TIF_FOREIGN_FPUSTATE)) {
		struct vfp_state *vfp_st = &current->thread.vfp_state;

		this_cpu_write(fpu_last_state, vfp_st);
		vfp_st->fcpu = smp_processor_id();
	}
	preempt_enable();
}
void vpu_update_current_state(struct vfp_state *st)
{
	preempt_disable();
	_vpu_load_state(st);
	if (test_and_clear_thread_flag(TIF_FOREIGN_VPUSTATE)) {
		struct vfp_state *vfp_st = &current->thread.vfp_state;

		this_cpu_write(vpu_last_state, vfp_st);
		vfp_st->vcpu = smp_processor_id();
	}
	preempt_enable();
}

/*
 * Invalidate live CPU copies of task t's VFP state
 */
void vfp_flush_task_state(struct task_struct *t)
{
	t->thread.vfp_state.fcpu = NR_CPUS;
	t->thread.vfp_state.vcpu = NR_CPUS;
}


/*
 * VFP support code initialisation.
 */
static int __init vfp_init(void)
{
	unsigned long fcr,fsr,vfcr,vfsr;
	
	hook_fault_code(IX_UNAVAIL_VEC, do_vpu_unavail, SIGSEGV, SEGV_MAPERR,
			"IX_UNAVAIL_VEC");	
	hook_fault_code(IX_UNAVAIL_FP, do_fpu_unavail, SIGSEGV, SEGV_MAPERR,
			"IX_UNAVAIL_FP");	
	hook_fault_code(IX_FP_INVALID, DO_VFP_EXC(IX_FP_INVALID), SIGFPE, 0,
			"IX_FP_INVALID");	
	hook_fault_code(IX_FP_DIVBYZERO, DO_VFP_EXC(IX_FP_DIVBYZERO), SIGFPE, 0,
			"IX_FP_DIVBYZERO");	
	hook_fault_code(IX_FP_OVERFLOW , DO_VFP_EXC(IX_FP_OVERFLOW), SIGFPE, 0,
			"IX_FP_OVERFLOW");	
	hook_fault_code(IX_FP_UNDERFLOW, DO_VFP_EXC(IX_FP_UNDERFLOW), SIGFPE, 0,
			"IX_FP_UNDERFLOW");	
	hook_fault_code(IX_FP_INEXACT, DO_VFP_EXC(IX_FP_INEXACT), SIGFPE, 0,
			"IX_FP_INEXACT");	
	hook_fault_code(IX_VFP_INVALID, DO_VFP_EXC(IX_VFP_INVALID), SIGFPE, 0,
			"IX_VFP_INVALID");	
	hook_fault_code(IX_VFP_DIVBYZERO, DO_VFP_EXC(IX_VFP_DIVBYZERO), SIGFPE, 0,
			"IX_VFP_DIVBYZERO");	
	hook_fault_code(IX_VFP_OVERFLOW, DO_VFP_EXC(IX_VFP_OVERFLOW), SIGFPE, 0,
			"IX_VFP_OVERFLOW");	
	hook_fault_code(IX_VFP_UNDERFLOW, DO_VFP_EXC(IX_VFP_UNDERFLOW), SIGFPE, 0,
			"IX_VFP_UNDERFLOW");	
	hook_fault_code(IX_VFP_INEXACT, DO_VFP_EXC(IX_VFP_INEXACT), SIGFPE, 0,
			"IX_VFP_INEXACT");			
	hook_fault_code(IX_VEC_LENGTH, DO_VFP_EXC(IX_VEC_LENGTH), SIGFPE, 0,
			"IX_VEC_LENGTH");			
	hook_fault_code(IX_VEC_TYPE, DO_VFP_EXC(IX_VEC_TYPE), SIGFPE, 0,
			"IX_VEC_TYPE");			
			
	this_cpu_write(fpu_last_state, NULL);
	this_cpu_write(vpu_last_state, NULL);

	open_fpu();
	fcr = fpu_get_fcr();
	fsr = fpu_get_fsr();  
	
	open_vpu();
	vfcr = vpu_get_fcr();
	vfsr = vpu_get_fsr();  	

	pr_info("VFP Init: FCR:%lx FSR:%lx VFCR:%lx VFSR:%lx\n",
		fcr,fsr,vfcr,vfsr);

	return 0;
}
core_initcall(vfp_init);
