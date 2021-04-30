#ifndef __ASM_GPT_VFP_H
#define __ASM_GPT_VFP_H

#include <asm/ptrace.h>

#define VPU_V_SIZE   1024
#define PSC_FPU_EN   (1UL<<PSC_OFFSE_unit_fpu)
#define PSC_VPU_EN   (1UL<<PSC_OFFSE_unit_vec)

struct vfp_state {
	union {
		struct user_fpu_state user_fpu_state;	
		struct {
			__u64 f_regs[16];
			__u64 fpsr;
			__u64 fpcr;	
		};
		unsigned int fcpu;
	};
	union {
		struct user_vpu_state user_vpu_state;
		struct {
			__u16 n_regs[8];
			__u64 m_regs[32];
			__u64 v_regs[1024];
			__u64 vfsr;
			__u64 vfcr;			
		};	
		unsigned int vcpu;
	};
};

struct task_struct;

extern void vfp_flush_thread(void);
extern void fpu_save_current_state(struct vfp_state *st);
extern void vpu_save_current_state(struct vfp_state *st);
extern void fpu_restore_current_state(struct vfp_state *st);
extern void vpu_restore_current_state(struct vfp_state *st);
extern void fpu_update_current_state(struct vfp_state *st);
extern void vpu_update_current_state(struct vfp_state *st);
extern void vfp_thread_switch(struct task_struct *next);
extern void vfp_flush_task_state(struct task_struct *t);
#endif
