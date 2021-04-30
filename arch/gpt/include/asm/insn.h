#ifndef __ASM_INSN_H
#define __ASM_INSN_H		

#define	GPT_INSN_SIZE		4

#ifndef __ASSEMBLY__

#include <machine/gptxinstr.h>

#define insn_get_opcode(insn)	\
	((insn>>20)&0xfff)	
#define insn_get_dest(insn)	\
	((insn>>16)&0xf)
#define insn_get_src(insn)	\
	((insn>>12)&0xf)
	
#define insn_gen0(op)					\
	(op<<20)	
#define insn_gen1(op, reg0)				\
	((op<<20)|(reg0<<16))	
#define insn_gen1t(op, reg0)				\
	((op<<20)|(0x7<<16)|(reg0<<7))	
#define insn_gen1_imm(op, reg0, imm)			\
	((op<<20)|(reg0<<16)|(imm&0xffff))	
#define insn_gen2(op, reg0, reg1)			\
	((op<<20)|(reg0<<16)|(reg1<<12))	
#define insn_gen2_imm(op, reg0, reg1, imm)		\
	((op<<20)|(reg0<<16)|(reg1<<12)|(imm&0xfff))	
#define insn_gen3(op, reg0, reg1, reg2)			\
	((op<<20)|(reg0<<16)|(reg1<<12)|(reg2<<8))	
#define insn_gen3_imm(op, reg0, reg1, reg2, imm)	\
	((op<<20)|(reg0<<16)|(reg1<<12)|(reg2<<8)|(imm&0xff))	


		
/*For VPU instructions*/
#define insn_get_vec_type(insn)	\
	((insn>>3)&0x3)
#define insn_get_vec_areg(insn)	\
	((insn>>12)&0xf)
#define insn_get_vec_vreg(insn)	\
	((insn>>17)&0x7)
#define insn_get_vec_nreg(insn)	\
	((insn>>5)&0x7)

enum vec_type {
	VEC_TYPE_BYTE   =0,
	VEC_TYPE_SHORT	=1,
	VEC_TYPE_WORD	=2
}; 	

static inline unsigned long get_ixa_va(struct pt_regs *regs, unsigned long user)
{
	unsigned int insn = sprget_gpt(IXI1H);
	unsigned int src = insn_get_src(insn);
	/* $a0 is user sp, so it's saved in regs->sp */
	if ((src == 0 && user) || (src == 15 && !user))
		return regs->sp;
	else
		return regs->a_regs[src];
}

int gpt_insn_read(void *addr, u32 *insnp, int size);
int gpt_insn_write(void *addr, u32* insn, int size);
u32 gpt_insn_gen_nop(void);
u32 gpt_insn_gen_call(void);
u32* gpt_insn_gen_branch(unsigned long pc, unsigned long addr, u32 insn[3]);

#endif /* !__ASSEMBLY__ */
#endif /* __ASM_INSN_H */
