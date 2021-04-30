#include <linux/moduleparam.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include <asm/insn.h>
#include <asm/vfp.h>

#define __get8_unaligned_check(ins,val,addr,err)		\
	asm volatile (						\
	"1:	"ins"	%1, %2, 1\n"				\
	"2:\n"							\
	"	.pushsection .fixup,\"ax\"\n"			\
	"	.align	3\n"					\
	"3:	rseti		%0,  1\n"			\
	"	taddpcil	$t4, 2b\n"			\
	"	taddti		$t1, $t4, 2b\n"			\
	"	j		$t1\n"				\
	"	.popsection\n"					\
	"	.pushsection __ex_table,\"a\"\n"		\
	"	.align	3\n"					\
	"	.dword	1b, 3b\n"				\
	"	.popsection\n"					\
	: "+r" (err), "=r" (val),"+a"(addr)			\
	: : "memory", "$t1", "$t4");			\

#define __get16_unaligned_check(ins,val,addr)			\
	do {							\
		unsigned long err = 0, v, a = addr;		\
		__get8_unaligned_check(ins,v,a,err);		\
		val =  v << 0;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 8;					\
		if(err)						\
			goto fault;				\
	} while (0)
#define __get32_unaligned_check(ins,val,addr)			\
	do {							\
		unsigned long err = 0, v, a = addr;		\
		__get8_unaligned_check(ins,v,a,err);		\
		val =  v << 0;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 8;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 16;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 24;					\
		if(err)						\
			goto fault;				\
	} while (0)	
#define __get64_unaligned_check(ins,val,addr)			\
	do {							\
		unsigned long err = 0, v, a = addr;		\
		__get8_unaligned_check(ins,v,a,err);		\
		val =  v << 0;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 8;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 16;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 24;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 32;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 40;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 48;					\
		__get8_unaligned_check(ins,v,a,err);		\
		val |= v << 56;					\
		if(err)						\
			goto fault;				\
	} while (0)	
	
#define get16_unaligned_check(val,addr) \
	__get16_unaligned_check("ldub",val,addr)	
#define get32_unaligned_check(val,addr) \
	__get32_unaligned_check("ldub",val,addr)	
#define get64_unaligned_check(val,addr) \
	__get64_unaligned_check("ldub",val,addr)	
	
#define put16_unaligned_check(val, addr)			\
	do {							\
		unsigned long err = 0, v = val, a = addr;	\
		asm volatile (					\
		"1:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"2:	stb	%1, %2\n"			\
		"3:\n"						\
		"	.pushsection .fixup,\"ax\"\n"		\
		"	.align	3\n"				\
		"4:	rseti	%0, 1\n"			\
		"	taddpcil	$t4, 3b\n"		\
		"	taddti		$t1, $t4, 3b\n"		\
		"	j	$t1\n"				\
		"	.popsection\n"				\
		"	.pushsection __ex_table,\"a\"\n"	\
		"	.align	3\n"				\
		"	.dword	1b, 4b\n"			\
		"	.dword	2b, 4b\n"			\
		"	.popsection\n"				\
		: "+r" (err), "+r" (v), "+a" (a)		\
		: : "memory", "$t1", "$t4");			\
		if(err)						\
			goto fault;				\
	} while (0)		
#define put32_unaligned_check(val, addr)			\
	do {							\
		unsigned long err = 0, v = val, a = addr;	\
		asm volatile (					\
		"1:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"2:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"3:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"4:	stb	%1, %2\n"			\
		"5:\n"						\
		"	.pushsection .fixup,\"ax\"\n"		\
		"	.align	3\n"				\
		"6:	rseti	%0, 1\n"			\
		"	taddpcil	$t4, 5b\n"		\
		"	taddti		$t1, $t4, 5b\n"		\
		"	j	$t1\n"				\
		"	.popsection\n"				\
		"	.pushsection __ex_table,\"a\"\n"	\
		"	.align	3\n"				\
		"	.dword	1b, 6b\n"			\
		"	.dword	2b, 6b\n"			\
		"	.dword	3b, 6b\n"			\
		"	.dword	4b, 6b\n"			\
		"	.popsection\n"				\
		: "+r" (err), "+r" (v), "+a" (a)		\
		: : "memory", "$t1", "$t4");			\
		if(err)						\
			goto fault;				\
	} while (0)	
	
#define put64_unaligned_check(val, addr)			\
	do {							\
		unsigned long err = 0, v = val, a = addr;	\
		asm volatile (					\
		"1:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"2:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"3:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"4:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"5:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"6:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"7:	stub	%1, %2, 1\n"			\
		"	shri	%1, %1, 8\n"			\
		"8:	stb	%1, %2\n"			\
		"9:\n"						\
		"	.pushsection .fixup,\"ax\"\n"		\
		"	.align	3\n"				\
		"10:	rseti	%0, 1\n"			\
		"	taddpcil	$t4, 9b\n"		\
		"	taddti		$t1, $t4, 9b\n"		\
		"	j	$t1\n"				\
		"	.popsection\n"				\
		"	.pushsection __ex_table,\"a\"\n"	\
		"	.align	3\n"				\
		"	.dword	1b, 10b\n"			\
		"	.dword	2b, 10b\n"			\
		"	.dword	3b, 10b\n"			\
		"	.dword	4b, 10b\n"			\
		"	.dword	5b, 10b\n"			\
		"	.dword	6b, 10b\n"			\
		"	.dword	7b, 10b\n"			\
		"	.dword	8b, 10b\n"			\
		"	.popsection\n"				\
		: "+r" (err), "+r" (v), "+a" (a)		\
		: : "memory", "$t1", "$t4");			\
		if(err)						\
			goto fault;				\
	} while (0)	

#define VPU_LOAD_V(v, from, num)	__asm__ __volatile__( \
			"rsetn_h $r14, $n0\n\t"    \
			"nsetr_h $n0, %1\n\t" \
			"ldv_b	"v", %0, $n0\n\t" \
			"nsetr_h $n0, $r14\n\t" \
			::"a"(from), "r"(num) : "memory", "$r14", v)

#define VPU_STORE_V(v, to, num)	__asm__ __volatile__( \
			"rsetn_h $r14, $n0\n\t"    \
			"nsetr_h $n0, %1\n\t" \
			"stv_b	"v", %0, $n0\n\t" \
			"nsetr_h $n0, $r14\n\t" \
			::"a"(to), "r"(num) : "memory", "$r14")

#define VPU_GET_N(n)	({         \
		int nv;             \
		__asm__ __volatile__( \
			"rsetn_h %0, "n"\n\t"     \
			:"=r"(nv));      \
			nv;	})     \

static inline unsigned int vpu_get_n(int id)
{
	int val;
	switch(id){
		case 0:
			val = VPU_GET_N("$n0");
			break;
		case 1:
			val = VPU_GET_N("$n1");
			break;
		case 2:
			val = VPU_GET_N("$n2");
			break;
		case 3:
			val = VPU_GET_N("$n3");
			break;
		case 4:
			val = VPU_GET_N("$n4");
			break;
		case 5:
			val = VPU_GET_N("$n5");
			break;
		case 6:
			val = VPU_GET_N("$n6");
			break;
		case 7:
			val = VPU_GET_N("$n7");
			break;
		default:
			BUG_ON(1);
			break;
	}
	return val;
}

static inline int vec_get_element_size(struct pt_regs *regs,u32 instr)
{
	switch(insn_get_vec_type(instr))
	{
		case VEC_TYPE_BYTE:
			return 1;
		case VEC_TYPE_SHORT:
			return 2;
		case VEC_TYPE_WORD:
			return 4;
		default:
			BUG_ON(1);
	}
}

static inline int vec_get_unaligned_check(struct pt_regs *regs,unsigned int instr, int update)
{
	unsigned int num, element_size = 0;
	unsigned char vp[VPU_V_SIZE], a, n, v;
	const u8* from;
	u8*to;

	a = insn_get_vec_areg(instr);
	n = insn_get_vec_nreg(instr);
	v = insn_get_vec_vreg(instr);
	element_size = vec_get_element_size(regs,instr);
	
	from =(const u8*)regs->a_regs[a];
	to = vp;

	pr_debug("%s: a %d n %d v %d from %p to %p pc %llx\n",__func__,a,n,v,from,to,regs->pc);
	num = vpu_get_n(n)*element_size;

	local_irq_enable();
	if(copy_from_user(to, from, num)){
		local_irq_disable();
		return -2;
	}
	local_irq_disable();

	switch(v){
		case 0:
			VPU_LOAD_V("$v0", to, num);
			break;
		case 1:
			VPU_LOAD_V("$v1", to, num);
			break;
		case 2:
			VPU_LOAD_V("$v2", to, num);
			break;
		case 3:
			VPU_LOAD_V("$v3", to, num);
			break;
		case 4:
			VPU_LOAD_V("$v4", to, num);
			break;
		case 5:
			VPU_LOAD_V("$v5", to, num);
			break;
		case 6:
			VPU_LOAD_V("$v6", to, num);
			break;
		case 7:
			VPU_LOAD_V("$v7", to, num);
			break;
		default:
			BUG_ON(1);
			break;
	}

	if(update)
		regs->a_regs[a] += num;

			
	return 0;
}

static inline int vec_put_unaligned_check(struct pt_regs *regs,unsigned int instr,int update)
{
	unsigned int num, element_size=0;	
	unsigned char vp[VPU_V_SIZE], a, n, v;
	const u8* from;
	u8*to;
	
	a = insn_get_vec_areg(instr);
	n = insn_get_vec_nreg(instr);
	v = insn_get_vec_vreg(instr);
	element_size=vec_get_element_size(regs,instr);
	
	to =(u8*)regs->a_regs[a];
	from = vp;

	pr_debug("%s: a %d n %d v %d from %p to %p\n",__func__,a,n,v,from,to);

	num = vpu_get_n(n)*element_size;

	switch(v){
		case 0:
			VPU_STORE_V("$v0", from, num);
			break;
		case 1:
			VPU_STORE_V("$v1", from, num);
			break;
		case 2:
			VPU_STORE_V("$v2", from, num);
			break;
		case 3:
			VPU_STORE_V("$v3", from, num);
			break;
		case 4:
			VPU_STORE_V("$v4", from, num);
			break;
		case 5:
			VPU_STORE_V("$v5", from, num);
			break;
		case 6:
			VPU_STORE_V("$v6", from, num);
			break;
		case 7:
			VPU_STORE_V("$v7", from, num);
			break;
		default:
			BUG_ON(1);
			break;
	}

	local_irq_enable();
	if(copy_to_user(to, from, num)){
		local_irq_disable();
		return -2;
	}		
	local_irq_disable();

	if(update)
		regs->a_regs[a] += num;

	
	return 0;
}
static void fixup_unaligned_load_store(unsigned long addr, unsigned int instr,
		    struct pt_regs *regs)
{
	unsigned int op_code = insn_get_opcode(instr);
	unsigned int ar=insn_get_src(instr);
	unsigned int rr=insn_get_dest(instr);
	long offset;
	asm (
	  "extrsi	%0,%1,0,12\n"
	  : "=r"(offset) : "r"(instr));

	pr_debug("instr %x op %x ar %x rr %x addr %lx\n",instr,op_code,ar,rr,addr);
	switch(op_code)
	{	
		case GPTXISA_OPCODE_LDH:	/* load instruction */
			get16_unaligned_check(regs->r_regs[rr],addr);
		break;	
		case GPTXISA_OPCODE_LDSH:
			get16_unaligned_check(regs->r_regs[rr],addr);
			regs->r_regs[rr] = (short)regs->r_regs[rr];
		break;	
		case GPTXISA_OPCODE_LDUH:		
			get16_unaligned_check(regs->r_regs[rr],addr);
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;	
		case GPTXISA_OPCODE_LDUSH:		
			get16_unaligned_check(regs->r_regs[rr],addr);
			regs->r_regs[rr] = (short)regs->r_regs[rr];
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		case GPTXISA_OPCODE_LDW:			
			get32_unaligned_check(regs->r_regs[rr],addr);
		break;	
		case GPTXISA_OPCODE_LDSW:		
			get32_unaligned_check(regs->r_regs[rr],addr);
			regs->r_regs[rr] = (int)regs->r_regs[rr];
		break;	
		case GPTXISA_OPCODE_LDUW:
			get32_unaligned_check(regs->r_regs[rr],addr);
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		case GPTXISA_OPCODE_LDUSW:		
			get32_unaligned_check(regs->r_regs[rr],addr);
			regs->r_regs[rr] = (int)regs->r_regs[rr];
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		//case GPTXISA_OPCODE_LDT:
		case GPTXISA_OPCODE_LDA:
			get64_unaligned_check(regs->a_regs[rr],addr);
		break;	
		case GPTXISA_OPCODE_LDL:
			get64_unaligned_check(regs->r_regs[rr],addr);
			//if (addr == 0x1048c84) {
			//	printk("exit: addr: %p, regs->r_regs[rr]: %lx\n", addr, regs->r_regs[rr]);
			//}
		break;	

		//case GPTXISA_OPCODE_LDUT:
		case GPTXISA_OPCODE_LDUL:
			get64_unaligned_check(regs->r_regs[rr],addr);
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		case GPTXISA_OPCODE_LDV:			
		case GPTXISA_OPCODE_LDVF:
			if(vec_get_unaligned_check(regs,instr, 0)){
				pr_alert("Unhandled floating point access,pc %llx address 0x%lx insn 0x%x\n",regs->pc,addr,instr);
				goto sigbus;
			}
		break;
		case GPTXISA_OPCODE_LDUV:
		case GPTXISA_OPCODE_LDUVF:
			if(vec_get_unaligned_check(regs,instr, 1)){
				pr_alert("Unhandled floating point access,pc %llx address 0x%lx insn 0x%x\n",regs->pc,addr,instr);
				goto sigbus;
			}
		break;
				
		case GPTXISA_OPCODE_STH:	/* store instruction */
			put16_unaligned_check(regs->r_regs[rr], addr);
		break;	
		case GPTXISA_OPCODE_STUH:		
			put16_unaligned_check(regs->r_regs[rr], addr);
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		case GPTXISA_OPCODE_STW:
			put32_unaligned_check(regs->r_regs[rr], addr);
		break;	
		case GPTXISA_OPCODE_STUW:		
			put32_unaligned_check(regs->r_regs[rr], addr);		
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		//case GPTXISA_OPCODE_STT:
		case GPTXISA_OPCODE_STA:
			put64_unaligned_check(regs->a_regs[rr], addr);				
		break;		
		case GPTXISA_OPCODE_STL:
			put64_unaligned_check(regs->r_regs[rr], addr);				
		break;		
		//case GPTXISA_OPCODE_STUT:
		case GPTXISA_OPCODE_STUL:
			put64_unaligned_check(regs->r_regs[rr], addr);
			regs->a_regs[ar] = offset + (long)regs->a_regs[ar];
		break;
		case GPTXISA_OPCODE_STV:			
		case GPTXISA_OPCODE_STVF:
			if(vec_put_unaligned_check(regs,instr, 0)){
				pr_alert("Unhandled vector access,pc %llx address 0x%lx insn 0x%x\n",regs->pc,addr,instr);
				goto sigbus;
			}
		break;	
		case GPTXISA_OPCODE_STUV:			
		case GPTXISA_OPCODE_STUVF:
			if(vec_put_unaligned_check(regs,instr, 1)){
				pr_alert("Unhandled vector access,pc %llx address 0x%lx insn 0x%x\n",regs->pc,addr,instr);
				goto sigbus;
			}
					
		break;	
		
		case GPTXISA_OPCODE_DCTL: 				
			goto fault;
		break;	
				
		default:
			pr_alert("Unhandled unaligned access,pc %llx address 0x%lx insn 0x%x\n",regs->pc,addr,instr);
			//force_sig(SIGSEGV, current);					
			//return;
			goto sigbus;
		break;		
	}
	regs->pc += 4;
	return;		
fault:	
	if (fixup_exception(regs))
		return;
	die_if_kernel("Unhandled kernel unaligned access", regs, addr);
	force_sig(SIGSEGV, current);					
	return;
sigbus:	
	die_if_kernel("Unhandled kernel unaligned access", regs, addr);
	force_sig(SIGBUS, current);
	return;	
}	

asmlinkage int do_alignment(struct pt_regs *regs, unsigned long ixi)
{
	unsigned int instr = ixi;
	unsigned long addr = get_ixa_va(regs, instr);
	
	fixup_unaligned_load_store(addr, instr, regs);
	return 0;
}
