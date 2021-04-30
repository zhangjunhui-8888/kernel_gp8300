/*
 * arch/gpt/kernel/ftrace.c, refer to arm64
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/ftrace.h>
#include <linux/uaccess.h>

#include <asm/ftrace.h>
#include <asm/cacheflush.h>
#include <asm/insn.h>

#ifdef CONFIG_DYNAMIC_FTRACE

static int ftrace_modify_code(unsigned long addr, u32 insn[], int cnt)
{
	int ret, size;
	size = cnt * GPT_INSN_SIZE;

	ret = gpt_insn_write((void*)addr, insn, size);
	if(!ret){
		addr = round_down(addr, L1_CACHE_BYTES);
		size = round_up(size, L1_CACHE_BYTES);
		__sync_dcache_range(addr, addr + size);
		__invalidate_icache_range(addr, addr + size);
	}		
	return ret;
}
/*
 * Replace tracer function in ftrace_caller()
 */
int ftrace_update_ftrace_func(ftrace_func_t func)
{
	u32 new[3];
	unsigned long pc;

	pc = (unsigned long)&ftrace_call;
	gpt_insn_gen_branch(pc, (unsigned long)func, new);

	return ftrace_modify_code(pc, new, 3);
}

/*
 * Turn on the call to ftrace_caller() in instrumented function
 */
int ftrace_make_call(struct dyn_ftrace *rec, unsigned long addr)
{
	u32 new[3];
	unsigned long pc = rec->ip;

	gpt_insn_gen_branch(pc, addr, new);

	return ftrace_modify_code(pc, new, 3);
}

/*
 * Turn off the call to ftrace_caller() in instrumented function
 */
int ftrace_make_nop(struct module *mod, struct dyn_ftrace *rec,
		    unsigned long addr)
{
	u32 new[3];
	unsigned long pc = rec->ip;
	
	new[0] = gpt_insn_gen_nop();
	new[1] = gpt_insn_gen_nop();	
	new[2] = gpt_insn_gen_nop();		

	return ftrace_modify_code(pc, new, 3);
}

int __init ftrace_dyn_arch_init(void)
{
	return 0;
}
#endif /* CONFIG_DYNAMIC_FTRACE */

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
/*
 * function_graph tracer expects ftrace_return_to_handler() to be called
 * on the way back to parent. For this purpose, this function is called
 * in _mcount() or ftrace_caller() to replace return address (*parent) on
 * the call stack to return_to_handler.
 *
 * Note that @frame_pointer is used only for sanity check later.
 */
void prepare_ftrace_return(unsigned long *parent, unsigned long self_addr,
			   unsigned long frame_pointer)
{
	unsigned long return_hooker = (unsigned long)&return_to_handler;
	unsigned long old;
	struct ftrace_graph_ent trace;
	int err;

	if (unlikely(atomic_read(&current->tracing_graph_pause)))
		return;

	/*
	 * Note:
	 * No protection against faulting at *parent, which may be seen
	 * on other archs.
	 */
	old = *parent;
	*parent = return_hooker;

	trace.func = self_addr;
	trace.depth = current->curr_ret_stack + 1;

	/* Only trace if the calling function expects to */
	if (!ftrace_graph_entry(&trace)) {
		*parent = old;
		return;
	}

	err = ftrace_push_return_trace(old, self_addr, &trace.depth,
				       frame_pointer);
	if (err == -EBUSY) {
		*parent = old;
		return;
	}
}

#ifdef CONFIG_DYNAMIC_FTRACE
/*
 * Turn on/off the call to ftrace_graph_caller() in ftrace_caller()
 * depending on @enable.
 */
static int ftrace_modify_graph_caller(bool enable)
{
	u32 insns[3];
	unsigned long pc = (unsigned long)&ftrace_graph_call;
	
	if (enable) {
		gpt_insn_gen_branch(pc,
				    (unsigned long)ftrace_graph_caller,
				    insns);
	} else {	
		insns[0] = gpt_insn_gen_nop();
		insns[1] = gpt_insn_gen_nop();
		insns[2] = gpt_insn_gen_nop();		
	}
	return ftrace_modify_code(pc, insns, 3);
}

int ftrace_enable_ftrace_graph_caller(void)
{
	return ftrace_modify_graph_caller(true);
}

int ftrace_disable_ftrace_graph_caller(void)
{
	return ftrace_modify_graph_caller(false);
}
#endif /* CONFIG_DYNAMIC_FTRACE */
#endif /* CONFIG_FUNCTION_GRAPH_TRACER */
