#ifndef _ASM_GPT_FTRACE_H
#define _ASM_GPT_FTRACE_H

#include <asm/insn.h>

#define MCOUNT_ADDR		((unsigned long)(_mcount))
#define MCOUNT_INSN_SIZE	GPT_INSN_SIZE /* sizeof mcount call */

#ifndef __ASSEMBLY__

extern void _mcount(void);
extern void *return_address(unsigned int);

struct dyn_arch_ftrace {
};

extern unsigned long ftrace_graph_call;

static inline unsigned long ftrace_call_adjust(unsigned long addr)
{
       /* reloction of mcount call site is the same as the address */
       return addr;
}

#define ftrace_return_address(n) return_address(n)

#endif /* __ASSEMBLY__ */

#endif /* _ASM_GPT_FTRACE_H */
