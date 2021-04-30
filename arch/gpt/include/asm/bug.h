#ifndef __ASM_GPT_BUG_H
#define __ASM_GPT_BUG_H

#include <linux/linkage.h>
#include <asm-generic/bug.h>

struct pt_regs;
struct siginfo;
/* arch/gpt/kernel/traps.c */
extern void die(const char *str, struct pt_regs *regs, int err);
extern void die_if_kernel(const char *str, struct pt_regs *regs, int err);
extern void gpt_notify_die(const char *str, struct pt_regs *regs,
		      struct siginfo *info, int err);
extern void hook_fault_code(int nr, int (*fn)(struct pt_regs *regs, unsigned long ixi),
		int sig, int code, const char *name);
#endif /* __ASM_GPT_BUG_H */
