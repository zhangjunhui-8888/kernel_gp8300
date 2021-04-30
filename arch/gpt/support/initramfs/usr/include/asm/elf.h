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

#ifndef __ASM_GPT_ELF_H
#define __ASM_GPT_ELF_H

/*
 * This files is partially exported to userspace.  This allows us to keep
 * the ELF bits in one place which should assist in keeping the kernel and
 * userspace in sync.
 */
#ifdef CONFIG_64BIT
#define ELF_GPT_R_SYM(r_info)         ELF64_R_SYM(r_info)
#define ELF_GPT_R_TYPE(r_info)        ELF64_R_TYPE(r_info)
#else
#define ELF_GPT_R_SYM(r_info)         ELF32_R_SYM(r_info)
#define ELF_GPT_R_TYPE(r_info)        ELF32_R_TYPE(r_info)
#endif


/* GPTX specific declarations */
#define R_GPTX_NONE                0
#define R_GPTX_REFWORD             1
#define R_GPTX_REFLONG             2
#define R_GPTX_32PCREL             3
#define R_GPTX_64PCREL             4
#define R_GPTX_PCOFF16             5
#define R_GPTX_UABS_G0             6
#define R_GPTX_UABS_G1             7
#define R_GPTX_UABS_G2             8
#define R_GPTX_UABS_G3             9
#define R_GPTX_APAGE_HIGH          10
#define R_GPTX_APAGE_LOW           11
#define R_GPTX_TPAGE_HIGH          12
#define R_GPTX_TPAGE_LOW           13
  /* Skip 14 - 17;   */
#define R_GPTX_GOT_HIGH            18
#define R_GPTX_GOT_LOW             19
#define R_GPTX_TLSGD_HIGH          20
#define R_GPTX_TLSGD_LOW           21
#define R_GPTX_TLSIE_GOTTPREL_HIGH 22
#define R_GPTX_TLSIE_GOTTPREL_LOW  23
#define R_GPTX_TLSLE_TPREL_HIGH    24
#define R_GPTX_TLSLE_TPREL_LOW     25
  /* Skip 26 - 29;   */
#define R_GPTX_COPY                30
#define R_GPTX_GLOB_DAT            31
#define R_GPTX_JUMP_SLOT           32
#define R_GPTX_RELATIVE            33
#define R_GPTX_TLS_DTPMOD          34
#define R_GPTX_TLS_DTPREL          35
#define R_GPTX_TLS_TPREL           36

/* for struct user_regs_struct definition */
#include <asm/ptrace.h>
#ifdef	CONFIG_VDSO
#include <uapi/asm/auxvec.h>
#endif

typedef unsigned long elf_greg_t;

/*
 * Note that NGREG is defined to ELF_NGREG in include/linux/elfcore.h, and is
 * thus exposed to user-space.
 */
#define ELF_NGREG (sizeof(struct user_pt_regs) / sizeof(elf_greg_t))
typedef elf_greg_t elf_gregset_t[ELF_NGREG];
typedef struct user_fpu_state elf_fpregset_t;

#ifdef	CONFIG_VDSO
#define ARCH_DLINFO							\
do {									\
	NEW_AUX_ENT(AT_SYSINFO_EHDR,					\
		    (elf_addr_t)current->mm->context.vdso_base);		\
} while (0)
#endif

/*
 * These are used to set parameters in the core dumps.
 */
#define ELF_ARCH	EM_GPT
#define ELF_CLASS	ELFCLASS64
#define ELF_DATA	ELFDATA2LSB
#define ELF_PLATFORM	("gptx")

#endif /* __ASM_GPT_ELF_H */
