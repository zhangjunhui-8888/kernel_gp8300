/* GPT loadable module support.
 *
 * Copyright (C) 2018, General Processor Techologies Inc.
 * scxie <scxie@hxgpt.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/elf.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleloader.h>
#include <linux/vmalloc.h>

#include <asm/insn.h>
#include <asm/pgtable.h>

void *module_alloc(unsigned long size)
{
	return __vmalloc_node_range(size, 1, MODULE_START, MODULE_END,
				GFP_KERNEL, PAGE_KERNEL, NUMA_NO_NODE,
				__builtin_return_address(0));
}

int apply_relocate_add(Elf64_Shdr *sechdrs,
		       const char *strtab,
		       unsigned int symindex,
		       unsigned int relsec,
		       struct module *me)
{
	unsigned int i;
	Elf64_Sym *sym;
	void *location;
	uint64_t value;
	Elf64_Rela *rel = (void *)sechdrs[relsec].sh_addr;
	int64_t upper, lower;
	uint32_t code;

	pr_debug("Applying relocate section %u to %u\n", relsec,
		 sechdrs[relsec].sh_info);
	pr_debug("rela->r_off | rela->addend | sym->st_value | ADDR | VALUE\n");
	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			+ rel[i].r_offset;
		/* This is the symbol it is referring to. */
		sym = (Elf64_Sym *)sechdrs[symindex].sh_addr
			+ ELF64_R_SYM(rel[i].r_info);

                if (IS_ERR_VALUE(sym->st_value)) {
                        /* Ignore unresolved weak symbol */
                        if (ELF_ST_BIND(sym->st_info) == STB_WEAK)
                                continue;
                        pr_warning("%s: Unknown symbol %s\n",
                                   me->name, strtab + sym->st_name);
                        return -ENOENT;
                }                
		value = sym->st_value + rel[i].r_addend;

		pr_debug("\t%llx\t\t%llx\t\t%llx  %p %llx [%s]\n",
			rel[i].r_offset, rel[i].r_addend,
			sym->st_value, location, value,
			strtab + sym->st_name);

		switch (ELF64_R_TYPE(rel[i].r_info)) {
		case R_GPTX_NONE:
			break;
		case R_GPTX_REFWORD:
			*(u32 *)location = (u32)value;
			break;
		case R_GPTX_REFLONG:
			*(u64 *)location = (u64)value;
			break;
		case R_GPTX_APAGE_HIGH:
			/*	aaddpcilb	$ax, 0
			 */
			code = *(uint32_t*)location;
			code = code & (~0xffff);
			upper = (int64_t)(((uint64_t)value >> 15) - ((uint64_t)location >> 15));
			*(uint32_t*)location = code | (upper & 0xffff);
			break;
		case R_GPTX_APAGE_LOW:
			/*	aadduib		$ax, 0
			 */
			code = *(uint32_t*)location;
			code = code & (~0xffff);
			lower = value & 0x7fff;
			*(uint32_t*)location = code | lower;
			break;
		case R_GPTX_TPAGE_HIGH:
			/*	taddpcil	$tx, 0
			 */
			code = *(uint32_t*)location;
			code = code & (~0xffff);
			upper = (int64_t)(((uint64_t)value >> 14) - ((uint64_t)location >> 14));
			*(uint32_t*)location = code | (upper & 0xffff);
			break;
		case R_GPTX_TPAGE_LOW:
			/*	taddti		$ty, $tx, 0
			 */
			code = *(uint32_t*)location;
			code = code & (~0xfff);
			lower = (value & 0x3fff)>>2;
			*(uint32_t*)location = code | lower;
			break;
		default:
			pr_err("module %s: Unknown relocation: %llu\n",
			       me->name, ELF64_R_TYPE(rel[i].r_info));
			return -ENOEXEC;
		}
	}

	return 0;
}
