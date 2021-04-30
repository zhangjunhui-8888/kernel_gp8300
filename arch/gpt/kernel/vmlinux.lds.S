/*
 * gpt vmlinux.lds.S
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
 *
 * ld script for gpt architecture
 */

#define LOAD_OFFSET  PAGE_OFFSET
#define LOAD_BASE    (PAGE_OFFSET + CONFIG_PHYSICAL_START)

#include <asm/thread_info.h>
#include <asm/bitsperlong.h>
#include <asm/page.h>
#include <asm/cache.h>
#include <asm-generic/vmlinux.lds.h>

jiffies = jiffies_64;

#define VECTOR_TEXT                                     \
	. = ALIGN(PAGE_SIZE*2);				\
	VMLINUX_SYMBOL(__vector_tables_start) = .;      \
	*(.vector_tables.text)                          \
	VMLINUX_SYMBOL(__vector_tables_end) = .;

SECTIONS
{
        /* Read-only sections, merged into text segment: */
        . = LOAD_BASE ;

        /* _s_kernel_ro must be page aligned */
        . = ALIGN(PAGE_SIZE);
        _s_kernel_ro = .;

        _text = .;	/* Text and read-only data */
        .text                   : AT(ADDR(.text) - LOAD_OFFSET)
        {
          _stext = .;
          HEAD_TEXT
          TEXT_TEXT
          SCHED_TEXT
          LOCK_TEXT
          KPROBES_TEXT
          IRQENTRY_TEXT
          VECTOR_TEXT
          *(.fixup)
          *(.text.__*)
          _etext = .;
        }

        _sdata = .;

        /* Page alignment required for RO_DATA_SECTION */
        RO_DATA_SECTION(PAGE_SIZE)
        _e_kernel_ro = .;

	EXCEPTION_TABLE(8)

        /* Whatever comes after _e_kernel_ro had better be page-aligend, too */

        /* 32 here is cacheline size... recheck this */
        RW_DATA_SECTION(L1_CACHE_BYTES, PAGE_SIZE, THREAD_SIZE)

        _edata  =  .;

        NOTES

        /* Init code and data */
        . = ALIGN(PAGE_SIZE);
        __init_begin = .;

        /* Page aligned */
        INIT_TEXT_SECTION(PAGE_SIZE)

        /* Align __setup_start on 16 byte boundary */
        INIT_DATA_SECTION(16)

        /* we have to discard exit text and such at runtime, not link time */
        .exit.text :
        {
                EXIT_TEXT
        }
        .exit.data :
        {
                EXIT_DATA
        }
        PERCPU_SECTION(L1_CACHE_BYTES)

        __init_end = .;

        __vmlinux_end = .;            /* last address of the physical file */

        BSS_SECTION(0, 0, 0x20)

        _end = .;

        /* Throw in the debugging sections */
        STABS_DEBUG
        DWARF_DEBUG

        /* Sections to be discarded -- must be last */
        DISCARDS
}
