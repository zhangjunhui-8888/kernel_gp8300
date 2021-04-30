#ifndef __ASM_GPT_SPARSEMEM_H
#define __ASM_GPT_SPARSEMEM_H

#ifdef __KERNEL__
/*
 * SECTION_SIZE_BITS		2^N: how big each section will be
 * MAX_PHYSADDR_BITS		2^N: how much physical address space we have
 * MAX_PHYSMEM_BITS		2^N: how much memory we can have in that space
 */
#define SECTION_SIZE_BITS	30
#define MAX_PHYSADDR_BITS	40
#define MAX_PHYSMEM_BITS	40

#endif

#endif /* __ASM_GPT_SPARSEMEM_H */
