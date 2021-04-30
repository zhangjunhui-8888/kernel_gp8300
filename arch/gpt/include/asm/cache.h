#ifndef __ASM_CACHE_H
#define __ASM_CACHE_H

#include <asm/page.h>
#include <machine/gpt_kernel.h>

#define ICACHE_LINE_BYTES	(1 << GPT_ICACHE_line_ln)
#define DCACHE_LINE_BYTES	(1 << GPT_DCACHE_line_ln)
#define L2CACHE_LINE_BYTES	(1 << GPT_2CACHE_line_ln)

#define ICACHE_LINE_COUNT	(1<<GPT_ICACHE_count_ln)
#define DCACHE_LINE_COUNT	(1<<GPT_DCACHE_count_ln)
#define L2CACHE_LINE_COUNT	(1<<GPT_2CACHE_count_ln)

#define L1_CACHE_BYTES		DCACHE_LINE_BYTES
#define L1_CACHE_SHIFT		GPT_DCACHE_line_ln

#define L2_CACHE_BYTES          L2CACHE_LINE_BYTES
#define ARCH_DMA_MINALIGN       L2_CACHE_BYTES
#define ARCH_SLAB_MINALIGN      L1_CACHE_BYTES


/*Notice: trap-chip2.S & atomic.h use the dctl_lock_l1 instruction too,
 * so please don't close the dcache, unless you know what you do!!*/
#define CACHE_MODE        ((_ONE << PSC_OFFSE_cache_ic) | (_ONE << PSC_OFFSE_cache_dc) | (_ONE << PSC_OFFSE_cache_l2c))

#endif /* __ASM_CACHE_H */
