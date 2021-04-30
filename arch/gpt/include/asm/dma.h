/*
 * linux/arch/gpt/include/asm/dma.h
 *
 * Copyright (C) hxgpt.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_GPT_DMA_H__
#define __ASM_GPT_DMA_H__

#define MAX_DMA_ADDRESS		PAGE_OFFSET

extern int request_dma(unsigned int dmanr, const char *device_id);
extern void free_dma(unsigned int dmanr);

#ifdef CONFIG_PCI
	extern int isa_dma_bridge_buggy;
#else
	#define isa_dma_bridge_buggy 	(0)
#endif

#endif /* __ASM_GPT_DMA_H__ */
