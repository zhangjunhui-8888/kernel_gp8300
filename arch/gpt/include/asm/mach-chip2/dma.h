#ifndef __CPU_DMA_H
#define __CPU_DMA_H

#define CDMA_ADDR_ALLOC			0x0
#define CDMA_ADDR_FREE			0x1
#define CDMA_ADDR_MMAP			0x3
#define CDMA_COPY			0x4

enum CDMA_TYPE {
	CDMA_PHYS_1D_1D,
};

struct dma_addr_desc {
	dma_addr_t phy_addr;
	char *cpu_addr;
	ulong user_virt;
	uint32_t len;
};

struct cdma_desc {
	struct dma_addr_desc *dst;
	struct dma_addr_desc *src;
	uint32_t len;
	enum CDMA_TYPE cmd;
};

#endif
