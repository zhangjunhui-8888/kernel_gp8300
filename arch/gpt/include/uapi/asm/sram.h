#ifndef __UAPI_ASM_SRAM_H__
#define __UAPI_ASM_SRAM_H__

#define SRAM_IOCTL_ALLOC	_IOW('S', 1, size_t) /* allocate sram buffer */
#define SRAM_IOCTL_FREE		_IO('S', 2) /* free sram buffer */
#define SRAM_IOCTL_COPY		_IOW('S', 4, size_t)

struct vdma_desc {
	dma_addr_t dst;
	dma_addr_t src;
	size_t size;
};

#endif /* __UAPI_ASM_SRAM_H__ */
