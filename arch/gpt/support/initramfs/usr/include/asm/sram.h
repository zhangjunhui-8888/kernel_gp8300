#ifndef __UAPI_ASM_SRAM_H__
#define __UAPI_ASM_SRAM_H__

#define SRAM_IOCTL_ALLOC	_IOW('S', 1, size_t) /* allocate sram buffer */
#define SRAM_IOCTL_FREE		_IO('S', 2) /* free sram buffer */

#endif /* __UAPI_ASM_SRAM_H__ */
