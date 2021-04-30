#ifndef __GPT_PROTOCOL_H__
#define __GPT_PROTOCOL_H__

#define IOCTL_GSNN_LOADCFG	_IOWR('G', 1, int)
#define IOCTL_GSNN_WRDATA	_IOWR('G', 2, int)
#define IOCTL_GSNN_GETRES	_IOWR('G', 3, int)
#define IOCTL_GSNN_ENABLE	_IOWR('G', 4, int)
#define IOCTL_GSNN_SECT		_IOWR('G', 5, int)
#define IOCTL_GSNN_REG_EN	_IOWR('G', 6, int)
#define IOCTL_GSNN_LOADNET	_IOWR('G', 7, int)
#define IOCTL_GSNN_GETPARAM	_IOWR('G', 8, int)
#define IOCTL_GSNN_RDATA	_IOWR('G', 9, int)
#define IOCTL_GSNN_GLOBAL	_IOWR('G', 10, int)
#define IOCTL_GSNN_S_REGMAP	_IOWR('G', 11, int)
#define IOCTL_GSNN_G_REGMAP	_IOWR('G', 12, int)
#define IOCTL_GSNN_REGREAD      _IOWR('G', 13, int)
#define IOCTL_GSNN_REGWRITE     _IOWR('G', 14, int)
#define IOCTL_GSNN_MEMREAD      _IOWR('G', 15, int)
#define IOCTL_GSNN_MEMWRITE     _IOWR('G', 16, int)
#define IOCTL_GSNN_REG_R     _IOWR('G', 17, int)
#define IOCTL_GSNN_REG_W     _IOWR('G', 18, int)
#define IOCTL_GSNN_MEM_R     _IOWR('G', 19, int)
#define IOCTL_GSNN_MEM_W     _IOWR('G', 20, int)
#define IOCTL_GSNN_SET_EXTSRC_CFG	 _IOWR('G', 21, int)
#define IOCTL_GSNN_MEM_DUMP    _IOWR('G', 22, int)

#if 0
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
#endif

#pragma pack(1)
struct gnx_header {
	uint32_t h_magic;//4B
	uint32_t h_version;//4B
	uint16_t h_filetype;//2B
	uint16_t h_headersize;//2B
	uint16_t h_numsection;//2B
	uint16_t h_sectionsize;//2B
	uint32_t h_sectionoffset;//4B
};
#pragma pack()

#pragma pack(1)
struct gnx_table {
	uint16_t s_engine;//2B
	uint16_t s_section;//2B
	uint32_t s_offset;//4B
	uint32_t s_size;//4B
	uint32_t s_addr;//4B
	uint32_t s_numsymtab;//4B
	uint32_t s_symtaboffset;//4B
};
#pragma pack()

struct gsnn_header {
	int addr;
	int length;
	int start;
};

struct gpt_gsnn_param {
	unsigned long hwdata_base;
	unsigned long hwdata_size;
};

#pragma pack(1)
struct gpt_gsnn_reg{
	uint32_t reg_addr;
	uint32_t value;
};	
#pragma pack()

#pragma pack(1)
struct gpt_gsnn_mem{
	uint64_t phys_addr;
	uint32_t len;
};	
#pragma pack()


#endif
