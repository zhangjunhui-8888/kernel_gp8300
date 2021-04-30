#ifndef __GPT_FB_H__
#define __GPT_FB_H__

/*config extcr_lo_reg and extcr_hi_reg*/
#define GPT_APIU_ENABLE_MASK	0xffff0000
#define GPT_EXTCR_IO_REG			0xf0000100
#define GPT_EXTCR_HI_REG			0xf0000104
#define GPT_VOUT_ENABLE			(0x1 << 7)
#define GPT_VIDMEM_ENABLE		(0x1 << 8)
#define GPT_VID_ENABLE			(0x1 << 13)
#define GPT_VIDMEM_SLEEP			(0x1 << 8)
#define GPT_VIDMEM_SHUTDOWN	(0x1 << 9)

/*vout register list*/
#define REG_DVO_ENABLE		0x00
#define REG_DVO_DMA_POSITION	0x04
#define REG_DVO_BACKGROUND	0x08
#define REG_DVO_HACTIVE		0x0c
#define REG_DVO_HDISPLAY	0x10
#define REG_DVO_HSYNC		0x14
#define REG_DVO_VACTIVE_ODD	0x18
#define REG_DVO_BUTTOM_BLK_ODD	0x1c
#define REG_DVO_VDISPLAY_ODD	0x20
#define REG_DVO_VSYNC_ODD	0x24
#define REG_DVO_VSYNC_HNUM_ODD	0x28
#define REG_DVO_VACTIVE_EVEN	0x2c
#define REG_DVO_BUTTOM_BLK_EVEN	0x30
#define REG_DVO_VDISPLAY_EVEN	0x34
#define REG_DVO_VSYNC_EVEN	0x38
#define REG_DVO_VSYNC_HNUM_EVEN	0x3c
#define REG_DVO_INT_ST		0x40
#define REG_DVO_INT_ST_RAW	0x44
#define REG_DVO_INT_MASK	0x48
#define REG_DVO_INT_CLEAR	0x4c
#define REG_DVO_INT_LINE_NUM	0x50
#define REG_DVO_DMA_BASE0	0x54
#define REG_DVO_DMA_BASE1	0x58
#define REG_DVO_DMA_HDAT	0x5c
#define REG_DVO_DMA_HIGH	0x60
#define REG_DVO_DMA_STRIDE	0x64
#define REG_DVO_DMA_CONFIG	0x68
#define REG_DVO_DMA_BASE0_H	0x6c
#define REG_DVO_DMA_BASE1_H	0x70
#define REG_DVO_AXI_CONFIG	0x74

/*video dvo_enable register*/
#define GPT_DVO_ENABLE		(0x01 << 0)
#define GPT_DVO_FORMAT_8BIT	(0x00 << 4)
#define GPT_DVO_FORMAT_10BIT	(0x01 << 4)
#define GPT_DVO_FORMAT_16BIT	(0x02 << 4)
#define GPT_DVO_INTER_EN	(0x01 << 8)
#define GPT_DVO_PROGR_EN	(0x00 << 8)
#define GPT_DVO_EVEN		(0x01 << 9)
#define GPT_DVO_SYNC_EN		(0x01 << 10)
#define GPT_DVO_SYNC_DIS	(0x00 << 10)
#define GPT_DVO_DE_HI		(0x01 << 11)
#define GPT_DVO_HSYNC_HI	(0x01 << 12)
#define GPT_DVO_VSYNC_HI	(0x01 << 13)
#define GPT_DVO_CLK_INV		(0x01 << 21)

/*video dvo_int register*/
#define GPT_DVO_INT_DMA_CMP	(0x01 << 0)
#define GPT_DVO_INT_FIFO_UNDER	(0x01 << 1)
#define GPT_DVO_INT_SOF		(0x01 << 2)
#define GPT_DVO_INT_EOF		(0x01 << 3)
#define GPT_DVO_INT_LINE	(0x01 << 4)

#define GPT_DVO_RAW_DMA_CMP	(0x01 << 0)
#define GPT_DVO_RAW_FIFO_UNDER	(0x01 << 1)
#define GPT_DVO_RAW_SOF		(0x01 << 2)
#define GPT_DVO_RAW_EOF		(0x01 << 3)
#define GPT_DVO_RAW_LINE	(0x01 << 4)

#define GPT_DVO_MSK_DMA_CMP	(0x01 << 0)
#define GPT_DVO_MSK_FIFO_UNDER	(0x01 << 1)
#define GPT_DVO_MSK_SOF		(0x01 << 2)
#define GPT_DVO_MSK_EOF		(0x01 << 3)
#define GPT_DVO_MSK_LINE	(0x01 << 4)

#define GPT_DVO_CLR_DMA_CMP	(0x01 << 0)
#define GPT_DVO_CLR_FIFO_UNDER	(0x01 << 1)
#define GPT_DVO_CLR_SOF		(0x01 << 2)
#define GPT_DVO_CLR_EOF		(0x01 << 3)
#define GPT_DVO_CLR_LINE	(0x01 << 4)

/*vedio dma_config register*/
#define GPT_DVO_DMA_SEMIP_EN	(0x01 << 9)
#define GPT_DVO_DMA_PACKE_EN	(0x00 << 9)
#define GPT_DVO_AXI_DMA_ARQOS	(0x0a << 8)

#define GPT_VIDMEM_ADDR		0x20000000
#ifdef CONFIG_FB_GPT_VOUT_MEM_VIDMEM
#define GPT_DMA_BASE_ADDR0	GPT_VIDMEM_ADDR
#else
#define GPT_DMA_BASE_ADDR0	0x220000000
#endif
#define GPT_DMA_BASE_ADDR1	0x230000000

#define GPT_VDO_DMA_START       0x00010000

//#define GPT_RGB2YUV_CONVERT
#define GPT_COLOR_CONVERT_FREQ		(30 * HZ / 1)
#define GPT_YUV_DATA_OFFSET	0x800000

#ifdef GPT_VDO_DEBUG
#endif

#define GPT_CAMARE_CLK		1
#define GPT_VIDEO_IN_CLK	0
#define GPT_EXTERN_CLK		3

#define FBIOGET_MODE		_IOR('F', 0x9, int)
#define FBIOSET_MODE		_IOW('F', 0x9, int)
#define FBIOGET_DMA_BASE_ADDR0	_IOR('F', 0x0A, int)
#define FBIOSET_DMA_BASE_ADDR0	_IOW('F', 0x0A, int)
#define FBIOGET_CLK_SOURCE	_IOR('F', 0x0B, int)
#define FBIOSET_CLK_SOURCE	_IOW('F', 0x0B, int)

struct gpt_fb_palette {
	struct fb_bitfield	r;
	struct fb_bitfield	g;
	struct fb_bitfield	b;
	struct fb_bitfield	a;
};


struct gpt_fbinfo {
	void __iomem	*regs;
	struct fb_info	*fbinfo;
	struct timer_list timer;
	struct task_struct *thread;
	struct gpt_fb_palette palette;
	unsigned int pseudo_palette[16];
	unsigned int *palette_buffer;
};

struct gpt_vdo_param {
	unsigned int vdo_mode;
	unsigned int data_mode;
	unsigned int sync_mode;
	unsigned int disp_mode;
	unsigned int vdo_polarity;

	/*for horizontal sync*/
	unsigned int dvo_ha;
	unsigned int dvo_hd;
	unsigned int dvo_hb;
	unsigned int dvo_hc;
	unsigned int dvo_he;
	unsigned int dvo_hf;
	/*for odd field*/
	unsigned int dvo_va_o;
	unsigned int dvo_vd_o;
	unsigned int dvo_vb_o;
	unsigned int dvo_vc_o;
	unsigned int dvo_ve_o;
	unsigned int dvo_vf_o;
	unsigned int dvo_vg_o;
	/*for even field*/
	unsigned int dvo_va_e;
	unsigned int dvo_vd_e;
	unsigned int dvo_vb_e;
	unsigned int dvo_vc_e;
	unsigned int dvo_ve_e;
	unsigned int dvo_vf_e;
	unsigned int dvo_vg_e;
	unsigned int hnum_even;
	unsigned int hnum_odd;
};

struct gpt_fbparam {
	unsigned int flag;	//de, vsync and hsync level
	unsigned int hstart;	//horizen position
	unsigned int vstart;	//vect position
	unsigned int hlen;	//horizen len
	unsigned int vlen;	//vect len

	unsigned int vbtm;
	//sync
	unsigned int hedge_one;
	unsigned int hedge_sec;
	unsigned int vedge_one;
	unsigned int vedge_sec;
} fbparam[] = {
{GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 160, 35, 640, 480, 10, 16, 112, 0, 1}, //640x480p
{GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 138, 42, 720, 480, 4, 16, 78, 0, 1}, //720x480p
{GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 370, 25, 1280, 720, 5, 110, 150, 0, 5}, //1280x720p
                                                     //ha  va  hd-ha hc-hb vf he   hf   ve vf
{GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 830, 40, 1920, 1080, 4, 638, 682, 0, 5}, //1920x1080p@24   ??????
{GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 279, 40, 1920, 1080, 4, 88, 132, 0, 5}, //1920x1080p@60   ??????
};

struct gpt_yuv_pla {
	int y;
	int u;
	int v;
} gpt_yuv[] = {
{0, 128, 128},
{19, 213, 114},
{99, 71, 56},
{119, 156, 42},
{50, 99, 213},
{70, 184, 199},
{100, 71, 177},
{169, 127, 127},
{84, 127, 127},
{104, 212, 113},
{184, 71, 56},
{204, 156, 42},
{135, 98, 212},
{155, 183, 198},
{235, 42, 141},
{254, 127, 127},
};

struct gpt_vdo_config {
	int bittype;
	int syncmode;
	int hostid;
	int resl;
};

enum {
	EM_SYNC_720P60,
	EM_SYNC_480P,
	EM_SYNC_480I,
	EM_SYNC_576P,
	EM_SYNC_576I,
	EM_SYNC_1080P50,
	EM_SYNC_1080P60,
	EM_SYNC_1080I30,
	EM_SYNC_1080P24,
	EM_SYNC_1080P30,  //add by lyy 2019-06-11
	EM_SYNC_480P10B,
	EM_SYNC_RESERVE,
};

struct gpt_vdo_config resolution[] = {

/*index0, 1080p 74.25M, sync packet prog*/
{16, 1, 0, EM_SYNC_RESERVE},
/*1080p sync plannar prog*/
{16, 1, 1, EM_SYNC_RESERVE},
/*1080p embeded packet prog*/
{16, 0, 2, EM_SYNC_1080I30},
/*1080p embeded plannar prog*/
{16, 0, 3, EM_SYNC_1080I30},

/*index4, 720p, 74.25M, sync mode, packet*/
{16, 1, 4, EM_SYNC_RESERVE},
/*720p, sync mode, semi planar*/
{16, 1, 5, EM_SYNC_RESERVE},
/*720p, embeded mode, packet*/
{16, 0, 6, EM_SYNC_720P60},
/*720p, embeded mode, packet, semi plannar*/
{16, 0, 7, EM_SYNC_720P60},

/*index8, 640x480p, 25.2M, sync mode*/
{16, 1, 8, EM_SYNC_RESERVE},
/*640x480p, sync mode, semi planar*/
{16, 1, 9, EM_SYNC_RESERVE},
/*640x480p, embeded mode*/
{16, 0, 10, EM_SYNC_480P},
/*640x480p, embeded mode, semi plannar*/
{16, 0, 11, EM_SYNC_480P},

/*index12, 480p, 25.2M, sync mode*/
{16, 1, 12, EM_SYNC_480P},
/*480p, sync mode, semi plannar*/
{16, 1, 13, EM_SYNC_480P},
/*480p embeded mode*/
{16, 0, 14, EM_SYNC_480P},
/*480p embeded mode, semi planar*/
{16, 0, 15, EM_SYNC_480P},

/*index16, 480i, sync mode interlace*/
{8, 1, 16, EM_SYNC_RESERVE},
/*480i, embeded mode interlace*/
{8, 1, 17, EM_SYNC_RESERVE},
/*480i, sync mode interlace*/
{8, 0, 18, EM_SYNC_480I},
/*480i,  sync mode interlace*/
{8, 0, 19, EM_SYNC_480I},

/*index20,8bit, 576i@50, 27M, sync, packet*/
{8, 1, 20, EM_SYNC_RESERVE},
/*8bit, 576i@50, 27M, sync, semi-plannar*/
{8, 1, 21, EM_SYNC_RESERVE},
/*8bit, 576i@50, 27M, embeded, packet*/
{8, 0, 22, EM_SYNC_576I},
/*8bit, 576i@50, 27M, embeded, semi-plannar*/
{8, 0, 23, EM_SYNC_576I},

/*index 24,720p@60 8bit syncmode, interlace*/
{8, 1, 24, EM_SYNC_720P60},	//----ok
/*720p@60 8bit syncmode, interlace*/
{8, 1, 25, EM_SYNC_720P60},	//---ok
/*720p@60 8bit syncmode, prog*/
{8, 0, 26, EM_SYNC_720P60},
/*720p@60 8bit embeded mode, interlace*/
{8, 0, 27, EM_SYNC_720P60},

/*index 28,1920x1080p@30 16bit syncmode, prog*/
{16, 1, 28, EM_SYNC_1080P24},
/*1920x1080p@60 16bit syncmode, prog*/
{16, 1, 29, EM_SYNC_1080P60},
/*1920x1080p@60 16bit syncmode, prog*/
{16, 1, 30, EM_SYNC_1080P60},
/*reserve*/
{16, 1, 31, 0},

/*index32 1920x1080i@60 16bit sync, packet*/
{16, 1, 32, EM_SYNC_1080P60},
/*1920x1080i@60 16bit sync, semi-plannar*/
{16, 1, 33, EM_SYNC_1080P60},
/*1920x1080i@60 16bit embeded , packet*/
{16, 0, 34, EM_SYNC_1080P60},
/*1920x1080i@60 16bit embeded, semi-plannar*/
{16, 0, 35, EM_SYNC_1080P60},

/*index36 480p@60 54M, 8bit sync, packet*/
{8, 1, 36, EM_SYNC_480P},
/*480p@30 54M, 8bit sync, semi-plannar*/
{8, 1, 37, EM_SYNC_480P},
/*480p@60 54M, 8bit sync, packet*/
{8, 0, 38, EM_SYNC_480P},
/*480p@60 54M, 8bit sync, semi-plannar*/
{8, 0, 39, EM_SYNC_480P},

/*index40 640x480p@60 8bit, 50.4M, sync, packet*/
{8, 1, 40, EM_SYNC_480P},
/*640x480p@60 8bit, 50.4M, sync, packet*/
{8, 1, 41, EM_SYNC_480P},
/*640x480p@60 8bit, 50.4M, sync, packet*/
{8, 0, 42, EM_SYNC_480P},
/*640x480p@60 8bit, 50.4M, sync, packet*/
{8, 0, 43, EM_SYNC_480P},

/*index44 480p@60 10bit, 54M, sync, packet*/
{10, 1, 44, EM_SYNC_480P},
/*480p@60 10bit, 54M, sync, packet*/
{10, 1, 45, EM_SYNC_480P},
/*480p@60 10bit, 54M, sync, packet*/
{10, 0, 46, EM_SYNC_480P},
/*480p@60 10bit, 54M, sync, packet*/
{10, 0, 47, EM_SYNC_480P},

/*index48 1080p@30 8bit, 148.5M, sync, packet*/
{8, 1, 48, EM_SYNC_1080P24},
/*1080p@30 8bit, 148.5M, sync, packet*/
{8, 1, 49, EM_SYNC_1080P24},
/*1080p@30 8bit, 148.5M, sync, packet*/
{8, 0, 50, EM_SYNC_1080P24},
/*1080p@30 8bit, 148.5M, sync, packet*/
{8, 0, 51, EM_SYNC_1080P24},

/*index52 640x480p@60 8bit, 50.4M, sync, packet*/
{10, 1, 52, EM_SYNC_480P},
/*640x480p@60 10bit, 50.4M, sync, packet*/
{10, 1, 53, EM_SYNC_480P},
/*640x480p@60 10bit, 50.4M, sync, packet*/
{10, 0, 54, EM_SYNC_480P},
/*640x480p@60 10bit, 50.4M, sync, packet*/
{10, 0, 55, EM_SYNC_480P},

/*index56 720p@60 10bit, 148.5M, sync, packet*/
{10, 1, 56, EM_SYNC_720P60},
/*720p@60 10bit, 148.5M, sync, semi-plannar*/
{10, 1, 57, EM_SYNC_720P60},
/*720p@60 10bit, 148.5M, embeded, packet*/
{10, 0, 58, EM_SYNC_720P60},
/*720p@60 10bit, 148.5M, embeded, semi-plannar*/
{10, 0, 59, EM_SYNC_720P60},

/*index60 1080p@24 10bit, 148.5M, sync, packet*/
{10, 1, 60, EM_SYNC_1080P24},
/*1080p@24 10bit, 148.5M, sync, semi-plannar*/
{10, 1, 61, EM_SYNC_1080P24},
/*1080p@24 10bit, 148.5M, embeded, packet*/
{10, 0, 62, EM_SYNC_1080P24},
/*1080p@24 10bit, 148.5M, embeded, semi-plannar*/
{10, 0, 63, EM_SYNC_1080P24},

/*index64, 480i, sync mode interlace*/
{10, 1, 64, EM_SYNC_480I},
/*480i, embeded mode interlace*/
{10, 1, 65, EM_SYNC_480I},
/*480i, sync mode interlace*/
{10, 1, 66, EM_SYNC_480P},
/*480i, sync mode interlace*/
{10, 1, 67, EM_SYNC_480P},

/*index68,8bit, 576i@50, 27M, sync, packet*/
{10, 1, 68, EM_SYNC_576I},
/*8bit, 576i@50, 27M, sync, semi-plannar*/
{10, 1, 69, EM_SYNC_576I},
/*8bit, 576i@50, 27M, embeded, packet*/
{10, 1, 70, EM_SYNC_576P},
/*8bit, 576i@50, 27M, embeded, semi-plannar*/
{10, 1, 71, EM_SYNC_576P},

/*index72 1080p@30 8bit, 148.5M, sync, packet*/
{8, 1, 72, EM_SYNC_1080P30},
/*1080p@30 8bit, 148.5M, sync, packet*/
{8, 1, 73, EM_SYNC_1080P30},
/*1080p@30 8bit, 148.5M, sync, packet*/
{8, 0, 74, EM_SYNC_1080P30},
/*1080p@30 8bit, 148.5M, sync, packet*/
{8, 0, 75, EM_SYNC_1080P30},

};


struct gpt_vdo_param resl[] = {
/*index0, 1080p 74.25M, sync packet prog*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 830, 2750, 830, 2750, 638, 638 + 44, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p sync plannar prog*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},	//p30
/*1080p embeded packet prog*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 830, 2750, 830, 2750, 638, 638 + 44, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p embeded plannar prog*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 830, 2750, 830, 2750, 638, 638 + 44, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index4, 720p, 74.25M, sync mode, packet*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 370, 1650, 370, 1650, 110, 110 + 40, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, sync mode, semi planar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 370, 1650, 370, 1650, 110, 110 + 40, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, embeded mode, packet*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 370, 1650, 370, 1650, 110, 110 + 40, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, embeded mode, packet, semi plannar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 370, 1650, 370, 1650, 110, 110 + 40, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index8, 640x480p, 25.2M, sync mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 160, 800, 160, 800, 16, (16 + 96), 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, sync mode, semi planar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 160, 800, 160, 800, 16, (16 + 96), 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, embeded mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 160, 800, 160, 800, 16, (16 + 96), 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, embeded mode, semi plannar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 160, 800, 160, 800, 16, (16 + 96), 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index12, 480p, 27M, sync mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p, sync mode, semi plannar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p embeded mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p embeded mode, semi planar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index16, 480i@60, 27M, sync, packet, interlace*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1, 1716 - 1, 276 - 1, 1716 - 1, 38 - 1, (38 + 124) -  1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},
/*480i@60, 27M, sync, semip, interlace*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1, 1716 - 1, 276 - 1, 1716 - 1, 38 - 1, (38 + 124) -  1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},
/*480i@60, 27M, embeded, packet, interlace*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1, 1716 - 1, 276 - 1, 1716 - 1, 38 - 1, (38 + 124) - 1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},
/*480i@60, 27M, embeded, semip, interlace*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1, 1716 - 1, 276 - 1, 1716 - 1, 38 - 1, (38 + 124) - 1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},

/*index20,8bit, 576i@60, 27M, sync, packet*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},
/*8bit, 576i@60, 27M, sync, semi-plannar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},
/*8bit, 576i@60, 27M, embeded, packet*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},
/*8bit, 576i@60, 27M, embeded, semi-plannar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},

/*index24, 720p, 8bit, 148.5M, sync mode, packet*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, 8bit, 148.5M, sync mode, semi planar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, 8bit, 148.5M, embeded mode, packet*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, 8bit, 148.5M, embeded mode, packet, semi plannar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index 28, 16bit 1080p@30, 74.25M sync mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*16bit 1080p@60, 148.5M sync mode*/
//{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 40, 1120, 40, 1120, 4, 8, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},		//p30
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 720, 2640, 720, 2640, 528, 528 + 44, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //p50
/*16bit 1080p@60, halt display sync mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*31 reserve*/
{},

/*index32, 1080i 74.25M, sync, packet*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 21, 561, 21, 561, 2, 7, 561, 22, 562, 22, 562, 2, 7, 562, 1100, 0},
/*1080i 74.25M, sync,  semi-plannar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 21, 561, 21, 561, 2, 7, 561, 22, 562, 22, 562, 2, 7, 562, 1100, 0},
/*1080i 74.25M, embeded, packet*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_INTER_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 21, 561, 21, 561, 2, 7, 561, 22, 562, 22, 562, 2, 7, 562, 1100, 0},
/*1080i 74.25M, embeded semi-plannar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_INTER_EN, GPT_DVO_DE_HI, 280, 2200, 280, 2200, 88, 88 + 44, 21, 561, 21, 561, 2, 7, 561, 22, 562, 22, 562, 2, 7, 562, 1100, 0},

/*index36, 480p@60, 8bit, 54M, sync, packet */
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 522, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p@60, 8bit, 54M, sync, semi-plannar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 522, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p@60p, 8bit, 54M, embeded , packet*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 522, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p@60, 8bit, 54M, sync mode*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 522, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index40, 640x480p, 8bit, 50.4M, sync mode*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, sync mode, semi planar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, embeded mode*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, embeded mode, semi plannar*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index44, 480p@60, 10bit, 54M, sync, packet */
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 521, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 31},
/*480p@60, 10bit, 54M, sync, semi-plannar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 521, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 31},
/*480p@60p, 10bit, 54M, embeded , packet*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 521, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 31},
/*480p@60, 10bit, 54M, sync mode*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 138 - 1, 2 * 858 - 1, 2 * 138 - 1, 2 * 858 - 1, 2 * 16 - 1, 2 * (16 + 62) - 1, 42, 521, 42, 521, 6, 12, 524, 0, 0, 0, 0, 0, 0, 0, 0, 31},

/*index48, 1080p@24, 8bit, 148.5M, sync packet prog*/
//{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*280-1, 2*2200-1, 2*280-1, 2*2200-1, 2*88-1, 2*(88 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@24, 8bit, 148.5M, sync plannar prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@24, 8bit, 148.5M, embeded packet prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@24, 8bit, 148.5M, embeded plannar prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index52, 640x480p, 10bit, 50.4M, sync mode*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, 50.4M, sync mode, semi planar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, 50.4M, embeded mode*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 - 1, 2 * 800 - 1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 - 1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*640x480p, 50.4M, embeded, semi plannar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 2 * 160 -1, 2 * 800 -1, 2 * 160 - 1, 2 * 800 - 1, 2 * 16 -1, 2 * (16 + 96) - 1, 35, 515, 35, 515, 0, 1, 525, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index56, 720p, 10bit, 148.5M, sync mode, packet*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, 10bit, 148.5M, sync mode, semi planar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, 10bit, 148.5M, embeded mode, packet*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*720p, 10bit, 148.5M, embeded mode, packet, semi plannar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2 * 370 - 1, 2*1650-1, 2*370-1, 2*1650-1, 2*110-1, 2*(110 + 40) - 1, 25, 745, 25, 745, 0, 5, 750, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index60, 1080p@30, 10bit, 148.5M, sync packet prog*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@30, 10bit, 148.5M, sync plannar prog*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@30, 10bit, 148.5M, embeded packet prog*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@30, 10bit, 148.5M, embeded plannar prog*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1124, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index64, 480i@60, 10bit, 27M, sync, packet, interlace*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1, 1716 - 1, 276 - 1, 1716 - 1, 38 - 1, (38 + 124) -  1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},
/*480i@60, 10bit, 27M, sync, semip, interlace*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1, 1716 - 1, 276 - 1, 1716 - 1, 38 - 1, (38 + 124) -  1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},
/*480i@60, 10bit, 27M, embeded, packet, interlace*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1 - 4, 1716 - 1 - 4, 276 - 1 - 4, 1716 - 1 - 4, 38 - 1 - 4, (38 + 124) -  1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},
/*480i@60, 10bit, 27M, embeded, semip, interlace*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 276 - 1 - 4, 1716 - 1 - 4, 276 - 1 - 4, 1716 - 1 - 4, 38 - 1, (38 + 124) - 1, 21, 261, 21, 261, 4, 7, 261, 22, 262, 22, 262, 4, 7, 262, 897 - 1, 39 - 1},

/*index68, 10bit, 576i@60, 27M, sync, packet*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},
/*10bit, 576i@60, 27M, sync, semi-plannar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},
/*10bit, 576i@60, 27M, embeded, packet*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},
/*10bit, 576i@60, 27M, embeded, semi-plannar*/
{GPT_DVO_FORMAT_10BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_INTER_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 288 - 1, 1728 - 1, 288 - 1, 1728 - 1, 24 - 1, (24 + 126) - 1, 23, 311, 23, 311, 2, 5, 311, 24, 312, 24, 312, 2, 5, 312, 864 + 23, 23},

/*index72, 1080p@30, 8bit, 148.5M, sync packet prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*280-1, 2*2200-1, 2*280-1, 2*2200-1, 2*88-1, 2*(88 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@30, 8bit, 148.5M, sync plannar prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*280-1, 2*2200-1, 2*280-1, 2*2200-1, 2*88-1, 2*(88 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@30, 8bit, 148.5M, embeded packet prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*280-1, 2*2200-1, 2*280-1, 2*2200-1, 2*88-1, 2*(88 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*1080p@30, 8bit, 148.5M, embeded plannar prog*/
{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 2*280-1, 2*2200-1, 2*280-1, 2*2200-1, 2*88-1, 2*(88 + 44) -1, 40, 1120, 40, 1120, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index76, 480p, 27M, sync mode halt windows*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a + 0x80, 0x35a - 0x80, 16, (16 + 62), 0x2a, 0x20a, 0x2a + 0x80, 0x20a - 0x80, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p, sync mode, semi plannar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a, 0x35a - 0x26c, 16, (16 + 62), 0x2a, 0x20a, 0x2a, 0x20a - 0x17c, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p embeded mode*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a + 0x80, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a + 0x80, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},
/*480p embeded mode, semi planar*/
{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_SEMIP_EN, GPT_DVO_SYNC_DIS, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 0x8a, 0x35a, 0x8a + 0x80, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a + 0x80, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},

};

void gpt_vdo_dumpdata(void *addr, int length);

static unsigned int g_vdo_mode = 72;//24;//48;  //1080p@30
unsigned int g_clk_src = GPT_EXTERN_CLK;

#endif
