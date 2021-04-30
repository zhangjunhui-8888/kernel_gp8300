#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/gfp.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>


#include "gptfb.h"

static int convert = 0;
module_param(convert, int, 0644);
MODULE_PARM_DESC(convert, "rgb 2 yuv convert enable(0-1)");

#define PXI_ENABLE	0x1
#define PXI_MASK 	0xfffffffUL
static uint64_t pxi_tag = GPT_VIDMEM_ADDR;

static void pxi_ctl(void)
{
        uint64_t tag = pxi_tag;
        uint64_t mask = PXI_MASK;

	tag = tag | PXI_ENABLE;
        __asm__ volatile (
                        "sprsetr   %0,%1\n\t"
                        : 
                        : "r"(mask), "i"(PXIMASK)
                        );
        __asm__ volatile (
                        "sprsetr   %0,%1\n\t"
                        : 
                        : "r"(tag), "i"(PXITAG)
                        );
}

static int gpt_vdo_write(struct gpt_fbinfo *info,
		int offset, unsigned int value)
{
	writel(value, info->regs + offset);
	return 0;
}

static unsigned int gpt_vdo_read(struct gpt_fbinfo *info, int offset)
{
	return readl(info->regs + offset);
}

void gpt_vdo_disable(struct gpt_fbinfo *info)
{
	unsigned int regs;
	int timeout = 0x100000;

	regs = gpt_vdo_read(info, REG_DVO_ENABLE);
	gpt_vdo_write(info, REG_DVO_ENABLE, regs & (~GPT_DVO_ENABLE));
	while (1) {
		regs = gpt_vdo_read(info, REG_DVO_ENABLE);
		timeout--;
		if (~(regs & 0x2))
			return;

		if (timeout < 0) {
			printk("wait underflow timeout\n");
			return;
		}
	}
}

void gpt_vdo_enable(struct gpt_fbinfo *info)
{
	unsigned int regs;

	/**/
	regs = gpt_vdo_read(info, REG_DVO_ENABLE);
	gpt_vdo_write(info, REG_DVO_ENABLE, regs | GPT_DVO_ENABLE | GPT_DVO_CLK_INV);
}

void gpt_vdo_polarity(struct gpt_fbinfo *info, int polarity)
{
	unsigned int regs;

	regs = gpt_vdo_read(info, REG_DVO_ENABLE);
	regs &= ~(GPT_DVO_DE_HI | GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI);
	gpt_vdo_write(info, REG_DVO_ENABLE, regs | polarity);
}


void gpt_vdo_dma_setup(struct gpt_fbinfo *info,
		int dma_hdat, int dma_high, int dma_stride)
{
	/**/
	gpt_vdo_write(info, REG_DVO_DMA_HIGH, dma_high);
	/**/
	gpt_vdo_write(info, REG_DVO_DMA_HDAT, dma_hdat);
	/**/
	gpt_vdo_write(info, REG_DVO_DMA_STRIDE, dma_stride);
}

int gpt_vdo_config_mode(struct gpt_fbinfo *info, int data_mode,
		int display_mode, int sync_mode, int dvo_format)
{
	unsigned int regs;

	regs = gpt_vdo_read(info, REG_DVO_ENABLE);
	regs &= ~((0x03 << 4) | (0x01 << 8) | (0x01 << 10));
	regs |= (display_mode | dvo_format) | (sync_mode);
	gpt_vdo_write(info, REG_DVO_ENABLE, regs);

	regs = gpt_vdo_read(info, REG_DVO_DMA_CONFIG);
	regs &= ~(0x01 << 9);
	regs |= data_mode;
	gpt_vdo_write(info, REG_DVO_DMA_CONFIG, regs);

    
    return 0;
}

void gpt_vdo_win_sync(struct gpt_fbinfo *info, int he, int hf,
			int ve_o, int vf_o, int ve_e, int vf_e)
{
	unsigned int regs;

	/**/
	regs = ((hf << 16) | (he << 0));
	gpt_vdo_write(info, REG_DVO_HSYNC, regs);

	regs = (((vf_o) << 16) | (ve_o << 0));
	gpt_vdo_write(info, REG_DVO_VSYNC_ODD, regs);

	regs = (((vf_e) << 16) | (ve_e << 0));
	gpt_vdo_write(info, REG_DVO_VSYNC_EVEN, regs);
}

void gpt_vdo_hwin_setup(struct gpt_fbinfo *info,
		int ha, int hd, int hb, int hc)
{
	unsigned int regs;

	/**/
	regs = ((hd << 16) | (ha << 0));
	gpt_vdo_write(info, REG_DVO_HACTIVE, regs);
	/**/
	regs = ((hc << 16) | (hb << 0));
	gpt_vdo_write(info, REG_DVO_HDISPLAY, regs);
}

int gpt_vdo_vwin_odd_setup(struct gpt_fbinfo *info,
		int va, int vd, int vb, int vc, int vg, int hnum)
{
	unsigned int regs;

	/**/
	regs = (vd << 16) | (va << 0);
	gpt_vdo_write(info, REG_DVO_VACTIVE_ODD, regs);
	/**/
	regs = (vc << 16) | (vb << 0);
	gpt_vdo_write(info, REG_DVO_VDISPLAY_ODD, regs);
	/**/
	regs = vg;
	gpt_vdo_write(info, REG_DVO_BUTTOM_BLK_ODD, regs);

	regs = hnum;
	gpt_vdo_write(info, REG_DVO_VSYNC_HNUM_ODD, regs);

	return 0;
}

int gpt_vdo_vwin_evn_setup(struct gpt_fbinfo *info,
		int va, int vd, int vb, int vc, int vg, int hnum)
{
	unsigned int regs;

	/**/
	regs = (vd << 16) | (va << 0);
	gpt_vdo_write(info, REG_DVO_VACTIVE_EVEN, regs);
	/**/
	regs = (vc << 16) | (vb << 0);
	gpt_vdo_write(info, REG_DVO_VDISPLAY_EVEN, regs);
	/**/
	regs = vg;
	gpt_vdo_write(info, REG_DVO_BUTTOM_BLK_EVEN, regs);

	regs = hnum;
	gpt_vdo_write(info, REG_DVO_VSYNC_HNUM_EVEN, regs);

	return 0;
}

static int gpt_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->bits_per_pixel != 16) {
		var->bits_per_pixel = 16;
		printk("gpt vdo support 16bits only\n");
	}

	info->screen_size = info->var.xres * info->var.yres * info->var.bits_per_pixel / 8;

	printk("%s:%d-->%dx%d (%dx%d) %d, %d\n", __func__, __LINE__,
		info->var.xres, info->var.yres,
		info->var.xres_virtual, info->var.yres_virtual,
		(int)info->screen_size, info->var.bits_per_pixel);

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;

	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if (var->yres_virtual <= 0) {
		int tmp = resl[g_vdo_mode].dvo_vd_o -resl[g_vdo_mode].dvo_va_o;
		if(abs( tmp - 1080) <= 5){
			var->yres_virtual = var->yres = 1080;
		} else if(abs(tmp - 720) <= 5){
			var->yres_virtual = var->yres = 720;
		} else if(abs(tmp - 576) <= 5){
			var->yres_virtual = var->yres = 576;
		} else if(abs(tmp - 480) <= 5){
			var->yres_virtual = var->yres = 480;
		}
		printk("confirm options, reset %d column\n", var->yres_virtual);
	}

	if (var->xoffset < 0)
		var->xoffset = 0;

	if (var->yoffset < 0)
		var->yoffset = 0;
	if (var->xoffset > var->xres_virtual - var->xres)
		var->xoffset = var->xres_virtual - var->xres - 1;
	if (var->yoffset > var->yres_virtual - var->yres)
		var->yoffset = var->yres_virtual - var->yres - 1;


	switch (var->bits_per_pixel) {
	case 32:	/* RGB A888 */
		var->red.length    = 8;
		var->green.length  = 8;
		var->blue.length   = 8;
		var->transp.length = 8;

		var->red.offset    = 0;
		var->green.offset  = 8;
		var->blue.offset   = 16;
		var->transp.offset = 24;
		break;
	case 24:	/* RGB A565 */
		var->red.length    = 8;
		var->green.length  = 8;
		var->blue.length   = 8;
		var->transp.length = 8;

		var->red.offset    = 0;
		var->green.offset  = 8;
		var->blue.offset   = 16;
		var->transp.offset = 24;
		break;
	case 16:
		var->red.offset   = 0;
		var->red.length   = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset  = 11;
		var->blue.length  = 5;
		break;
	case 8:
		var->red.offset   = 1;
		var->red.length   = 1;
		var->green.offset = 1;
		var->green.length = 1;
		var->blue.offset  = 1;
		var->blue.length  = 1;
		break;
	default:
		printk("unsuported type (%d)\n", var->bits_per_pixel);
		return -EINVAL;
	}
	printk("%s:%d-->%dx%d (%dx%d) %d, %d\n", __func__, __LINE__,
		var->xres, var->yres, var->xres_virtual, var->yres_virtual,
		(int)info->screen_size, var->bits_per_pixel);


	return 0;
}

static int gpt_fb_set_par(struct fb_info *fbinfo)
{
	int i;
	int id = -1;
	struct gpt_fbinfo *info = fbinfo->par;
	struct fb_var_screeninfo *var = &fbinfo->var;
	unsigned int hend, vend, vbtm;
	printk("%s:%d-->%dx%d (%dx%d) %d, %d\n", __func__, __LINE__,
		fbinfo->var.xres, fbinfo->var.yres,
		fbinfo->var.xres_virtual, fbinfo->var.yres_virtual,
		(int)fbinfo->screen_size, fbinfo->var.bits_per_pixel);

	fbinfo->screen_size = info->fbinfo->var.xres *
		info->fbinfo->var.yres * info->fbinfo->var.bits_per_pixel / 8;

	fbinfo->fix.line_length = (var->xres_virtual * var->bits_per_pixel) / 8;
	printk("bytes per line %d\n", fbinfo->fix.line_length);
        fbinfo->fix.visual = FB_VISUAL_TRUECOLOR;
	for (i = sizeof(fbparam) / sizeof(struct gpt_fbparam); i > 0; i--) {
		if ((fbparam[i].hlen == fbinfo->var.xres) &&
			(fbparam[i].vlen == fbinfo->var.yres)) {
			id = i;
		}
	}

	if (id < 0) {
		printk("vdo not support %dx%d\n", fbinfo->var.xres, fbinfo->var.yres);
		return 0;
	}

	hend = fbparam[id].hstart + fbparam[id].hlen;
	vend = fbparam[id].vstart + fbparam[id].vlen;
	vbtm = vend + fbparam[id].vbtm;
	gpt_vdo_hwin_setup(info, fbparam[id].hstart, hend, fbparam[id].hstart, hend);
	gpt_vdo_vwin_odd_setup(info, fbparam[id].vstart, vend, fbparam[id].vstart, vend, vbtm, 0x0);
	gpt_vdo_win_sync(info, fbparam[id].hedge_one, fbparam[id].hedge_sec, fbparam[id].vedge_one, fbparam[id].vedge_sec, 0, 0);
	gpt_vdo_dma_setup(info, fbparam[id].hlen * 2, fbparam[id].vlen, fbparam[id].hlen * 2);
	gpt_vdo_polarity(info, fbparam[id].flag);

	return 0;
}

static int gpt_fb_blank(int blank_mode, struct fb_info *info)
{
	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	case FB_BLANK_UNBLANK:
		memset(info->screen_base, 0, info->screen_size); //clean frame buff.
		break;
	default:
		memset(info->screen_base, 0, info->screen_size); //clean frame buff.
		break;
	}

	return 0;
}

static int gpt_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp, struct fb_info *info)
{
	struct gpt_fbinfo *fbinfo = info->par;
	u32 *pal = fbinfo->pseudo_palette;

	printk("%s: %d => rgb=%x/%x/%x\n",
		__func__, regno, red, green, blue);


	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno < 16) {
#if 0
			pal[regno] = (gpt_yuv[regno].y << 24) | (gpt_yuv[regno].v << 16)
					| (gpt_yuv[regno].y << 8) | gpt_yuv[regno].u;
#else
			pal[regno] = red;
			pal[regno] |= (blue << 24) | (green << 16);
#endif
			printk("lqq-> pal reg:%d, val: 0x%.8x\n", regno, pal[regno]);
		}
		break;

	default:
		printk("default\n");
		return 1;	/* unknown type */
	}

	return 0;
}

ssize_t gpt_fb_write(struct fb_info *info, const char __user *buf,
                            size_t count, loff_t *ppos)
{
	gpt_vdo_dumpdata(info->screen_base, 0x100);
	memcpy(info->screen_base, buf, 0x4000);
	gpt_vdo_dumpdata(info->screen_base, 0x100);
	return 1;
}

int gpt_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	//return vm_iomap_memory(vma, info->fix.smem_start, info->fix.smem_len);
	if (!vm_iomap_memory(vma, info->fix.smem_start, 0x800000)) {
		vma->vm_pgoff += info->fix.smem_start >> PAGE_SHIFT;
		return 0;
	}

	return -EAGAIN;
}

void gpt_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	cfb_imageblit(info, image);
}

static int gpt_extapbccr_cfg_voutclk(int clk_val)
{
	uint32_t *extsrc = ioremap(0xf0000010,0x40);
	if(extsrc == NULL){
		printk("%s:%d-->get memory/io resource filed\n", __func__, __LINE__);
		return -ENXIO;
	}
	/*config clk*/
	writel((readl(extsrc) & 0x9FFFFFFF) |(clk_val<<29), extsrc);
	//writel((readl(extsrc) & 0x9FFF0FFF) |(clk_val<<29) | 8 << 12, extsrc);// 1.6G vout 266MHz

	iounmap(extsrc);
	
	return 0;
}

static int gpt_extcr_cfg(void)
{
	uint32_t *extcr = NULL, *extcr_h = NULL;

	extcr = ioremap(GPT_EXTCR_IO_REG, 0x40);
	if (extcr == NULL) {
		printk("%s:%d--> get memory/io resource failed\n", __func__, __LINE__);
		return -ENXIO;
	}
	printk("GPT EXTSRC address(%x) remap to %p\n", GPT_EXTCR_IO_REG, extcr);
	/*vout  pclk vidmem enable */
	writel((readl(extcr) | GPT_VOUT_ENABLE | GPT_VID_ENABLE)  | 
			GPT_VIDMEM_ENABLE | GPT_APIU_ENABLE_MASK, extcr); 
	printk("GPT EXTSRC address(%x) remap to %p\n", GPT_EXTCR_IO_REG, extcr);
	
	iounmap(extcr);

	extcr_h = ioremap_nocache(GPT_EXTCR_HI_REG, 0x40);
	if (extcr_h == NULL) {
		printk("%s:%d-->get memory/io resource failed\n",  __func__, __LINE__);
		return -ENXIO;
	}
	printk("GPT EXTSRC address(%x) remap to %p\n", GPT_EXTCR_HI_REG, extcr_h);
	
	/*write 0 set vidmem don't sleep and shutdown.*/
	//*extcr_h = (readl(extcr_h) | (((GPT_VIDMEM_SLEEP) | (GPT_VIDMEM_SHUTDOWN))  << 16)) & 
	//		(~((GPT_VIDMEM_SLEEP) | (GPT_VIDMEM_SHUTDOWN)));
	writel((readl(extcr_h) | ((GPT_VIDMEM_SLEEP | GPT_VIDMEM_SHUTDOWN)  << 16)) & 
			~(GPT_VIDMEM_SLEEP | GPT_VIDMEM_SHUTDOWN), extcr_h);

	iounmap(extcr_h);
	
	return 0;
}

static void gpt_vdo_hwinit(struct gpt_fbinfo *info, int index)
{
	int i = index;
	int tmp = 0;
#ifdef GPT_RGB2YUV_CONVERT
	ulong dma_addr0 = info->fbinfo->fix.smem_start + GPT_YUV_DATA_OFFSET;
#else
	ulong dma_addr0 = info->fbinfo->fix.smem_start +
				info->fbinfo->fix.line_length * info->fbinfo->var.yoffset;
#endif
	/* mask all interrupt*/
	//gpt_vdo_write(info, REG_DVO_INT_MASK, 0xffffffff);  // disable all interrupt.
	gpt_vdo_write(info, REG_DVO_INT_MASK, 0x0);       // enable all interrupt.
    
    //int int_mask = gpt_vdo_read(info, REG_DVO_INT_MASK);
    //printk("intr mask = 0x%x 0x%x\n", int_mask, int_mask & ~GPT_DVO_INT_DMA_CMP);
	//gpt_vdo_write(info, REG_DVO_INT_MASK, int_mask & ~GPT_DVO_INT_DMA_CMP);
	//gpt_vdo_write(info, REG_DVO_INT_MASK, 0x0b);// only enable sof
	gpt_vdo_write(info, REG_DVO_INT_MASK, 0xfc);
	gpt_vdo_write(info, REG_DVO_INT_LINE_NUM, 0x100);  //setting line interrupt cannot by mask and trigger all one flush.
	
	tmp = gpt_vdo_read(info, REG_DVO_AXI_CONFIG);
	gpt_vdo_write(info, REG_DVO_AXI_CONFIG, tmp | GPT_DVO_AXI_DMA_ARQOS);

	/* vdo dma config */
	gpt_vdo_write(info, REG_DVO_DMA_POSITION, GPT_VDO_DMA_START);

	/* config dma mem address*/
	gpt_vdo_write(info, REG_DVO_DMA_BASE0, dma_addr0 & 0xffffffff);
	gpt_vdo_write(info, REG_DVO_DMA_BASE0_H, (dma_addr0 >> 32) & 0xffffffff);

    /*config dma mem address*/
    gpt_vdo_write(info, REG_DVO_DMA_BASE1, (dma_addr0 + info->fbinfo->screen_size/2) & 0xffffffff);
    gpt_vdo_write(info, REG_DVO_DMA_BASE1_H, ((dma_addr0 + info->fbinfo->screen_size/2) >> 32) & 0xffffffff);

	gpt_vdo_config_mode(info, resl[i].data_mode, resl[i].disp_mode, resl[i].sync_mode, resl[i].vdo_mode);
	//gpt_vdo_config_mode(info, GPT_DVO_DMA_PACKE_EN, GPT_DVO_PROGR_EN, GPT_DVO_SYNC_EN, GPT_DVO_FORMAT_16BIT);
}

/*index12, 480p, 27M, sync mode*/
//{GPT_DVO_FORMAT_16BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI, 
//0x8a, 0x35a, 0x8a, 0x35a, 16, (16 + 62), 0x2a, 0x20a, 0x2a, 0x20a, 6, 12, 0x20d, 0, 0, 0, 0, 0, 0, 0, 0, 0},

/*index48, 1080p@24, 8bit, 148.5M, sync packet prog*/
//{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 
//2*280-1, 2*2200-1, 2*280-1, 2*2200-1, 2*88-1, 2*(88 + 44) -1, 40, 1121, 40, 1121, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//{GPT_DVO_FORMAT_8BIT, GPT_DVO_DMA_PACKE_EN, GPT_DVO_SYNC_EN, GPT_DVO_PROGR_EN, GPT_DVO_DE_HI, 
//2*830-1, 2*2750-1, 2*830-1, 2*2750-1, 2*638-1, 2*(638 + 44) -1, 40, 1121, 40, 1121, 0, 5, 1125, 0, 0, 0, 0, 0, 0, 0, 0, 0},
static int gpt_vdo_default_res(struct gpt_fbinfo *info, int index)
{
	int i = index; //48/12;
	//gpt_vdo_hwin_setup(info, 0x8a, 0x35a, 0x8a, 0x35a);
	gpt_vdo_hwin_setup(info, resl[i].dvo_ha, resl[i].dvo_hd, resl[i].dvo_hb, resl[i].dvo_hc);

	//gpt_vdo_vwin_odd_setup(info, 0x2a, 0x20a, 0x2a, 0x20a, 0x20d, 0x0);
	gpt_vdo_vwin_odd_setup(info, resl[i].dvo_va_o, resl[i].dvo_vd_o, resl[i].dvo_vb_o, resl[i].dvo_vc_o, resl[i].dvo_vg_o, resl[i].hnum_odd);

	//gpt_vdo_vwin_evn_setup(info, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0);
	gpt_vdo_vwin_evn_setup(info, resl[i].dvo_va_e, resl[i].dvo_vd_e, resl[i].dvo_vb_e, resl[i].dvo_vc_e, resl[i].dvo_vg_e, resl[i].hnum_even);

	//gpt_vdo_win_sync(info, 16, 16 + 62, 6, 12, 0, 0);
	gpt_vdo_win_sync(info, resl[i].dvo_he, resl[i].dvo_hf, resl[i].dvo_ve_o, resl[i].dvo_vf_o, resl[i].dvo_ve_e, resl[i].dvo_vf_e);

	//gpt_vdo_dma_setup(info, (0x35a - 0x8a) * 2, 0x20a - 0x2a, (0x35a - 0x8a) * 2);
	if(resl[i].vdo_mode == GPT_DVO_FORMAT_16BIT){
        if(resl[g_vdo_mode].data_mode == GPT_DVO_DMA_SEMIP_EN){
		    gpt_vdo_dma_setup(info, (resl[i].dvo_hd - resl[i].dvo_ha), resl[i].dvo_vd_o - resl[i].dvo_va_o, (resl[i].dvo_hc - resl[i].dvo_hb));
        } else {
            gpt_vdo_dma_setup(info, (resl[i].dvo_hd - resl[i].dvo_ha) * 2, resl[i].dvo_vd_o - resl[i].dvo_va_o, (resl[i].dvo_hc - resl[i].dvo_hb) * 2);
        }

	} else {
        if(resl[g_vdo_mode].data_mode == GPT_DVO_DMA_SEMIP_EN){
		    gpt_vdo_dma_setup(info, (resl[i].dvo_hd - resl[i].dvo_ha) / 2, resl[i].dvo_vd_o - resl[i].dvo_va_o, (resl[i].dvo_hc - resl[i].dvo_hb) / 2);
        } else {
            gpt_vdo_dma_setup(info, (resl[i].dvo_hd - resl[i].dvo_ha), resl[i].dvo_vd_o - resl[i].dvo_va_o, (resl[i].dvo_hc - resl[i].dvo_hb));
        }
	}


	//gpt_vdo_polarity(info, GPT_DVO_HSYNC_HI | GPT_DVO_VSYNC_HI | GPT_DVO_DE_HI);
	gpt_vdo_polarity(info, resl[i].vdo_polarity);
	return 0;
}

void gpt_vdo_dumpdata(void *addr, int length)
{
	int i;
	unsigned int *buff = addr;

        for (i = 0; i < (length / 4); i++) {
                if (i % 4 == 0)
                        printk("%.8x: ", i* 4);
                printk("%.8x%c", buff[i], i % 4 == 3?'\n':' ');
        }
        printk("\n");
}

void gpt_vdo_dumpregs(struct gpt_fbinfo *info)
{
	int i;
	uint32_t *reg_t = info->regs;
        if (reg_t == NULL) {
                printk("please init vdo host first\n");
                return;
        }

        printk("vido out regs: \n");
        for (i = 0; i < (0x80 / 4); i++) {
                if (i % 4 == 0)
                        printk("%.8x: ", i* 4);
                printk("%.8x%c", reg_t[i], i % 4 == 3?'\n':' ');
        }
        printk("\n");
}

#ifdef GPT_RGB2YUV_CONVERT

struct gpt_pixel {
	int rc;
	int bc;
	int gc;
	int y;
	int cb;
	int cr;
};

void gpt_rgb2yuv_601(struct gpt_pixel *p)
{
	int rc = p->rc;
	int gc = p->gc;
	int bc = p->bc;
#if 0
	p->y  = 0.257  * rc + 0.564 * gc + 0.098 * bc + 16;
	p->cb = -0.148 * rc - 0.291 * gc + 0.439 * bc + 128;
	p->cr = 0.439  * rc - 0.368 * gc - 0.071 * bc + 128;
#else
	p->y  = (257  * rc + 564 * gc + 98 * bc) / 1000 + 16;
	p->cb = (-148 * rc - 291 * gc + 439 * bc) / 1000 + 128;
	p->cr = (439  * rc - 368 * gc - 71 * bc) / 1000 + 128;

#endif
	if (p->y < 0)
		p->y = 0;
	if (p->cb < 0)
		p->cb = 0;
	if (p->cr < 0)
		p->cr = 0;

	p->y  = p->y  > 0xff ? 0xff : p->y;
	p->cb = p->cb > 0xff ? 0xff : p->cb;
	p->cr = p->cr > 0xff ? 0xff : p->cr;
}

/*
 * ARGB to uyuv
*/
int gpt_vdo_color_convert(struct fb_info *fbinfo)
{
	int i, j;
	struct gpt_pixel p1;
	struct gpt_pixel p2;
	int rindex = 0;
	int yindex = 0;
	volatile uint8_t *src = (void *)fbinfo->screen_base;
	volatile uint8_t *dst = (void *)fbinfo->screen_base + GPT_YUV_DATA_OFFSET;
	unsigned char global =  *src;

	printk("convert (%dx%d) to yuv, global: 0x%x\n", fbinfo->var.xres, fbinfo->var.yres, global);
	
	convert = 0;
	for (i = 0; i < fbinfo->var.yres; i++) {
		for (j = 0; j < fbinfo->var.xres; j++, rindex += 2, yindex++) {
			p1.rc = src[rindex * 4 + 0];
			p1.gc = src[rindex * 4 + 1];
			p1.bc = src[rindex * 4 + 2];
			gpt_rgb2yuv_601(&p1);
			p2.rc = src[(rindex + 1) * 4 + 0];
			p2.gc = src[(rindex + 1) * 4 + 1];
			p2.bc = src[(rindex + 1) * 4 + 2];
			gpt_rgb2yuv_601(&p2);
#if 0
			if (p1.rc != global || p1.gc != global || p1.bc != global ||
				p2.rc != global || p2.gc != global || p2.bc != global ) {
				printk("rindex: %x, rc1: %x, rc2: %x, bc1: %x, bc2:%x, gc1: %x, gc2: %x, global: %x\n",
							rindex, p1.rc, p2.rc, p1.bc, p2.bc, p1.gc, p2.gc, global);
			}
#endif
			dst[yindex * 4 + 0] = (p1.cb + p2.cb) / 2;
			dst[yindex *4 + 1] = p1.y;
			dst[yindex * 4 + 2] = (p1.cr + p2.cr) / 2;
			dst[yindex * 4 + 3] = p2.y;
		}
	}

	return 0;
}

static int gpt_rgb2yuv_thread(void *data)
{
	struct gpt_fbinfo *info = (struct gpt_fbinfo *)data;
	struct fb_info *fbinfo = info->fbinfo;

	while (1) {

		if (kthread_should_stop())
			break;

		set_current_state(TASK_INTERRUPTIBLE);
//		schedule_timeout(1 * HZ);
		if (convert)
			gpt_vdo_color_convert(fbinfo);
		
		msleep(10000);
	}

	return 0;
}

static void gpt_rgb2yuv_timer(unsigned long data)
{
	struct gpt_fbinfo *info = (struct gpt_fbinfo *)data;
	struct fb_info *fbinfo = info->fbinfo;
	gpt_vdo_color_convert(fbinfo);
	mod_timer(&info->timer, round_jiffies(jiffies + GPT_COLOR_CONVERT_FREQ));
}
#endif

/**
 * gpt_fb_set_fix() - set fixed framebuffer parameters from variable settings.
 * @info:	framebuffer information pointer
 * @return:	0 on success or negative error code on failure.
 */
static int gpt_fb_set_fix(struct fb_info *fbi)
{
	struct fb_fix_screeninfo *fix = &fbi->fix;
	struct fb_var_screeninfo *var = &fbi->var;

	strncpy(fix->id, "GPTFB BG", 8);
	
	fix->smem_start = GPT_DMA_BASE_ADDR0;
	fix->smem_len = fbi->screen_size;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 0;
	fix->ypanstep = 1;

	return 0;
}

static int gpt_vdo_fill_var(struct fb_info * fbinfo)
{
	struct gpt_fbinfo *info = fbinfo->par;
	int tmp = 0;
	
	if(resl[g_vdo_mode].vdo_mode == GPT_DVO_FORMAT_16BIT){
		tmp = resl[g_vdo_mode].dvo_hd - resl[g_vdo_mode].dvo_ha;
	} else {
		tmp = (resl[g_vdo_mode].dvo_hd - resl[g_vdo_mode].dvo_ha) / 2;
	}
	if(abs( tmp - 1920) <= 5){
		fbinfo->var.xres	= 1920;
	} else if(abs(tmp - 1280) <= 5){
		fbinfo->var.xres	= 1280;
	} else if(abs(tmp - 720) <= 5){
		fbinfo->var.xres	= 720;
	} else if(abs(tmp - 640) <= 5){
		fbinfo->var.xres	= 640;
	}

	tmp = resl[g_vdo_mode].dvo_vd_o -resl[g_vdo_mode].dvo_va_o;
	if(abs( tmp - 1080) <= 5){
		fbinfo->var.yres	= 1080;
	} else if(abs(tmp - 720) <= 5){
		fbinfo->var.yres	= 720;
	} else if(abs(tmp - 576) <= 5){
		fbinfo->var.yres	= 576;
	} else if(abs(tmp - 480) <= 5){
		fbinfo->var.yres	= 480;
	}
	
	//if(resl[g_vdo_mode].vdo_mode == GPT_DVO_FORMAT_16BIT){
		fbinfo->var.bits_per_pixel = 16; 
	//}else if(resl[g_vdo_mode].vdo_mode == GPT_DVO_FORMAT_8BIT){
		//fbinfo->var.bits_per_pixel = 8; 
	//}
	//else if (resl[g_vdo_mode].vdo_mode == GPT_DVO_FORMAT_10BIT){
		//fbinfo->var.bits_per_pixel = 10; 
	//}
	fbinfo->var.xres_virtual = info->fbinfo->var.xres;
	fbinfo->var.yres_virtual = info->fbinfo->var.yres * 2;

	return 0;
}

static int gpt_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fb_info)
{
#if 0
	struct gpt_fbinfo *info = fb_info->par;

    if(var->yoffset != info->fbinfo->var.yoffset){
        info->fbinfo->var.yoffset = var->yoffset;
        printk("oldyoffset=%d, yoffset=%d\n", info->fbinfo->var.yoffset = var->yoffset);
    }
#endif

	if(var->xoffset != 0){
		return -EINVAL;
	}

	return 0;
}


static int gpt_fb_open(struct fb_info *fbinfo, int user)
{
	struct gpt_fbinfo *info = fbinfo->par;
	
	gpt_vdo_fill_var(fbinfo);
	gpt_fb_check_var(&fbinfo->var,fbinfo);
	//gpt_fb_set_fix(fbinfo);
	gpt_vdo_hwinit(info, g_vdo_mode);
	gpt_vdo_default_res(info, g_vdo_mode);
	//gpt_fb_blank(FB_BLANK_NORMAL, fbinfo);
	gpt_vdo_enable(info);	

	return 0;
}

static int gpt_fb_release(struct fb_info *fb_info, int user)
{
	//struct gpt_fbinfo *info = fb_info->par;

	//gpt_vdo_disable(info);

	return 0;
}

static int gpt_vdo_remove(struct platform_device *pdev)
{
	struct gpt_fbinfo *info = platform_get_drvdata(pdev);

	if (info->regs)	{
		iounmap(info->regs);
		info->regs = NULL;
	}

#ifdef GPT_RGB2YUV_CONVERT
//	del_timer_sync(&info->timer);
	kthread_stop(info->thread);
#endif
	if (info->fbinfo) {

	#if 1
		iounmap(info->fbinfo->screen_base);
	#else
		dma_free_writecombine(&pdev->dev,
			0x100000, info->fbinfo->screen_base,
			info->fbinfo->fix.smem_start);
	#endif
		unregister_framebuffer(info->fbinfo);
		framebuffer_release(info->fbinfo);
		info->fbinfo = NULL;
	}
	return 0;
}

static int gpt_vdo_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int gpt_vdo_resume(struct platform_device *pdev)
{
	return 0;
}

static int gpt_fb_ioctl (struct fb_info *fbinfo, unsigned int cmd,
					unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int ret = 0;
	int user_data = 0;
	unsigned long dma_base0 = 0;
	struct gpt_fbinfo *info = fbinfo->par;
	
	switch (cmd) {
	case FBIOGET_MODE:
		ret = copy_to_user(argp, &g_vdo_mode, sizeof(g_vdo_mode)) ? -EFAULT : 0;
		printk("g_vdo_mode=%d\n", g_vdo_mode);
		break;

	case FBIOSET_MODE:
		if(copy_from_user(&user_data, argp, sizeof(user_data))){
			return -EFAULT;
		}
		g_vdo_mode = user_data;
		//TODO...
		gpt_fb_open(fbinfo, 1);
		break;

	case FBIOGET_DMA_BASE_ADDR0:
		dma_base0 = ((unsigned long)gpt_vdo_read(info, REG_DVO_DMA_BASE0_H) << 32) + gpt_vdo_read(info, REG_DVO_DMA_BASE0);
		ret = copy_to_user(argp, &dma_base0, sizeof(dma_base0)) ? -EFAULT : 0;
		break;
		
	case FBIOSET_DMA_BASE_ADDR0:
		if(copy_from_user(&dma_base0, argp, sizeof(dma_base0))){
			return -EFAULT;
		}
		gpt_vdo_write(info, REG_DVO_DMA_BASE0_H, (unsigned int)(dma_base0 >> 32));
		gpt_vdo_write(info, REG_DVO_DMA_BASE0, (unsigned int)dma_base0);
		break;

	case FBIOGET_CLK_SOURCE:
		ret = copy_to_user(argp, &g_clk_src, sizeof(g_clk_src)) ? -EFAULT : 0;
		break;

	case FBIOSET_CLK_SOURCE:
		if (copy_from_user(&user_data, argp, sizeof(user_data))) {
			return -EFAULT;
		}
		g_clk_src = user_data;
		gpt_extapbccr_cfg_voutclk(g_clk_src);
		break;
		
	default:
		break;

	}
	return 0;
}

static struct fb_ops gpt_fb_ops = {

	.owner		= THIS_MODULE,
	.fb_open		= gpt_fb_open,
	.fb_release	= gpt_fb_release,
	//.fb_write	= gpt_fb_write,
	.fb_check_var	= gpt_fb_check_var,
	.fb_set_par	= gpt_fb_set_par,
	.fb_blank	= gpt_fb_blank,
	.fb_pan_display	= gpt_fb_pan_display,
	.fb_setcolreg	= gpt_fb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
//	.fb_imageblit	= cfb_imageblit,
	.fb_imageblit	= gpt_fb_imageblit,
	.fb_ioctl       = gpt_fb_ioctl,
	.fb_mmap	= gpt_fb_mmap,
};


struct gptfb_format {
	const char *name;
	u32 bits_per_pixel;
	u32 fourcc;
};

//static struct gptfb_format gptfb_formats[] = ;

struct gptfb_params {
	u32 mode;
	u32 clkSrc;
	u32 width;
	u32 height;
	u32 stride;
	struct gptfb_format *format;
};

static int gptfb_parse_dt(struct platform_device *pdev,
			   struct gptfb_params *params)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
	//const char *format;
	//int i;

	ret = of_property_read_u32(np, "mode", &params->mode);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse mode property\n");
		return ret;
	}
	printk("%s,%d: mode=%d\n", __func__, __LINE__, params->mode);
	g_vdo_mode = params->mode;

	ret = of_property_read_u32(np, "clk-src", &params->clkSrc);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse clk-src property\n");
		return ret;
	}
	g_clk_src = params->clkSrc;
/*
	ret = of_property_read_u32(np, "width", &params->width);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse width property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "height", &params->height);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse height property\n");
		return ret;
	}

	ret = of_property_read_u32(np, "stride", &params->stride);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse stride property\n");
		return ret;
	}

	ret = of_property_read_string(np, "format", &format);
	if (ret) {
		dev_err(&pdev->dev, "Can't parse format property\n");
		return ret;
	}
	params->format = NULL;
	for (i = 0; i < ARRAY_SIZE(gptfb_formats); i++) {
		if (strcmp(format, gptfb_formats[i].name))
			continue;
		params->format = &gptfb_formats[i];
		break;
	}
	if (!params->format) {
		dev_err(&pdev->dev, "Invalid format value\n");
		return -EINVAL;
	}
*/
	return 0;
}

static int gpt_vout_parse_dt(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *pinctrl;
	const char *pctrl_state;
	struct pinctrl_state *states;
	struct device_node *np = pdev->dev.of_node;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	pctrl_state = devm_kzalloc(&pdev->dev, sizeof(pctrl_state), GFP_KERNEL);
	if (!pctrl_state) {
		dev_err(&pdev->dev, "Cannot allocate pctrl_state\n");
		return -ENOMEM;
	}

	states = devm_kzalloc(&pdev->dev, sizeof(states), GFP_KERNEL);
	if (!states) {
		dev_err(&pdev->dev, "Cannot allocate states\n");
		return -ENOMEM;
	}

	ret = of_property_read_string_index(np, "pinctrl-names", 0,
							&pctrl_state);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot parse pinctrl-names %d\n", ret);
		return ret;
	}

	states = pinctrl_lookup_state(pinctrl, pctrl_state);
	if (IS_ERR(states)) {
		dev_err(&pdev->dev, "Lookup state failed\n");
		return IS_ERR(states);
	}

	ret = pinctrl_select_state(pinctrl, states);
	if (ret < 0) {
		dev_err(&pdev->dev, "Select state failed\n");
		return ret;
	}

	return 0;
}

static irqreturn_t gptfb_irq(int irq, void *dev_id)
{
    struct gpt_fbinfo *info = dev_id;
    struct fb_info *fb_info = info->fbinfo;
    int intr = 0;
    static unsigned int old_offset = 0;
	volatile unsigned int offset;
   
    //intr_raw = gpt_vdo_read(info, REG_DVO_INT_ST_RAW); //not real.
    intr = gpt_vdo_read(info, REG_DVO_INT_ST); 

    if(0 == intr){
        return IRQ_NONE;
    }

#if 0
    static long index = 0; 
    if(0 == ++index % 60){
        pr_debug("%s:%d %ld intr=0x%x\n", __func__, __LINE__, index, intr);
    }
#endif

	/* handle and clear interrupt */
    if(intr & GPT_DVO_INT_DMA_CMP){
        offset = info->fbinfo->fix.line_length * info->fbinfo->var.yoffset;
    
        /* update on next VSYNC */
	    if (old_offset != offset){
		    /* have eof interrupt */
			gpt_vdo_write(info, REG_DVO_DMA_BASE0, 
				fb_info->fix.smem_start + offset);
            if(resl[g_vdo_mode].data_mode == GPT_DVO_DMA_SEMIP_EN){
                gpt_vdo_write(info, REG_DVO_DMA_BASE1, 
                    fb_info->fix.smem_start + offset + fb_info->screen_size/2);
	        }
            //printk("old_offset=%d offset=%d\n", old_offset, offset);
            old_offset = offset;
        }

	    gpt_vdo_write(info, REG_DVO_INT_CLEAR, GPT_DVO_CLR_DMA_CMP);
    }
    
    if(intr & GPT_DVO_INT_FIFO_UNDER){
		printk("*** gpt frame buffer under flow !!! *****\n");
	    gpt_vdo_write(info, REG_DVO_INT_CLEAR, GPT_DVO_CLR_FIFO_UNDER);
    }

    if(intr & GPT_DVO_INT_SOF)
	    gpt_vdo_write(info, REG_DVO_INT_CLEAR, GPT_DVO_CLR_SOF);
    if(intr & GPT_DVO_INT_EOF)
        gpt_vdo_write(info, REG_DVO_INT_CLEAR, GPT_DVO_CLR_EOF);
    if(intr & GPT_DVO_INT_LINE)
	    gpt_vdo_write(info, REG_DVO_INT_CLEAR, GPT_DVO_CLR_LINE);

    return IRQ_HANDLED;
}

static int gpt_vdo_probe(struct platform_device *pdev)
{
	int ret = 0;

#if 0
	dma_addr_t map_dma;
#endif
	struct resource *res;
	struct gpt_fbinfo *info;
	struct fb_info *fbinfo;
	struct gptfb_params params;
    int irq;

	gpt_extcr_cfg(); //Fixed clk

	gpt_vout_parse_dt(pdev);
	if (pdev->dev.of_node) {
		ret = gptfb_parse_dt(pdev, &params);
	}
	if (ret) {
		printk("%s:%d--> parse device tree failed\n", __func__, __LINE__);
		//return ret;
	}
	
	/*0:adv7612 1:camare 3:extern*/
	gpt_extapbccr_cfg_voutclk(g_clk_src);//other

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk("%s:%d--> get memory/io resource failed\n", __func__, __LINE__);
		return -EINVAL;
	}

    irq = platform_get_irq(pdev, 0);
    if(irq < 0){
        dev_err(&pdev->dev, "no irq for device\n");
        return -ENOENT;
    }

	fbinfo = framebuffer_alloc(sizeof(struct gpt_fbinfo), &pdev->dev);
	if (!fbinfo) {
		printk("%s:%d--> alloc framebuffer failed\n", __func__, __LINE__);
		return -ENXIO;
	}

	info = fbinfo->par;
	info->regs = ioremap(res->start, resource_size(res));
	if (info->regs == NULL) {
		printk("%s:%d--> get memory/io resource failed\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto framebuff_exit;
	}
	printk("GPT VDO address(%llx) remap to %p\n", res->start, info->regs);

	info->fbinfo = fbinfo;
	fbinfo->fbops	= &gpt_fb_ops;
	fbinfo->flags	= FBINFO_FLAG_DEFAULT;

    ret = request_irq(irq, gptfb_irq, 0, pdev->name, info);
    //printk("%s:%d irq=%d\n", __func__, __LINE__, irq);
    if(ret) {
        dev_err(&pdev->dev, "cannot get irq %d - err %d\n", irq, ret);
        ret = -EBUSY;
        goto framebuff_exit;
    }

	/* fill var struct.*/
	gpt_vdo_fill_var(fbinfo);
	ret = gpt_fb_check_var(&fbinfo->var, fbinfo);
	if (ret < 0) {
		printk("check_var failed on initial video params\n");
		ret = -1;
		goto regs_remap_exit;
	}

	gpt_fb_set_fix(fbinfo);
#ifdef CONFIG_FB_GPT_VOUT_MEM_VIDMEM
	pxi_tag = fbinfo->fix.smem_start;
	pxi_ctl();
#endif

#if 1
	fbinfo->screen_base = ioremap(GPT_DMA_BASE_ADDR0, 0x800000);
	printk("GPT VDO address(%lx) remap to %p\n", GPT_DMA_BASE_ADDR0, fbinfo->screen_base);
#else
    dma_addr_t map_dma;
    unsigned map_size = PAGE_ALIGN(fbinfo->fix.smem_len * 2);
	fbinfo->screen_base = dma_alloc_writecombine(&pdev->dev, map_size,
						&map_dma, GFP_KERNEL);
    fbinfo->fix.smem_start = map_dma;
#endif
	if (!fbinfo->screen_base) {
		printk("%s:%d alloc dma space failed\n", __func__, __LINE__);
		ret = -1;
		goto regs_remap_exit;
	}
	//gpt_fb_blank(FB_BLANK_NORMAL, fbinfo);

	fbinfo->pseudo_palette  = &info->pseudo_palette;

	platform_set_drvdata(pdev, info);

	ret =  register_framebuffer(fbinfo);
	if (ret < 0) {
		printk("%s:%d register framebuffer failed\n", __func__, __LINE__);
		ret = -1;
		goto dma_free_exit;
	}

	printk("register fb%d device\n", fbinfo->node);

#ifdef GPT_RGB2YUV_CONVERT
	info->thread = kthread_run(gpt_rgb2yuv_thread, info, "vdo_rgb2yuv");
	if (IS_ERR(info->thread))
		goto dma_free_exit;

	/* Set the timer to convert rgb to yuv. */
	init_timer(&info->timer);
	info->timer.expires = round_jiffies(jiffies + GPT_COLOR_CONVERT_FREQ);
	info->timer.data = (unsigned long)info;
	info->timer.function = gpt_rgb2yuv_timer;
//	add_timer(&info->timer);
#endif

	gpt_vdo_hwinit(info, g_vdo_mode);
	gpt_vdo_default_res(info, g_vdo_mode);
	gpt_vdo_enable(info);

	return 0;

dma_free_exit:
	iounmap(fbinfo->screen_base);
regs_remap_exit:
	iounmap(info->regs);
framebuff_exit:
	framebuffer_release(fbinfo);

	return ret;
}

static const struct of_device_id gpt_vdo_match[] = {
        { .name = "gpt-vdo", },
        {},
};
MODULE_DEVICE_TABLE(of, gpt_vdo_match);

static struct platform_driver gpt_vdo_driver = {
	.probe = gpt_vdo_probe,
	.remove = gpt_vdo_remove,
	.suspend = gpt_vdo_suspend,
	.resume = gpt_vdo_resume,
	.driver = {
		.name = "gpt-vdo",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(gpt_vdo_match),
	},
};

static int __init gpt_vdo_driver_init(void)
{
	return platform_driver_register(&gpt_vdo_driver);
}

static void __exit gpt_vdo_driver_exit(void)
{
	platform_driver_unregister(&gpt_vdo_driver);
}

module_init(gpt_vdo_driver_init);
module_exit(gpt_vdo_driver_exit);

MODULE_DESCRIPTION("Video Out Controller framebuffer driver");
MODULE_LICENSE("GPL");
