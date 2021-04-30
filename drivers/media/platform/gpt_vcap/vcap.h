/*
 * VCAP header file
 *
 * Copyright (C) 2018 GPT
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef VCAP_H
#define VCAP_H

#include <linux/io.h>
#include <linux/videodev2.h>
#include <media/gpt/vcap_type.h>

/* Maximum channel allowed */
#define VCAP_CAPTURE_NUM_CHANNELS	3
#define VCAP_CHANNEL_0 0
#define VCAP_CHANNEL_1 1
#define VCAP_CHANNEL_2 2

#define VIDEO_480P59H 0
#define VIDEO_720P30H 1
#define VIDEO_1080P30H 2
#define VIDEO_1080P60H 2
#define VIDEO_2K30H 3
#define VIDEO_2x720P30H 4

void *vcap_base0;
void *vcap_base1;
void *vcap_base2;
void *vcap_base;

#define AXI_BUS_32

#ifdef AXI_BUS_32
#define AXI_BUS_WIDTH 32
#define AXI_BUS_BYTE 4
#elif defined (AXI_BUS_256)
#define AXI_BUS_WIDTH 256
#define AXI_BUS_BYTE  32
#endif

//store mode
#define Packed 1
#define Semi 0

//rgb mode
#define RGB_601 1
#define RGB_709 2
#define CLOSE_RGB 0

#define VCAP0 0
#define VCAP1 1
#define VCAP2 2

//sync mode
#define SYNC 1
#define BT 0

//Bit width
#define BIT_8 8
#define BIT_10 10
#define BIT_16 16

//scan mode
#define I 0
#define P 1

//interface pin_select low/high 8 bit
#define LOW 0
#define HIGH 1

/* Register Address Offsets */
#define sync_ctrl_reg 0x00
#define vin_ctrl_reg 0x04
#define axi_wrconfig_reg 0x08
#define yuv2rgb_mult_y_reg 0x10
#define yuv2rgb_mult_u_reg 0x14
#define yuv2rgb_mult_v_reg 0x18
#define yuv2rgb_offset_reg 0x1C
#define yuv2rgb_limited_reg 0x20
#define int_raw_reg 0x100
#define int_mask_reg 0x104
#define INTR_STATUS_REG 0x108
#define INTR_STATUS_CLR 0x10C
#define int_line0_reg 0x110
#define int_line1_reg 0x114
#define cap_time_reg 0x118
#define early_record_reg 0x11c
#define win0_ctrl_reg 0x200
#define win0_oddstart_reg 0x204
#define win0_oddend_reg 0x208
#define win0_evenstart_reg 0x20c
#define win0_evenend_reg 0x210
#define error_type_reg 0x214
#define wr_base_0_low 0x218
#define wr_base_0_high 0x21c
#define wr_base_1_low 0x220
#define wr_base_1_high 0x224
#define WR_HDAT 0x228
#define WR_HIGH 0x22c
#define wr_stride 0x230
#define WR_CONFIG 0x234
#define wr_start 0x238
#define dma_line_record 0x23c

/*sync_ctrl_reg define*/
#define  XYZ_SEL 0x01
#define  VSYNC_SEL 0x10
#define  VSYNC_INV 0x20
#define  VSYNC_MODE 0xC0
#define  HSYNC_SEL 0x100
#define  HSYNC_INV 0x200
#define  HSYNC_AND 0x00
#define  HSYNC_MODE 0x3000
#define  DE_SEL 0x20000 //high
#define  X2_XYZ 0x100000
#define  START_FIELD 0x1000000

/*packed or simi_planner*/
#define TRANS_FORM 0x100000 //packed
#define WR_SEMI  0x200  // 1:semi 0:mix
#define Y_C_SWITCH  0x20000  // 0:c first 1:y first
#define WR_LENTH 0x8 

/*P or I*/
#define PROG_IN 0x2000 //P

/*offset_choice_en*/
#define OFF_CHOICE_EN 0x10000 //interlace must config 1,progress muster config 0

/*vin_ctrl_reg */
#define  WIDTH_SEL_8  0x1
#define  WIDTH_SEL_10  0x0
#define  WIDTH_SEL_12  0x2
#define  WIDTH_SEL_10_TO_8  0x3
#define  WIDTH_SEL_16  0x4
#define  WIDTH_SEL_8_2XYZ  0x5
#define Enable_Rgb 0x2000000

#define SEL_H8B  0x8
#define DITHER_EN 0x100

/*input pin select, 16 bit interlace bus ,select low 8 bit or high 8 bit*/
#define SEL_DIN 0x10

/*line_stride*/
#define LINE_STRIDE 0x3000 //16K Byte

/*dma control*/
#define EN_DMA 0x1
#define DIS_DMA 0x2

/*start capture*/
#define START_CAPTURE 0x1
/*stop capture*/
#define STOP_CAPTURE 0x0

/*clean capture down intr*/
#define CAP_INT 0x10000

/*clean dma intr*/
#define CLEAN_DMA 0x4
#define DMA_DONE 0x4

/*dma frame int flag*/
#define FRAME_INT  0x4

/*error raw int flag*/
#define ERROE_RAW_INT  0x2

/*timeout int flag*/
#define TIME_OUT  0x20000


/*LINE_INTR_EN*/
#define LINE_EN 0x80000000

/*INTR MASK FLAG*/
#define CHECK_LINE0 0x10
#define CHECK_LINE1 0x20
#define CHECK_SOF 0x100
#define CHECK_EOF 0x200
#define CHECK_SOL 0x400
#define CHECK_EOL 0x800
#define CHECK_WIN0_CAP_DOWN 0x10000

/*error type*/
#define FRAME_EARLY 0x10
#define LINE_EARLY 0x20


#define FRAME_INTERVAL 0xA00000 //10M 1080P one frame <4M

#define GPT_VDI_INT_FIFO		(0x1 << 0)
#define GPT_VDI_INT_ERROR		(0x1 << 1)
#define GPT_VDI_DMA_DONE		(0x1 << 2)
#define GPT_VDI_INT_LINE0		(0x1 << 4)
#define GPT_VDI_INT_LINE1		(0x1 << 5)
#define GPT_VDI_INT_SOF			(0x1 << 8)
#define GPT_VDI_INT_EOF			(0x1 << 9)
#define GPT_VDI_INT_SOL			(0x1 << 10)
#define GPT_VDI_INT_EOL			(0x1 << 11)
#define GPT_VDI_INT_WIN0		(0x1 << 16)
#define GPT_VDI_INT_TIMEOUT		(0x1 << 17)

#define GPT_VDI_IRQ_MASK	(GPT_VDI_INT_FIFO | GPT_VDI_INT_ERROR | \
        GPT_VDI_INT_WIN0 | GPT_VDI_INT_TIMEOUT | GPT_VDI_INT_SOF | \
        GPT_VDI_INT_SOL | GPT_VDI_INT_EOF | GPT_VDI_INT_EOL | GPT_VDI_INT_LINE0 | \
        GPT_VDI_INT_LINE1)

#define GPT_VDI_IRQ_DISABLE (GPT_VDI_INT_WIN0 | GPT_VDI_INT_TIMEOUT | GPT_VDI_INT_SOF | \
        GPT_VDI_INT_SOL | GPT_VDI_INT_EOF | GPT_VDI_INT_EOL | GPT_VDI_INT_LINE0 | GPT_VDI_INT_LINE1)

#define GPT_VDI_IRQ_ENABLE	(GPT_VDI_DMA_DONE)

#define GPT_VDI_ERROR_TYPE_WIN0_INT_MISS	(0x1 << 0)
#define GPT_VDI_FRAME_EARLY			(0x1 << 4)
#define GPT_VDI_LINE_EARLY			(0x1 << 5)
#define GPT_VDI_Y_REQ_OVERFLOW			(0x1 << 8)
#define GPT_VDI_C_REQ_OVERFLOW			(0x1 << 9)

#define VCAP_MAX_NAME	(30)
#define VCAP_CH_PARAM_ROW_MAX	3
#define VCAP_CH_PARAM_COLUMN_MAX 6

#define VCAP_CH0_MAX_MODES	22
#define VCAP_CH1_MAX_MODES	2
#define VCAP_CH2_MAX_MODES	15
//#define VIDEO_DEBUG

void clear_capture_down(int dev_id);
void clear_dma(int dev_id);
void get_error_type_reg(int dev_id);

/* This structure will store size parameters as per the mode selected by user */

typedef struct {
    char name[VCAP_MAX_NAME];	/* Name of the mode */
    u16  odd_h_start;
    u16  odd_h_end;
    u16  odd_v_start;
    u16  odd_v_end;
    u16  even_h_start;
    u16  even_h_end;
    u16  even_v_start;
    u16  even_v_end;
} vcap_channel_config_params;

/* Macros to read/write registers */
static int gpt_vcap_write(void *addr, unsigned int value,int offset)
{
    writel(value, addr + offset);
    return 0;
}

static int gpt_vcap_read(void *addr,int offset)
{
    return readl(addr + offset);
}

void* get_base_addr(int dev_id);
/* inline function to set buffer addresses in case of Y/C seprate(semi) mode */
static inline void set_videobuf_addr(u64 base0,u64 base1,int dev_id)
{
    vcap_base = get_base_addr(dev_id);
    //addr_base0
    gpt_vcap_write(vcap_base, base0 & 0xffffffff, wr_base_0_low);
    gpt_vcap_write(vcap_base, (base0>>32)&0xff, wr_base_0_high);
    //addr_base1
    gpt_vcap_write(vcap_base, base1 & 0xffffffff, wr_base_1_low);
    gpt_vcap_write(vcap_base, (base1>>32)&0xff, wr_base_1_high);
}

/*
 * vcap_ch_params: video standard configuration parameters for vcap
 *
 * The table must include all presets from supported subdevices.
 */
vcap_channel_config_params sh_isp = {
    .name = "1080p30fps",
    .odd_h_start = 146,
    .odd_h_end = 2065,
    .odd_v_start = 41,
    .odd_v_end = 1120,
    .even_h_start = 146,
    .even_h_end = 2065,
    .even_v_start = 41,
    .even_v_end = 1120,
};
EXPORT_SYMBOL(sh_isp);

vcap_channel_config_params hi3519 = {
    .name = "1080p60fps",
    .odd_h_start = 276,
    .odd_h_end = 2195,
    .odd_v_start = 0,
    .odd_v_end = 1079,
    .even_h_start = 276,
    .even_h_end = 2195,
    .even_v_start = 0,
    .even_v_end = 1079,
};
EXPORT_SYMBOL(hi3519);

vcap_channel_config_params chaokong = {
    .name = "2*720P30Hz",
    .odd_h_start = 0,
    .odd_h_end = 2559,
    .odd_v_start = 0,
    .odd_v_end = 719,
    .even_h_start = 0,
    .even_h_end = 2559,
    .even_v_start = 0,
    .even_v_end = 719,
};
EXPORT_SYMBOL(chaokong);

vcap_channel_config_params nt99141= {
   .name = "720p30Hz",
   .odd_h_start = 0,
   .odd_h_end = 1279,
   .odd_v_start = 0,
   .odd_v_end = 719,
   .even_h_start = 0,
   .even_h_end = 1279,
   .even_v_start = 0,
   .even_v_end = 719,
};
EXPORT_SYMBOL(nt99141);

vcap_channel_config_params adv7612= {
   .name = "1080p60fps",
   .odd_h_start = 276,
   .odd_h_end = 2195,
   .odd_v_start = 0,
   .odd_v_end = 1079,
   .even_h_start = 276,
   .even_h_end = 2195,
   .even_v_start = 0,
   .even_v_end = 1079,
};
EXPORT_SYMBOL(adv7612);

/* ----------------------------------------------------------------------- */
void* get_base_addr(int dev_id){

    if(dev_id ==0){
        vcap_base = vcap_base0;
    }else if(dev_id == 1){
        vcap_base = vcap_base1;
    }else if(dev_id == 2){
        vcap_base = vcap_base2;
    }else{
        return NULL;
    }
    return vcap_base;
}
EXPORT_SYMBOL(get_base_addr);

/* ----------------------------------------------------------------------- */

int vcap_intr_status(int dev_id)
{
    if (dev_id < 0 || dev_id > 3)
        return 0;

    vcap_base = get_base_addr(dev_id);
    if((gpt_vcap_read(vcap_base,INTR_STATUS_REG)&DMA_DONE) !=  0){
        clear_capture_down(dev_id);
        clear_dma(dev_id);
        return 1;
    }else{
        get_error_type_reg(dev_id);
        return 0;
    }
}
EXPORT_SYMBOL(vcap_intr_status);
/* ----------------------------------------------------------------------- */

void start_capture(int dev_id){

    vcap_base = get_base_addr(dev_id);
    /*start capture data*/
    gpt_vcap_write(vcap_base,START_CAPTURE, win0_ctrl_reg);//4
}
EXPORT_SYMBOL(start_capture);
/* ----------------------------------------------------------------------- */

void stop_capture(int dev_id){

    vcap_base = get_base_addr(dev_id);
    /*stop capture data*/
    gpt_vcap_write(vcap_base,STOP_CAPTURE, win0_ctrl_reg);//4
}
EXPORT_SYMBOL(stop_capture);
/* ----------------------------------------------------------------------- */
void enable_dma_intr(int dev_id){
    unsigned int value;

    vcap_base = get_base_addr(dev_id);
    value = gpt_vcap_read(vcap_base, int_mask_reg);
    gpt_vcap_write(vcap_base,~GPT_VDI_IRQ_ENABLE & value, int_mask_reg);
}

/* ----------------------------------------------------------------------- */
void enable_all_intr(int dev_id){

    vcap_base = get_base_addr(dev_id);
    gpt_vcap_write(vcap_base,0,int_mask_reg);
}
/* ----------------------------------------------------------------------- */
void set_intr_mask(int val,int dev_id){
    unsigned int value;

    vcap_base = get_base_addr(dev_id);
    value = gpt_vcap_read(vcap_base, int_mask_reg);

    gpt_vcap_write(vcap_base, val | value, int_mask_reg);

}
EXPORT_SYMBOL(set_intr_mask);
/* ----------------------------------------------------------------------- */

void clear_capture_down(int dev_id){

    vcap_base = get_base_addr(dev_id);
    gpt_vcap_write(vcap_base,CAP_INT, INTR_STATUS_CLR);
}
/* ----------------------------------------------------------------------- */
void line0_intr(unsigned int num,int dev_id){

    unsigned int val=0;

    vcap_base = get_base_addr(dev_id);
    val = gpt_vcap_read(vcap_base, int_line0_reg);

    gpt_vcap_write(vcap_base,val | num , int_line0_reg);
}

/* ----------------------------------------------------------------------- */
void line1_intr(unsigned int num,int dev_id){

    unsigned int val=0;

    vcap_base = get_base_addr(dev_id);
    val = gpt_vcap_read(vcap_base, int_line1_reg);

    gpt_vcap_write(vcap_base,val | num , int_line1_reg);
}

/* ----------------------------------------------------------------------- */
void clear_line_intr(int dev_id){

    vcap_base = get_base_addr(dev_id);
    gpt_vcap_write(vcap_base,CHECK_LINE0|CHECK_LINE1, INTR_STATUS_CLR);
}
/* ----------------------------------------------------------------------- */

void clear_dma(int dev_id){

    vcap_base = get_base_addr(dev_id);
    gpt_vcap_write(vcap_base,CLEAN_DMA, INTR_STATUS_CLR);
}
/* ----------------------------------------------------------------------- */

void clear_all_intr(int dev_id){
    vcap_base = get_base_addr(dev_id);
    gpt_vcap_write(vcap_base,0x3FFFF, INTR_STATUS_CLR);
}
/* ----------------------------------------------------------------------- */
void set_other_reg(int dev_id){
    vcap_base = get_base_addr(dev_id);
    /*axi bus width and packed or simi mode*/
    gpt_vcap_write(vcap_base,0x208, WR_CONFIG);//4 

}
/* ----------------------------------------------------------------------- */
void disable_dma(int dev_id){
    unsigned int val;

    vcap_base = get_base_addr(dev_id);
    val = gpt_vcap_read(vcap_base, wr_start);
    /*dma stop*/
    gpt_vcap_write(vcap_base,DIS_DMA &val, wr_start);//4 
}
EXPORT_SYMBOL(disable_dma);
/* ----------------------------------------------------------------------- */

void get_error_type_reg(int dev_id){

    unsigned int val=0;

    vcap_base = get_base_addr(dev_id);
    val = gpt_vcap_read(vcap_base, error_type_reg);
    printk("%s %d channel_id 0x%d ,error_type_reg val 0x%x \n",__FUNCTION__,__LINE__,dev_id,val);

    if((val & GPT_VDI_FRAME_EARLY)!=0){
        printk("frame_early and frame num is 0x%x\n",gpt_vcap_read(vcap_base, early_record_reg));

    }else if((val & GPT_VDI_LINE_EARLY)!=0){
        printk("frame_early and line num is 0x%x\n",gpt_vcap_read(vcap_base, early_record_reg));

    }else if((val & GPT_VDI_ERROR_TYPE_WIN0_INT_MISS)==1){
        printk("GPT_VDI_ERROR_TYPE_WIN0_INT_MISS error ...........\n");

    }else if((val & GPT_VDI_Y_REQ_OVERFLOW)!=0){
        printk("GPT_VDI_Y_REQ_OVERFLOW error ...........\n");

    }else if((val & GPT_VDI_C_REQ_OVERFLOW)!=0){
        printk("GPT_VDI_C_REQ_OVERFLOW error ...........\n");
    }

    clear_all_intr(dev_id);
    stop_capture(dev_id);
    disable_dma(dev_id);
}

/* ----------------------------------------------------------------------- */
int gpt_intr(int dev_id){
    int status = 0;
    unsigned regs=0;

    vcap_base = get_base_addr(dev_id);
    regs = gpt_vcap_read(vcap_base, INTR_STATUS_REG);
    if (regs & GPT_VDI_INT_FIFO) {
        printk("GPT_VDI_INT_FIFO\n");
    }

    if (regs & GPT_VDI_INT_ERROR) {
        get_error_type_reg(dev_id);
    }

    if (regs & GPT_VDI_DMA_DONE) {
        clear_capture_down(dev_id);
        clear_dma(dev_id);
    }

    if (regs & GPT_VDI_INT_LINE0) {
        printk("get line0 intr .........................\n");
    }

    if (regs & GPT_VDI_INT_LINE1) {
        printk("get line1 intr .........................\n");
    }

    if (regs & GPT_VDI_INT_SOF) {
        printk("get sof intr .........................\n");
    }

    if (regs & GPT_VDI_INT_EOF) {
        printk("get eof intr .........................\n");
    }

    if (regs & GPT_VDI_INT_SOL) {
        printk("get sol intr .........................\n");
    }

    if (regs & GPT_VDI_INT_EOL) {
        printk("get eol intr .........................\n");
    }

    if (regs & GPT_VDI_INT_TIMEOUT) {
        printk("get timeout intr .........................\n");
    }

    return status;
}

/* ----------------------------------------------------------------------- */
void get_dma_line_record(int dev_id){
    unsigned int val;

    vcap_base = get_base_addr(dev_id);

    printk("dma_record_line is  %d ............\n ",gpt_vcap_read(vcap_base, dma_line_record));
}

int dma_int_check(int dev_id){
    unsigned int val,val1;

    vcap_base = get_base_addr(dev_id);
    val = gpt_vcap_read(vcap_base, int_raw_reg);
    val1 = gpt_vcap_read(vcap_base,error_type_reg);

    if((val&FRAME_INT) !=  0){
        clear_dma(dev_id);
        printk("dma_record_line is  %d ............\n ",gpt_vcap_read(vcap_base, dma_line_record));
        return 1;
    }else{
#ifdef VIDEO_DEBUG
        printk("function id %s ,line %d  int_raw_reg 0x%x, error_type_reg 0x%x \n ",__func__,__LINE__,val,val1);
#endif
        return 0;
    }
}
/* ----------------------------------------------------------------------- */

int error_raw_int_check(int dev_id){
    unsigned int val=0;

    vcap_base = get_base_addr(dev_id);
    val = gpt_vcap_read(vcap_base, int_raw_reg);
    if((val&ERROE_RAW_INT) !=  0){
        printk("error_type_reg 0x%x \n ",val);
        gpt_vcap_write(vcap_base,0x2, INTR_STATUS_CLR);
        return 1;
    }else{
        return 0;
    }
}
/* ----------------------------------------------------------------------- */

void enable_dma(int dev_id){

    vcap_base = get_base_addr(dev_id);
    /*dma start*/
    gpt_vcap_write(vcap_base,EN_DMA, wr_start);//4 
}
EXPORT_SYMBOL(enable_dma);
/* ----------------------------------------------------------------------- */

void set_trans_form_type(int dev_id){
    volatile int value,val,val1;

    vcap_base = get_base_addr(dev_id);
    value = gpt_vcap_read(vcap_base, vin_ctrl_reg);
    val = gpt_vcap_read(vcap_base, WR_CONFIG);
    val1 = gpt_vcap_read(vcap_base,WR_HDAT);

    if(value&0x100000 != 0x100000){//simi
        gpt_vcap_write(vcap_base,~TRANS_FORM & value, vin_ctrl_reg);
        gpt_vcap_write(vcap_base,WR_SEMI|val, WR_CONFIG);
        val1 = val1/2;
        gpt_vcap_write(vcap_base,val1, WR_HDAT);
#ifdef VIDEO_DEBUG
        printk("semi mode vin_ctrl_reg is 0x%x and WR_CONFIG is 0x%x,wr_hdata is 0x%x \n",gpt_vcap_read(vcap_base, vin_ctrl_reg),gpt_vcap_read(vcap_base, WR_CONFIG),val1);
#endif
    }else if( value&0x100000 == 0x100000)
        {//packaged
        gpt_vcap_write(vcap_base,TRANS_FORM |value,  vin_ctrl_reg);
        gpt_vcap_write(vcap_base,~WR_SEMI & val, WR_CONFIG);
#ifdef VIDEO_DEBUG
        printk("packed mode vin_ctrl_reg is 0x%x and WR_CONFIG is 0x%x\n",gpt_vcap_read(vcap_base, WR_CONFIG),gpt_vcap_read(vcap_base, vin_ctrl_reg));
#endif
    }else{//packaged
        gpt_vcap_write(vcap_base,TRANS_FORM |value,  vin_ctrl_reg);
        gpt_vcap_write(vcap_base,~WR_SEMI& val, WR_CONFIG);
#ifdef VIDEO_DEBUG
        printk("packed mode vin_ctrl_reg is 0x%x and WR_CONFIG is 0x%x \n",gpt_vcap_read(vcap_base, WR_CONFIG),gpt_vcap_read(vcap_base, vin_ctrl_reg));
#endif
    }
}
EXPORT_SYMBOL(set_trans_form_type);
/* ----------------------------------------------------------------------- */

/* ----------------------------------------------------------------------- */
void set_sync_vin_reg(int sync_val,int vin_val,int dev_id){

    vcap_base = get_base_addr(dev_id);
    /*sync and vin reg config*/
    gpt_vcap_write(vcap_base,0,sync_ctrl_reg);
    gpt_vcap_write(vcap_base,sync_val,sync_ctrl_reg);
    
    gpt_vcap_write(vcap_base,0,vin_ctrl_reg);
    gpt_vcap_write(vcap_base,vin_val,vin_ctrl_reg);

    set_other_reg(dev_id);

}
EXPORT_SYMBOL(set_sync_vin_reg);

/* ----------------------------------------------------------------------- */
void set_sync_reg(int sync_val,int dev_id){

    vcap_base = get_base_addr(dev_id);
    /*sync and vin reg config*/
    gpt_vcap_write(vcap_base,0,sync_ctrl_reg);
    gpt_vcap_write(vcap_base,sync_val,sync_ctrl_reg);
}
EXPORT_SYMBOL(set_sync_reg);


/* ----------------------------------------------------------------------- */
void set_vin_reg(int reg_val,int dev_id){

    vcap_base = get_base_addr(dev_id);
    /*sync and vin reg config*/
    gpt_vcap_write(vcap_base,0,vin_ctrl_reg);
    gpt_vcap_write(vcap_base,reg_val,vin_ctrl_reg);

    set_other_reg(dev_id);
}
EXPORT_SYMBOL(set_vin_reg);

/* ----------------------------------------------------------------------- */
void open_rgb(int val,int dev_id){

    unsigned int value;

    vcap_base = get_base_addr(dev_id);
    value = gpt_vcap_read(vcap_base,vin_ctrl_reg);

    if((value & Enable_Rgb)!=0){
        printk("yuv2rgb enabled \n");        
    }

    if(val == 1){//601
        gpt_vcap_write(vcap_base,0x48c,yuv2rgb_mult_y_reg);
        gpt_vcap_write(vcap_base,0x07E11E78,yuv2rgb_mult_u_reg);
        gpt_vcap_write(vcap_base,0x1CD3063c,yuv2rgb_mult_v_reg);
        gpt_vcap_write(vcap_base,0x018001F0,yuv2rgb_offset_reg);
        gpt_vcap_write(vcap_base,0x00FF,yuv2rgb_limited_reg);

    }else if(val == 2){//709
        gpt_vcap_write(vcap_base,0x3E8,yuv2rgb_mult_y_reg);
        gpt_vcap_write(vcap_base,0xE7401F45,yuv2rgb_mult_u_reg);
        gpt_vcap_write(vcap_base,0x1E2CE627,yuv2rgb_mult_v_reg);
        gpt_vcap_write(vcap_base,0x1F80E000,yuv2rgb_offset_reg);
        gpt_vcap_write(vcap_base,0x00FF,yuv2rgb_limited_reg);

    }else if(val == 3){
        gpt_vcap_write(vcap_base,0x000,yuv2rgb_mult_y_reg);
        gpt_vcap_write(vcap_base,0x000,yuv2rgb_mult_u_reg);
        gpt_vcap_write(vcap_base,0x000,yuv2rgb_mult_v_reg);
        gpt_vcap_write(vcap_base,0x000,yuv2rgb_offset_reg);

    }
}
EXPORT_SYMBOL(open_rgb);

/* ----------------------------------------------------------------------- */
void set_hdat_high_stride(vcap_channel_config_params param,u8 dev_id){

    unsigned int wr_hdat =0;
    unsigned int wr_high = 0;
    unsigned int reg_val,width_sel_val;

    vcap_base = get_base_addr(dev_id);
#ifdef VIDEO_DEBUG
    printk("odd/even value is : %d %d %d %d %d %d %d %d\n",param.even_h_end,param.even_h_start,
            param.even_v_end,param.even_v_start,param.odd_h_end,param.odd_h_start,
            param.odd_v_end,param.odd_v_start);
#endif

    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base,win0_oddstart_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.odd_h_start | (param.odd_v_start<<16)), win0_oddstart_reg);
#ifdef VIDEO_DEBUG
    printk("win0_oddstart_reg value is 0x%x \n",gpt_vcap_read(vcap_base, win0_oddstart_reg));
#endif
    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, win0_oddend_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.odd_h_end| (param.odd_v_end<<16)), win0_oddend_reg);

#ifdef VIDEO_DEBUG
    printk("win0_oddend_reg value is 0x%x  \n",gpt_vcap_read(vcap_base, win0_oddend_reg));
#endif

    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, win0_evenstart_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.even_h_start | (param.even_v_start<<16)), win0_evenstart_reg);

#ifdef VIDEO_DEBUG
    printk("win0_evenstart_reg value is 0x%x  \n",gpt_vcap_read(vcap_base, win0_evenstart_reg));
#endif

    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, win0_evenend_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.even_h_end | (param.even_v_end<<16)), win0_evenend_reg);

#ifdef VIDEO_DEBUG
    printk("win0_evenend_reg value is 0x%x  \n",gpt_vcap_read(vcap_base, win0_evenend_reg));
#endif


    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, WR_CONFIG);
    width_sel_val = 0;
    width_sel_val = gpt_vcap_read(vcap_base, vin_ctrl_reg);
#ifdef VIDEO_DEBUG
    printk("WR_CONFIG 0x%x,vin_ctrl_reg id 0x%x\n",reg_val,width_sel_val);
#endif
    if((reg_val&WR_SEMI)==0){
        if(((width_sel_val&WIDTH_SEL_8_2XYZ)==WIDTH_SEL_8_2XYZ)||((width_sel_val&WIDTH_SEL_8)==WIDTH_SEL_8)||((width_sel_val&WIDTH_SEL_10_TO_8)==WIDTH_SEL_10_TO_8)||((width_sel_val&WIDTH_SEL_16)==WIDTH_SEL_16)){
            if(((param.even_h_end-param.even_h_start+1)*2)%AXI_BUS_BYTE)
            {
                wr_hdat=(((param.even_h_end-param.even_h_start+1)*2)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1)*2;
            }
        }
        if(((width_sel_val&WIDTH_SEL_8_2XYZ)!=WIDTH_SEL_8_2XYZ)&&((width_sel_val&WIDTH_SEL_8)!=WIDTH_SEL_8)&&((width_sel_val&WIDTH_SEL_10_TO_8)!=WIDTH_SEL_10_TO_8)&&((width_sel_val&WIDTH_SEL_16)!=WIDTH_SEL_16)){
            if(((param.even_h_end-param.even_h_start+1)*4)%AXI_BUS_BYTE){
                wr_hdat=(((param.even_h_end-param.even_h_start+1)*4)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1)*4;
            }
        }
    }

    if((reg_val&WR_SEMI)==0x200){

        if(((width_sel_val&WIDTH_SEL_8_2XYZ)==WIDTH_SEL_8_2XYZ)||((width_sel_val&WIDTH_SEL_8)==WIDTH_SEL_8)||((width_sel_val&WIDTH_SEL_10_TO_8)==WIDTH_SEL_10_TO_8)||((width_sel_val&WIDTH_SEL_16)==WIDTH_SEL_16))
        {
            if(((param.even_h_end-param.even_h_start+1))%AXI_BUS_BYTE){
                wr_hdat=((param.even_h_end-param.even_h_start+1)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1);
            }
        }
        if(((width_sel_val&WIDTH_SEL_8_2XYZ)!=WIDTH_SEL_8_2XYZ)&&((width_sel_val&WIDTH_SEL_8)!=WIDTH_SEL_8)&&((width_sel_val&WIDTH_SEL_10_TO_8)!=WIDTH_SEL_10_TO_8)&&((width_sel_val&WIDTH_SEL_16)!=WIDTH_SEL_16))
        {
            if(((param.even_h_end-param.even_h_start+1)*2)%AXI_BUS_BYTE){
                wr_hdat=(((param.even_h_end-param.even_h_start+1)*2)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1)*2;
            }
        }
    }

#ifdef VIDEO_DEBUG
    printk("wr_hdat value is : 0x%x \n",wr_hdat);
#endif
    reg_val = gpt_vcap_read(vcap_base,vin_ctrl_reg);
    /*active data in a line */
    if((reg_val >> 25)&0x1 >0){
        gpt_vcap_write(vcap_base,wr_hdat*2, WR_HDAT);
        gpt_vcap_write(vcap_base,wr_hdat*2, wr_stride);//4
    }else{
        gpt_vcap_write(vcap_base,wr_hdat, WR_HDAT);
        gpt_vcap_write(vcap_base,wr_hdat, wr_stride);//4
    }

    /*active pixels  in a column */
    wr_high = param.even_v_end-param.even_v_start+1;
    gpt_vcap_write(vcap_base,wr_high, WR_HIGH);

#ifdef VIDEO_DEBUG
    printk("wr_hdat value is : 0x%x  wr_stride value is 0x%x ,wr_high is 0x%x \n",
            wr_hdat,gpt_vcap_read(vcap_base, wr_stride),gpt_vcap_read(vcap_base,WR_HIGH));
#endif
}
EXPORT_SYMBOL(set_hdat_high_stride);
/* ----------------------------------------------------------------------- */
void set_hdat_high_stride_for_2channel(vcap_channel_config_params param,u8 dev_id){

    unsigned int wr_hdat =0;
    unsigned int wr_high = 0;
    unsigned int reg_val,width_sel_val;

    vcap_base = get_base_addr(dev_id);
#ifdef VIDEO_DEBUG
    printk("odd/even value is : %d %d %d %d %d %d %d %d\n",param.even_h_end,param.even_h_start,
            param.even_v_end,param.even_v_start,param.odd_h_end,param.odd_h_start,
            param.odd_v_end,param.odd_v_start);
#endif

    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base,win0_oddstart_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.odd_h_start | (param.odd_v_start<<16)), win0_oddstart_reg);
#ifdef VIDEO_DEBUG
    printk("win0_oddstart_reg value is 0x%x \n",gpt_vcap_read(vcap_base, win0_oddstart_reg));
#endif
    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, win0_oddend_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.odd_h_end| (param.odd_v_end<<16)), win0_oddend_reg);

#ifdef VIDEO_DEBUG
    printk("win0_oddend_reg value is 0x%x  \n",gpt_vcap_read(vcap_base, win0_oddend_reg));
#endif

    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, win0_evenstart_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.even_h_start | (param.even_v_start<<16)), win0_evenstart_reg);

#ifdef VIDEO_DEBUG
    printk("win0_evenstart_reg value is 0x%x  \n",gpt_vcap_read(vcap_base, win0_evenstart_reg));
#endif

    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, win0_evenend_reg);
    gpt_vcap_write(vcap_base,(reg_val | param.even_h_end | (param.even_v_end<<16)), win0_evenend_reg);

#ifdef VIDEO_DEBUG
    printk("win0_evenend_reg value is 0x%x  \n",gpt_vcap_read(vcap_base, win0_evenend_reg));
#endif


    reg_val = 0;
    reg_val = gpt_vcap_read(vcap_base, WR_CONFIG);
    width_sel_val = 0;
    width_sel_val = gpt_vcap_read(vcap_base, vin_ctrl_reg);
#ifdef VIDEO_DEBUG
    printk("WR_CONFIG 0x%x,vin_ctrl_reg id 0x%x\n",reg_val,width_sel_val);
#endif
    if((reg_val&WR_SEMI)==0){
        if(((width_sel_val&WIDTH_SEL_8_2XYZ)==WIDTH_SEL_8_2XYZ)||((width_sel_val&WIDTH_SEL_8)==WIDTH_SEL_8)||((width_sel_val&WIDTH_SEL_10_TO_8)==WIDTH_SEL_10_TO_8)||((width_sel_val&WIDTH_SEL_16)==WIDTH_SEL_16)){
            if(((param.even_h_end-param.even_h_start+1)*2)%AXI_BUS_BYTE)
            {
                wr_hdat=(((param.even_h_end-param.even_h_start+1)*2)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1)*2;
            }
        }
        if(((width_sel_val&WIDTH_SEL_8_2XYZ)!=WIDTH_SEL_8_2XYZ)&&((width_sel_val&WIDTH_SEL_8)!=WIDTH_SEL_8)&&((width_sel_val&WIDTH_SEL_10_TO_8)!=WIDTH_SEL_10_TO_8)&&((width_sel_val&WIDTH_SEL_16)!=WIDTH_SEL_16)){
            if(((param.even_h_end-param.even_h_start+1)*4)%AXI_BUS_BYTE){
                wr_hdat=(((param.even_h_end-param.even_h_start+1)*4)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1)*4;
            }
        }
    }

    if((reg_val&WR_SEMI)==0x200){

        if(((width_sel_val&WIDTH_SEL_8_2XYZ)==WIDTH_SEL_8_2XYZ)||((width_sel_val&WIDTH_SEL_8)==WIDTH_SEL_8)||((width_sel_val&WIDTH_SEL_10_TO_8)==WIDTH_SEL_10_TO_8)||((width_sel_val&WIDTH_SEL_16)==WIDTH_SEL_16))
        {
            if(((param.even_h_end-param.even_h_start+1))%AXI_BUS_BYTE){
                wr_hdat=((param.even_h_end-param.even_h_start+1)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1);
            }
        }
        if(((width_sel_val&WIDTH_SEL_8_2XYZ)!=WIDTH_SEL_8_2XYZ)&&((width_sel_val&WIDTH_SEL_8)!=WIDTH_SEL_8)&&((width_sel_val&WIDTH_SEL_10_TO_8)!=WIDTH_SEL_10_TO_8)&&((width_sel_val&WIDTH_SEL_16)!=WIDTH_SEL_16))
        {
            if(((param.even_h_end-param.even_h_start+1)*2)%AXI_BUS_BYTE){
                wr_hdat=(((param.even_h_end-param.even_h_start+1)*2)/AXI_BUS_BYTE+1)*AXI_BUS_BYTE;
            }else{
                wr_hdat=(param.even_h_end-param.even_h_start+1)*2;
            }
        }
    }

#ifdef VIDEO_DEBUG
    printk("wr_hdat value is : 0x%x \n",wr_hdat);
#endif
    reg_val = gpt_vcap_read(vcap_base,vin_ctrl_reg);
    /*active data in a line */
    if((reg_val >> 25)&0x1 >0){
        gpt_vcap_write(vcap_base,wr_hdat*2, WR_HDAT);
        gpt_vcap_write(vcap_base,wr_hdat*2, wr_stride);//4
    }else{
        gpt_vcap_write(vcap_base,wr_hdat, WR_HDAT);
        gpt_vcap_write(vcap_base,wr_hdat*2, wr_stride);//4
    }

    /*active pixels  in a column */
    wr_high = param.even_v_end-param.even_v_start+1;
    gpt_vcap_write(vcap_base,wr_high, WR_HIGH);

#ifdef VIDEO_DEBUG
    printk("wr_hdat value is : 0x%x  wr_stride value is 0x%x ,wr_high is 0x%x \n",
            wr_hdat,gpt_vcap_read(vcap_base, wr_stride),gpt_vcap_read(vcap_base,WR_HIGH));
#endif
}


void set_time_out(int res,int dev_id){

    vcap_base = get_base_addr(dev_id); 
    switch(res){
        case 0://1080P25HZ
            break;
        case 1://720P 1280*720 50HZ

            gpt_vcap_write(vcap_base,0xFc23d0,cap_time_reg);
#ifdef VIDEO_DEBUG
            printk("cap_time_reg is  0x%x...........................\n",gpt_vcap_read(vcap_base,cap_time_reg));
#endif
            break;
        case 2://PAL 720*576I60HZ

            break;
        case 3://720*480P60HZ
            gpt_vcap_write(vcap_base,0xFc23d0,cap_time_reg);

#ifdef VIDEO_DEBUG
            printk("cap_time_reg is  0x%x...........................\n",gpt_vcap_read(vcap_base,cap_time_reg));
#endif
            break;
        default://720*480P60HZ NTSC

            break;
    }
}

/* ----------------------------------------------------------------------- */
void print_vcap0_reg(int dev_id){
    int i;

    vcap_base = get_base_addr(dev_id);
    printk("print vcap%d reg \n", dev_id);
    printk("sync_ctrl_reg val 0x%x,vin_ctrl_reg 0x%x \n",gpt_vcap_read(vcap_base,sync_ctrl_reg),gpt_vcap_read(vcap_base,vin_ctrl_reg));
    printk("win0_ctrl_reg 0x%x \n ",gpt_vcap_read(vcap_base,win0_ctrl_reg));
    printk("win0_oddstart_reg 0x%x \n ",gpt_vcap_read(vcap_base,win0_oddstart_reg));
    printk("win0_oddend_reg 0x%x \n ",gpt_vcap_read(vcap_base,win0_oddend_reg));
    printk("win0_evenstart_reg 0x%x \n ",gpt_vcap_read(vcap_base,win0_evenstart_reg));
    printk("win0_evenend_reg 0x%x \n ",gpt_vcap_read(vcap_base,win0_evenend_reg));
    printk("error_type_reg 0x%x \n ",gpt_vcap_read(vcap_base,error_type_reg));
    printk("wr_base_0_low 0x%x \n ",gpt_vcap_read(vcap_base,wr_base_0_low));
    printk("wr_base_0_high 0x%x \n ",gpt_vcap_read(vcap_base,wr_base_0_high));
    printk("wr_base_1_low 0x%x \n ",gpt_vcap_read(vcap_base,wr_base_1_low));
    printk("wr_base_1_high 0x%x \n ",gpt_vcap_read(vcap_base,wr_base_1_high)); 
    printk("wr_hdat 0x%x \n ",gpt_vcap_read(vcap_base,WR_HDAT));
    printk("wr_high 0x%x \n ",gpt_vcap_read(vcap_base,WR_HIGH));
    printk("wr_stride 0x%x \n ",gpt_vcap_read(vcap_base,wr_stride));
    printk("wr_config 0x%x \n ",gpt_vcap_read(vcap_base,WR_CONFIG));
    printk("intr_mask 0x%x \n ",gpt_vcap_read(vcap_base,int_mask_reg));
    printk("wr_start 0x%x \n ",gpt_vcap_read(vcap_base,wr_start));
    printk("dma_line_record 0x%x \n ",gpt_vcap_read(vcap_base,0x23c));

    printk("sync_ctrl_reg val 0x%x,vin_ctrl_reg 0x%x \n",gpt_vcap_read(vcap_base,sync_ctrl_reg),gpt_vcap_read(vcap_base,vin_ctrl_reg));

    for(i = 0 ;i<= 0x20; i = i + 4)
    {
        printk(" reg num 0x%x val 0x%x \n ",i, gpt_vcap_read(vcap_base,i));
    }
    for(i = 0x100 ;i<= 0x11c; i = i + 4)
    {
        printk(" reg num 0x%x val 0x%x \n ",i, gpt_vcap_read(vcap_base,i));
    }
    for(i = 0x200 ;i<= 0x23c; i = i + 4)
    {
        printk(" reg num 0x%x val 0x%x \n ",i, gpt_vcap_read(vcap_base,i));
    }
}
/* ----------------------------------------------------------------------- */
#endif				/* End of #ifndef VCAP_H */

