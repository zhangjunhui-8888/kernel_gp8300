/*
 * Copyright (C)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_graph.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include "vcap.h"
#include "vcap_capture.h"

#define vcap_err(fmt, arg...)	v4l2_err(&vcap_obj.v4l2_dev, fmt, ## arg)
#define vcap_dbg(level, debug, fmt, arg...)	\
    v4l2_dbg(level, debug, &vcap_obj.v4l2_dev, fmt, ## arg)
//#define DEBUG
//#define VIDEO_DEBUG
#define VCAP_DRIVER_NAME	"gpt-vcap"
#define MAX_SD_NUM 3

static int debug = 0;
static char *vcap = "SH_ISP";
static u8 channel_first_int[VCAP_NUMBER_OF_OBJECTS][2] = { {1, 1},{1, 1},{1,1} };

module_param(debug, int, 0644);
module_param(vcap, charp, 0644);

MODULE_PARM_DESC(debug, "Debug level 0-1");

/* global variables */
static struct vcap_device vcap_obj = { {NULL} };
static struct device *vcap_dev;
struct v4l2_subdev *sd[MAX_SD_NUM];

void init_vcap(int dev_id,vcap_channel_config_params para);
static int gpt_extsrc_cfg(void);
int get_sd_flag = -1;

/* ----------------------------------------------------------------------- */
static int strcmp_zxp(const char *cs,const char *ct)
{
    signed char __res;

    while(1){
        if((__res = *cs - *ct++)!=0 || !*cs++)
            break;
    }
    return __res; 
}

/* ----------------------------------------------------------------------- */
static inline struct vcap_cap_buffer *to_vcap_buffer(struct vb2_buffer *vb)
{
    return container_of(vb, struct vcap_cap_buffer, vb);
}
/* ----------------------------------------------------------------------- */

/**
 * vcap_buffer_prepare :  callback function for buffer prepare
 * @vb: ptr to vb2_buffer
 *
 * This is the callback function for buffer prepare when vb2_qbuf()
 * function is called. The buffer is prepared and user space virtual address
 * or user address is converted into  physical address
 */
static int vcap_buffer_prepare(struct vb2_buffer *vb)
{
    struct vb2_queue *q = vb->vb2_queue;
    struct channel_obj *ch = vb2_get_drv_priv(q);
    struct common_obj *common;

    common = &ch->common;

    vb2_set_plane_payload(vb, 0, common->fmt.fmt.pix.sizeimage);
    if (vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
        return -EINVAL;

    vb->v4l2_buf.field = common->fmt.fmt.pix.field;
#ifdef VIDEO_DEBUG
    printk("entry %s %d and sizeimage 0x%x ,field %d\n",__FUNCTION__,__LINE__,common->fmt.fmt.pix.sizeimage,common->fmt.fmt.pix.field);
#endif

    return 0;
}
/* ----------------------------------------------------------------------- */

/**
 * vcap_buffer_queue_setup : Callback function for buffer setup.
 * @vq: vb2_queue ptr
 * @fmt: v4l2 format
 * @nbuffers: ptr to number of buffers requested by application
 * @nplanes:: contains number of distinct video planes needed to hold a frame
 * @sizes[]: contains the size (in bytes) of each plane.
 * @alloc_ctxs: ptr to allocation context
 *
 * This callback function is called when reqbuf() is called to adjust
 * the buffer count and buffer size
 */
static int vcap_buffer_queue_setup(struct vb2_queue *vq,
        const struct v4l2_format *fmt,
        unsigned int *nbuffers, unsigned int *nplanes,
        unsigned int sizes[], void *alloc_ctxs[])
{
    struct channel_obj *ch = vb2_get_drv_priv(vq);
    struct common_obj *common;
#ifdef VIDEO_DEBUG
    printk("entry %s %d,%d,0x%x \n",__FUNCTION__,__LINE__,*nbuffers,sizes[0]);
#endif
    common = &ch->common;

    if (fmt && fmt->fmt.pix.sizeimage < common->fmt.fmt.pix.sizeimage)
        return -EINVAL;

    if (vq->num_buffers + *nbuffers < 3)
        *nbuffers = 3 - vq->num_buffers;

    *nplanes = 1;
    sizes[0] = fmt ? fmt->fmt.pix.sizeimage : common->fmt.fmt.pix.sizeimage;
    alloc_ctxs[0] = common->alloc_ctx;

    return 0;
}
/* ----------------------------------------------------------------------- */

/**
 * vcap_buffer_queue : Callback function to add buffer to DMA queue
 * @vb: ptr to vb2_buffer
 */
static void vcap_buffer_queue(struct vb2_buffer *vb)
{
    struct channel_obj *ch = vb2_get_drv_priv(vb->vb2_queue);
    struct vcap_cap_buffer *buf = to_vcap_buffer(vb);
    struct common_obj *common;
    unsigned long flags;
#ifdef VIDEO_DEBUG
    printk("entry %s %d \n",__FUNCTION__,__LINE__);
#endif
    common = &ch->common;

    spin_lock_irqsave(&common->irqlock, flags);
    /* add the buffer to the DMA queue */
    list_add_tail(&buf->list, &common->dma_queue);
    spin_unlock_irqrestore(&common->irqlock, flags);
}

/* ----------------------------------------------------------------------- */

/**
 * vcap_start_streaming : Starts the DMA engine for streaming
 * @vb: ptr to vb2_buffer
 * @count: number of buffers
 */
static int vcap_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct channel_obj *ch = vb2_get_drv_priv(vq);
    struct common_obj *common = &ch->common;
    u64 addr;
    unsigned long flags;
    int  err;
#ifdef VIDEO_DEBUG
    printk("entry %s , %d ch_id is %d.....n",__FUNCTION__,__LINE__,ch->channel_id);
#endif

    spin_lock_irqsave(&common->irqlock, flags);
    /*reg set_addr function*/
    common->set_addr = set_videobuf_addr;

    /* Get the next frame from the buffer queue */
    common->cur_frm = common->next_frm = list_entry(common->dma_queue.next,
            struct vcap_cap_buffer, list);
    /* Remove buffer from the buffer queue */
    list_del(&common->cur_frm->list);
    spin_unlock_irqrestore(&common->irqlock, flags);

#ifdef CONFIG_DIRECT_DISPLAY
    addr = 0x220000000;
#else
    addr = vb2_dma_contig_plane_dma_addr(&common->next_frm->vb, 0);
#endif
    common->set_addr(addr,0x230000000,ch->channel_id);


    channel_first_int[VCAP_VIDEO_INDEX][ch->channel_id] = 1;

    if(get_sd_flag>=0){
        err = v4l2_subdev_call(sd[ch->channel_id],video,s_stream,0);
        if(err<0){
            printk("%s %d s_stream failed \n",__func__,__LINE__);
        }
    }
    start_capture(ch->channel_id);
    enable_dma(ch->channel_id);
    return 0;

}
/* ----------------------------------------------------------------------- */

/**
 * vcap_stop_streaming : Stop the DMA engine
 * @vq: ptr to vb2_queue
 *
 * This callback stops the DMA engine and any remaining buffers
 * in the DMA queue are released.
 */
static void vcap_stop_streaming(struct vb2_queue *vq)
{
    struct channel_obj *ch = vb2_get_drv_priv(vq);
    struct common_obj *common;
    unsigned long flags;

#ifdef VIDEO_DEBUG
    printk("%s,%d \n",__FUNCTION__,__LINE__);
#endif
    common = &ch->common;
    stop_capture(ch->channel_id);
    disable_dma(ch->channel_id);

    /* release all active buffers */
    spin_lock_irqsave(&common->irqlock, flags);
    if (common->cur_frm == common->next_frm) {
        vb2_buffer_done(&common->cur_frm->vb, VB2_BUF_STATE_ERROR);
    } else {
        if (common->cur_frm != NULL)
            vb2_buffer_done(&common->cur_frm->vb,
                    VB2_BUF_STATE_ERROR);
        if (common->next_frm != NULL)
            vb2_buffer_done(&common->next_frm->vb,
                    VB2_BUF_STATE_ERROR);
    }

    while (!list_empty(&common->dma_queue)) {
        common->next_frm = list_entry(common->dma_queue.next,
                struct vcap_cap_buffer, list);
        list_del(&common->next_frm->list);
        vb2_buffer_done(&common->next_frm->vb, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&common->irqlock, flags);
}

static struct vb2_ops video_qops = {
    .queue_setup		= vcap_buffer_queue_setup,//reqbuf call function
    .buf_prepare		= vcap_buffer_prepare,//vb2_qbuf() call function
    .start_streaming	= vcap_start_streaming,
    .stop_streaming		= vcap_stop_streaming,
    .buf_queue		= vcap_buffer_queue,//add buffer to DMA queue
};
/* ----------------------------------------------------------------------- */

/**
 * vcap_process_buffer_complete: process a completed buffer
 * @common: ptr to common channel object
 *
 * This function time stamp the buffer and mark it as DONE. It also
 * wake up any process waiting on the QUEUE and set the next buffer
 * as current
 */
static void vcap_process_buffer_complete(struct common_obj *common)
{
#ifdef VIDEO_DEBUG
    printk("[zxp] vcap_process_buffer_complete\n");
#endif
    v4l2_get_timestamp(&common->cur_frm->vb.v4l2_buf.timestamp);
    vb2_buffer_done(&common->cur_frm->vb,VB2_BUF_STATE_DONE);
    /* Make curFrm pointing to nextFrm */
    common->cur_frm = common->next_frm;
}
/* ----------------------------------------------------------------------- */

/**
 * vcap_schedule_next_buffer: set next buffer address for capture
 * @common : ptr to common channel object
 *
 * This function will get next buffer from the dma queue and
 * set the buffer address in the vcap register for capture.
 * the buffer is marked active
 */
static void vcap_schedule_next_buffer(struct common_obj *common,int dev_id)
{
    u64 addr = 0;

    spin_lock(&common->irqlock);
    common->next_frm = list_entry(common->dma_queue.next,
            struct vcap_cap_buffer, list);

    /* Remove that buffer from the buffer queue */
    list_del(&common->next_frm->list);
    spin_unlock(&common->irqlock);

#ifdef CONFIG_DIRECT_DISPLAY
    addr = 0x220000000;
#else
    addr = vb2_dma_contig_plane_dma_addr(&common->next_frm->vb, 0);
#endif
    /* Set top and bottom field addresses in VCAP registers */
    common->set_addr(addr,0x230000000,dev_id);

}

/* ----------------------------------------------------------------------- */

/**
 * vcap_channel_isr : ISR handler for vcap capture
 * @irq: irq number
 * @dev_id: dev_id ptr
 *
 * It changes status of the captured buffer, takes next buffer from the queue
 * and sets its address in VCAP registers
 */
static irqreturn_t vcap_channel_isr(int irq, void *dev_id)
{
    struct vcap_device *dev = &vcap_obj;
    struct common_obj *common;
    struct channel_obj *ch;
    int channel_id = 0;

    channel_id = *(int *)(dev_id);
#ifdef DEBUG
    printk("irq channel_id is %d \n",channel_id);
    /*only for debug ,must del when working */
    gpt_intr(channel_id);
    get_dma_line_record(channel_id);
#endif
    if (!vcap_intr_status(channel_id))
        return IRQ_HANDLED;
    ch = dev->dev[channel_id];

    common = &ch->common;

    /* Progressive mode */
    if (!list_empty(&common->dma_queue)) {

        if (!channel_first_int[0][channel_id])
            vcap_process_buffer_complete(common);
        channel_first_int[0][channel_id] = 0;

        vcap_schedule_next_buffer(common,channel_id);

        channel_first_int[0][channel_id] = 0;

    }
    return IRQ_HANDLED;
}

/* ----------------------------------------------------------------------- */

/**
 * vcap_update_std_info() - update standard related info
 * @ch: ptr to channel object
 *
 * For a given standard selected by application, update values
 * in the device data structures
 */
static int vcap_update_std_info(struct channel_obj *ch)
{
    struct common_obj *common = &ch->common;
    struct vcap_params *vcapparams = &ch->vcapparams;
    vcap_channel_config_params *config;
    vcap_channel_config_params *std_info = &vcapparams->std_info;
    struct video_obj *vid_ch = &ch->video;
    int index;

    vcap_dbg(2, debug, "vcap_update_std_info\n");

#if 0
    config = &vcap_ch_params[ch->channel_id][index];
    memcpy(std_info, config, sizeof(*config));

    /* standard not found */
    if (index == vcap_ch_params_count)
        return -EINVAL;

    common->fmt.fmt.pix.width = std_info->width;
    common->width = std_info->width;
    common->fmt.fmt.pix.height = std_info->height;
    common->height = std_info->height;
    common->fmt.fmt.pix.sizeimage = common->height * common->width * 2;
    common->fmt.fmt.pix.bytesperline = std_info->width;

    if (vid_ch->stdid)
        common->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
    else
        common->fmt.fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

    if (ch->vcapparams.std_info.frm_fmt)
        common->fmt.fmt.pix.field = V4L2_FIELD_NONE;
    else
        common->fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    common->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

#endif
    return 0;
}

/* ----------------------------------------------------------------------- */

/**
 * vcap_get_default_field() - Get default field type based on interface
 * @vcap_params - ptr to vcap params
 */
static inline enum v4l2_field vcap_get_default_field(
        struct vcap_interface *iface)
{
    return (iface->if_type == VCAP_IF_RAW_BAYER) ? V4L2_FIELD_NONE :
        V4L2_FIELD_INTERLACED;
}

/* ----------------------------------------------------------------------- */
/**
 * vcap_enum_fmt_vid_cap() - ENUM_FMT handler
 * @file: file ptr
 * @priv: file handle
 * @index: input index
 */
static int vcap_enum_fmt_vid_cap(struct file *file, void  *priv,
        struct v4l2_fmtdesc *fmt)
{
    struct video_device *vdev = video_devdata(file);
    struct channel_obj *ch = video_get_drvdata(vdev);

#ifdef VIDEO_DEBUG
    printk("\n entry %s %d \n",__FUNCTION__,__LINE__);
#endif

    if (fmt->index != 0) {
        vcap_dbg(1, debug, "Invalid format index\n");
        return -EINVAL;
    }

    /* Fill in the information about format */
    if (ch->vcapparams.iface.if_type == VCAP_IF_RAW_BAYER) {
        fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        strcpy(fmt->description, "Raw Mode -Bayer Pattern GrRBGb");
        fmt->pixelformat = V4L2_PIX_FMT_SBGGR8;
    } else {
        fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        strcpy(fmt->description, "YCbCr4:2:2 YC Planar");
        fmt->pixelformat = V4L2_PIX_FMT_YUV422P;
    }

    return 0;
}
/* ----------------------------------------------------------------------- */

/**
 * vcap_g_fmt_vid_cap() - Set INPUT handler
 * @file: file ptr
 * @priv: file handle
 * @fmt: ptr to v4l2 format structure
 */
static int vcap_g_fmt_vid_cap(struct file *file, void *priv,
        struct v4l2_format *fmt)
{
    struct video_device *vdev = video_devdata(file);
    struct channel_obj *ch = video_get_drvdata(vdev);
    struct common_obj *common = &ch->common;

#ifdef VIDEO_DEBUG
    printk("entry %s %d \n",__FUNCTION__,__LINE__);
#endif
    /* Check the validity of the buffer type */
    if (common->fmt.type != fmt->type)
        return -EINVAL;

    //	common->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* Fill in the information about format */
    *fmt = common->fmt;
    printk("entry %s %d %d %d %d %d %d \n",__FUNCTION__,__LINE__,fmt->type,common->fmt.type,fmt->fmt.pix.width,fmt->fmt.pix.height,fmt->fmt.pix.pixelformat);
    return 0;
}

/* ----------------------------------------------------------------------- */
/**
 * vcap_try_fmt_vid_cap() - TRY_FMT handler
 * @file: file ptr
 * @priv: file handle
 * @fmt: ptr to v4l2 format structure
 */
static int vcap_try_fmt_vid_cap(struct file *file, void *priv,
        struct v4l2_format *fmt)
{
    struct video_device *vdev = video_devdata(file);
    struct channel_obj *ch = video_get_drvdata(vdev);
    struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

#ifdef VIDEO_DEBUG
    printk("entry %s %d ,ch_id is %d , %d,%d,0x%x \n",__FUNCTION__,__LINE__,ch->channel_id,pixfmt->width,pixfmt->height,pixfmt->sizeimage);
#endif
    memcpy(&(ch->common.fmt),fmt,sizeof(struct v4l2_format));

    vcap_update_std_info(ch);

    return 0;
}

/**
 * vcap_s_fmt_vid_cap() - Set FMT handler
 * @file: file ptr
 * @priv: file handle
 * @fmt: ptr to v4l2 format structure
 */
static int vcap_s_fmt_vid_cap(struct file *file, void *priv,
        struct v4l2_format *fmt)
{
    struct video_device *vdev = video_devdata(file);
    struct channel_obj *ch = video_get_drvdata(vdev);
    struct common_obj *common = &ch->common;
    int ret;

#ifdef VIDEO_DEBUG
    printk("entry %s %d,ch_id is %d ,type is %d,%d,%d,0x%x \n",__FUNCTION__,__LINE__,ch->channel_id,fmt->type,fmt->fmt.pix.width,fmt->fmt.pix.height,fmt->fmt.pix.sizeimage);
#endif

    if (vb2_is_busy(&common->buffer_queue))
        return -EBUSY;
#if 1
    ret = vcap_try_fmt_vid_cap(file, priv, fmt);
    if (ret)
        return ret;
#endif
    /* store the format in the channel object */
    common->fmt = *fmt;


    return 0;
}
/* ----------------------------------------------------------------------- */

/**
 * vcap_querycap() - QUERYCAP handler
 * @file: file ptr
 * @priv: file handle
 * @cap: ptr to v4l2_capability structure
 */
static int vcap_querycap(struct file *file, void  *priv,
        struct v4l2_capability *cap)
{

#ifdef VIDEO_DEBUG
    printk("entry %s %d \n",__FUNCTION__,__LINE__);
#endif
    cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

    strlcpy(cap->driver, VCAP_DRIVER_NAME, sizeof(cap->driver));

    strlcpy(cap->bus_info, "platform:", sizeof(cap->bus_info));
    return 0;
}
/* ----------------------------------------------------------------------- */
static int vcap_g_crop_vid_cap(struct file *file, void *fh,struct v4l2_crop *a)
{

    return 0;
}
/* ----------------------------------------------------------------------- */
static	int vcap_s_crop_vid_cap(struct file *file, void *fh,const struct v4l2_crop *a)
{

    return 0;
}
/* ----------------------------------------------------------------------- */
static int vcap_g_parm_vid_cap(struct file *file, void *fh,struct v4l2_streamparm *a)
{

    return 0;
}
/* ----------------------------------------------------------------------- */
static int vcap_s_parm_vid_cap(struct file *file, void *fh,struct v4l2_streamparm *a)
{
    return 0;
}

/* ----------------------------------------------------------------------- */
static int vcap_enum_framesize(struct file *file, void *fh,struct v4l2_frmsizeenum *fsize)
{
    return 0;
}

/* ----------------------------------------------------------------------- */
static int vcap_enum_frameintervals(struct file *file, void *fh,struct v4l2_frmivalenum *fival)
{
    return 0;
}

/* ----------------------------------------------------------------------- */
static int vcap_g_std (struct file *file, void *fh, v4l2_std_id *norm)
{
    return 0;
}
/* ----------------------------------------------------------------------- */
static int vcap_s_std (struct file *file, void *fh, v4l2_std_id norm)
{
    return 0;
}
/* ----------------------------------------------------------------------- */
/* vcap capture ioctl operations */
static const struct v4l2_ioctl_ops vcap_ioctl_ops = {
    .vidioc_querycap		= vcap_querycap, //
    .vidioc_g_fmt_vid_cap		= vcap_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap		= vcap_s_fmt_vid_cap, //

    .vidioc_enum_fmt_vid_cap	= vcap_enum_fmt_vid_cap,
    .vidioc_g_crop			= vcap_g_crop_vid_cap,
    .vidioc_s_crop			= vcap_s_crop_vid_cap,
    .vidioc_g_parm			= vcap_g_parm_vid_cap,
    .vidioc_s_parm			= vcap_s_parm_vid_cap,
    .vidioc_enum_framesizes		= vcap_enum_framesize,
    .vidioc_enum_frameintervals	= vcap_enum_frameintervals,
    .vidioc_g_std			=vcap_g_std,
    .vidioc_s_std			=vcap_s_std,
    .vidioc_reqbufs			= vb2_ioctl_reqbufs,//
    .vidioc_create_bufs		= vb2_ioctl_create_bufs,
    .vidioc_querybuf		= vb2_ioctl_querybuf,//
    .vidioc_qbuf			= vb2_ioctl_qbuf,//
    .vidioc_dqbuf			= vb2_ioctl_dqbuf,
    .vidioc_expbuf			= vb2_ioctl_expbuf,
    .vidioc_streamon		= vb2_ioctl_streamon,//
    .vidioc_streamoff		= vb2_ioctl_streamoff,//
};

/* vcap file operations */
static struct v4l2_file_operations vcap_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
    .unlocked_ioctl = video_ioctl2,
    .mmap = vb2_fop_mmap,
    .poll = vb2_fop_poll
};

/* ----------------------------------------------------------------------- */

/**
 * initialize_vcap() - Initialize vcap data structures
 *
 * Allocate memory for data structures and initialize them
 */
static int initialize_vcap(void)
{
    int err, i, j;
    int free_channel_objects_index;

    /* Allocate memory for 3 channel objects */
    for (i = 0; i < VCAP_CAPTURE_MAX_DEVICES; i++) {
        vcap_obj.dev[i] =
            kzalloc(sizeof(*vcap_obj.dev[i]), GFP_KERNEL);
        /* If memory allocation fails, return error */
        if (!vcap_obj.dev[i]) {
            free_channel_objects_index = i;
            err = -ENOMEM;
            goto vcap_init_free_channel_objects;
        }
    }
    return 0;

vcap_init_free_channel_objects:
    for (j = 0; j < free_channel_objects_index; j++)
        kfree(vcap_obj.dev[j]);
    return err;
}

static int gpt_extsrc_cfg(void)
{
    uint32_t *extsrc = ioremap(0xf0000100, 0x40);
    if (extsrc == NULL) {
        printk("%s:%d--> get memory/io resource failed\n",__func__, __LINE__);
        return -ENXIO;
    }
    /*reset vcap0/1/2*/
    writel(readl(extsrc) & 0xffffff8f, extsrc);

    writel((readl(extsrc) | (1 << 4) | (1 << 5)| (1 << 6) | (1 << 13)) | 0xffff0000, extsrc);
#ifdef DEBUG
    printk("GPT EXTSRC address(%x) remap to 0x%x\n", 0xf0000100, readl(extsrc));
#endif
    iounmap(extsrc);
    return 0;
}

void init_vcap(int dev_id,vcap_channel_config_params para){

    set_trans_form_type(dev_id);
    //open_rgb(0,dev_id);
    set_hdat_high_stride(para,dev_id);

    //set_time_out(3,dev_id);

    set_intr_mask(GPT_VDI_IRQ_DISABLE,dev_id);
#ifdef VIDEO_DEBUG
    print_vcap0_reg(dev_id);
#endif
}

static int vcap_probe_complete(void)
{
    struct common_obj *common;
    struct video_device *vdev;
    struct channel_obj *ch;
    struct vb2_queue *q;
    int i, j, err, k;
#ifdef DEBUG
    printk("entry %s , %d.............................\n",__FUNCTION__,__LINE__);
#endif
    for (j = 0; j < VCAP_CAPTURE_MAX_DEVICES; j++) {

        ch = vcap_obj.dev[j];
        ch->channel_id = j;
        common = &ch->common;
        spin_lock_init(&common->irqlock);
        mutex_init(&common->lock);

        /* set initial format */
        ch->video.stdid = V4L2_STD_525_60;
        vcap_update_std_info(ch);

        /* Initialize vb2 queue */
        q = &common->buffer_queue;
        q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
        q->drv_priv = ch;
        q->ops = &video_qops;
        q->mem_ops = &vb2_dma_contig_memops;
        q->buf_struct_size = sizeof(struct vcap_cap_buffer);
        q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
        q->min_buffers_needed = 2;
        q->lock = &common->lock;

        err = vb2_queue_init(q);
        if (err) {
            vcap_err("vcap_capture: vb2_queue_init() failed\n");
            goto probe_out;
        }

        common->alloc_ctx = vb2_dma_contig_init_ctx(vcap_dev);
        if (IS_ERR(common->alloc_ctx)) {
            vcap_err("Failed to get the context\n");
            err = PTR_ERR(common->alloc_ctx);
            goto probe_out;
        }
        INIT_LIST_HEAD(&common->dma_queue);

        /* Initialize the video_device structure */
        vdev = ch->video_dev;
        strlcpy(vdev->name, VCAP_DRIVER_NAME, sizeof(vdev->name));
        vdev->release = video_device_release;
        vdev->fops = &vcap_fops;
        vdev->ioctl_ops = &vcap_ioctl_ops;
        vdev->v4l2_dev = &vcap_obj.v4l2_dev;
        vdev->vfl_dir = VFL_DIR_RX;
        vdev->queue = q;
        vdev->lock = &common->lock;
        video_set_drvdata(ch->video_dev, ch);
        err = video_register_device(vdev,
                VFL_TYPE_GRABBER, (j ? 1 : 0));
        if (err)
            goto probe_out;
    }

    v4l2_info(&vcap_obj.v4l2_dev, "VCAP capture driver initialized\n");
    return 0;

probe_out:
    for (k = 0; k < j; k++) {
        /* Get the pointer to the channel object */
        ch = vcap_obj.dev[k];
        common = &ch->common;
        vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
        /* Unregister video device */
        video_unregister_device(ch->video_dev);
    }
    kfree(vcap_obj.sd);
    for (i = 0; i < VCAP_CAPTURE_MAX_DEVICES; i++) {
        ch = vcap_obj.dev[i];
        /* Note: does nothing if ch->video_dev == NULL */
        video_device_release(ch->video_dev);
    }
    v4l2_device_unregister(&vcap_obj.v4l2_dev);

    return err;
}

static int vcap_prase_dts(struct device_node *np){
    int i,ret;	
    u32 data[4];

    if(np == NULL)
    {
        printk("node is null %s\n",__func__);
        return -EINVAL;
    }

    ret = of_property_read_u32_array(np,"sync_ctrl",data,3);
    if (ret){
        printk("value is Error %s\n",__func__);
        return ret;
    }else{
        for(i=0; i<3; i++){
            set_sync_reg(data[i],i);
        }
    }
    ret = of_property_read_u32_array(np,"vin_ctrl",data,3);
    if (ret){
        printk("value is Error %s\n",__func__);
        return ret;
    }else{
        for(i=0; i<3; i++){
            set_vin_reg(data[i],i);
        }
    }

    return 0;
}

/* ----------------------------------------------------------------------- */
static int get_sd(struct device_node *np){
    int i=0;
    struct i2c_client*client;
    struct device_node *epn = NULL, *ren;

    if(np == NULL)
    {
        printk("node is null %s\n",__func__);
        return -EINVAL;
    }
    for (i = 0; ; i++) {
        epn = of_graph_get_next_endpoint(np, epn);
        if (!epn){
            printk("can not find endpointi %s\n",__func__);
            return -EINVAL;
        }

        ren = of_graph_get_remote_port(epn);
        if (!ren) {
            printk("no remote for %s\n",of_node_full_name(epn));
            continue;
        }

        client = of_find_i2c_device_by_node(ren->parent);
        if(client == NULL){
            printk("[zaf] client is NULL\n");
            return -EINVAL;
        }
        sd[i] = (struct v4l2_subdev *)i2c_get_clientdata(client);
        if(sd[i] != NULL){
            strcpy(sd[i]->name,epn->name);
            v4l_info(client, "vcap subdevice chip found @ 0x%02x (%s)\n",
                    client->addr << 1, client->adapter->name);
        }else{
            printk("node is null %s\n",__func__);
            return -EINVAL;
        }
    }
    return 0;
}

static int gpt_vcap_parse_dt(struct platform_device *pdev)
{
    int ret,pinctrl_num,i;
    struct pinctrl *pinctrl;
    const char **pctrl_state;
    struct pinctrl_state **states;
    struct device_node *np = pdev->dev.of_node;

    if(np == NULL)
    {
        printk("node is null %s\n",__func__);
        return -EINVAL;
    }

    pinctrl = devm_pinctrl_get(&pdev->dev);
    if(pinctrl == NULL)
    {
        printk("pinctrl is null %s\n",__func__);
        return -EINVAL;
    }

    pinctrl_num = of_property_count_strings(np, "pinctrl-names");
    pctrl_state = devm_kzalloc(&pdev->dev, sizeof(*pctrl_state) * pinctrl_num, GFP_KERNEL);
    if (!pctrl_state) {
        dev_err(&pdev->dev, "Cannot allocate pctrl_state\n");
        return -ENOMEM;
    }

    states = devm_kzalloc(&pdev->dev, sizeof(*states) * pinctrl_num, GFP_KERNEL);
    if (!states) {
        dev_err(&pdev->dev, "Cannot allocate states\n");
        return -ENOMEM;
    }

    for(i = 0; i < pinctrl_num; i++){
        ret = of_property_read_string_index(np, "pinctrl-names", i,
                &pctrl_state[i]);
        if (ret < 0) {
            dev_err(&pdev->dev, "Cannot parse pinctrl-names %d\n", ret);
            return ret;
        }
    }

    for(i = 0; i < pinctrl_num; i++){
        states[i] = pinctrl_lookup_state(pinctrl, pctrl_state[i]);
        if (IS_ERR(states[i])) {
            dev_err(&pdev->dev, "Lookup state failed\n");
            return IS_ERR(states[i]);
        }
    }

    for(i = 0; i < pinctrl_num; i++){
        ret = pinctrl_select_state(pinctrl, states[i]);
        if (ret < 0) {
            dev_err(&pdev->dev, "Select state failed\n");
            return ret;
        }
    }

    return 0;
}

/* ----------------------------------------------------------------------- */

/**
 * vcap_probe : This function probes the vcap capture driver
 * @pdev: platform device pointer
 *
 * This creates device entries by register itself to the V4L2 driver and
 * initializes fields of each channel objects
 */

static __init int vcap_probe(struct platform_device *pdev)
{
    int i, j,k = 0,err;
    int res_idx = 0;
    struct channel_obj *ch;
    struct video_device *vfd;
    struct resource *res,*res1,*res2;
    int subdev_count;
    struct device_node*node = pdev->dev.of_node;

#ifdef VIDEO_DEBUG
    printk("entry %s , %d.............................\n",__FUNCTION__,__LINE__);
#endif

    gpt_vcap_parse_dt(pdev);

    gpt_extsrc_cfg();
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    res2 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
    vcap_base0 = ioremap(res->start,0x1000);
    vcap_base1 = ioremap(res1->start,0x1000);
    vcap_base2 = ioremap(res2->start,0x1000);


#ifdef VIDEO_DEBUG
    printk("baseo 0x%x, base1 0x%x,base2 0x%x vcap0 is %s \n", (u32)res->start,(u32)res1->start,(u32)res2->start,vcap0);
#endif
    vcap_dev = &pdev->dev;

    /*vcap_obj.dev alloc memory*/
    err = initialize_vcap();
    if (err) {
        v4l2_err(vcap_dev->driver, "Error initializing vcap\n");
        return err;
    }

    err = v4l2_device_register(vcap_dev, &vcap_obj.v4l2_dev);
    if (err) {
        v4l2_err(vcap_dev->driver, "Error registering v4l2 device\n");
        return err;
    }


if(strcmp_zxp(vcap,"chaokong")==0){
    printk("entry %s , %d config chaokong \n",__FUNCTION__,__LINE__);
    set_sync_vin_reg(0x2021041,0x122004,VCAP_CHANNEL_0);
    init_vcap(VCAP_CHANNEL_0,chaokong);
    set_sync_vin_reg(0x2021041,0x122004,VCAP_CHANNEL_1);
    init_vcap(VCAP_CHANNEL_1,chaokong);
}

if(strcmp_zxp(vcap,"HI3519")==0){
    printk("entry %s , %d config HI3519 \n",__FUNCTION__,__LINE__);
    set_sync_vin_reg(0x2021040,0x122004,VCAP_CHANNEL_0);
    init_vcap(VCAP_CHANNEL_0,hi3519);
}

if(strcmp_zxp(vcap,"SH_ISP")==0){
    printk("entry %s , %d config SH_ISP\n",__FUNCTION__,__LINE__);
    set_sync_vin_reg(0x2021241,0x122004,VCAP_CHANNEL_0);
    init_vcap(VCAP_CHANNEL_0,sh_isp);
}

if(strcmp_zxp(vcap,"evk")==0){
    printk("entry %s , %d config evk board\n",__FUNCTION__,__LINE__);
    set_sync_vin_reg(0x2021040,0x122004,VCAP_CHANNEL_0);
    init_vcap(VCAP_CHANNEL_0,adv7612);
    set_sync_vin_reg(0x2021041,0x122004,VCAP_CHANNEL_1);
    init_vcap(VCAP_CHANNEL_1,nt99141);
} 
    
    while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, res_idx))) {
        vcap_obj.dev[res_idx]->channel_id = res_idx;
#ifdef VIDEO_DEBUG
        printk("irq%d is %d,ch_id is %d\n",res_idx,res->start,vcap_obj.dev[res_idx]->channel_id);
#endif
        err = devm_request_irq(&pdev->dev,res->start, vcap_channel_isr,
                IRQF_SHARED, VCAP_DRIVER_NAME,
                (void *)(&(vcap_obj.dev[res_idx]->channel_id)));

        if (err) {
            err = -EINVAL;
            goto vcap_unregister;
        }
        res_idx++;
    }

    for (i = 0; i < VCAP_CAPTURE_MAX_DEVICES; i++) {
        /* Get the pointer to the channel object */
        ch = vcap_obj.dev[i];

        /* Allocate memory for video device */
        vfd = video_device_alloc();
        if (NULL == vfd) {
            for (j = 0; j < i; j++) {
                ch = vcap_obj.dev[j];
                video_device_release(ch->video_dev);
            }
            err = -ENOMEM;
            goto vcap_unregister;
        }

        /* Set video_dev to the video device */
        ch->video_dev = vfd;
    }

    /*subdev alloc memory*/
    vcap_obj.sd = kzalloc(sizeof(struct v4l2_subdev *) * VCAP_CAPTURE_MAX_DEVICES,GFP_KERNEL);
    *sd = kzalloc(sizeof(struct v4l2_subdev *) * MAX_SD_NUM,GFP_KERNEL);
    if (vcap_obj.sd == NULL || sd == NULL) {
        vcap_err("unable to allocate memory for subdevice pointers\n");
        err = -ENOMEM;
        goto vcap_sd_error;
    }
    /*make v4l2_device and v4l2_subdev register*/
    get_sd_flag =  get_sd(node); 
    if(get_sd_flag>=0){
        subdev_count = MAX_SD_NUM;
        for (i = 0;i<subdev_count; i++) {

            if(sd[i] != NULL){
                if (v4l2_device_register_subdev(&vcap_obj.v4l2_dev, sd[i])==0)
                {
                    err = v4l2_subdev_call(sd[i],core,init,0);
                    if(err>=0){
                        vcap_obj.sd[k] = sd[i];
                        k++;
                        printk("%s find camera name %s \n",__func__,sd[i]->name);
                    }else{
                        v4l2_device_unregister_subdev(sd[i]);
                        printk("%s check camera %s failed \n",__func__,sd[i]->name);
                    }
                }else{
                    v4l2_device_unregister_subdev(sd[i]);
                    printk("v4l2_device_register_subdev failed\n");
                }

            }else{
                printk("get sd is NULL\n");
            }
        }
    }   

    vcap_probe_complete();

    return 0;

vcap_sd_error:
    for (i = 0; i < VCAP_CAPTURE_MAX_DEVICES; i++) {
        ch = vcap_obj.dev[i];
        /* Note: does nothing if ch->video_dev == NULL */
        video_device_release(ch->video_dev);
    }

vcap_unregister:
    v4l2_device_unregister(&vcap_obj.v4l2_dev);
    return err;
}

/**
 * vcap_remove() - driver remove handler
 * @device: ptr to platform device structure
 *
 * The vidoe device is unregistered
 */
static int vcap_remove(struct platform_device *device)
{
    int i;
    struct common_obj *common;
    struct channel_obj *ch;

    v4l2_device_unregister(&vcap_obj.v4l2_dev);

    kfree(vcap_obj.sd);
    /* un-register device */
    for (i = 0; i < VCAP_CAPTURE_MAX_DEVICES; i++) {
        /* Get the pointer to the channel object */
        ch = vcap_obj.dev[i];
        common = &ch->common;
        vb2_dma_contig_cleanup_ctx(common->alloc_ctx);
        /* Unregister video device */
        video_unregister_device(ch->video_dev);
        kfree(vcap_obj.dev[i]);
    }

    return 0;
}

static const struct of_device_id vcap_of_match[] = {
    //	{ .name = VCAP_DRIVER_NAME,},
    { .compatible = "gpt,gpt-vcap", },
    {},
};

MODULE_DEVICE_TABLE(of, vcap_of_match);
static struct platform_driver v4l2_driver_video = {
    .probe = vcap_probe,
    .remove = vcap_remove,
    .driver	= {
        .name	= VCAP_DRIVER_NAME,
        .owner	= THIS_MODULE,
        .of_match_table = of_match_ptr(vcap_of_match),
    },

};

static int __init vcap_init(void)
{
    printk("start vcap_init \n");

    return platform_driver_register(&v4l2_driver_video);
}

static void __exit vcap_exit(void)
{
    printk("vcap_init cleanup! \n");

    platform_driver_unregister(&v4l2_driver_video);

}

module_init(vcap_init);
module_exit(vcap_exit);

MODULE_AUTHOR("xinpozhou");
MODULE_DESCRIPTION("GPT VCAP Capture driver");
MODULE_LICENSE("GPL");
