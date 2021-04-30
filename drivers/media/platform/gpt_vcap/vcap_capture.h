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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef VCAP_CAPTURE_H
#define VCAP_CAPTURE_H

/* Header files */
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>

/* Macros */
#define VCAP_CAPTURE_VERSION		"0.0.2"

#define VCAP_CAPTURE_MAX_DEVICES        3
#define VCAP_VIDEO_INDEX		0
#define VCAP_NUMBER_OF_OBJECTS		3
#define I2C_NAME_SIZE                   20

/* Enumerated data type to give id to each device per channel */
enum vcap_channel_id {
	VCAP_CHANNEL0_VIDEO = 0,
	VCAP_CHANNEL1_VIDEO,
	VCAP_CHANNEL2_VIDEO,
};

struct video_obj {
	enum v4l2_field buf_field;
	/* Currently selected or default standard */
	v4l2_std_id stdid;
	struct v4l2_dv_timings dv_timings;
};

struct vcap_cap_buffer {
	struct vb2_buffer vb;
	struct list_head list;
};
/* structure for vcap parameters */
struct vcap_video_params {
	__u8 storage_mode;	/* Indicates field or frame mode */
	v4l2_std_id stdid;
};


struct vcap_params {
	struct vcap_interface iface;
	struct vcap_video_params video_params;
	vcap_channel_config_params std_info;
};

struct common_obj {
	/* Pointer pointing to current v4l2_buffer */
	struct vcap_cap_buffer *cur_frm;
	/* Pointer pointing to current v4l2_buffer */
	struct vcap_cap_buffer *next_frm;
	/* Used to store pixel format */
	struct v4l2_format fmt;
	/* Buffer queue used in video-buf */
	struct vb2_queue buffer_queue;
	/* allocator-specific contexts for each plane */
	struct vb2_alloc_ctx *alloc_ctx;

	/* Queue of filled frames ,is a list*/
	struct list_head dma_queue;

	/* Used in video-buf */
	spinlock_t irqlock;
	/* lock used to access this structure */
	struct mutex lock;
	/* Function pointer to set the addresses */
	void (*set_addr) (u64,u64,int);
	/* Indicates width of the image data */
	u32 width;
	/* Indicates height of the image data */
	u32 height;
};

struct channel_obj {
	/* Identifies video device for this channel */
	struct video_device *video_dev;
	/* Identifies channel */
	enum vcap_channel_id channel_id;
	/* vcap configuration params */
	struct vcap_params vcapparams;
	/* common object array */
	struct common_obj common;
	/* video object */
	struct video_obj video;
};

struct vcap_device {
	struct v4l2_device v4l2_dev;// root or parent
	struct v4l2_subdev **sd; //camera or hdmi
	/*one channel_obj is a vcap*/
	struct channel_obj *dev[VCAP_CAPTURE_NUM_CHANNELS];
};

#endif				/* VCAP_CAPTURE_H */
