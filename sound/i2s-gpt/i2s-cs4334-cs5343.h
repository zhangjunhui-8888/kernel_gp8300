#ifndef __GPT_I2S_CS4334_CS5343_H__
#define __GPT_I2S_CS4334_CS5343_H__

#include <linux/types.h>

/* IOCTL commands */
#define I2S_IOC_MAGIC           'x'

#define I2S_IOC_RD_MODE					       _IOR(I2S_IOC_MAGIC, 1, __u8)
#define I2S_IOC_RD_ALIGN_MODE			       _IOR(I2S_IOC_MAGIC, 2, __u8)
#define I2S_IOC_RD_SAMPLE_SIZE			       _IOR(I2S_IOC_MAGIC, 3, __u16)
#define I2S_IOC_RD_FRAME_SIZE	               _IOR(I2S_IOC_MAGIC, 4, __u16)
#define I2S_IOC_RD_MAX_BUF_SIZE                _IOR(I2S_IOC_MAGIC, 5, __u32)
#define I2S_IOC_RD_MAX_BUF_NUM                 _IOR(I2S_IOC_MAGIC, 6, __u8)
#define I2S_IOC_RD_SCLK_FREQ                   _IOR(I2S_IOC_MAGIC, 7, __u32)

#define I2S_IOC_WR_MODE					       _IOW(I2S_IOC_MAGIC, 1, __u8)
#define I2S_IOC_WR_ALIGN_MODE			       _IOW(I2S_IOC_MAGIC, 2, __u8)
#define I2S_IOC_WR_SAMPLE_SIZE			       _IOW(I2S_IOC_MAGIC, 3, __u16)
#define I2S_IOC_WR_FRAME_SIZE			       _IOW(I2S_IOC_MAGIC, 4, __u16)
#define I2S_IOC_WR_SCLK_FREQ                   _IOW(I2S_IOC_MAGIC, 5, __u32)
#define I2S_IOC_WR_PLAYBACK_BUF_SIZE           _IOW(I2S_IOC_MAGIC, 6, __u32)
#define I2S_IOC_WR_CAPTURE_BUF_SIZE            _IOW(I2S_IOC_MAGIC, 7, __u32)
#define I2S_IOC_WR_PLAYBACK_BUF_NUM            _IOW(I2S_IOC_MAGIC, 8, __u8)
#define I2S_IOC_WR_CAPTURE_BUF_NUM             _IOW(I2S_IOC_MAGIC, 9, __u8)
#define I2S_IOC_WR_PLAYBACK_REQUEST_BUF        _IOW(I2S_IOC_MAGIC, 10, __u8)
#define I2S_IOC_WR_CAPTURE_REQUEST_BUF		   _IOW(I2S_IOC_MAGIC, 11, __u8)
#define I2S_IOC_WR_PLAYBACK_RELEASE_BUF        _IOW(I2S_IOC_MAGIC, 12, __u8)
#define I2S_IOC_WR_CAPTURE_RELEASE_BUF		   _IOW(I2S_IOC_MAGIC, 13, __u8)
#define I2S_IOC_WR_I2S_ENABLE			       _IOW(I2S_IOC_MAGIC, 14, __u8)
#define I2S_IOC_WR_PLAYBACK				       _IOW(I2S_IOC_MAGIC, 15, __u8)
#define I2S_IOC_WR_CAPTURE				       _IOW(I2S_IOC_MAGIC, 16, __u8)
#define I2S_IOC_WR_PLAYBACK_QUEUE              _IOW(I2S_IOC_MAGIC, 17, __u8)

#endif
