#ifndef GPT_rn6864m_H_
#define GPT_rn6864m_H_
/* rn6864m registers */

/*
 * the sensor's autoexposure is buggy when setting total_height low.
 * It tries to expose longer than 1 frame period without taking care of it
 * and this leads to weird output. So we set 1000 lines as minimum.
 */
#define RN6864M_PAGE 0x0
#define RN6864M_PAGE_MAX 0X1

#define REG_CHIP_ID_LOW 0xFD
#define REG_CHIP_ID_HIGH 0xFE

struct regval_list {
	u16 reg_num;
	u8 value;
};
/* ----------------------------------------------------------------------- */
static struct regval_list RN675x_init_cfg[] = {
// 720P@25 BT601
// Slave address is 0x58
// Register, data

// if clock source(Xin) of RN6752 is 26MHz, please add these procedures marked first
//0xD2, 0x85, // disable auto clock detect
//0xD6, 0x37, // 27MHz default
//0xD8, 0x18, // switch to 26MHz clock
//delay(100), // delay 100ms

	{ 0x81, 0x01 }, // turn on video decoder
	{ 0xA3, 0x04 },
	{ 0xDF, 0xFE }, // enable HD format

	{ 0x88, 0x40 }, // disable SCLK0B out
	{ 0xF6, 0x40 }, // disable SCLK3A out
	 // ch0
	{ 0xFF, 0x00 }, // switch to ch0 (default; optional)
	{ 0x00, 0x20 }, // internal use*
	{ 0x06, 0x08 }, // internal use*
	{ 0x07, 0x63 }, // HD format
	{ 0x2A, 0x01 }, // filter control
	{ 0x3A, 0x00 }, // No Insert Channel ID in SAV/EAV code
	{ 0x3F, 0x10 }, // channel ID
	{ 0x4C, 0x37 }, // equalizer
	{ 0x4F, 0x03 }, // sync control
	{ 0x50, 0x02 }, // 720p resolution
	{ 0x56, 0x05 }, // 72M mode and BT601 mode
	{ 0x5F, 0x40 }, // blank level
	{ 0x63, 0xF5 }, // filter control
	{ 0x59, 0x00 }, // extended register access
	{ 0x5A, 0x42 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x59, 0x33 }, // extended register access
	{ 0x5A, 0x23 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x51, 0xE1 }, // scale factor1
	{ 0x52, 0x88 }, // scale factor2
	{ 0x53, 0x12 }, // scale factor3
	{ 0x5B, 0x07 }, // H-scaling control
	{ 0x5E, 0x08 }, // enable H-scaling control
	{ 0x6A, 0x82 }, // H-scaling control
	{ 0x28, 0x92 }, // cropping
	{ 0x03, 0x80 }, // saturation
	{ 0x04, 0x80 }, // hue
	{ 0x05, 0x00 }, // sharpness
	{ 0x57, 0x23 }, // black/white stretch
	{ 0x68, 0x32 }, // coring
	{ 0x3A, 0x04 },
	{ 0x3E, 0x32 },
	{ 0x40, 0x04 },
	{ 0x46, 0x23 },
	{ 0x37, 0x33 },
	{ 0x61, 0x6C },

	{ 0x8E, 0x00 }, // single channel output for VP
	{ 0x8F, 0x80 }, // 720p mode for VP
	{ 0x8D, 0x31 }, // enable VP out
	{ 0x89, 0x09 }, // select 72MHz for SCLK
	{ 0x88, 0x41 }, // enable SCLK out

	{ 0x96, 0x00 }, // select AVID & VBLK as status indicator
	{ 0x97, 0x0B }, // enable status indicator out on AVID,VBLK & VSYNC 
	{ 0x98, 0x00 }, // video timing pin status
	{ 0x9A, 0x40 }, // select AVID & VBLK as status indicator 
	{ 0x9B, 0xE1 }, // enable status indicator out on HSYNC
	{ 0x9C, 0x00 }, // video timing pin status

//	{ 0xff, 0x00 }, //color bar 
//	{ 0x00, 0x80 }, //color bar
	{ 0xffff, 0xff},
	};

/* ----------------------------------------------------------------------- */
static struct regval_list RN675x_init_cfg_2chin_2chout[] = {
// 720P@25 BT601
// Slave address is 0x58
// Register, data

// if clock source(Xin) of RN6752 is 26MHz, please add these procedures marked first
//0xD2, 0x85, // disable auto clock detect
//0xD6, 0x37, // 27MHz default
//0xD8, 0x18, // switch to 26MHz clock
//delay(100), // delay 100ms

	{ 0x81, 0x0F }, // turn on video decoder
	{ 0xA3, 0x04 },
	{ 0xA7, 0x04 },
	{ 0xDF, 0xF0 }, // enable HD format
	{ 0xF0, 0xC0 },

	{ 0x88, 0x40 }, // disable SCLK0B out
	{ 0xF6, 0x40 }, // disable SCLK3A out
	 // ch0
	{ 0xFF, 0x00 }, // switch to ch0 (default; optional)
	{ 0x00, 0x20 }, // internal use*
	{ 0x06, 0x08 }, // internal use*
	{ 0x07, 0x63 }, // HD format
	{ 0x2A, 0x01 }, // filter control
	{ 0x3A, 0x00 }, // No Insert Channel ID in SAV/EAV code
	{ 0x3F, 0x10 }, // channel ID
	{ 0x4C, 0x37 }, // equalizer
	{ 0x4F, 0x03 }, // sync control
	{ 0x50, 0x02 }, // 720p resolution
	{ 0x56, 0x06 }, // 72M mode and BT601 mode
	{ 0x5F, 0x40 }, // blank level
	{ 0x63, 0xF5 }, // filter control
	{ 0x59, 0x00 }, // extended register access
	{ 0x5A, 0x44 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x59, 0x33 }, // extended register access
	{ 0x5A, 0x23 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x51, 0x4E }, // scale factor1
	{ 0x52, 0x87 }, // scale factor2
	{ 0x53, 0x12 }, // scale factor3
	{ 0x5B, 0x07 }, // H-scaling control
	{ 0x5E, 0x08 }, // enable H-scaling control
	{ 0x6A, 0x82 }, // H-scaling control
	{ 0x28, 0x92 }, // cropping
	{ 0x03, 0x80 }, // saturation
	{ 0x04, 0x80 }, // hue
	{ 0x05, 0x00 }, // sharpness
	{ 0x57, 0x23 }, // black/white stretch
	{ 0x68, 0x32 }, // coring
	{ 0x3A, 0x04 },
	{ 0x3E, 0x32 },
	{ 0x40, 0x04 },
	{ 0x46, 0x23 },
	{ 0x37, 0x33 },
	{ 0x61, 0x6C },
	 // ch1
	{ 0xFF, 0x01 }, // switch to ch0 (default; optional)
	{ 0x00, 0x20 }, // internal use*
	{ 0x06, 0x08 }, // internal use*
	{ 0x07, 0x63 }, // HD format
	{ 0x2A, 0x01 }, // filter control
	{ 0x3A, 0x00 }, // No Insert Channel ID in SAV/EAV code
	{ 0x3F, 0x10 }, // channel ID
	{ 0x4C, 0x37 }, // equalizer
	{ 0x4F, 0x03 }, // sync control
	{ 0x50, 0x02 }, // 720p resolution
	{ 0x56, 0x06 }, // 72M mode and BT601 mode
	{ 0x5F, 0x40 }, // blank level
	{ 0x63, 0xF5 }, // filter control
	{ 0x59, 0x00 }, // extended register access
	{ 0x5A, 0x44 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x59, 0x33 }, // extended register access
	{ 0x5A, 0x23 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x51, 0x4E }, // scale factor1
	{ 0x52, 0x87 }, // scale factor2
	{ 0x53, 0x12 }, // scale factor3
	{ 0x5B, 0x07 }, // H-scaling control
	{ 0x5E, 0x08 }, // enable H-scaling control
	{ 0x6A, 0x82 }, // H-scaling control
	{ 0x28, 0x92 }, // cropping
	{ 0x03, 0x80 }, // saturation
	{ 0x04, 0x80 }, // hue
	{ 0x05, 0x00 }, // sharpness
	{ 0x57, 0x23 }, // black/white stretch
	{ 0x68, 0x32 }, // coring
	{ 0x3A, 0x04 },
	{ 0x3E, 0x32 },
	{ 0x40, 0x04 },
	{ 0x46, 0x23 },
	{ 0x37, 0x33 },
	{ 0x61, 0x6C },
 // ch2
	{ 0xFF, 0x02 }, // switch to ch0 (default; optional)
	{ 0x00, 0x20 }, // internal use*
	{ 0x06, 0x08 }, // internal use*
	{ 0x07, 0x63 }, // HD format
	{ 0x2A, 0x01 }, // filter control
	{ 0x3A, 0x00 }, // No Insert Channel ID in SAV/EAV code
	{ 0x3F, 0x10 }, // channel ID
	{ 0x4C, 0x37 }, // equalizer
	{ 0x4F, 0x03 }, // sync control
	{ 0x50, 0x02 }, // 720p resolution
	{ 0x56, 0x06 }, // 72M mode and BT601 mode
	{ 0x5F, 0x40 }, // blank level
	{ 0x63, 0xF5 }, // filter control
	{ 0x59, 0x00 }, // extended register access
	{ 0x5A, 0x44 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x59, 0x33 }, // extended register access
	{ 0x5A, 0x23 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x51, 0x4E }, // scale factor1
	{ 0x52, 0x87 }, // scale factor2
	{ 0x53, 0x12 }, // scale factor3
	{ 0x5B, 0x07 }, // H-scaling control
	{ 0x5E, 0x08 }, // enable H-scaling control
	{ 0x6A, 0x82 }, // H-scaling control
	{ 0x28, 0x92 }, // cropping
	{ 0x03, 0x80 }, // saturation
	{ 0x04, 0x80 }, // hue
	{ 0x05, 0x00 }, // sharpness
	{ 0x57, 0x23 }, // black/white stretch
	{ 0x68, 0x32 }, // coring
	{ 0x3A, 0x04 },
	{ 0x3E, 0x32 },
	{ 0x40, 0x04 },
	{ 0x46, 0x23 },
	{ 0x37, 0x33 },
	{ 0x61, 0x6C },
 // ch3
	{ 0xFF, 0x03 }, // switch to ch0 (default; optional)
	{ 0x00, 0x20 }, // internal use*
	{ 0x06, 0x08 }, // internal use*
	{ 0x07, 0x63 }, // HD format
	{ 0x2A, 0x01 }, // filter control
	{ 0x3A, 0x00 }, // No Insert Channel ID in SAV/EAV code
	{ 0x3F, 0x10 }, // channel ID
	{ 0x4C, 0x37 }, // equalizer
	{ 0x4F, 0x03 }, // sync control
	{ 0x50, 0x02 }, // 720p resolution
	{ 0x56, 0x06 }, // 72M mode and BT601 mode
	{ 0x5F, 0x40 }, // blank level
	{ 0x63, 0xF5 }, // filter control
	{ 0x59, 0x00 }, // extended register access
	{ 0x5A, 0x44 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x59, 0x33 }, // extended register access
	{ 0x5A, 0x23 }, // data for extended register
	{ 0x58, 0x01 }, // enable extended register write
	{ 0x51, 0x4E }, // scale factor1
	{ 0x52, 0x87 }, // scale factor2
	{ 0x53, 0x12 }, // scale factor3
	{ 0x5B, 0x07 }, // H-scaling control
	{ 0x5E, 0x08 }, // enable H-scaling control
	{ 0x6A, 0x82 }, // H-scaling control
	{ 0x28, 0x92 }, // cropping
	{ 0x03, 0x80 }, // saturation
	{ 0x04, 0x80 }, // hue
	{ 0x05, 0x00 }, // sharpness
	{ 0x57, 0x23 }, // black/white stretch
	{ 0x68, 0x32 }, // coring
	{ 0x3A, 0x04 },
	{ 0x3E, 0x32 },
	{ 0x40, 0x04 },
	{ 0x46, 0x23 },
	{ 0x37, 0x33 },
	{ 0x61, 0x6C },

	{ 0x8E, 0x08 }, // single channel output for VP
	{ 0x8F, 0x80 }, // 720p mode for VP
	{ 0x8D, 0x31 }, // enable VP out
    { 0x89, 0x0A }, // select 72MHz for SCLK
	{ 0x88, 0x41 }, // enable SCLK out

    { 0x94, 0x0C }, // single channel output for VP
	{ 0x95, 0x80 }, // 720p mode for VP
	{ 0x93, 0x31 }, // enable VP out
	{ 0xF7, 0x0A }, // select 72MHz for SCLK
	{ 0xF6, 0x41 }, // enable SCLK out

	{ 0x96, 0x00 }, // select AVID & VBLK as status indicator
	{ 0x97, 0x0B }, // enable status indicator out on AVID,VBLK & VSYNC 
	{ 0x98, 0x00 }, // video timing pin status
	{ 0x9A, 0x80 }, // select AVID & VBLK as status indicator 
	{ 0x9B, 0xE6 }, // enable status indicator out on HSYNC
	{ 0x9C, 0x00 }, // video timing pin status

//	{ 0xff, 0x00 }, //color bar 
//	{ 0x00, 0x80 }, //color bar
	{ 0xffff, 0xff},
	};

static void RN675xM_Pre_initial(struct v4l2_subdev *sd);
#endif /*GPT_rn6864m_H_*/
