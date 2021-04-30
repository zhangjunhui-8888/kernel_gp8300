/*
 * Analog Devices ADV7511 HDMI Transmitter Device Driver
 *
 * Copyright 2013 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ADV7511_H
#define ADV7511_H

/* notify events */
#define ADV7511_MONITOR_DETECT 0
#define ADV7511_EDID_DETECT 1

#define MASK_ADV7511_EDID_RDY_INT   0x04
#define MASK_ADV7511_MSEN_INT       0x40
#define MASK_ADV7511_HPD_INT        0x80

#define MASK_ADV7511_HPD_DETECT     0x40
#define MASK_ADV7511_MSEN_DETECT    0x20
#define MASK_ADV7511_EDID_RDY       0x10

#define EDID_MAX_RETRIES (8)
#define EDID_DELAY 250
#define EDID_MAX_SEGM 8

#define ADV7511_MAX_WIDTH 1920
#define ADV7511_MAX_HEIGHT 1200
#define ADV7511_MIN_PIXELCLOCK 20000000
#define ADV7511_MAX_PIXELCLOCK 225000000

#define ADV7511_REG_I2C_FREQ_ID_CFG		0x15
#define ADV7511_REG_VIDEO_INPUT_CFG1		0x16
#define ADV7511_REG_CSC_UPPER(x)		(0x18 + (x) * 2)
#define ADV7511_REG_CSC_LOWER(x)		(0x19 + (x) * 2)
#define ADV7511_REG_SYNC_DECODER(x)		(0x30 + (x))
#define ADV7511_REG_DE_GENERATOR		(0x35 + (x))
#define ADV7511_REG_PIXEL_REPETITION		0x3b
#define ADV7511_REG_VIC_MANUAL			0x3c
#define ADV7511_REG_VIC_SEND			0x3d
#define ADV7511_REG_VIC_DETECTED		0x3e
#define ADV7511_REG_AUX_VIC_DETECTED		0x3f
#define ADV7511_REG_PACKET_ENABLE0		0x40
#define ADV7511_REG_POWER			0x41
#define ADV7511_REG_STATUS			0x42
#define ADV7511_REG_EDID_I2C_ADDR		0x43
#define ADV7511_REG_PACKET_ENABLE1		0x44
#define ADV7511_REG_PACKET_I2C_ADDR		0x45
#define ADV7511_REG_DSD_ENABLE			0x46
#define ADV7511_REG_VIDEO_INPUT_CFG2		0x48
#define ADV7511_REG_INFOFRAME_UPDATE		0x4a
#define ADV7511_REG_GC(x)			(0x4b + (x)) /* 0x4b - 0x51 */
#define ADV7511_REG_AVI_INFOFRAME_VERSION	0x52
#define ADV7511_REG_AVI_INFOFRAME_LENGTH	0x53
#define ADV7511_REG_AVI_INFOFRAME_CHECKSUM	0x54
#define ADV7511_REG_AVI_INFOFRAME(x)		(0x55 + (x)) /* 0x55 - 0x6f */
#define ADV7511_REG_TIMING_GEN_SEQ      0xd0

enum adv7511_input_clock {
	ADV7511_INPUT_CLOCK_1X,
	ADV7511_INPUT_CLOCK_2X,
	ADV7511_INPUT_CLOCK_DDR,
};

enum adv7511_input_justification {
	ADV7511_INPUT_JUSTIFICATION_EVENLY = 0,
	ADV7511_INPUT_JUSTIFICATION_RIGHT = 1,
	ADV7511_INPUT_JUSTIFICATION_LEFT = 2,
};

enum adv7511_input_sync_pulse {
	ADV7511_INPUT_SYNC_PULSE_DE = 0,
	ADV7511_INPUT_SYNC_PULSE_HSYNC = 1,
	ADV7511_INPUT_SYNC_PULSE_VSYNC = 2,
	ADV7511_INPUT_SYNC_PULSE_NONE = 3,
};

/**
 * enum adv7511_sync_polarity - Polarity for the input sync signals
 * @ADV7511_SYNC_POLARITY_PASSTHROUGH:  Sync polarity matches that of
 *				       the currently configured mode.
 * @ADV7511_SYNC_POLARITY_LOW:	    Sync polarity is low
 * @ADV7511_SYNC_POLARITY_HIGH:	    Sync polarity is high
 *
 * If the polarity is set to either LOW or HIGH the driver will configure the
 * ADV7511 to internally invert the sync signal if required to match the sync
 * polarity setting for the currently selected output mode.
 *
 * If the polarity is set to PASSTHROUGH, the ADV7511 will route the signal
 * unchanged. This is used when the upstream graphics core already generates
 * the sync signals with the correct polarity.
 */
enum adv7511_sync_polarity {
	ADV7511_SYNC_POLARITY_PASSTHROUGH,
	ADV7511_SYNC_POLARITY_LOW,
	ADV7511_SYNC_POLARITY_HIGH,
};
enum hdmi_colorspace {
    HDMI_COLORSPACE_RGB,
    HDMI_COLORSPACE_YUV422,
    HDMI_COLORSPACE_YUV444,
};
/**
 * struct adv7511_link_config - Describes adv7511 hardware configuration
 * @input_color_depth:		Number of bits per color component (8, 10 or 12)
 * @input_colorspace:		The input colorspace (RGB, YUV444, YUV422)
 * @input_clock:		The input video clock style (1x, 2x, DDR)
 * @input_style:		The input component arrangement variant
 * @input_justification:	Video input format bit justification
 * @clock_delay:		Clock delay for the input clock (in ps)
 * @embedded_sync:		Video input uses BT.656-style embedded sync
 * @sync_pulse:			Select the sync pulse
 * @vsync_polarity:		vsync input signal configuration
 * @hsync_polarity:		hsync input signal configuration
 */
struct adv7511_link_config {
	unsigned int input_color_depth;
	enum hdmi_colorspace input_colorspace;
	enum adv7511_input_clock input_clock;
	unsigned int input_style;
	enum adv7511_input_justification input_justification;

	int clock_delay;

	bool embedded_sync;
	enum adv7511_input_sync_pulse sync_pulse;
	enum adv7511_sync_polarity vsync_polarity;
	enum adv7511_sync_polarity hsync_polarity;
};


struct adv7511_monitor_detect {
	int present;
};

struct adv7511_edid_detect {
	int present;
	int segment;
};

struct adv7511_cec_arg {
	void *arg;
	u32 f_flags;
};

struct adv7511_platform_data {
	uint8_t i2c_edid;
	uint8_t i2c_cec;
	uint32_t cec_clk;
};

#endif
