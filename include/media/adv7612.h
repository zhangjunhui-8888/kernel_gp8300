/*
 * adv7612 - Analog Devices ADV7612 video decoder driver
 *
 * Copyright 2012 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
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
 *
 */

#ifndef _ADV7612_
#define _ADV7612_

#include <linux/types.h>
/****************************************/
/* ADV7612 I2C slave address definition */
/****************************************/
#if 0
#define ADV7612_I2C_IO			0x4C	/* IO Map */
#define ADV7612_I2C_CEC			0x40	/* CEC Map */
#define ADV7612_I2C_INFOFRAME	0x3E	/* InfoFrame Map */
#define ADV7612_I2C_DPLL		0x26	/* DPLL Map */
#define ADV7612_I2C_KSV			0x32	/* KSV(Repeater) Map */
#define ADV7612_I2C_EDID		0x36	/* EDID Map */
#define ADV7612_I2C_HDMI		0x34	/* HDMI Map */
#define ADV7612_I2C_CP			0x22	/* CP Map */
#define ADV7612_I2C_EOR		0xFF	/* End Mark */
#endif


enum adv7612_i2c_addr {
	ADV7612_IO_ADDR=0x4C,
	ADV7612_CEC_ADDR=0x40,
	ADV7612_INFOFRAME=0x3E,
	ADV7612_DPLL_ADDR=0x26,
	ADV7612_KSV_ADDR=0x32,
	ADV7612_EDID_ADDR=0x36,
	ADV7612_HDMI_ADDR=0x34,
	ADV7612_CP_ADDR=0x22,
};

/****************************************/
/* ADV7612 other definition             */
/****************************************/

#define ADV7612_MAX_WIDTH		1920
#define ADV7612_MAX_HEIGHT		1080

/****************************************/
/* ADV7612 structure definition         */
/****************************************/

/* Structure for register values */
struct adv7612_reg_value {
	u8 addr;				/* i2c slave address */
	u8 reg;					/* sub (register) address */
	u8 value;				/* register value */
};


/****************************************/
/* ADV7612 IO register definition       */
/****************************************/
#define ADV7612_RDINFO 0x2041

#define IO_RD_INFO_MSB  0xea
#define IO_RD_INFO_LSB  0xeb

/* Power Management */
#define ADV7612_IO_PWR_MAN_REG	0x0C	/* Power management register */
#define ADV7612_IO_PWR_ON		0x42	/* Power on */
#define ADV7612_IO_PWR_OFF		0x62	/* Power down */


/****************************************/
/* ADV7612 CP register definition       */
/****************************************/

/* Contrast Control */
#define ADV7612_CP_CON_REG	0x3a	/* Contrast (unsigned) */
#define ADV7612_CP_CON_MIN	0		/* Minimum contrast */
#define ADV7612_CP_CON_DEF	128		/* Default */
#define ADV7612_CP_CON_MAX	255		/* Maximum contrast */

/* Saturation Control */
#define ADV7612_CP_SAT_REG	0x3b	/* Saturation (unsigned) */
#define ADV7612_CP_SAT_MIN	0		/* Minimum saturation */
#define ADV7612_CP_SAT_DEF	128		/* Default */
#define ADV7612_CP_SAT_MAX	255		/* Maximum saturation */

/* Brightness Control */
#define ADV7612_CP_BRI_REG	0x3c	/* Brightness (signed) */
#define ADV7612_CP_BRI_MIN	-128	/* Luma is -512d */
#define ADV7612_CP_BRI_DEF	0		/* Luma is 0 */
#define ADV7612_CP_BRI_MAX	127		/* Luma is 508d */

/* Hue Control */
#define ADV7612_CP_HUE_REG	0x3d	/* Hue (unsigned) */
#define ADV7612_CP_HUE_MIN	0		/* -90 degree */
#define ADV7612_CP_HUE_DEF	0		/* -90 degree */
#define ADV7612_CP_HUE_MAX	255		/* +90 degree */

/* Video adjustment register */
#define ADV7612_CP_VID_ADJ_REG		0x3e
/* Video adjustment mask */
#define ADV7612_CP_VID_ADJ_MASK		0x7F
/* Enable color controls */
#define ADV7612_CP_VID_ADJ_ENABLE	0x80


/****************************************/
/* ADV7612 HDMI register definition     */
/****************************************/

/* HDMI status register */
#define ADV7612_HDMI_STATUS1_REG		0x07
/* VERT_FILTER_LOCKED flag */
#define ADV7612_HDMI_VF_LOCKED_FLG		0x80
/* DE_REGEN_FILTER_LOCKED flag */
#define ADV7612_HDMI_DERF_LOCKED_FLG	0x20
/* LINE_WIDTH[12:8] mask */
#define ADV7612_HDMI_LWIDTH_MSBS_MASK	0x1F

/* LINE_WIDTH[7:0] register */
#define ADV7612_HDMI_LWIDTH_REG			0x08

/* FIELD0_HEIGHT[12:8] register */
#define ADV7612_HDMI_F0HEIGHT_MSBS_REG	0x09
/* FIELD0_HEIGHT[12:8] mask */
#define ADV7612_HDMI_F0HEIGHT_MSBS_MASK	0x1F

/* FIELD0_HEIGHT[7:0] register */
#define ADV7612_HDMI_F0HEIGHT_LSBS_REG	0x0A

/* HDMI status register */
#define ADV7612_HDMI_STATUS2_REG		0x0B
/* DEEP_COLOR_MODE[1:0] mask */
#define ADV7612_HDMI_DCM_MASK			0xC0
/* HDMI_INTERLACED flag */
#define ADV7612_HDMI_IP_FLAG			0x20
/* FIELD1_HEIGHT[12:8] mask */
#define ADV7612_HDMI_F1HEIGHT_MSBS_MASK	0x1F

/* FIELD1_HEIGHT[7:0] register */
#define ADV7612_HDMI_F1HEIGHT_REG		0x0C



/* Analog input muxing modes (AFE register 0x02, [2:0]) */
enum adv7612_ain_sel {
	ADV7612_AIN1_2_3_NC_SYNC_1_2 = 0,
	ADV7612_AIN4_5_6_NC_SYNC_2_1 = 1,
	ADV7612_AIN7_8_9_NC_SYNC_3_1 = 2,
	ADV7612_AIN10_11_12_NC_SYNC_4_1 = 3,
	ADV7612_AIN9_4_5_6_SYNC_2_1 = 4,
};

/*
 * Bus rotation and reordering. This is used to specify component reordering on
 * the board and describes the components order on the bus when the ADV7612
 * outputs RGB.
 */
enum adv7612_bus_order {
	ADV7612_BUS_ORDER_RGB,		/* No operation	*/
	ADV7612_BUS_ORDER_GRB,		/* Swap 1-2	*/
	ADV7612_BUS_ORDER_RBG,		/* Swap 2-3	*/
	ADV7612_BUS_ORDER_BGR,		/* Swap 1-3	*/
	ADV7612_BUS_ORDER_BRG,		/* Rotate right	*/
	ADV7612_BUS_ORDER_GBR,		/* Rotate left	*/
};

/* Input Color Space (IO register 0x02, [7:4]) */
enum adv7612_inp_color_space {
	ADV7612_INP_COLOR_SPACE_LIM_RGB = 0,
	ADV7612_INP_COLOR_SPACE_FULL_RGB = 1,
	ADV7612_INP_COLOR_SPACE_LIM_YCbCr_601 = 2,
	ADV7612_INP_COLOR_SPACE_LIM_YCbCr_709 = 3,
	ADV7612_INP_COLOR_SPACE_XVYCC_601 = 4,
	ADV7612_INP_COLOR_SPACE_XVYCC_709 = 5,
	ADV7612_INP_COLOR_SPACE_FULL_YCbCr_601 = 6,
	ADV7612_INP_COLOR_SPACE_FULL_YCbCr_709 = 7,
	ADV7612_INP_COLOR_SPACE_AUTO = 0xf,
};

/* Select output format (IO register 0x03, [4:2]) */
enum adv7612_op_format_mode_sel {
	ADV7612_OP_FORMAT_MODE0 = 0x00,
	ADV7612_OP_FORMAT_MODE1 = 0x04,
	ADV7612_OP_FORMAT_MODE2 = 0x08,
};

enum adv7612_drive_strength {
	ADV7612_DR_STR_MEDIUM_LOW = 1,
	ADV7612_DR_STR_MEDIUM_HIGH = 2,
	ADV7612_DR_STR_HIGH = 3,
};

enum adv7612_int1_config {
	ADV7612_INT1_CONFIG_OPEN_DRAIN,
	ADV7612_INT1_CONFIG_ACTIVE_LOW,
	ADV7612_INT1_CONFIG_ACTIVE_HIGH,
	ADV7612_INT1_CONFIG_DISABLED,
};

enum adv7612_page {
	ADV7612_PAGE_IO,
	ADV7612_PAGE_CEC,//0xF4
	ADV7612_PAGE_INFOFRAME,//0xF5
	NOP0,//0xF6
	NOP1,//0xF7
	ADV7612_PAGE_DPLL,//0xF8
	ADV7612_PAGE_REP,//0xF9
	ADV7612_PAGE_EDID,//0xFA
	ADV7612_PAGE_HDMI,//0xFB
	NOP2,//0xFC
	ADV7612_PAGE_CP,//0xFD
	ADV7612_PAGE_MAX,
};


struct adv7612_format_info {
	u32 code;
	u8 op_ch_sel;
	bool rgb_out;
	bool swap_cb_cr;
	u8 op_format_sel;
};

enum adv7612_type {
	ADV7604,
	ADV7611,
	ADV7612,
};

/* Platform dependent definition */
struct adv7612_platform_data {
	/* DIS_PWRDNB: 1 if the PWRDNB pin is unused and unconnected */
	unsigned disable_pwrdnb:1;

	/* DIS_CABLE_DET_RST: 1 if the 5V pins are unused and unconnected */
	unsigned disable_cable_det_rst:1;

	int default_input;

	/* Analog input muxing mode */
	enum adv7612_ain_sel ain_sel;

	/* Bus rotation and reordering */
	enum adv7612_bus_order bus_order;

	/* Select output format mode */
	enum adv7612_op_format_mode_sel op_format_mode_sel;

	/* Configuration of the INT1 pin */
	enum adv7612_int1_config int1_config;

	/* IO register 0x02 */
	unsigned alt_gamma:1;
	unsigned op_656_range:1;
	unsigned alt_data_sat:1;

	/* IO register 0x05 */
	unsigned blank_data:1;
	unsigned insert_av_codes:1;
	unsigned replicate_av_codes:1;

	/* IO register 0x06 */
	unsigned inv_vs_pol:1;
	unsigned inv_hs_pol:1;
	unsigned inv_llc_pol:1;

	/* IO register 0x14 */
	enum adv7612_drive_strength dr_str_data;
	enum adv7612_drive_strength dr_str_clk;
	enum adv7612_drive_strength dr_str_sync;

	/* IO register 0x30 */
	unsigned output_bus_lsb_to_msb:1;

	/* Free run */
	unsigned hdmi_free_run_mode;

	/* i2c addresses: 0 == use default */
	u8 i2c_addresses[ADV7612_PAGE_MAX];
};

enum adv7612_pad {
	ADV7612_PAD_HDMI_PORT_A = 0,
	ADV7612_PAD_HDMI_PORT_B = 1,
	ADV7612_PAD_HDMI_PORT_C = 2,
	ADV7612_PAD_HDMI_PORT_D = 3,
	ADV7612_PAD_VGA_RGB = 4,
	ADV7612_PAD_VGA_COMP = 5,
	/* The source pad is either 1 (ADV7611) or 6 (ADV7612) */
	ADV7612_PAD_SOURCE = 6,
	ADV7611_PAD_SOURCE = 1,
	ADV7612_PAD_MAX = 7,
};

#define V4L2_CID_ADV_RX_ANALOG_SAMPLING_PHASE	(V4L2_CID_DV_CLASS_BASE + 0x1000)
#define V4L2_CID_ADV_RX_FREE_RUN_COLOR_MANUAL	(V4L2_CID_DV_CLASS_BASE + 0x1001)
#define V4L2_CID_ADV_RX_FREE_RUN_COLOR		(V4L2_CID_DV_CLASS_BASE + 0x1002)

/* notify events */
#define ADV7612_HOTPLUG		1
#define ADV7612_FMT_CHANGE	2

#endif
