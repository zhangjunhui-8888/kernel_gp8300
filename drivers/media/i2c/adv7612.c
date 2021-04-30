/*
 * drivers/media/video/adv7612.c
 *
 * Copyright (C) 2013 Renesas Electronics Corporation
 *
 * adv7612.c Analog Devices ADV7612 HDMI receiver driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/mutex.h>
#include <media/soc_camera.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <media/adv7612.h>
#define DRIVER_NAME "adv7612,adv"
//#define DEBUG

static int adv7612_ready(struct v4l2_subdev *sd,u32 val);
void adv7612_480P_no_freerun(struct v4l2_subdev *sd);
void adv7612_720P_no_freerun(struct v4l2_subdev *sd);
void adv7612_1080P60_freerun(struct v4l2_subdev *sd);
void adv7612_1080P60(struct v4l2_subdev *sd);

/* ----------------------------------------------------------------------- */
/*256 bit edid */
static unsigned char Edid_buf[] = {

0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x52,0x74,0x01,0x00,0x01,0x00,0x00,0x00,
0x00,0x00,0x01,0x03,0x80,0x29,0x1A,0x78,0x0A,0xE5,0xB5,0xA3,0x55,0x49,0x99,0x27,
0x13,0x50,0x54,0x23,0x08,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x02,0x3A,0x80,0x18,0x71,0x38,0x2D,0x40,0x58,0x2C,
0x45,0x00,0x80,0x68,0x21,0x00,0x00,0x1E,0x01,0x1D,0x80,0x18,0x71,0x38,0x2D,0x40,
0x58,0x2C,0x45,0x00,0x80,0x68,0x21,0x00,0x00,0x1E,0x00,0x00,0x00,0xFC,0x00,0x54,
0x45,0x53,0x54,0x20,0x54,0x56,0x0A,0x20,0x0A,0x20,0x20,0x20,0x00,0x00,0x00,0xFD,
0x00,0x38,0x3E,0x1E,0x44,0x0F,0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x01,0x89,
0x02,0x03,0x1F,0x70,0x49,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x10,0x22,0x26,0x09,
0x57,0x03,0x15,0x07,0x50,0x83,0x01,0x00,0x00,0x65,0x03,0x0C,0x00,0x11,0x00,0x01,
0x1D,0x80,0x18,0x71,0x1C,0x16,0x20,0x58,0x2C,0x25,0x00,0x80,0x68,0x01,0x00,0x00,
0x9E,0x01,0x1D,0x00,0x72,0x51,0xD0,0x1E,0x20,0x6E,0x28,0x55,0x00,0x80,0x68,0x01,
0x00,0x00,0x1E,0x8C,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,0x10,0x10,0x3E,0x96,0x00,0x80,
0x68,0x01,0x00,0x00,0x18,0x8C,0x0A,0xA0,0x14,0x51,0xF0,0x16,0x00,0x26,0x7C,0x43,
0x00,0x80,0x68,0x01,0x00,0x00,0x98,0x8C,0x0A,0xD0,0x8A,0x20,0xE0,0x2D,0x10,0x10,
0x3E,0x96,0x00,0x90,0x2C,0x01,0x00,0x00,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x4E 
};

/* ----------------------------------------------------------------------- */
struct adv7612_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct mutex		mutex; /* mutual excl. when accessing chip */
	bool			autodetect;
	u32			width;
	u32			height;
	enum v4l2_field		scanmode;

		/* i2c clients */
	struct i2c_client *i2c_clients[ADV7612_PAGE_MAX];

	/* Regmaps */
	struct regmap *regmap[ADV7612_PAGE_MAX];

	struct adv7612_platform_data pdata;
};

/*****************************************************************************/
/*  Private functions                                                        */
/*****************************************************************************/

#define to_adv7612_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct adv7612_state,	\
					    ctrl_hdl)->sd)

static inline struct adv7612_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7612_state, sd);
}

/* ------------------------ I2C ----------------------------------------------- */
static int adv7612_read_check(struct adv7612_state *state,
			     int client_page, u8 reg)
{
	struct i2c_client *client = state->i2c_clients[client_page];
	int err;
	unsigned int val;

	err = regmap_read(state->regmap[client_page], reg, &val);

	if (err) {
		v4l_err(client, "error reading %02x, %02x\n",
				client->addr, reg);
		return err;
	}
	return val;
}

/* adv7612_write_block(): Write raw data with a maximum of I2C_SMBUS_BLOCK_MAX
 * size to one or more registers.
 *
 * A value of zero will be returned on success, a negative errno will
 * be returned in error cases.
 */
static int adv7612_write_block(struct adv7612_state *state, int client_page,
			      unsigned int init_reg, const void *val,
			      size_t val_len)
{
	struct regmap *regmap = state->regmap[client_page];

	if (val_len > I2C_SMBUS_BLOCK_MAX)
		val_len = I2C_SMBUS_BLOCK_MAX;

	return regmap_raw_write(regmap, init_reg, val, val_len);
}

/* ----------------------------------------------------------------------- */

static inline int io_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_IO, reg);

#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif

	return ret;
}

static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);
#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_IO], reg, val);
	io_read(sd,reg);
	return ret;
}

static inline int io_write_clr_set(struct v4l2_subdev *sd, u8 reg, u8 mask,
				   u8 val)
{
	return io_write(sd, reg, (io_read(sd, reg) & ~mask) | val);
}

static inline int cec_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_CEC, reg);
#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif
	return ret;
}

static inline int cec_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_CEC], reg, val);
	cec_read(sd,reg);

	return ret;
}

static inline int cec_write_clr_set(struct v4l2_subdev *sd, u8 reg, u8 mask,
				   u8 val)
{
	return cec_write(sd, reg, (cec_read(sd, reg) & ~mask) | val);
}

static inline int infoframe_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_INFOFRAME, reg);
#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif
	return ret;
}

static inline int infoframe_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	infoframe_read(sd,reg);
	ret = regmap_write(state->regmap[ADV7612_PAGE_DPLL], reg, val);

	return ret;
}

static inline int dpll_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_DPLL, reg);

#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif

	return ret;
}

static inline int dpll_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);
#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_DPLL], reg, val);
	dpll_read(sd,reg);

	return ret;
}

static inline int dpll_write_clr_set(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return dpll_write(sd, reg, (dpll_read(sd, reg) & ~mask) | val);
}

static inline int rep_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_REP, reg);
#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif
	return ret;
}

static inline int rep_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_REP], reg, val);
	rep_read(sd,reg);

	return ret;
}

static inline int rep_write_clr_set(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return rep_write(sd, reg, (rep_read(sd, reg) & ~mask) | val);
}

static inline int edid_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_EDID, reg);
#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif
	return ret;

}

static inline int edid_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_EDID], reg, val);
	edid_read(sd,reg);

	return ret;
}

static inline int edid_write_block(struct v4l2_subdev *sd,
					unsigned int total_len, const u8 *val)
{
	struct adv7612_state *state = to_state(sd);
	int err = 0;
	int i = 0;
	int len = 0;

	while (!err && i < total_len) {
		len = (total_len - i) > I2C_SMBUS_BLOCK_MAX ?
				I2C_SMBUS_BLOCK_MAX :
				(total_len - i);

		err = adv7612_write_block(state, ADV7612_PAGE_EDID,
				i, val + i, len);
		i += len;
	}

	return err;
}

static inline int hdmi_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_HDMI, reg);
#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif
	return ret;

}

static inline int hdmi_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);
#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_HDMI], reg, val);
	hdmi_read(sd,reg);
	return ret;
}

static inline int hdmi_write_clr_set(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return hdmi_write(sd, reg, (hdmi_read(sd, reg) & ~mask) | val);
}

static inline int cp_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct adv7612_state *state = to_state(sd);

	ret = adv7612_read_check(state, ADV7612_PAGE_CP, reg);
#ifdef DEBUG
	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);
#endif

	return ret;

}

static inline int cp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct adv7612_state *state = to_state(sd);
#ifdef DEBUG
	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
#endif
	ret = regmap_write(state->regmap[ADV7612_PAGE_CP], reg, val);
	cp_read(sd,reg);

	return ret;
}

static inline int cp_write_clr_set(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return cp_write(sd, reg, (cp_read(sd, reg) & ~mask) | val);
}

/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_subdev_video_ops                       */
/*****************************************************************************/
/*
 * adv7612_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 * Currently no implementation.
 */
static int adv7612_s_stream(struct v4l2_subdev *sd, int enable)
{
	//adv7612_1080P60_freerun(sd);
	adv7612_1080P60(sd);
    return 0;
}

/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_ctrl_ops                               */
/*****************************************************************************/


/*
 * adv7612_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in ADV7612 decoder device.
 */
static int adv7612_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_adv7612_sd(ctrl);
	struct adv7612_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	return ret;
}


static const struct v4l2_subdev_core_ops adv7612_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.init = adv7612_ready,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.querymenu = v4l2_subdev_querymenu,
};

static const struct v4l2_subdev_video_ops adv7612_video_ops = {
	.s_stream		= adv7612_s_stream,
};

static const struct v4l2_subdev_ops adv7612_ops = {
	.core = &adv7612_core_ops,
	.video = &adv7612_video_ops,
};

static const struct v4l2_ctrl_ops adv7612_ctrl_ops = {
	.s_ctrl = adv7612_s_ctrl,
};

/*****************************************************************************/
/*  i2c driver interface handlers                                            */
/*****************************************************************************/

static int reset_adv7612(struct device_node *devnod){
	struct gpio_desc *gpio0_desc;
	int gpio;

	gpio = of_get_named_gpio_flags(devnod, "reset-gpios", 0, NULL);
	if (gpio < 0) {
		if (gpio != -EPROBE_DEFER)
			pr_err("%s: Can't get 'reset-gpios' DT property\n",
			       __func__);
		return -1;
	}

	gpio0_desc = gpio_to_desc(gpio);
	gpiod_direction_output(gpio0_desc,0);
	msleep(500);
	gpiod_direction_output(gpio0_desc,1);
	msleep(500);
	return 1;
}

static int adv7612_ready(struct v4l2_subdev *sd,u32 val){
		unsigned int err,val1,val2;

		struct adv7612_state *state = to_state(sd);
		err = regmap_read(state->regmap[ADV7612_PAGE_IO],
				0xea,
				&val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val2 = val1 << 8;
		err = regmap_read(state->regmap[ADV7612_PAGE_IO],
			    0xeb,
			    &val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val1 |= val2;
		if (val1 != ADV7612_RDINFO) {
			v4l2_err(sd, "adv7612  not ready ........\n");
			return -ENODEV;
		}else{
			v4l2_err(sd, "adv7612 ready ........\n");
			return 1;
		}

}

static struct i2c_client *adv7612_dummy_client(struct v4l2_subdev *sd,
							u8 addr, u8 io_reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (addr)
		io_write(sd, io_reg, addr << 1);
	return i2c_new_dummy(client->adapter, io_read(sd, io_reg) >> 1);
}

static const struct regmap_config adv7612_regmap_cnf[] = {
	{
		.name			= "io",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "cec",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "infoframe",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "nop1",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "nop2",
		.reg_bits		= 8,
		.val_bits		= 8,
		.max_register		= 0xff,
		.cache_type 	= REGCACHE_NONE,
	},
	{
		.name			= "dpll",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "rep",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "edid",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "nop2",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type 	= REGCACHE_NONE,
	},

	{
		.name			= "hdmi",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
	{
		.name			= "cp",
		.reg_bits		= 8,
		.val_bits		= 8,

		.max_register		= 0xff,
		.cache_type		= REGCACHE_NONE,
	},
};

static int configure_regmap(struct adv7612_state *state, int region)
{
	int err;

	if (!state->i2c_clients[region])
		return -ENODEV;

	state->regmap[region] =
		devm_regmap_init_i2c(state->i2c_clients[region],
				     &adv7612_regmap_cnf[region]);

	if (IS_ERR(state->regmap[region])) {
		err = PTR_ERR(state->regmap[region]);
		v4l_err(state->i2c_clients[region],
			"Error initializing regmap %d with error %d\n",
			region, err);
		return -EINVAL;
	}

	return 0;
}

static int adv7612_parse_dt(struct adv7612_state *state)
{
	struct device_node *np;

	np = state->i2c_clients[ADV7612_PAGE_IO]->dev.of_node;

	/* Use the default I2C addresses. */
#if 1
	state->pdata.i2c_addresses[ADV7612_PAGE_CEC] = 0x4F;
	state->pdata.i2c_addresses[ADV7612_PAGE_INFOFRAME] = 0x4E;
	state->pdata.i2c_addresses[NOP0] = 0x01;
	state->pdata.i2c_addresses[NOP1] = 0x02;
	state->pdata.i2c_addresses[ADV7612_PAGE_DPLL] = 0x49;
	state->pdata.i2c_addresses[ADV7612_PAGE_REP] = 0x4A;
	state->pdata.i2c_addresses[ADV7612_PAGE_EDID] = 0x4D;
	state->pdata.i2c_addresses[ADV7612_PAGE_HDMI] = 0x4B;
	state->pdata.i2c_addresses[NOP2] = 0x03;
	state->pdata.i2c_addresses[ADV7612_PAGE_CP] = 0x48;
#else
	state->pdata.i2c_addresses[ADV7612_PAGE_CEC] = 0x40;
	state->pdata.i2c_addresses[ADV7612_PAGE_INFOFRAME] = 0x3e;
	state->pdata.i2c_addresses[NOP0] = 0x01;
	state->pdata.i2c_addresses[NOP1] = 0x02;
	state->pdata.i2c_addresses[ADV7612_PAGE_DPLL] = 0x26;
	state->pdata.i2c_addresses[ADV7612_PAGE_REP] = 0x32;
	state->pdata.i2c_addresses[ADV7612_PAGE_EDID] = 0x36;
	state->pdata.i2c_addresses[ADV7612_PAGE_HDMI] = 0x34;
	state->pdata.i2c_addresses[NOP2] = 0x03;
	state->pdata.i2c_addresses[ADV7612_PAGE_CP] = 0x22;
#endif
	/* Hardcode the remaining platform data fields. */
	state->pdata.disable_pwrdnb = 0;
	state->pdata.disable_cable_det_rst = 0;
	state->pdata.blank_data = 1;
	state->pdata.op_format_mode_sel = ADV7612_OP_FORMAT_MODE0;
	state->pdata.bus_order = ADV7612_BUS_ORDER_RGB;

	return 0;
}

static int configure_regmaps(struct adv7612_state *state)
{
	int i, err;

	for (i = ADV7612_PAGE_INFOFRAME ; i < ADV7612_PAGE_MAX; i++) {
		err = configure_regmap(state, i);
		if (err && (err != -ENODEV))
			return err;
	}
	return 0;
}

/* ----------------------------------------------------------------------- */
static void Init_Edid(struct v4l2_subdev *sd){
  	int i ;
	int Edid_Lenth = 128;

	for(i=0;i<Edid_Lenth;i++){
     		edid_write(sd, (0x00+i), Edid_buf[i]);
  	}
}

/* ----------------------------------------------------------------------- */

void print_adv7612_all_reg(struct v4l2_subdev *sd){
	int i=0;

	for(i=0;i<=0xFF;i++){
		printk("IO reg is %x, IO  reg  value is 0x%x......\n",i,io_read(sd,i));

	}
	printk("DPLL IO  reg  value is 0x%x......\n",dpll_read(sd,0xA0));
	printk("DPLL IO  reg  value is 0x%x......\n",dpll_read(sd,0xB5));

	for(i=0;i<=0xF6;i++){
		printk("HDMI reg is %x, HDMI  reg  value is 0x%x......\n",i,hdmi_read(sd,i));
	}
	for(i=0x2A;i<=0xFF;i++){
		printk("CP reg is %x, CP  reg  value is 0x%x......\n",i,cp_read(sd,i));
	}
}

/* ----------------------------------------------------------------------- */
void check_hdmi_interface(struct v4l2_subdev *sd){
		int res = 0;

	/*check adv7612 work mode*/
		printk("PADS_PDN value is 0x%x......\n",io_read(sd,0x0C));
		printk("Tristates HPA output pin for Port A 0x%x......\n",io_read(sd,0x20));

		if(io_read(sd,0x6f)&0x1)
		{
			printk("port A cable access ok 0x%x......\n",io_read(sd,0x6f));
		}else{
			printk("port A cable not access  0x%x......\n",io_read(sd,0x6f));
		}

		if(io_read(sd,0x6A)&0x10)
		{
			printk("TMDS clock detected on port A 0x%x......\n",io_read(sd,0x6A));
		}else{
			printk("TMDS clock not detected on port A 0x%x.......\n",io_read(sd,0x6A));
		}

		if(hdmi_read(sd, 0x04)&0x2)
		{
			printk("HDMI PLL lock 0x%x......\n",hdmi_read(sd, 0x04));
		}else{
			printk("HDMI PLL unlock 0x%x.......\n",hdmi_read(sd, 0x04));
		}
		if(hdmi_read(sd, 0x05)&0x40)
		{
			printk("HDCP is open 0x%x......\n",hdmi_read(sd, 0x05));
		}else{
			printk("HDCP is close 0x%x.......\n",hdmi_read(sd, 0x05));
		}

		if(hdmi_read(sd, 0x05)&0x80)
		{
			printk("HDMI is open 0x%x......\n",hdmi_read(sd, 0x05));
		}else{
			printk("DVI is open 0x%x.......\n",hdmi_read(sd, 0x05));
		}

		if(hdmi_read(sd, 0x07)&0x20)
		{
			printk("DE regeneration locked to incoming DE 0x%x......\n",hdmi_read(sd,0x07));
		}else{
			printk("DE regeneration not locked 0x%x.......\n",hdmi_read(sd, 0x07));
		}

		res =  hdmi_read(sd, 0x51)+hdmi_read(sd, 0x52)/128;
		printk("HDMI input single frequency is  %d.......\n",res);
		printk("VID_STD is  0x%x.......\n",io_read(sd, 0x00));
		printk("PriMode reg is  0x%x.......\n",io_read(sd, 0x01));
		printk("reg 0x2 is  0x%x.......\n",io_read(sd, 0x02));
		printk("reg 0x3  op_force_sel is  0x%x.......\n",io_read(sd, 0x03));
		printk("reg 0x5  is  0x%x.......\n",io_read(sd, 0x05));
		if(cp_read(sd, 0xFF)&0x10)
		{
			printk("CP is free runing......\n");
		}else{
			printk("CP is not free runing.......\n");
		}

}



/* ----------------------------------------------------------------------- */
/*480P,sync,packed,8bit*/
void set_480P_sync_8bit(struct v4l2_subdev *sd){
}

/* ----------------------------------------------------------------------- */
void adv7612_720P_no_freerun(struct v4l2_subdev *sd)
{
	/*edid init start*/
	//Manual Hot Plug Detect  1s delay ,hpa_man_value_x
	hdmi_write(sd,0x6C,0xA1);

	//Deassert HPD ,set hpa_a pin 0V,hpa_a pin active
	hdmi_write(sd,0x20,0x30);
	//Enable internal EDID ram, edid for port a enable
	rep_write(sd, 0x74,0x01);

	/*write edid to adv7619*/
	Init_Edid(sd);

	/*Assert HPD*/
	io_write(sd,0x20,0xB0);
	/*edid init end*/

	io_write(sd,0x00,0x0A);

	/*Prim_Mode =110b HDMI-GR 60HZ */
	io_write(sd, 0x01,0x05);

	/*8 bit SDR 422 Mode 0*/
	io_write(sd, 0x03,0x0);

	/*output complete SAV/EAV*/
	io_write(sd,0x05,0x28);


	/*Invert VS,HS pins*/
	io_write(sd,0x0B,0x44);

	  /*Power up part,include chip ,cp clock and pads of the digital output pins*/
	io_write(sd,0x0C,0x42);

	  /*enable Tristate of Pins */
	io_write(sd,0x14,0x7F);
	io_write(sd,0x15,0x80);

	  /*LLC DLL phase*/
	io_write(sd,0x19,0xC0);

	 /*LLC DLL MUX enable*/
	io_write(sd,0x33,0x40);

	 /*Set HDMI FreeRun*/
	cp_write(sd, 0xBA,0x01);
	cp_write(sd, 0xBF,0x17);

	cp_write(sd, 0xC0, 0xA0);
	cp_write(sd, 0xC1, 0xA1);
	cp_write(sd, 0xC2, 0xA2);
	cp_write(sd, 0xC9,0x05);		//   TBD

#ifdef DEBUG
	check_hdmi_interface(sd);

	print_adv7612_all_reg(sd);
#endif
}

/* ----------------------------------------------------------------------- */
void adv7612_1080P60(struct v4l2_subdev *sd)
{
	/*edid init start*/
	//Manual Hot Plug Detect  1s delay ,hpa_man_value_x
	hdmi_write(sd,0x6C,0xA1);

	//Deassert HPD ,set hpa_a pin 0V,hpa_a pin active
	hdmi_write(sd,0x20,0x30);
	//Enable internal EDID ram, edid for port a enable
	rep_write(sd, 0x74,0x01);

	/*write edid to adv7619*/
	Init_Edid(sd);

	/*Assert HPD*/
	io_write(sd,0x20,0xB0);
	/*edid init end*/

	io_write(sd,0x00,0x1E);

	/*Prim_Mode =110b HDMI-GR 60HZ */
	io_write(sd, 0x01,0x05);
	io_write(sd, 0x02,0xf5);

	/*16 bit SDR 422 Mode 0*/
	io_write(sd, 0x03,0x80);

	/*output sync*/
	io_write(sd,0x05,0x28);
	io_write(sd,0x06,0xa6);

	/*Invert VS,HS pins*/
	io_write(sd,0x0B,0x44);

	  /*Power up part,include chip ,cp clock and pads of the digital output pins*/
	io_write(sd,0x0C,0x42);

	  /*enable Tristate of Pins */
	io_write(sd,0x14,0x7F);
	io_write(sd,0x15,0x80);

	  /*LLC DLL phase*/
	io_write(sd,0x19,0x83);

	 /*LLC DLL MUX enable*/
	io_write(sd,0x33,0x40);

	 /*Set HDMI FreeRun*/
	cp_write(sd, 0xBA,0x01);
//	cp_write(sd, 0xBF,0x17);//output blue
	rep_write(sd, 0x40,0x81);

	cp_write(sd, 0xC0, 0xFF);
	cp_write(sd, 0xC1, 0xFF);
	cp_write(sd, 0xC2, 0xFF);

    cp_write(sd, 0xC9,0x05);		//

    hdmi_write(sd,0x9b,0x03);
    hdmi_write(sd,0x00,0x08);
    hdmi_write(sd,0x02,0x03);
    hdmi_write(sd,0x14,0x1F);
    hdmi_write(sd,0x83,0xFC);
    hdmi_write(sd,0x6F,0x0C);
    hdmi_write(sd,0x85,0x1F);
    hdmi_write(sd,0x87,0x70);
    hdmi_write(sd,0x8D,0x04);
    hdmi_write(sd,0x8E,0x1E);
    hdmi_write(sd,0x1A,0x8A);
    hdmi_write(sd,0x57,0xDA);
    hdmi_write(sd,0x58,0x01);
    hdmi_write(sd,0x03,0x98);
    hdmi_write(sd,0x75,0x10);
    hdmi_write(sd,0x90,0x04);
    hdmi_write(sd,0x91,0x1E);

//#ifdef DEBUG
	check_hdmi_interface(sd);

	//print_adv7612_all_reg(sd);
//#endif
}

/* ----------------------------------------------------------------------- */
void adv7612_1080P60_freerun(struct v4l2_subdev *sd)
{
	/*edid init start*/
	//Manual Hot Plug Detect  1s delay ,hpa_man_value_x
	hdmi_write(sd,0x6C,0xA1);

	//Deassert HPD ,set hpa_a pin 0V,hpa_a pin active
	hdmi_write(sd,0x20,0x30);
	//Enable internal EDID ram, edid for port a enable
	rep_write(sd, 0x74,0x01);

	/*write edid to adv7619*/
	Init_Edid(sd);

	/*Assert HPD*/
	io_write(sd,0x20,0xB0);
	/*edid init end*/

	io_write(sd,0x00,0x1E);

	/*Prim_Mode =110b HDMI-GR 60HZ */
	io_write(sd, 0x01,0x05);
	io_write(sd, 0x02,0xf5);

	/*16 bit SDR 422 Mode 0*/
	io_write(sd, 0x03,0x80);

	/*output sync*/
	io_write(sd,0x05,0x28);
	io_write(sd,0x06,0xa6);

	/*Invert VS,HS pins*/
	io_write(sd,0x0B,0x44);

	  /*Power up part,include chip ,cp clock and pads of the digital output pins*/
	io_write(sd,0x0C,0x42);

	  /*enable Tristate of Pins */
	io_write(sd,0x14,0x7F);
	io_write(sd,0x15,0x80);

	  /*LLC DLL phase*/
	io_write(sd,0x19,0x83);

	 /*LLC DLL MUX enable*/
	io_write(sd,0x33,0x40);

	 /*Set HDMI FreeRun*/
	cp_write(sd, 0xBA,0x01);
//	cp_write(sd, 0xBF,0x17);//output blue
	rep_write(sd, 0x40,0x81);

	cp_write(sd, 0xC0, 0xFF);
	cp_write(sd, 0xC1, 0xFF);
	cp_write(sd, 0xC2, 0xFF);

    cp_write(sd, 0xC9,0x05);		//def_col_man_val

//#ifdef DEBUG
	check_hdmi_interface(sd);

	//print_adv7612_all_reg(sd);
//#endif
}

/* ----------------------------------------------------------------------- */
void adv7612_480P_no_freerun(struct v4l2_subdev *sd)
{

	/*edid init start*/
	//Manual Hot Plug Detect  1s delay ,hpa_man_value_x
	hdmi_write(sd,0x6C,0xA1);

	//Deassert HPD ,set hpa_a pin 0V,hpa_a pin active
	hdmi_write(sd,0x20,0x30);
	//Enable internal EDID ram, edid for port a enable
	rep_write(sd, 0x74,0x01);

	/*write edid to adv7619*/
	Init_Edid(sd);

	/*Assert HPD*/
	io_write(sd,0x20,0xB0);
	/*edid init end*/
#if 0
	/*VID_STD =  720 * 480 480p */
	io_write(sd, 0x00,0x02);

	/*Prim_Mode =110b HDMI-GR 60HZ */
	io_write(sd, 0x01,0x06);
#else
	/*VID_STD =  720 * 480 480p */
	io_write(sd, 0x00,0x0A);

	/*Prim_Mode =110b HDMI-GR 60HZ */
	io_write(sd, 0x01,0x05);
#endif
/*Auto CSC, YCrCb out, Set op_656 bit*/
	io_write(sd, 0x02,0xF5);

	/*8 bit SDR 422 Mode 0*/
	io_write(sd, 0x03,0x0);

	/*output complete SAV/EAV*/
	io_write(sd,0x05,0x3D);


	/*Invert VS,HS pins*/
	io_write(sd,0x06,0xA0);
	io_write(sd,0x0B,0x44);

	  /*Power up part,include chip ,cp clock and pads of the digital output pins*/
	io_write(sd,0x0C,0x42);

	  /*enable Tristate of Pins */
	io_write(sd,0x15,0x88);

	  /*LLC DLL phase*/
	io_write(sd,0x19,0xC0);

	 /*LLC DLL MUX enable*/
	io_write(sd,0x33,0x40);

	 /*Set HDMI FreeRun*/
	cp_write(sd, 0xBA,0x01);

	cp_write(sd, 0xC9,0x2D);		//   TBD
//	cp_write(sd, 0xC9,0x2C);		//   TBD
	cp_write(sd, 0xC0, 0xA0);
	cp_write(sd, 0xC1, 0xA1);
	cp_write(sd, 0xC2, 0xA2);
	 /*Required ADI write*/
	cp_write(sd, 0x6C,0x00);	//   TBD

	 /*Disable HDCP 1.1 features*/
	rep_write(sd,0x40,0x81);

	 /*Setting MCLK to 256Fs*/
	dpll_write(sd, 0xB5,0x01);		//   TBD


	 /*Required ADI write*/
	hdmi_write(sd, 0xC0,0x03);		//   TBD

	 /*Set HDMI Input Port A (BG_MEAS_PORT_SEL = 001b)*/
	hdmi_write(sd, 0x00,0x08);
	 /*ALL BG Ports enabled*/
	hdmi_write(sd,0x02,0x00);

	 /*ADI Required Write*/
	hdmi_write(sd, 0x03,0x98);
	hdmi_write(sd, 0x10,0xA5);	//   TBD
	hdmi_write(sd, 0x1B,0x08);	//   TBD
	hdmi_write(sd, 0x45,0x04);	//   TBD
	hdmi_write(sd, 0x97,0xC0);	//   TBD
	hdmi_write(sd, 0x3D,0x10);	//   TBD
	hdmi_write(sd, 0x3E,0x69);	//   TBD

	hdmi_write(sd, 0x3F,0x46);	//   TBD
	hdmi_write(sd, 0x4E,0xFE);	//   TBD
	hdmi_write(sd, 0x4F,0x08);	//   TBD
	hdmi_write(sd, 0x50,0x00);	//   TBD
	hdmi_write(sd, 0x57,0xA3);
	hdmi_write(sd, 0x58,0x07);
	hdmi_write(sd, 0x6F,0x08);


	 /* Enable clock terminators for port A & B*/
	hdmi_write(sd, 0x83,0xF8);

	 /*ADI Required Write*/
	hdmi_write(sd, 0x84,0x03);	//   TBD
	hdmi_write(sd, 0x85,0x10);
	hdmi_write(sd, 0x86,0x9B);	//   TBD
	hdmi_write(sd, 0x89,0x03);	//   TBD

	hdmi_write(sd, 0x9B,0x03);
	hdmi_write(sd, 0x93,0x03);	//   TBD
	hdmi_write(sd, 0x5A,0x80);	//   TBD
	hdmi_write(sd, 0x9C,0x80);	//   TBD
	hdmi_write(sd, 0x9C,0xC0);	//   TBD
	hdmi_write(sd, 0x9C,0x00);	//   TBD
	msleep(1000);

#ifdef DEBUG
	check_hdmi_interface(sd);

	//print_adv7612_all_reg(sd);
#endif
}

static void adv7612_unregister_clients(struct adv7612_state *state)
{
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(state->i2c_clients); ++i) {
		if (state->i2c_clients[i])
			i2c_unregister_device(state->i2c_clients[i]);
	}
}


/*
 * adv7612_probe - Probe a ADV7612 device
 * @client: pointer to i2c_client structure
 * @id: pointer to i2c_device_id structure
 *
 * Initialize the ADV7612 device
 */
static int adv7612_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adv7612_state *state;
	struct v4l2_subdev *sd;
	struct device_node *devnod = client->dev.of_node;
	int ret,i,err;

	printk("entry %s , %d.............................\n",__FUNCTION__,__LINE__);

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	state = kzalloc(sizeof(struct adv7612_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	state->i2c_clients[ADV7612_PAGE_IO] = client;

	mutex_init(&state->mutex);
	state->autodetect = true;
	sd = &state->sd;
	state->width		= ADV7612_MAX_WIDTH;
	state->height		= ADV7612_MAX_HEIGHT;
	state->scanmode		= V4L2_FIELD_NONE;

	err = adv7612_parse_dt(state);
	if (err < 0) {
		v4l_err(client, "DT parsing error\n");
		goto err_i2c;
	}

	err = reset_adv7612(devnod);
	if (err < 0) {
		v4l_err(client, "can not find reset chip\n");
		goto err_i2c;
	}
	v4l2_i2c_subdev_init(sd, client, &adv7612_ops);
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		id->name, i2c_adapter_id(client->adapter),
		client->addr);

	err = configure_regmap(state,ADV7612_PAGE_IO);
	if (err) {
		v4l2_err(sd, "Error configuring IO regmap region\n");
		return -ENODEV;
	}

	for (i = 1; i < ADV7612_PAGE_MAX; ++i) {
		state->i2c_clients[i] =
			adv7612_dummy_client(sd, state->pdata.i2c_addresses[i],
					 0xf3 + i);
		if (!state->i2c_clients[i]) {
			err = -ENOMEM;
			v4l2_err(sd, "failed to create i2c client %u\n", i);
			goto err_i2c;
		}
	}

	/* Configure regmaps */
	err = configure_regmaps(state);
	if (err)
		goto err;
	printk(" adv7612 probe end!\n");

	return 0;

err_i2c:
		adv7612_unregister_clients(state);
		return err;

err:
	printk(KERN_ERR DRIVER_NAME ": Failed to probe: %d\n", ret);
	return ret;
}

/*
 * adv7612_remove - Remove ADV7612 device support
 * @client: pointer to i2c_client structure
 *
 * Reset the ADV7612 device
 */
static  int adv7612_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7612_state *state = to_state(sd);

	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id adv7612_dev_id[] = {
		{ "adv7612", 0 },
		{ }
		};

#ifdef CONFIG_PM
/*
 * adv7612_suspend - Suspend ADV7612 device
 * @client: pointer to i2c_client structure
 * @state: power management state
 *
 * Power down the ADV7612 device
 */
static int adv7612_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;
#if 0
	ret = adv7612_write_register(client, ADV7612_I2C_IO,
				ADV7612_IO_PWR_MAN_REG, ADV7612_IO_PWR_OFF);
#endif
	return ret;
}

/*
 * adv7612_resume - Resume ADV7612 device
 * @client: pointer to i2c_client structure
 *
 * Power on and initialize the ADV7612 device
 */
static int adv7612_resume(struct i2c_client *client)
{
	int ret;
	return ret;
}
#endif

static const struct of_device_id adv7612_dt_match[]={
		{.compatible = "adv7612,adv",},
		{},
};

static struct i2c_driver adv7612_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "adv7612",
		.of_match_table = of_match_ptr(adv7612_dt_match),
	},
	.probe		= adv7612_probe,
	.remove		= adv7612_remove,
#ifdef CONFIG_PM
	.suspend = adv7612_suspend,
	.resume = adv7612_resume,
#endif
	.id_table	= adv7612_dev_id,
};

static __init int init_camera(void){
	return i2c_add_driver(&adv7612_driver);
}

static __exit void exit_camera(void){
	i2c_del_driver(&adv7612_driver);
}

module_init(init_camera);
module_exit(exit_camera);

MODULE_DESCRIPTION("HDMI Receiver ADV7612 video decoder driver");
MODULE_LICENSE("GPL v2");
