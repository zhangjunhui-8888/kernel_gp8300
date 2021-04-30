/*
 * drivers/media/video/ov5642.c
 *
 * Copyright (C) 2013 Renesas Electronics Corporation
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
#include <linux/string.h>
#include "ov5642.h"

#define DRIVER_NAME "ov5642"

#define ov5642_RDINFO 0x5642

struct ov5642_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct mutex		mutex; /* mutual excl. when accessing chip */
	bool			autodetect;
	u32			width;
	u32			height;
	enum v4l2_field		scanmode;

		/* i2c clients */
	struct i2c_client *i2c_clients[ov5642_PAGE_MAX];

	/* Regmaps */
	struct regmap *regmap[ov5642_PAGE_MAX];
};

static int ov5642_get_id(struct v4l2_subdev *sd,u32 val);
static void ov5642_init(struct v4l2_subdev *sd,struct ov5642_state *state);

/*****************************************************************************/
/*  Private functions                                                        */
/*****************************************************************************/
#define to_ov5642_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct ov5642_state,	\
					    ctrl_hdl)->sd)

/* ----------------------------------------------------------------------- */
static inline struct ov5642_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5642_state, sd);
}

/* ----------------------------------------------------------------------- */
static int ov5642_read_check(struct ov5642_state *state,
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

/* ----------------------------------------------------------------------- */
static inline int io_read(struct v4l2_subdev *sd, u8 reg)
{
	int ret;
	struct ov5642_state *state = to_state(sd);

	ret = ov5642_read_check(state, ov5642_PAGE, reg);

	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct ov5642_state *state = to_state(sd);

	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
	ret = regmap_write(state->regmap[ov5642_PAGE], reg, val);
	io_read(sd,reg);
	return ret;
}


/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_subdev_video_ops                       */
/*****************************************************************************/
/*
 * ov5642_g_input_status() - V4L2 decoder i/f handler for g_input_status
 * @sd: ptr to v4l2_subdev struct
 * @status: video input status flag
 *
 * Obtains the video input status flags.
 */
static int ov5642_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	printk("[zaf] subdev: ov5642_g_input_status\n");
	return 0;
}

/*
 * ov5642_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 * Currently no implementation.
 */
static int ov5642_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5642_state *state = to_state(sd);

//	ov5642_init(sd, state);
	printk("[zaf] subdev: ov5642_s_stream\n");

	return 0;
}

/*
 * ov5642_cropcap() - V4L2 decoder i/f handler for cropcap
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets cropping limits, default cropping rectangle and pixel aspect.
 */
static int ov5642_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	printk("[zaf] subdev: ov5642_cropcap \n");

	return 0;
}

/*
 * ov5642_cropcap() - V4L2 decoder i/f handler for g_crop
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets current cropping rectangle.
 */
static int ov5642_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	printk("[zaf] subdev: ov5642_g_crop \n");
	a->c.left	= 0;
	a->c.top	= 0;

	return 0;
}

/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_ctrl_ops                               */
/*****************************************************************************/
/*
 * ov5642_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in ov5642 decoder device.
 */
static int ov5642_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_ov5642_sd(ctrl);
	struct ov5642_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	printk("[zaf] subdev: ov5642_s_ctrl \n");
	if (ret)
		return ret;

	return ret;
}


static const struct v4l2_subdev_core_ops ov5642_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.init = ov5642_get_id,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.querymenu = v4l2_subdev_querymenu,
};

static const struct v4l2_subdev_video_ops ov5642_video_ops = {
	.g_input_status		= ov5642_g_input_status,
	.s_stream		= ov5642_s_stream,
	.cropcap		= ov5642_cropcap,
	.g_crop			= ov5642_g_crop,
};

static const struct v4l2_subdev_ops ov5642_ops = {
	.core = &ov5642_core_ops,
	.video = &ov5642_video_ops,
};

static const struct v4l2_ctrl_ops ov5642_ctrl_ops = {
	.s_ctrl = ov5642_s_ctrl,
};

/*****************************************************************************/
/*  reset ov5642 and power on                                            */
/*****************************************************************************/

static int reset_ov5642(struct device_node *devnod){
	struct gpio_desc *gpio0_desc;
	int gpio = 0;

	gpio = of_get_named_gpio_flags(devnod, "rst-gpios", 0, NULL);
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
	return 0;
}

static void ov5642_init(struct v4l2_subdev *sd,struct ov5642_state *state){
	int i = 0;
	unsigned short tmp_reg ;

	while(1){

		if((END_FLAG ==INIT_TAB_745M_720P[i]) && (END_FLAG == INIT_TAB_745M_720P[i+1])\
			&& (END_FLAG ==INIT_TAB_745M_720P[i+2])){
			break;
		}
		else {
			tmp_reg = INIT_TAB_745M_720P[i];
			tmp_reg = (tmp_reg<<8) | INIT_TAB_745M_720P[i+1];
			regmap_write(state->regmap[ov5642_PAGE], tmp_reg, INIT_TAB_745M_720P[i+2]);

		}
		i=i+3;
	}
}

/*
*   read ov5642 id
*/
static int ov5642_get_id(struct v4l2_subdev *sd,u32 val){

		unsigned int err,val1=0,val2=0;

		struct ov5642_state *state = to_state(sd);
		err = regmap_read(state->regmap[ov5642_PAGE],
					REG_CHIP_ID_HIGH,
					&val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val2 = val1 << 8;
		err = regmap_read(state->regmap[ov5642_PAGE],
			    REG_CHIP_ID_LOW,&val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val1 |= val2;
		if (val1 != ov5642_RDINFO) {
			v4l2_err(sd, "ov5642 is not ready ........\n");
			return -ENODEV;
		}else{
			ov5642_init(sd, state);
			printk("ov5642 ready ........\n");
			return 1;
		}

}

static const struct regmap_config ov5642_regmap_cnf[] = {
	{
		.name			= "io",
		.reg_bits		= 0x10,
		.val_bits		= 0x8,
		.max_register		= 0xffff,
		.cache_type		= REGCACHE_NONE,
	},

};

static int configure_regmap(struct ov5642_state *state, int region)
{
	int err;

	if (!state->i2c_clients[region])
		return -ENODEV;

	state->regmap[region] =
		devm_regmap_init_i2c(state->i2c_clients[region],
				     &ov5642_regmap_cnf[region]);

	if (IS_ERR(state->regmap[region])) {
		err = PTR_ERR(state->regmap[region]);
		v4l_err(state->i2c_clients[region],
			"Error initializing regmap %d with error %d\n",
			region, err);
		return -EINVAL;
	}

	return 0;
}

static int ov5642_parse_dt(struct ov5642_state *state)
{
	struct device_node *np;

	np = state->i2c_clients[ov5642_PAGE]->dev.of_node;
	return 0;
}


static void ov5642_unregister_clients(struct ov5642_state *state)
{
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(state->i2c_clients); ++i) {
		if (state->i2c_clients[i])
			i2c_unregister_device(state->i2c_clients[i]);
	}
}
/*
 * ov5642_probe - Probe a ov5642 device
 * @client: pointer to i2c_client structure
 * @id: pointer to i2c_device_id structure
 *
 * Initialize the ov5642 device
 */
static int ov5642_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov5642_state *state;
	struct v4l2_subdev *sd;
	struct device_node *devnod = client->dev.of_node;
	int ret,err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	state = kzalloc(sizeof(struct ov5642_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	state->i2c_clients[ov5642_PAGE] = client;
	mutex_init(&state->mutex);
	state->autodetect = true;
	sd = &state->sd;
	state->width		= ov5642_MAX_WIDTH;
	state->height		= ov5642_MAX_HEIGHT;
	state->scanmode		= V4L2_FIELD_NONE;
	strcpy(sd->name, "ov5642-gpt");

	err = ov5642_parse_dt(state);
	if (err < 0) {
		v4l_err(client, "DT parsing error\n");
		return err;
	}

	err = reset_ov5642(devnod);
	if (err < 0) {
		v4l_err(client, "can not find reset chip\n");
		goto err_i2c;
	}
	v4l2_i2c_subdev_init(sd, client, &ov5642_ops);
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		id->name, i2c_adapter_id(client->adapter),
		client->addr);
	err = configure_regmap(state,ov5642_PAGE);
	if (err) {
		v4l2_err(sd, "Error configuring IO regmap region\n");
		return -ENODEV;
	}

	printk("%s %d .... probe end\n",__func__,__LINE__);
	return 0;

err_i2c:
	ov5642_unregister_clients(state);
	return err;
err:
	printk(KERN_ERR DRIVER_NAME ": Failed to probe: %d\n", ret);
	return ret;
}

/*
 * ov5642_remove - Remove ov5642 device support
 * @client: pointer to i2c_client structure
 *
 * Reset the ov5642 device
 */
static  int ov5642_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5642_state *state = to_state(sd);

	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

#ifdef CONFIG_PM
/*
 * nt9941_suspend - Suspend ov5642 device
 * @client: pointer to i2c_client structure
 * @state: power management state
 *
 * Power down the ov5642 device
 */
static int ov5642_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;
	return ret;
}

/*
 * ov5642_resume - Resume ov5642 device
 * @client: pointer to i2c_client structure
 *
 * Power on and initialize the ov5642 device
 */
static int ov5642_resume(struct i2c_client *client)
{
	int ret;
	return ret;
}
#endif

static const struct i2c_device_id ov5642_dev_id[] = {
		{ "ov5642", 0 },
		{ }
};

static struct i2c_driver ov5642_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ov5642",
	},
	.probe		= ov5642_probe,
	.remove		= ov5642_remove,
	.id_table	= ov5642_dev_id,
};

module_i2c_driver(ov5642_driver);
MODULE_DESCRIPTION("ov5642 camera  driver");
MODULE_LICENSE("GPL v2");
