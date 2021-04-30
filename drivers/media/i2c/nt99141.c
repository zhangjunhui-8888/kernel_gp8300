/*
 * drivers/media/video/nt99141.c
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
#include "nt99141.h"

#define DRIVER_NAME "nt99141"

#define NT99141_RDINFO 0x1410

struct nt99141_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct mutex		mutex; /* mutual excl. when accessing chip */
	bool			autodetect;
	u32			width;
	u32			height;
	enum v4l2_field		scanmode;

		/* i2c clients */
	struct i2c_client *i2c_clients[NT99141_PAGE_MAX];

	/* Regmaps */
	struct regmap *regmap[NT99141_PAGE_MAX];
};

static int nt99141_get_id(struct v4l2_subdev *sd,u32 val);
static void nt99141_init(struct v4l2_subdev *sd,struct nt99141_state *state);

/*****************************************************************************/
/*  Private functions                                                        */
/*****************************************************************************/
#define to_nt99141_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct nt99141_state,	\
					    ctrl_hdl)->sd)

/* ----------------------------------------------------------------------- */
static inline struct nt99141_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nt99141_state, sd);
}

/* ----------------------------------------------------------------------- */
static int nt99141_read_check(struct nt99141_state *state,
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
	struct nt99141_state *state = to_state(sd);

	ret = nt99141_read_check(state, NT99141_PAGE, reg);

	printk("%s read addr is %x val is %x \n",__FUNCTION__,reg,ret);

	return ret;
}

/* ----------------------------------------------------------------------- */
static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct nt99141_state *state = to_state(sd);

	printk("%s write addr is %x val is %x \n",__FUNCTION__,reg,val);
	ret = regmap_write(state->regmap[NT99141_PAGE], reg, val);
	io_read(sd,reg);
	return ret;
}


/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_subdev_video_ops                       */
/*****************************************************************************/
/*
 * nt99141_g_input_status() - V4L2 decoder i/f handler for g_input_status
 * @sd: ptr to v4l2_subdev struct
 * @status: video input status flag
 *
 * Obtains the video input status flags.
 */
static int nt99141_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	printk("[zaf] subdev: nt99141_g_input_status\n");
	return 0;
}

/*
 * nt99141_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 * Currently no implementation.
 */
static int nt99141_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct nt99141_state *state = to_state(sd);

//	nt99141_init(sd, state);
	printk("[zaf] subdev: nt99141_s_stream\n");

	return 0;
}

/*
 * nt99141_cropcap() - V4L2 decoder i/f handler for cropcap
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets cropping limits, default cropping rectangle and pixel aspect.
 */
static int nt99141_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	printk("[zaf] subdev: nt99141_cropcap \n");

	return 0;
}

/*
 * nt99141_cropcap() - V4L2 decoder i/f handler for g_crop
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets current cropping rectangle.
 */
static int nt99141_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	printk("[zaf] subdev: nt99141_g_crop \n");
	a->c.left	= 0;
	a->c.top	= 0;

	return 0;
}

/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_ctrl_ops                               */
/*****************************************************************************/
/*
 * nt99141_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in nt99141 decoder device.
 */
static int nt99141_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_nt99141_sd(ctrl);
	struct nt99141_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	printk("[zaf] subdev: nt99141_s_ctrl \n");
	if (ret)
		return ret;

	return ret;
}


static const struct v4l2_subdev_core_ops nt99141_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.init = nt99141_get_id,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.querymenu = v4l2_subdev_querymenu,
};

static const struct v4l2_subdev_video_ops nt99141_video_ops = {
	.g_input_status		= nt99141_g_input_status,
	.s_stream		= nt99141_s_stream,
	.cropcap		= nt99141_cropcap,
	.g_crop			= nt99141_g_crop,
};

static const struct v4l2_subdev_ops nt99141_ops = {
	.core = &nt99141_core_ops,
	.video = &nt99141_video_ops,
};

static const struct v4l2_ctrl_ops nt99141_ctrl_ops = {
	.s_ctrl = nt99141_s_ctrl,
};

/*****************************************************************************/
/*  reset nt99141 and power on                                            */
/*****************************************************************************/

static int reset_nt99141(struct device_node *devnod){
	struct gpio_desc *gpio0_desc;
	struct gpio_desc *gpio0_desc_pwdw;
	int gpio = 0, gpio_pwdw = 0;

	gpio_pwdw = of_get_named_gpio_flags(devnod, "pwdw-gpios", 0, NULL);
	if (gpio < 0) {
		if (gpio != -EPROBE_DEFER)
			pr_err("%s: Can't get 'pwdw-gpios' DT property\n",
			       __func__);
		return -1;
	}

	gpio = of_get_named_gpio_flags(devnod, "reset-gpios", 0, NULL);
	if (gpio < 0) {
		if (gpio != -EPROBE_DEFER)
			pr_err("%s: Can't get 'reset-gpios' DT property\n",
			       __func__);
		return -1;
	}

	gpio0_desc = gpio_to_desc(gpio);
	gpio0_desc_pwdw = gpio_to_desc(gpio_pwdw);
	gpiod_direction_output(gpio0_desc,0);
	gpiod_direction_output(gpio0_desc_pwdw,0);
	msleep(500);
	gpiod_direction_output(gpio0_desc,1);
	msleep(500);
	return 0;
}

static void nt99141_init(struct v4l2_subdev *sd,struct nt99141_state *state){
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
			regmap_write(state->regmap[NT99141_PAGE], tmp_reg, INIT_TAB_745M_720P[i+2]);

		}
		i=i+3;
	}
}

/*
*   read nt99141 id
*/
static int nt99141_get_id(struct v4l2_subdev *sd,u32 val){

		unsigned int err,val1=0,val2=0;

		struct nt99141_state *state = to_state(sd);
		err = regmap_read(state->regmap[NT99141_PAGE],
					0x3000,
					&val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val2 = val1 << 8;
		err = regmap_read(state->regmap[NT99141_PAGE],
			    0x3001,
			    &val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val1 |= val2;
		if (val1 != NT99141_RDINFO) {
			v4l2_err(sd, "nt99141 is not ready ........\n");
			return -ENODEV;
		}else{

			nt99141_init(sd, state);
			printk("nt99141 ready ........\n");
			return 1;
		}

}

static const struct regmap_config nt99141_regmap_cnf[] = {
	{
		.name			= "io",
		.reg_bits		= 0x10,
		.val_bits		= 0x8,
		.max_register		= 0xffff,
		.cache_type		= REGCACHE_NONE,
	},

};

static int configure_regmap(struct nt99141_state *state, int region)
{
	int err;

	if (!state->i2c_clients[region])
		return -ENODEV;

	state->regmap[region] =
		devm_regmap_init_i2c(state->i2c_clients[region],
				     &nt99141_regmap_cnf[region]);

	if (IS_ERR(state->regmap[region])) {
		err = PTR_ERR(state->regmap[region]);
		v4l_err(state->i2c_clients[region],
			"Error initializing regmap %d with error %d\n",
			region, err);
		return -EINVAL;
	}

	return 0;
}

static int nt99141_parse_dt(struct nt99141_state *state)
{
	struct device_node *np;

	np = state->i2c_clients[NT99141_PAGE]->dev.of_node;
	return 0;
}


static void nt99141_unregister_clients(struct nt99141_state *state)
{
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(state->i2c_clients); ++i) {
		if (state->i2c_clients[i])
			i2c_unregister_device(state->i2c_clients[i]);
	}
}
/*
 * nt99141_probe - Probe a nt99141 device
 * @client: pointer to i2c_client structure
 * @id: pointer to i2c_device_id structure
 *
 * Initialize the nt99141 device
 */
static int nt99141_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct nt99141_state *state;
	struct v4l2_subdev *sd;
	struct device_node *devnod = client->dev.of_node;
	int ret,err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	state = kzalloc(sizeof(struct nt99141_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	state->i2c_clients[NT99141_PAGE] = client;
	mutex_init(&state->mutex);
	state->autodetect = true;
	sd = &state->sd;
	state->width		= NT99141_MAX_WIDTH;
	state->height		= NT99141_MAX_HEIGHT;
	state->scanmode		= V4L2_FIELD_NONE;
	strcpy(sd->name, "nt99141-gpt");

	err = nt99141_parse_dt(state);
	if (err < 0) {
		v4l_err(client, "DT parsing error\n");
		return err;
	}

	err = reset_nt99141(devnod);
	if (err < 0) {
		v4l_err(client, "can not find reset chip\n");
		goto err_i2c;
	}

	v4l2_i2c_subdev_init(sd, client, &nt99141_ops);
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		id->name, i2c_adapter_id(client->adapter),
		client->addr);

	err = configure_regmap(state,NT99141_PAGE);
	if (err) {
		v4l2_err(sd, "Error configuring IO regmap region\n");
		return -ENODEV;
	}

	return 0;

err_i2c:
	nt99141_unregister_clients(state);
	return err;
err:
	printk(KERN_ERR DRIVER_NAME ": Failed to probe: %d\n", ret);
	return ret;
}

/*
 * nt99141_remove - Remove nt99141 device support
 * @client: pointer to i2c_client structure
 *
 * Reset the nt99141 device
 */
static  int nt99141_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nt99141_state *state = to_state(sd);

	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

#ifdef CONFIG_PM
/*
 * nt9941_suspend - Suspend nt99141 device
 * @client: pointer to i2c_client structure
 * @state: power management state
 *
 * Power down the nt99141 device
 */
static int nt99141_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;
	return ret;
}

/*
 * nt99141_resume - Resume nt99141 device
 * @client: pointer to i2c_client structure
 *
 * Power on and initialize the NT99141 device
 */
static int nt99141_resume(struct i2c_client *client)
{
	int ret;
	return ret;
}
#endif

static const struct i2c_device_id nt99141_dev_id[] = {
		{ "nt99141", 0 },
		{ }
};


static const struct of_device_id nt99141_dt_match[]={
		{.compatible = "nt99141,camera",},
		{},
};

static struct i2c_driver nt99141_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "nt99141",
		.of_match_table = of_match_ptr(nt99141_dt_match),
	},
	.probe		= nt99141_probe,
	.remove		= nt99141_remove,
#ifdef CONFIG_PM
	.suspend = nt99141_suspend,
	.resume = nt99141_resume,
#endif
	.id_table	= nt99141_dev_id,
};

static __init int init_camera(void){
	return i2c_add_driver(&nt99141_driver);
}

static __exit void exit_camera(void){
	i2c_del_driver(&nt99141_driver);
}

module_init(init_camera);
module_exit(exit_camera);

MODULE_DESCRIPTION("nt99141 camera  driver");
MODULE_LICENSE("GPL v2");
