/*
 * drivers/media/video/rn6864m.c
 *
 * Copyright (C) 2019 GPT
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
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include "rn6864m.h"

#define DRIVER_NAME "rn6864m"

#define RN6864M_RDINFO 0x0510

struct rn6864m_state {
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_subdev	sd;
	struct mutex		mutex;
	/* i2c clients */
	struct i2c_client *i2c_clients[RN6864M_PAGE_MAX];

	/* Regmaps */
	struct regmap *regmap[RN6864M_PAGE_MAX];
};

static int rn6864m_get_id(struct v4l2_subdev *sd,u32 val);
static void rn6864m_init(struct v4l2_subdev *sd);

/*****************************************************************************/
/*  Private functions                                                        */
/*****************************************************************************/
#define to_rn6864m_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct rn6864m_state,	\
					    ctrl_hdl)->sd)

/* ----------------------------------------------------------------------- */
static inline struct rn6864m_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rn6864m_state, sd);
}

/* ----------------------------------------------------------------------- */
static int rn6864m_read_check(struct rn6864m_state *state,
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
	struct rn6864m_state *state = to_state(sd);

	ret = rn6864m_read_check(state, RN6864M_PAGE, reg);

	return ret;
}

/* ----------------------------------------------------------------------- */
static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int ret;
	struct rn6864m_state *state = to_state(sd);

	ret = regmap_write(state->regmap[RN6864M_PAGE], reg, val);
	io_read(sd,reg);
	return ret;
}

/*
 * convenience function to write 16 bit register values that are split up
 * into two consecutive high and low parts
 */
static int reg_write16(struct v4l2_subdev *sd, u16 reg, u16 val16)
{
	int ret;

	struct rn6864m_state *state = to_state(sd);

	ret = regmap_write(state->regmap[RN6864M_PAGE], reg, val16 >> 8);
	if (ret)
		return ret;
	return regmap_write(state->regmap[RN6864M_PAGE], reg + 1, val16 & 0x00ff);
}

/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_subdev_video_ops                       */
/*****************************************************************************/
/*
 * rn6864m_g_input_status() - V4L2 decoder i/f handler for g_input_status
 * @sd: ptr to v4l2_subdev struct
 * @status: video input status flag
 *
 * Obtains the video input status flags.
 */
static int rn6864m_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	printk("subdev: rn6864m_g_input_status\n");
	return 0;
}

/*
 * rn6864m_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 * Currently no implementation.
 */
static int rn6864m_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rn6864m_state *state = to_state(sd);

	return 0;
}

/*
 * rn6864m_cropcap() - V4L2 decoder i/f handler for cropcap
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets cropping limits, default cropping rectangle and pixel aspect.
 */
static int rn6864m_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	printk("subdev: rn6864m_cropcap \n");

	return 0;
}

/*
 * rn6864m_cropcap() - V4L2 decoder i/f handler for g_crop
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 cropcap structure
 *
 * Gets current cropping rectangle.
 */
static int rn6864m_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	printk("subdev: rn6864m_g_crop \n");
	a->c.left	= 0;
	a->c.top	= 0;

	return 0;
}

/*****************************************************************************/
/*  V4L2 decoder i/f handler for v4l2_ctrl_ops                               */
/*****************************************************************************/
/*
 * rn6864m_s_ctrl() - V4L2 decoder i/f handler for s_ctrl
 * @ctrl: pointer to standard V4L2 control structure
 *
 * Set a control in rn6864m decoder device.
 */
static int rn6864m_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_rn6864m_sd(ctrl);
	struct rn6864m_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);

	printk("subdev: rn6864m_s_ctrl \n");
	if (ret)
		return ret;

	return ret;
}


static const struct v4l2_subdev_core_ops rn6864m_core_ops = {
	.queryctrl = v4l2_subdev_queryctrl,
	.init = rn6864m_get_id,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.querymenu = v4l2_subdev_querymenu,
};

static const struct v4l2_subdev_video_ops rn6864m_video_ops = {
	.g_input_status		= rn6864m_g_input_status,
	.s_stream		= rn6864m_s_stream,
	.cropcap		= rn6864m_cropcap,
	.g_crop			= rn6864m_g_crop,
};

static const struct v4l2_subdev_ops rn6864m_ops = {
	.core = &rn6864m_core_ops,
	.video = &rn6864m_video_ops,
};

static const struct v4l2_ctrl_ops rn6864m_ctrl_ops = {
	.s_ctrl = rn6864m_s_ctrl,
};

/*****************************************************************************/
/*  reset rn6864m and power on                                            */
/*****************************************************************************/

/* ----------------------------------------------------------------------- */
static int reset_rn6864m(struct device_node *devnod){
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
	gpiod_direction_output(gpio0_desc,1);
	msleep(500);
	gpiod_direction_output(gpio0_desc,0);
	msleep(500);
	gpiod_direction_output(gpio0_desc,1);
	msleep(500);
	return 0;
}
/* ----------------------------------------------------------------------- */
static int rn6864m_write_array(struct v4l2_subdev *sd,struct regval_list *vals)
{
	struct rn6864m_state *state = to_state(sd);

	while (vals->reg_num != 0xffff || vals->value != 0xff) {
		int ret =regmap_write(state->regmap[RN6864M_PAGE], vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	printk("Register list loaded\n");
	return 0;
}

/* ----------------------------------------------------------------------- */
static void rn6864m_init(struct v4l2_subdev *sd){
	int ret;

	//ret = rn6864m_write_array(sd,RN675x_init_cfg);
	ret = rn6864m_write_array(sd,RN675x_init_cfg_2chin_2chout);
	return ret;
}

/* ----------------------------------------------------------------------- */
static void print_all_reg(struct v4l2_subdev *sd){
	int i , val1;

	struct rn6864m_state *state = to_state(sd);

	for(i = 0; i < 0xff ; i++ ){
		regmap_read(state->regmap[RN6864M_PAGE],
				i,
				&val1);
		printk("reg 0x%d i value is 0x%x \n",i,val1);
	}
}

/* ----------------------------------------------------------------------- */
/*
*   read rn6864m id
*/
static int rn6864m_get_id(struct v4l2_subdev *sd,u32 val){

		unsigned int err,val1=0,val2=0;

		struct rn6864m_state *state = to_state(sd);

		err = regmap_read(state->regmap[RN6864M_PAGE],
					REG_CHIP_ID_HIGH,
					&val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val2 = val1 << 8;
		err = regmap_read(state->regmap[RN6864M_PAGE],
			    REG_CHIP_ID_LOW,&val1);
		if (err) {
			v4l2_err(sd, "Error %d reading IO Regmap\n", err);
			return -ENODEV;
		}
		val1 |= val2;
		if (val1 != RN6864M_RDINFO) {
			v4l2_err(sd, "rn6864m is not ready and value is 0x%x .......\n",val1);
			return -ENODEV;
		}else{
			printk("rn6864m ready ........\n");
			RN675xM_Pre_initial(sd);
			rn6864m_init(sd);
			printk("rn6864m reg init end........\n");
            return 1;
		}

}
/* ----------------------------------------------------------------------- */
/*
* pre_initial_start
*/
void RN675xM_Pre_initial(struct v4l2_subdev *sd) {
	char rom_byte1, rom_byte2, rom_byte3, rom_byte4, rom_byte5, rom_byte6;

	io_write(sd, 0xE1, 0x80);
	io_write(sd, 0xFA, 0x81);
	rom_byte1 = io_read(sd,0xFB);
	rom_byte2 = io_read(sd,0xFB);
	rom_byte3 = io_read(sd,0xFB);
	rom_byte4 = io_read(sd,0xFB);
	rom_byte5 = io_read(sd,0xFB);
	rom_byte6 = io_read(sd,0xFB);

	// config. decoder accroding to rom_byte5 and rom_byte6
	if ((rom_byte6 == 0x00) && (rom_byte5 == 0x00)) {
		io_write(sd,0xEF, 0xAA);
		io_write(sd,0xE7, 0xFF);
		io_write(sd,0xFF, 0x09);
		io_write(sd,0x03, 0x0C);
		io_write(sd,0xFF, 0x0B);
		io_write(sd,0x03, 0x0C);
	}else if (((rom_byte6 == 0x34) && (rom_byte5 == 0xA9)) ||
         ((rom_byte6 == 0x2C) && (rom_byte5 == 0xA8))) {
		io_write(sd,0xEF, 0xAA);
		io_write(sd,0xE7, 0xFF);
		io_write(sd,0xFC, 0x60);
		io_write(sd,0xFF, 0x09);
		io_write(sd,0x03, 0x18);
		io_write(sd,0xFF, 0x0B);
		io_write(sd,0x03, 0x18);
	}else {
		io_write(sd,0xEF, 0xAA);
		io_write(sd,0xFC, 0x60);
		io_write(sd,0xFF, 0x09);
		io_write(sd,0x03, 0x18);
		io_write(sd,0xFF, 0x0B);
		io_write(sd,0x03, 0x18);
	}
}

/* ----------------------------------------------------------------------- */
static const struct regmap_config rn6864m_regmap_cnf[] = {
	{
		.name			= "io",
		.reg_bits		= 0x8,
		.val_bits		= 0x8,
		.max_register		= 0xffff,
		.cache_type		= REGCACHE_NONE,
	},

};

/* ----------------------------------------------------------------------- */
static int configure_regmap(struct rn6864m_state *state, int region)
{
	int err;

	if (!state->i2c_clients[region])
		return -ENODEV;

	state->regmap[region] =
		devm_regmap_init_i2c(state->i2c_clients[region],
				     &rn6864m_regmap_cnf[region]);

	if (IS_ERR(state->regmap[region])) {
		err = PTR_ERR(state->regmap[region]);
		v4l_err(state->i2c_clients[region],
			"Error initializing regmap %d with error %d\n",
			region, err);
		return -EINVAL;
	}

	return 0;
}

/* ----------------------------------------------------------------------- */
static int rn6864m_parse_dt(struct rn6864m_state *state)
{
	struct device_node *np;

	np = state->i2c_clients[RN6864M_PAGE]->dev.of_node;
	return 0;}


/* ----------------------------------------------------------------------- */
static void rn6864m_unregister_clients(struct rn6864m_state *state)
{
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(state->i2c_clients); ++i) {
		if (state->i2c_clients[i])
			i2c_unregister_device(state->i2c_clients[i]);
	}
}

/* ----------------------------------------------------------------------- */
/*
 * rn6864m_probe - Probe a rn6864m device
 * @client: pointer to i2c_client structure
 * @id: pointer to i2c_device_id structure
 *
 * Initialize the rn6864m device
 */
static int rn6864m_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct rn6864m_state *state;
	struct v4l2_subdev *sd;
	struct device_node *devnod = client->dev.of_node;
	int ret,err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);
	state = kzalloc(sizeof(struct rn6864m_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	state->i2c_clients[RN6864M_PAGE] = client;
	mutex_init(&state->mutex);
	sd = &state->sd;

	strcpy(sd->name, "rn6864m-gpt");

	err = rn6864m_parse_dt(state);
	if (err < 0) {
		v4l_err(client, "DT parsing error\n");
		return err;
	}

	err = reset_rn6864m(devnod);
	if (err < 0) {
		v4l_err(client, "can not find reset chip\n");
		goto err_i2c;
	}
	v4l2_i2c_subdev_init(sd, client, &rn6864m_ops);
	snprintf(sd->name, sizeof(sd->name), "%s %d-%04x",
		id->name, i2c_adapter_id(client->adapter),
		client->addr);
	err = configure_regmap(state,RN6864M_PAGE);
	if (err) {
		v4l2_err(sd, "Error configuring IO regmap region\n");
		return -ENODEV;
	}

	printk("%s %d .... probe end\n",__func__,__LINE__);
	return 0;

err_i2c:
	rn6864m_unregister_clients(state);
	return err;
err:
	printk(KERN_ERR DRIVER_NAME ": Failed to probe: %d\n", ret);
	return ret;
}

/* ----------------------------------------------------------------------- */
/*
 * rn6864m_remove - Remove rn6864m device support
 * @client: pointer to i2c_client structure
 *
 * Reset the rn6864m device
 */
static  int rn6864m_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct rn6864m_state *state = to_state(sd);

	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

#ifdef CONFIG_PM

/* ----------------------------------------------------------------------- */
/*
 * nt9941_suspend - Suspend rn6864m device
 * @client: pointer to i2c_client structure
 * @state: power management state
 *
 * Power down the rn6864m device
 */
static int rn6864m_suspend(struct i2c_client *client, pm_message_t state)
{
	int ret;
	return ret;
}

/* ----------------------------------------------------------------------- */
/*
 * rn6864m_resume - Resume rn6864m device
 * @client: pointer to i2c_client structure
 *
 * Power on and initialize the rn6864m device
 */
static int rn6864m_resume(struct i2c_client *client)
{
	int ret;
	return ret;
}
#endif

/* ----------------------------------------------------------------------- */
static const struct i2c_device_id rn6864m_dev_id[] = {
		{ "rn6864m", 0 },
		{ }
};

static struct i2c_driver rn6864m_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rn6864m",
	},
	.probe		= rn6864m_probe,
	.remove		= rn6864m_remove,
	.id_table	= rn6864m_dev_id,
};

module_i2c_driver(rn6864m_driver);
MODULE_DESCRIPTION("rn6864m camera  driver");
MODULE_LICENSE("GPL v2");
