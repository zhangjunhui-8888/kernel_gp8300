/*
   Driver for the Epson RTC module RX-8035 SA

   Copyright(C) SEIKO EPSON CORPORATION 2013-2019. All rights reserved.

   Derived from RX-8025 driver: 
   Copyright (C) 2009 Wolfgang Grandegger <wg@grandegger.com>

   Copyright (C) 2005 by Digi International Inc.
   All rights reserved.

   Modified by fengjh at rising.com.cn
   <http://lists.lm-sensors.org/mailman/listinfo/lm-sensors>
   2006.11

   Code cleanup by Sergei Poselenov, <sposelenov@emcraft.com>
   Converted to new style by Wolfgang Grandegger <wg@grandegger.com>
   Alarm and periodic interrupt added by Dmitry Rakhchev <rda@emcraft.com>

   This driver software is distributed as is, without any warranty of any kind,
   either express or implied as further specified in the GNU Public License. This
   software may be used and distributed according to the terms of the GNU Public
   License, version 2 as published by the Free Software Foundation.
   See the file COPYING in the main directory of this archive for more details.

   You should have received a copy of the GNU General Public License along with
   this program. If not, see <http://www.gnu.org/licenses/>.
   */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/bcd.h>
#include <linux/i2c.h>
#include <linux/list.h>
#include <linux/rtc.h>
#include <linux/of_gpio.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>

// RX-8035 Register definitions
#define RX8035_B0_SEC				0x00
#define RX8035_B0_MIN				0x01
#define RX8035_B0_HOUR				0x02
#define RX8035_B0_WEEK_DAY			0x03
#define RX8035_B0_MONTH_DAY			0x04
#define RX8035_B0_MONTH				0x05
#define RX8035_B0_YEAR				0x06
#define RX8035_B0_DIGITAL_OFFSET		0x07
#define RX8035_B0_ALARM_WK_MIN			0x08
#define RX8035_B0_ALARM_WK_HOUR			0x09
#define RX8035_B0_ALARM_WK_DAY			0x0A
#define RX8035_B0_ALARM_MO_MIN			0x0B
#define RX8035_B0_ALARM_MO_HOUR			0x0C
#define RX8035_B0_RAM				0x0D
#define RX8035_B0_CTRL_1			0x0E
#define RX8035_B0_CTRL_2			0x0F
#define RX8035_B1_TS_SEC			0x10
#define RX8035_B1_TS_MIN			0x11
#define RX8035_B1_TS_HOUR			0x12
#define RX8035_B1_TS_WEEK_DAY			0x13
#define RX8035_B1_TS_MONTH_DAY			0x14
#define RX8035_B1_TS_MONTH			0x15
#define RX8035_B1_TS_YEAR			0x16
#define RX8035_B1_DIGITAL_OFFSET		0x17
// 0x18 is reserved
// 0x19 is reserved
// 0x1A is reserved
#define RX8035_B1_ALARM_MO_DAY			0x1B
#define RX8035_B1_ALARM_MO_MONTH		0x1C
#define RX8035_B1_RAM				0x1D
#define RX8035_B1_CTRL_1			0x1E
#define RX8035_B1_CTRL_2			0x1F

// Flag CTRL_1 Register bit positions
#define RX8035_CTRL_1_CT1 			(1 << 0)
#define RX8035_CTRL_1_CT2 			(1 << 1)
#define RX8035_CTRL_1_CT3 			(1 << 2)
#define RX8035_CTRL_1_TEST 			(1 << 3) 
#define RX8035_CTRL_1_EDEN 			(1 << 4)
#define RX8035_CTRL_1_DBSL 			(1 << 5) 
#define RX8035_CTRL_1_MO_ALE			(1 << 6)
#define RX8035_CTRL_1_WK_ALE			(1 << 7)

// Flag CTRL_2 Register bit positions
#define RX8035_CTRL_2_MO_AFG 			(1 << 0)
#define RX8035_CTRL_2_WK_AFG 			(1 << 1)
#define RX8035_CTRL_2_CTFG 			(1 << 2)
#define RX8035_CTRL_2_EDFG 			(1 << 3)
#define RX8035_CTRL_2_PON 			(1 << 4)
#define RX8035_CTRL_2_XSTP 			(1 << 5)
#define RX8035_CTRL_2_VLOW 			(1 << 6)
#define RX8035_CTRL_2_BTSFG			(1 << 7)

static const struct i2c_device_id rx8035_id[] = {
	{ "rx8035", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rx8035_id);

struct rx8035_data {
	struct i2c_client *client;
	struct rtc_device *rtc;
	struct work_struct work_a;
	struct work_struct work_b;
	u8 ctrlreg;
	int irq_a;
	int irq_b;
	unsigned exiting:1;
};

typedef struct {
	u8 number;
	u8 value;
}reg_data;

#define SE_RTC_REG_READ		_IOWR('p', 0x20, reg_data)
#define SE_RTC_REG_WRITE	_IOW('p',  0x21, reg_data)

static int rx8035_read_reg(struct i2c_client *client, u8 number, u8 *value)
{
	int ret = i2c_smbus_read_byte_data(client, number<<4) ;

	//check for error
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read register #%d\n", number);
		return ret;
	}

	*value = (u8)ret;
	return 0;
}

static int rx8035_read_regs(struct i2c_client *client, u8 number,
		u8 length, u8 *values)
{
	int ret = i2c_smbus_read_i2c_block_data(client, number<<4, 
			length, values);

	//check for length error
	if (ret != length) {
		dev_err(&client->dev, "Unable to read registers #%d..#%d\n", 
				number, number + length - 1);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int rx8035_write_reg(struct i2c_client *client, u8 number, u8 value)
{
	int ret = i2c_smbus_write_byte_data(client, number<<4, value);

	//check for error
	if (ret)
		dev_err(&client->dev, "Unable to write register #%d\n", number);

	return ret;
}

static int rx8035_write_regs(struct i2c_client *client, 
		u8 number, u8 length, u8 *values)
{
	int ret = i2c_smbus_write_i2c_block_data(client, number<<4, 
			length, values);

	//check for error
	if (ret)
		dev_err(&client->dev, "Unable to write registers #%d..#%d\n", 
				number, number + length - 1);

	return ret;
}

static irqreturn_t rx8035_irq_a(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct rx8035_data *rx8035 = i2c_get_clientdata(client);
	disable_irq_nosync(irq);
	schedule_work(&rx8035->work_a);

	return IRQ_HANDLED;
}

static void rx8035_work_a(struct work_struct *work)
{
	struct rx8035_data *rx8035 = container_of(work, struct rx8035_data, work_a);
	struct i2c_client *client = rx8035->client;
	struct mutex *lock = &rx8035->rtc->ops_lock;
	u8 status;

	mutex_lock(lock);

	if (rx8035_read_reg(client, RX8035_B0_CTRL_2, &status))
		goto out;

	dev_dbg(&client->dev, "%s REG[%02xh]=>%02xh\n", __func__, 
			RX8035_B0_CTRL_2, status);

	if (status & RX8035_CTRL_2_PON)
	{
		if (status & RX8035_CTRL_2_XSTP)
			dev_warn(&client->dev, "Supply voltage has dropped to 0 V\n");
		else
			dev_warn(&client->dev, "Power supply is likely flickering\n");
		dev_warn(&client->dev, "Initialization is required.\n");
	}

	// month day alarm interrupt
	if (status & RX8035_CTRL_2_MO_AFG) { 
		status &= ~RX8035_CTRL_2_MO_AFG;
		local_irq_disable();
		rtc_update_irq(rx8035->rtc, 1, RTC_AF | RTC_IRQF);
		local_irq_enable();
		dev_dbg(&client->dev, "%s: month day alarm interrupt, status: %xh\n", 
				__func__, status);
	}

	// event interrupt
	if (status & RX8035_CTRL_2_EDFG) { 
		u8 ram;
		status &= ~RX8035_CTRL_2_EDFG; 
		// you can replace this part with something more useful
		if (rx8035_read_reg(client, RX8035_B0_RAM, &ram))
			goto out;

		ram++;

		if (rx8035_write_reg(client, RX8035_B0_RAM, ram))
			goto out;

		dev_dbg(&client->dev, "%s: event interrupt, status: %xh\n", 
				__func__, status);
	}

	// acknowledge IRQ (clear flags)
	rx8035_write_reg(client, RX8035_B0_CTRL_2, 0x0f & status);

out:
	if (!rx8035->exiting)
		enable_irq(rx8035->irq_a);

	mutex_unlock(lock);
}

static irqreturn_t rx8035_irq_b(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct rx8035_data *rx8035 = i2c_get_clientdata(client);
	disable_irq_nosync(irq);
	schedule_work(&rx8035->work_b);

	return IRQ_HANDLED;
}

static void rx8035_work_b(struct work_struct *work)
{
	struct rx8035_data *rx8035 = container_of(work, struct rx8035_data, work_b);
	struct i2c_client *client = rx8035->client;
	struct mutex *lock = &rx8035->rtc->ops_lock;
	unsigned long events = 0;
	u8 status;

	mutex_lock(lock);

	if (rx8035_read_reg(client, RX8035_B0_CTRL_2, &status))
		goto out;

	dev_dbg(&client->dev, "%s REG[%02xh]=>%02xh\n", __func__, 
			RX8035_B0_CTRL_2, status);

	if (status & RX8035_CTRL_2_PON)
	{
		if (status & RX8035_CTRL_2_XSTP)
			dev_warn(&client->dev, "Supply voltage has dropped to 0 V\n");
		else
			dev_warn(&client->dev, "Power supply is likely flickering\n");
		dev_warn(&client->dev, "Initialization is required.\n");
	}

	// week day alarm interrupt
	if (status & RX8035_CTRL_2_WK_AFG) { 
		status &= ~RX8035_CTRL_2_WK_AFG;
		events |= RTC_AF | RTC_IRQF;
		dev_dbg(&client->dev, "%s: week day alarm interrupt, status: %xh\n", 
				__func__, status);
	}

	// time update interrupt
	if (status & RX8035_CTRL_2_CTFG) { 
		status &= ~RX8035_CTRL_2_CTFG; 
		events |= RTC_UF | RTC_IRQF;
		dev_dbg(&client->dev, "%s: time update interrupt, status: %xh\n", 
				__func__, status);
	}

	if ( events ){
		local_irq_disable();
		rtc_update_irq(rx8035->rtc, 1, events);
		local_irq_enable();
	}

	// acknowledge IRQ (clear flags)
	rx8035_write_reg(client, RX8035_B0_CTRL_2, 0x0f & status);

out:
	if (!rx8035->exiting)
	{
		if (rx8035->irq_b > 0)
			enable_irq(rx8035->irq_b);
		else
			enable_irq(client->irq);
	}

	mutex_unlock(lock);
}

static int rx8035_get_time(struct device *dev, struct rtc_time *dt)
{
	struct rx8035_data *rx8035 = dev_get_drvdata(dev);
	u8 date[7];
	int err;

	err = rx8035_read_regs(rx8035->client, RX8035_B0_SEC, 7, date);
	if (err)
		return err;

	dev_dbg(dev, "%s: read 0x%02x 0x%02x "
			"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", __func__,
			date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

	dt->tm_sec  = bcd2bin(date[RX8035_B0_SEC] & 0x7f);
	dt->tm_min  = bcd2bin(date[RX8035_B0_MIN] & 0x7f);
	dt->tm_hour = bcd2bin(date[RX8035_B0_HOUR] & 0x3f);
	dt->tm_wday = bcd2bin(date[RX8035_B0_WEEK_DAY] & 0x07);
	dt->tm_mday = bcd2bin(date[RX8035_B0_MONTH_DAY] & 0x3f);
	dt->tm_mon  = bcd2bin(date[RX8035_B0_MONTH] & 0x1f) - 1;
	dt->tm_year = bcd2bin(date[RX8035_B0_YEAR]);

	if (dt->tm_year < 70)
		dt->tm_year += 100;

	dev_dbg(dev, "%s: date %ds %dm %dh %dmd %dm %dy\n", __func__,
			dt->tm_sec, dt->tm_min, dt->tm_hour,
			dt->tm_mday, dt->tm_mon, dt->tm_year);

	return rtc_valid_tm(dt);
}

static int rx8035_set_time(struct device *dev, struct rtc_time *dt)
{
	struct rx8035_data *rx8035 = dev_get_drvdata(dev);
	u8 date[7];
	int ret = 0;

	date[RX8035_B0_SEC]  = bin2bcd(dt->tm_sec);
	date[RX8035_B0_MIN]  = bin2bcd(dt->tm_min);
	date[RX8035_B0_HOUR] = bin2bcd(dt->tm_hour) + 0x80;	//only 24hr time

	date[RX8035_B0_MONTH_DAY] = bin2bcd(dt->tm_mday);
	date[RX8035_B0_MONTH]     = bin2bcd(dt->tm_mon + 1);
	date[RX8035_B0_YEAR]      = bin2bcd(dt->tm_year % 100);
	date[RX8035_B0_WEEK_DAY]  = bin2bcd(dt->tm_wday);

	dev_dbg(dev, "%s: write 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__, date[0], date[1], date[2], date[3], date[4], date[5], date[6]);

	ret =  rx8035_write_regs(rx8035->client, RX8035_B0_SEC, 7, date);

	return ret;
}

static int rx8035_init_client(struct i2c_client *client, int *need_reset)
{
	u8 ctrl[2];
	int need_clear = 0;
	int err = 0;

	err = rx8035_read_regs(client, RX8035_B0_CTRL_1, 2, ctrl);
	if (err)
		goto out;

	switch (ctrl[1] & (RX8035_CTRL_2_PON | RX8035_CTRL_2_XSTP | RX8035_CTRL_2_VLOW))
	{
		case (RX8035_CTRL_2_XSTP):
			dev_warn(&client->dev, "Status: clock has stopped temporarily, \
					possibly due to mechanical clash, etc.\n");
			*need_reset = 1;
			break;

		case (RX8035_CTRL_2_XSTP | RX8035_CTRL_2_VLOW):
			dev_warn(&client->dev, "Status: clock has stopped, maybe due \
					to drop in backup power supply\n");
			*need_reset = 1;
			break;

		case (RX8035_CTRL_2_VLOW):
			dev_warn(&client->dev, "Status: exchange the battery\n");
			need_clear = 1;
			break;

		case (RX8035_CTRL_2_PON | RX8035_CTRL_2_XSTP):
			dev_warn(&client->dev, "Status: initialization is required due \
					to power dropp\n");
			*need_reset = 1;
			break;

		case (RX8035_CTRL_2_PON):
			dev_warn(&client->dev, "Status: initialization is required due \
					to power flickering\n");
			*need_reset = 1;
			break;

		case (0):
			dev_warn(&client->dev, "Status: normal\n");
			break;

		default:
			dev_warn(&client->dev, "Status: initialization is required\n");
			*need_reset = 1;
			need_clear = 1;
			break;
	}

	if ( ctrl[1] & RX8035_CTRL_2_MO_AFG ){
		dev_warn(&client->dev, "Month Alarm was detected\n");
		*need_reset = 1;
	}

	if ( ctrl[1] & RX8035_CTRL_2_WK_AFG ){
		dev_warn(&client->dev, "Week Alarm was detected\n");
		*need_reset = 1;
	}

	if ( ctrl[1] & RX8035_CTRL_2_CTFG ){
		dev_warn(&client->dev, "Periodic timer was detected\n");
		*need_reset = 1;
	}

	if ( ctrl[1] & RX8035_CTRL_2_EDFG ){
		dev_warn(&client->dev, "Event was detected\n");
		*need_reset = 1;
	}

	//clear adjust register
	err = rx8035_write_reg(client, RX8035_B0_DIGITAL_OFFSET, 0x00);
	if (err)
		goto out;

	//reset or clear needed?
	if (*need_reset || need_clear) {
		//clear flag register
		err = rx8035_write_reg(client, RX8035_B0_CTRL_1, 0x00);
		if (err)
			goto out;

		//clear ctrl register
		err = rx8035_write_reg(client, RX8035_B0_CTRL_2, 0x00);
		if (err)
			goto out;
	}

out:
	return err;
}

static int rx8035_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct rx8035_data *rx8035 = dev_get_drvdata(dev);
	struct i2c_client *client = rx8035->client;
	u8 alarmvals[2];		//< minute, hour values
	u8 ctrl[1];			//< flag values
	int err;

	if (client->irq <= 0)
		return -EINVAL;

	// get current minute, hour, alarm values
	err = rx8035_read_regs(client, RX8035_B0_ALARM_MO_MIN, 2, alarmvals);
	if (err)
		return err;

	dev_dbg(dev, "%s: minutes:0x%02x hours:0x%02x\n",
			__func__, alarmvals[0], alarmvals[1]);

	// get current flag register values
	err = rx8035_read_reg(client, RX8035_B0_CTRL_2, ctrl);
	if (err)
		return err;

	// Hardware alarm precision is 1 minute
	t->time.tm_sec = 0;
	t->time.tm_min = bcd2bin(alarmvals[0] & 0x7f);
	t->time.tm_hour = bcd2bin(alarmvals[1] & 0x3f);

	t->time.tm_wday = -1;
	t->time.tm_mday = -1;
	t->time.tm_mon = -1;
	t->time.tm_year = -1;

	dev_dbg(dev, "%s: date: %ds %dm %dh %dmd %dm %dy\n",
			__func__,
			t->time.tm_sec, t->time.tm_min, t->time.tm_hour,
			t->time.tm_mday, t->time.tm_mon, t->time.tm_year);

	//check if enabled
	t->enabled = !!(rx8035->ctrlreg & RX8035_CTRL_1_MO_ALE);
	//check if flag is triggered 
	t->pending = (ctrl[0] & RX8035_CTRL_2_MO_AFG) && t->enabled; 

	dev_dbg(dev, "%s: t->enabled: %d t->pending: %d\n",
			__func__,
			t->enabled, t->pending);

	return err;
}

static int rx8035_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8035_data *rx8035 = dev_get_drvdata(dev);
	u8 alarmvals[2];		//minute, hour
	u8 flagreg;			//flag register
	int err;

	if (client->irq <= 0)
		return -EINVAL;

	//get current flag register
	err = rx8035_read_reg(client, RX8035_B0_CTRL_2, &flagreg);
	if (err <0)
		return err;

	// Hardware alarm precision is 1 minute
	alarmvals[0] = bin2bcd(t->time.tm_min);
	alarmvals[1] = bin2bcd(t->time.tm_hour);

	dev_dbg(dev, "%s: write 0x%02x 0x%02x\n", __func__, 
			alarmvals[0], alarmvals[1]);

	//check interrupt enable and disable
	if (rx8035->ctrlreg & RX8035_CTRL_1_MO_ALE) {
		rx8035->ctrlreg &= ~RX8035_CTRL_1_MO_ALE;
		err = rx8035_write_reg(rx8035->client, RX8035_B0_CTRL_1, rx8035->ctrlreg);
		if (err)
			return err;
	}

	//write the new minute and hour values
	err = rx8035_write_regs(rx8035->client, RX8035_B0_ALARM_MO_MIN, 2, alarmvals);
	if (err)
		return err;

	//clear Alarm Flag
	flagreg &= ~RX8035_CTRL_2_MO_AFG;
	err = rx8035_write_reg(rx8035->client, RX8035_B0_CTRL_2, flagreg);
	if (err)
		return err;

	//re-enable interrupt if required
	if (t->enabled) {
		if ( rx8035->rtc->uie_rtctimer.enabled )
			//set update interrupt enable
			rx8035->ctrlreg |= RX8035_CTRL_1_CT3;	//once per second
		if ( rx8035->rtc->aie_timer.enabled )
			//set alarm interrupt enable
			rx8035->ctrlreg |= (RX8035_CTRL_1_MO_ALE | RX8035_CTRL_1_CT3);

		err = rx8035_write_reg(rx8035->client, RX8035_B0_CTRL_1, rx8035->ctrlreg);
		if (err)
			return err;
	}

	return 0;
}

static int rx8035_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rx8035_data *rx8035 = dev_get_drvdata(dev);
	u8 flagreg;
	u8 ctrl;
	int err;

	//get the current ctrl settings
	ctrl = rx8035->ctrlreg;

	if (enabled)
	{
		if ( rx8035->rtc->uie_rtctimer.enabled )
			//set update interrupt enable
			ctrl |= RX8035_CTRL_1_CT3; 
		if ( rx8035->rtc->aie_timer.enabled )
			//set alarm interrupt enable
			ctrl |= (RX8035_CTRL_1_MO_ALE | RX8035_CTRL_1_CT3);
	}
	else
	{
		if ( ! rx8035->rtc->uie_rtctimer.enabled )
			//clear update interrupt enable
			ctrl &= ~RX8035_CTRL_1_CT3;
		if ( ! rx8035->rtc->aie_timer.enabled )
		{
			if ( rx8035->rtc->uie_rtctimer.enabled )
				ctrl &= ~RX8035_CTRL_1_MO_ALE; 
			else //clear alarm interrupt enable
				ctrl &= ~(RX8035_CTRL_1_MO_ALE | RX8035_CTRL_1_CT3); 
		}
	}

	//clear alarm flag
	err = rx8035_read_reg(client, RX8035_B0_CTRL_2, &flagreg);
	if (err <0)
		return err;
	flagreg &= ~RX8035_CTRL_1_MO_ALE;
	err = rx8035_write_reg(rx8035->client, RX8035_B0_CTRL_2, flagreg); 
	if (err)
		return err;

	//update the Control register if the setting changed
	if (ctrl != rx8035->ctrlreg) {
		rx8035->ctrlreg = ctrl;
		err = rx8035_write_reg(rx8035->client, RX8035_B0_CTRL_1, rx8035->ctrlreg); 
		if (err)
			return err;
	}

	return 0;
}

static int rx8035_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;
	int tmp;
	void __user *argp = (void __user *)arg;
	reg_data reg;

	dev_dbg(dev, "%s: cmd=%x\n", __func__, cmd);

	switch (cmd) {
		case SE_RTC_REG_READ:
			if (copy_from_user(&reg, argp, sizeof(reg)))
				return -EFAULT;
			if (reg.number > RX8035_B1_CTRL_2)
				return -EFAULT;
			ret = rx8035_read_reg(client, reg.number, &reg.value);
			if (! ret )				
				return copy_to_user(argp, &reg, sizeof(reg)) ? -EFAULT : 0;
			break;

		case SE_RTC_REG_WRITE:
			if (copy_from_user(&reg, argp, sizeof(reg)))
				return -EFAULT;
			if (reg.number > RX8035_B1_CTRL_2)
				return -EFAULT;				
			ret = rx8035_write_reg(client, reg.number, reg.value);
			break;

		case RTC_VL_READ:
			ret = rx8035_read_reg(client, RX8035_B0_CTRL_2, &reg.value);
			if (! ret)	
			{
				tmp = !!(reg.value & RX8035_CTRL_2_VLOW);
				return copy_to_user(argp, &tmp, sizeof(tmp)) ? -EFAULT : 0;
			}			
			break;			

		case RTC_VL_CLR:
			ret = rx8035_read_reg(client, RX8035_B0_CTRL_2, &reg.value);
			if (! ret)
			{
				reg.value &= ~RX8035_CTRL_2_VLOW;
				ret = rx8035_write_reg(client, RX8035_B0_CTRL_2, reg.value);
			}
			break;

		default:
			return -ENOIOCTLCMD;
	}

	return ret;
}

static struct rtc_class_ops rx8035_rtc_ops = {
	.read_time = rx8035_get_time,
	.set_time = rx8035_set_time,
	.read_alarm = rx8035_read_alarm,
	.set_alarm = rx8035_set_alarm,
	.alarm_irq_enable = rx8035_alarm_irq_enable,
	.ioctl = rx8035_ioctl,
};

static int rx8035_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node; 
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct rx8035_data *rx8035;
	int err, gpio, i, irqs_success = 0, need_reset = 0;
	const char * irq_name[2] = {"rx8035-irq_a", "rx8035-irq_b"};

	dev_dbg(&client->dev, "IRQ %d supplied\n", client->irq);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA | 
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&adapter->dev, "doesn't support required functionality\n");
		err = -EIO;
		goto errout;
	}

	rx8035 = devm_kzalloc(&client->dev, sizeof(struct rx8035_data), GFP_KERNEL);
	if (!rx8035) {
		dev_err(&adapter->dev, "failed to alloc memory\n");
		err = -ENOMEM;
		goto errout;
	}

	rx8035->client = client;
	i2c_set_clientdata(client, rx8035);

	err = rx8035_init_client(client, &need_reset);
	if (err)
		goto errout;

	if (need_reset) {
		struct rtc_time tm;
		rtc_time_to_tm(0, &tm);		// set to 1970/1/1
		rx8035_set_time(&client->dev, &tm);
		dev_warn(&client->dev, " - time set to 1970/1/1\n");
	}

	rx8035->rtc = rtc_device_register(client->name, &client->dev, 
			&rx8035_rtc_ops, THIS_MODULE);

	if (IS_ERR(rx8035->rtc)) {
		err = PTR_ERR(rx8035->rtc);
		dev_err(&client->dev, "unable to register the class device\n");
		goto errout;
	}

	// get interrupts
	rx8035->irq_a = rx8035->irq_b = -1;
	for (i = 0; i < 2; i ++)
	{
		gpio = of_get_named_gpio(np, irq_name[i], 0);
		if (gpio_is_valid(gpio)) {
			int irq;
			err = devm_gpio_request_one(&client->dev, gpio, GPIOF_DIR_IN, 
					irq_name[i]);
			if (err) {
				dev_err(&client->dev, "cannot request %s\n", irq_name[i]);
				goto errout_reg;
			}
			irq = gpio_to_irq(gpio);
			dev_dbg(&client->dev, "%s %d\n", irq_name[i], irq);
			if (irq <= 0) {
				dev_warn(&client->dev, "Failed to "
						"convert gpio #%d to %s\n",
						gpio, irq_name[i]);
				goto errout_reg;
			}
			err = devm_request_threaded_irq(&client->dev,irq, NULL, 
					i==0 ? rx8035_irq_a : rx8035_irq_b, 
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					irq_name[i], client);
			if (err) {
				dev_err(&client->dev, "unable to request %s\n", irq_name[i]);
				goto errout_reg;
			}
			if (i == 0)
			{
				rx8035->irq_a = irq;
				INIT_WORK(&rx8035->work_a, rx8035_work_a);
			}
			else
			{
				rx8035->irq_b = irq;
				INIT_WORK(&rx8035->work_b, rx8035_work_b);
			}
			irqs_success++;
		} else {
			dev_warn(&client->dev, "%s missing or invalid\n", 
					irq_name[i]);
		}
	}

	if (!irqs_success && client->irq > 0) {
		dev_info(&client->dev, "IRQ %d supplied\n", client->irq);
		err = devm_request_threaded_irq(&client->dev,client->irq, NULL, 
				rx8035_irq_b, IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"rx8035", client);

		if (err) {
			dev_err(&client->dev, "unable to request IRQ\n");
			goto errout_reg;
		}
		INIT_WORK(&rx8035->work_b, rx8035_work_b);
	}

	rx8035->rtc->irq_freq = 1;
	rx8035->rtc->max_user_freq = 1;

	return 0;

errout_reg:
	rtc_device_unregister(rx8035->rtc);

errout:
	dev_err(&adapter->dev, "probing for rx8035 failed\n");
	return err;
}

static int rx8035_remove(struct i2c_client *client)
{
	struct rx8035_data *rx8035 = i2c_get_clientdata(client);
	struct mutex *lock = &rx8035->rtc->ops_lock;

	if (client->irq > 0 || rx8035->irq_a > 0 || rx8035->irq_b > 0) {
		mutex_lock(lock);
		rx8035->exiting = 1;
		mutex_unlock(lock);

		//cancel_work
		if (rx8035->irq_a > 0)
			cancel_work_sync(&rx8035->work_a);
		if (rx8035->irq_b > 0 || client->irq > 0)
			cancel_work_sync(&rx8035->work_b);
	}

	rtc_device_unregister(rx8035->rtc);

	return 0;
}

static struct i2c_driver rx8035_driver = {
	.driver = {
		.name = "rtc-rx8035",
		.owner = THIS_MODULE,
	},
	.probe		= rx8035_probe,
	.remove		= rx8035_remove,
	.id_table	= rx8035_id,
};

module_i2c_driver(rx8035_driver);

MODULE_AUTHOR("Val Krutov <val.krutov@ea.epson.com>");
MODULE_DESCRIPTION("RX-8035 SA/LC RTC driver");
MODULE_LICENSE("GPL");
