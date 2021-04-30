/*
 *gpt GPIO device driver
 *
 * Copyright (C) 2009 - 2014 Xilinx, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 */

#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <asm-generic/delay.h>
#include <linux/delay.h>
#include <asm/mach-chip2/apiu.h>
#include <linux/pinctrl/consumer.h>

#include "gpiolib.h"
#define DRIVER_NAME "gpt-gpio"


#define BIT_INT_MASK(PIN)		(0x1 << (16 + PIN))
#define PIN_INPUT(PIN)			(0x1 << PIN)

#define GPT_MASK_SHIFT			0x10
/* Register offsets for the GPIO device */
/* BANK0 CTRL*/
#define GPT_GPIO_CTL			(0xFFC)
/* USFR */
#define GPT_GPIO_USFR			(0xFF8)
/* Data Register */
#define GPT_GPIO_DATA			(0x000)
/* Direction Reg */
#define GPT_GPIO_DIRCTION		(0x400)
/* Bit write reg */
#define GPT_GPIO_DATA_BIT_WRITE		(0x004)
/* Interrupt type */
#define GPT_GPIO_INTTYPE		(0x404)
/* Rising / High - Level reg */
#define GPT_GPIO_R_HL			(0x408)
/* Failing /Low - Level reg */
#define GPT_GPIO_F_LL			(0x40C)
/* Interrupt enable reg */
#define GPT_GPIO_INT_ENABLE		(0x410)
/* Direct Interrupt reg */
#define GPT_GPIO_DIRECT_INT		(0x414)
/* Interrupt clear reg */
#define GPT_GPIO_INT_CLEAR		(0x41C)

/* Disable all interrupts mask */
#define GPT_GPIO_IXR_DISABLE_ALL	0x000000FF

/**
 * struct gpt_gpio - gpio device private data structure
 * @chip:	instance of the gpio_chip
 * @base_addr:	base address of the GPIO device
 * @clk:	clock resource for this controller
 * @irq:	interrupt for the GPIO device
 */
struct gpt_gpio {
	struct gpio_chip chip;
	void __iomem *base_addr;
	struct clk *clk;
	int irq;
	int pgroups;
	struct pinctrl_state **states;
};

static struct irq_chip gpt_gpio_irqchip;

void __iomem *gpio_base_addr;

/**
 * gpt_gpio_get_value - Get the state of the specified pin of GPIO device
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 *
 * This function reads the state of the specified pin of the GPIO device.
 *
 * Return: 0 if the pin is low, 1 if pin is high.
 */
static int gpt_gpio_get_value(struct gpio_chip *chip, unsigned int pin)
{
	u32 data;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);

	data = readl(gpio->base_addr + GPT_GPIO_DATA);

	return (data >> pin) & 1;
}

/**
 * gpt_gpio_set_value - Modify the state of the pin with specified value
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 * @state:	value used to modify the state of the specified pin
 *
 * This function calculates the register offset (i.e to lower 16 bits or
 * upper 16 bits) based on the given pin number and sets the state of a
 * gpio pin to the specified value. The state is either 0 or non-zero.
 */
static void gpt_gpio_set_value(struct gpio_chip *chip, unsigned int pin,
				int state)
{
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);
	unsigned int  tmp ;

	tmp = readl(gpio->base_addr + GPT_GPIO_DATA);

	if(state)
	{
		state = tmp | (BIT(pin));
	} else {
		state = tmp & (~BIT(pin));
	}

	writel(state , gpio->base_addr + GPT_GPIO_DATA);
}

/**
 * gpt_gpio_dir_in - Set the direction of the specified GPIO pin as input
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 *
 * This function uses the read-modify-write sequence to set the direction of
 * the gpio pin as input.
 *
 * Return: 0 always
 */
static int gpt_gpio_dir_in(struct gpio_chip *chip, unsigned int pin)
{
	u32 reg;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);

	/* set the bit in direction mode reg to set the pin as input */

	reg = readl(gpio->base_addr + GPT_GPIO_DIRCTION);
	reg |= BIT(pin);
	writel(reg, gpio->base_addr + GPT_GPIO_DIRCTION);

	return 0;
}

/**
 * gpt_gpio_dir_out - Set the direction of the specified GPIO pin as output
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 * @state:	value to be written to specified pin
 *
 * This function sets the direction of specified GPIO pin as output, configures
 * the Output Enable register for the pin and uses gpt_gpio_set to set
 * the state of the pin to the value specified.
 *
 * Return: 0 always
 */
static int gpt_gpio_dir_out(struct gpio_chip *chip, unsigned int pin,
			     int state)
{
	u32 reg;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);

	/* clear the GPIO pin as output */
	reg = readl(gpio->base_addr + GPT_GPIO_DIRCTION);
	reg &= ~BIT(pin);
	writel(reg, gpio->base_addr + GPT_GPIO_DIRCTION);

	/* set the state of the pin */
	gpt_gpio_set_value(chip, pin, state);
	return 0;
}

/**
 * gpt_gpio_irq_unmask - Enable the interrupts for a gpio pin
 * @irq_data:	irq data containing irq number of gpio pin for the interrupt
 *		to enable
 *
 * This function calculates the gpio pin number from irq number and sets the
 * bit in the Interrupt Enable register of the corresponding bank to enable
 * interrupts for that pin.
 */
static void gpt_gpio_irq_unmask(struct irq_data *irq_data)
{
	unsigned int device_pin_num;
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(irq_data);
	unsigned int tmp ;
	unsigned int clear_bit =0;

	device_pin_num = irq_data->hwirq;

	tmp = readl(gpio->base_addr + GPT_GPIO_INT_ENABLE);
	tmp |= BIT(device_pin_num);
	writel(tmp, gpio->base_addr + GPT_GPIO_INT_ENABLE);

	clear_bit = readl(gpio->base_addr + GPT_GPIO_INT_CLEAR);
	clear_bit = clear_bit | (BIT(device_pin_num));
	writel(clear_bit, gpio->base_addr + GPT_GPIO_INT_CLEAR);
}

/**
 * gpt_gpio_irq_mask - Disable the interrupts for a gpio pin
 * @irq_data:	per irq and chip data passed down to chip functions
 *
 * This function calculates gpio pin number from irq number and sets the
 * bit in the Interrupt Disable register of the corresponding bank to disable
 * interrupts for that pin.
 */
static void gpt_gpio_irq_mask(struct irq_data *irq_data)
{
	unsigned int device_pin_num;
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(irq_data);
	unsigned int tmp ;

	device_pin_num = irq_data->hwirq;

        tmp = readl(gpio->base_addr + GPT_GPIO_INT_ENABLE);
        /* 0 disable  0xffc ctl reg*/
        tmp &= ~BIT(device_pin_num);
        /*enable register*/
        writel(tmp, gpio->base_addr + GPT_GPIO_INT_ENABLE);
}

/**
 * gpt_gpio_irq_enable - Enable the interrupts for a gpio pin
 * @irq_data:	irq data containing irq number of gpio pin for the interrupt
 *		to enable
 *
 * Clears the INTSTS bit and unmasks the given interrrupt.
 */
static void gpt_gpio_irq_enable(struct irq_data *irq_data)
{
	/*
	 * The gpt GPIO controller does not disable interrupt detection when
	 * the interrupt is masked and only disables the propagation of the
	 * interrupt. This means when the controller detects an interrupt
	 * condition while the interrupt is logically disabled it will propagate
	 * that interrupt event once the interrupt is enabled. This will cause
	 * the interrupt consumer to see spurious interrupts to prevent this
	 * first make sure that the interrupt is not asserted and then enable
	 * it.
	 */
	gpt_gpio_irq_unmask(irq_data);
}

/**
 * gpt_gpio_set_irq_type - Set the irq type for a gpio pin
 * @irq_data:	irq data containing irq number of gpio pin
 * @type:	interrupt type that is to be set for the gpio pin
 *
 * This function gets the gpio pin number and its bank from the gpio pin number
 * and configures the INT_TYPE, INT_POLARITY and INT_ANY registers.
 *
 * Return: 0, negative error otherwise.
 * TYPE-EDGE_RISING,  INT_TYPE - 1, INT_POLARITY - 1,  INT_ANY - 0;
 * TYPE-EDGE_FALLING, INT_TYPE - 1, INT_POLARITY - 0,  INT_ANY - 0;
 * TYPE-EDGE_BOTH,    INT_TYPE - 1, INT_POLARITY - NA, INT_ANY - 1;
 * TYPE-LEVEL_HIGH,   INT_TYPE - 0, INT_POLARITY - 1,  INT_ANY - NA;
 * TYPE-LEVEL_LOW,    INT_TYPE - 0, INT_POLARITY - 0,  INT_ANY - NA
 */
static int gpt_gpio_set_irq_type(struct irq_data *irq_data, unsigned int type)
{
	u32 int_type, int_rhl, int_fll;
	unsigned int device_pin_num;
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(irq_data);

	device_pin_num = irq_data->hwirq;

	int_type = readl(gpio->base_addr + GPT_GPIO_INTTYPE);
	int_rhl  = readl(gpio->base_addr + GPT_GPIO_R_HL);
	int_fll  = readl(gpio->base_addr + GPT_GPIO_F_LL);

	/*
	 * based on the type requested, configure the INT_TYPE, INT_POLARITY
	 * and INT_ANY registers
	 */
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		int_type |= BIT(device_pin_num);
		int_rhl |= BIT(device_pin_num);
		int_fll &= ~BIT(device_pin_num);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		int_type |= BIT(device_pin_num);
		int_fll |= BIT(device_pin_num);
		int_rhl &= ~BIT(device_pin_num);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		int_type |= BIT(device_pin_num);
		int_rhl |= BIT(device_pin_num);
		int_fll |= BIT(device_pin_num);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		int_type &= ~BIT(device_pin_num);
		int_rhl |= BIT(device_pin_num);
		int_fll &= ~BIT(device_pin_num);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		int_type &= ~BIT(device_pin_num);
		int_rhl &= ~BIT(device_pin_num);
		int_fll |= BIT(device_pin_num);
		break;
	default:
		return -EINVAL;
	}

	writel(GPIO_EN | BIT_INT_MASK(device_pin_num) | GPIO_COMP_EN,
					gpio->base_addr + GPT_GPIO_CTL);
	writel(readl((void*)(gpio->base_addr + GPT_GPIO_DIRCTION)) | PIN_INPUT(device_pin_num),
				gpio->base_addr + GPT_GPIO_DIRCTION);

	writel(int_type,gpio->base_addr + GPT_GPIO_INTTYPE);
	writel(int_rhl, gpio->base_addr + GPT_GPIO_R_HL);
	writel(int_fll, gpio->base_addr + GPT_GPIO_F_LL);

	__irq_set_chip_handler_name_locked(irq_data->irq,
			&gpt_gpio_irqchip, handle_level_irq, NULL);

	return 0;
}

static int gpt_gpio_set_wake(struct irq_data *data, unsigned int on)
{
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(data);

	irq_set_irq_wake(gpio->irq, on);

	return 0;
}

/* irq chip descriptor */
static struct irq_chip gpt_gpio_irqchip = {
	.name		= DRIVER_NAME,
	.irq_enable	= gpt_gpio_irq_enable,
	.irq_mask	= gpt_gpio_irq_mask,
	.irq_unmask	= gpt_gpio_irq_unmask,
	.irq_set_type	= gpt_gpio_set_irq_type,
	.irq_set_wake	= gpt_gpio_set_wake,
	.flags		= IRQCHIP_EOI_THREADED | IRQCHIP_EOI_IF_HANDLED |
			  IRQCHIP_MASK_ON_SUSPEND | IRQCHIP_ONESHOT_SAFE,
};

static void gpt_gpio_handle_bank_irq(struct gpt_gpio *gpio,
					unsigned long pending)
{
	struct irq_domain *irqdomain = gpio->chip.irqdomain;
	int offset;

	if (!pending)
		return;

	for_each_set_bit(offset, &pending, 8) {
		unsigned int gpio_irq;

		gpio_irq = irq_find_mapping(irqdomain, offset);
		generic_handle_irq(gpio_irq);
		writel(offset , gpio->base_addr + GPT_GPIO_INT_CLEAR);
	}
}

/**
 * gpt_gpio_irqhandler - IRQ handler for the gpio banks of a gpio device
 * @irq:	irq number of the gpio bank where interrupt has occurred
 *		it is virtual irq of mpic,
 * @desc:	irq descriptor instance of the 'irq'
 *
 * This function reads the Interrupt Status Register of each bank to get the
 * gpio pin number which has triggered an interrupt. It then acks the triggered
 * interrupt and calls the pin specific handler set by the higher layer
 * application for that pin.
 * Note: A bug is reported if no handler is set for the gpio pin.
 */
static void gpt_gpio_irqhandler(unsigned int irq, struct irq_desc *desc)
{
	u32 int_sts, int_enb;

	struct gpt_gpio *gpio = irq_get_handler_data(irq);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);

	chained_irq_enter(irqchip, desc);

	int_sts = readl(gpio->base_addr + GPT_GPIO_USFR);
	int_enb = readl(gpio->base_addr + GPT_GPIO_INT_ENABLE);
	dev_dbg(gpio->chip.dev, "Func:%s, irq = %d, int_sts:%x, int_enb:%x\n",
		__func__,irq, int_sts, int_enb);
	gpt_gpio_handle_bank_irq(gpio, int_sts & int_enb);

	chained_irq_exit(irqchip, desc);
}

static int __maybe_unused gpt_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 0);
	struct irq_data *data = irq_get_irq_data(irq);

	if(!irqd_is_wakeup_set(data))
		return pm_runtime_force_suspend(dev);
	return 0;
}

static int __maybe_unused gpt_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 0);
	struct irq_data *data = irq_get_irq_data(irq);

	if(!irqd_is_wakeup_set(data))
		return pm_runtime_force_resume(dev);
	return 0;
}

static int __maybe_unused gpt_gpio_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpt_gpio *gpio = platform_get_drvdata(pdev);

	clk_disable_unprepare(gpio->clk);
	return 0;
}

static int gpt_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void gpt_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static const struct dev_pm_ops gpt_gpio_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gpt_gpio_suspend, gpt_gpio_resume)
	SET_PM_RUNTIME_PM_OPS(gpt_gpio_runtime_suspend,
			gpt_gpio_runtime_resume, NULL)
};

static int gpt_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return irq_find_mapping(chip->irqdomain, offset);
}

static int gpt_gpio_pinctrl_parse_dt(struct gpt_gpio *gpio,
					struct platform_device *pdev)
{
	int i, ret;
	struct pinctrl *pinctrl;
	const char **pctrl_state;
	struct device_node *np = pdev->dev.of_node;

	pinctrl = devm_pinctrl_get(&pdev->dev);
	gpio->pgroups = of_property_count_strings(np, "pinctrl-names");
	if (gpio->pgroups < 0) {
		return 0;
        }

	pctrl_state = devm_kzalloc(&pdev->dev,
		sizeof(*pctrl_state) * gpio->pgroups, GFP_KERNEL);
	if (!pctrl_state) {
		dev_err(&pdev->dev, "Cannot allocate pctrl_state\n");
		return -ENOMEM;
	}
	gpio->states = devm_kzalloc(&pdev->dev,
		sizeof(*gpio->states) * gpio->pgroups, GFP_KERNEL);
	if (!gpio->states) {
		dev_err(&pdev->dev, "Cannot allocate states\n");
		return -ENOMEM;
	}

	for ( i = 0; i < gpio->pgroups; i++) {
		ret = of_property_read_string_index(np, "pinctrl-names", i,
						&pctrl_state[i]);
		if (ret < 0) {
			dev_err(&pdev->dev, "Cannot parse pinctrl-names %d\n",
					ret);
			return ret;
		}
		dev_dbg(&pdev->dev, "pinctrl-names state:%s\n", pctrl_state[i]);
	}

	for ( i = 0; i < gpio->pgroups; i++) {
		gpio->states[i] = pinctrl_lookup_state(pinctrl, pctrl_state[i]);
		if (IS_ERR(gpio->states[i])) {
			dev_err(&pdev->dev, "Lookup state failed\n");
			return IS_ERR(gpio->states[i]);
		}
	}

	for ( i = 0; i < gpio->pgroups; i++) {
		ret = pinctrl_select_state(pinctrl, gpio->states[i]);
		if ( ret < 0) {
			dev_err(&pdev->dev, "Select state failed\n");
			return ret;
		}
	}

	return 0;
}

static struct lock_class_key gpio_lock_class;

/**
 * gpt_gpio_probe - Initialization method for a gpt_gpio device
 * @pdev:	platform device instance
 *
 * This function allocates memory resources for the gpio device and registers
 * all the banks of the device. It will also set up interrupts for the gpio
 * pins.
 * Note: Interrupts are disabled for all the banks during initialization.
 *
 * Return: 0 on success, negative error otherwise.
 */
static int gpt_gpio_probe(struct platform_device *pdev)
{
	int ret, pin, pin_start, gpio_base;
	struct gpt_gpio *gpio;
	struct gpio_chip *chip;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct of_phandle_args pinspec;
	int nr_gpios;

	gpio = devm_kzalloc(&pdev->dev, sizeof(struct gpt_gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpio);
	gpt_gpio_pinctrl_parse_dt(gpio, pdev);
	if (of_property_read_u32(np, "gpt,ngpio", &nr_gpios)) {
		dev_err(&pdev->dev, "invalid gpt,ngpio");
	}

	if(of_parse_phandle_with_fixed_args(np, "gpio-ranges", 3, 0, &pinspec)) {
		dev_err(&pdev->dev, "invalid gpio-range");
		return -1;
	}
	dev_dbg(&pdev->dev, " gpio-ranges args0:%d, args1:%d, args2:%d", 
		pinspec.args[0], pinspec.args[1], pinspec.args[2]);
	pin_start = pinspec.args[1];

	if (of_property_read_u32(np, "gpt,base", &gpio_base)) {
		dev_err(&pdev->dev, "invalid gpt,base");
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gpio->base_addr))
		return PTR_ERR(gpio->base_addr);

	gpio->irq = platform_get_irq(pdev, 0);
	if(gpio->irq < 0) {
		dev_err(&pdev->dev, "invalid IRQ\n");
		return gpio->irq;
	}
	
	/* configure the gpio chip */
	chip = &gpio->chip;
	chip->label = "gpt_gpio";
	chip->owner = THIS_MODULE;
	chip->dev = &pdev->dev;
	chip->get = gpt_gpio_get_value;
	chip->set = gpt_gpio_set_value;
	chip->request = gpt_gpio_request;
	chip->free = gpt_gpio_free;
	chip->direction_input = gpt_gpio_dir_in;
	chip->direction_output = gpt_gpio_dir_out;
	chip->base = gpio_base;
	chip->ngpio = nr_gpios;
	chip->to_irq = gpt_gpio_to_irq;
	chip->exported = true;

	chip->irqdomain = irq_domain_add_linear(np,
					   chip->ngpio,
					   &irq_domain_simple_ops, NULL);
	/* Enable GPIO clock */
	gpio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(gpio->clk)) {
		dev_err(&pdev->dev, "input clock not found.\n");
		return PTR_ERR(gpio->clk);
	}

	ret = gpiochip_add(chip);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add gpio chip\n");
		goto err_rm_gpiochip;
	}

	/* disable interrupts for all banks  */
	writel(GPT_GPIO_IXR_DISABLE_ALL, (void*)(gpio->base_addr + GPT_GPIO_INT_CLEAR));

	for(pin = 0; pin < chip->ngpio; pin++) {
		u32 gpio_irq = irq_create_mapping(chip->irqdomain, pin);
		dev_dbg(&pdev->dev, "pin:%d, irq:%d\n", pin, gpio_irq);
		
		irq_set_lockdep_class(gpio_irq, &gpio_lock_class);
		irq_set_chip_and_handler(gpio_irq, &gpt_gpio_irqchip,
					handle_simple_irq);
		irq_set_chip_data(gpio_irq, (void *)chip);
	}
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	irq_set_handler_data(res->start, gpio);
	irq_set_chained_handler(res->start, gpt_gpio_irqhandler);

	dev_info(&pdev->dev, "GPIO subsystem successfully registered\n");

	return 0;

err_rm_gpiochip:
	gpiochip_remove(chip);

	return ret;

}

//gpio 0 is used for xilinx network irq in fpga board

/**
 * gpt_gpio_remove - Driver removal function
 * @pdev:	platform device instance
 *
 * Return: 0 always
 */
static int gpt_gpio_remove(struct platform_device *pdev)
{
	struct gpt_gpio *gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&gpio->chip);
	device_set_wakeup_capable(&pdev->dev, 0);
	return 0;
}

static struct of_device_id gpt_gpio_of_match[] = {
	{ .compatible = "gpt,gpio",},
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, gpt_gpio_of_match);

static struct platform_driver gpt_gpio_driver = {
	.driver	= {
		.name = DRIVER_NAME,
		.of_match_table = gpt_gpio_of_match,
	},
	.probe = gpt_gpio_probe,
	.remove = gpt_gpio_remove,
};

/**
 * gpt_gpio_init - Initial driver registration call
 *
 * Return: value from platform_driver_register
 */
static int __init gpt_gpio_init(void)
{
	return platform_driver_register(&gpt_gpio_driver);
}
subsys_initcall(gpt_gpio_init);

MODULE_AUTHOR("GPT Inc.");
MODULE_DESCRIPTION("gpt GPIO driver");
MODULE_LICENSE("GPL");
