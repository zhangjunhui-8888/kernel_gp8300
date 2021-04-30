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

#include <linux/bitops.h>
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

#include "gpiolib.h"
#define DRIVER_NAME "gpt-gpio"


#define AF_PRINT
#ifdef  AF_PRINT
#define  SP_PRINTK(format,arg...) do { \
                                         printk(format,##arg);\
                                 }while(0)
#else
#define  SP_PRINTK(format,arg...) do { \
                                         }while(0)
#endif

/* Maximum banks */
#define GPT_GPIO_MAX_BANK	8

#define GPT_GPIO_BANK0_NGPIO	8
#define GPT_GPIO_BANK1_NGPIO	8
#define GPT_GPIO_BANK2_NGPIO	8
#define GPT_GPIO_BANK3_NGPIO	8
#define GPT_GPIO_BANK4_NGPIO	8
#define GPT_GPIO_BANK5_NGPIO	8
#define GPT_GPIO_BANK6_NGPIO	8
#define GPT_GPIO_BANK7_NGPIO	8

#define GPT_GPIO_NR_GPIOS	(GPT_GPIO_BANK0_NGPIO + \
				 GPT_GPIO_BANK1_NGPIO + \
				 GPT_GPIO_BANK2_NGPIO + \
				 GPT_GPIO_BANK3_NGPIO + \
				 GPT_GPIO_BANK4_NGPIO + \
				 GPT_GPIO_BANK5_NGPIO + \
				 GPT_GPIO_BANK6_NGPIO + \
				 GPT_GPIO_BANK7_NGPIO)

#define GPT_GPIO_BANK0_PIN_MIN	0
#define GPT_GPIO_BANK0_PIN_MAX	(GPT_GPIO_BANK0_PIN_MIN + \
					GPT_GPIO_BANK0_NGPIO - 1)
#define GPT_GPIO_BANK1_PIN_MIN	(GPT_GPIO_BANK0_PIN_MAX + 1)
#define GPT_GPIO_BANK1_PIN_MAX	(GPT_GPIO_BANK1_PIN_MIN + \
					GPT_GPIO_BANK1_NGPIO - 1)
#define GPT_GPIO_BANK2_PIN_MIN	(GPT_GPIO_BANK1_PIN_MAX + 1)
#define GPT_GPIO_BANK2_PIN_MAX	(GPT_GPIO_BANK2_PIN_MIN + \
					GPT_GPIO_BANK2_NGPIO - 1)
#define GPT_GPIO_BANK3_PIN_MIN	(GPT_GPIO_BANK2_PIN_MAX + 1)
#define GPT_GPIO_BANK3_PIN_MAX	(GPT_GPIO_BANK3_PIN_MIN + \
					GPT_GPIO_BANK3_NGPIO - 1)
#define GPT_GPIO_BANK4_PIN_MIN	(GPT_GPIO_BANK3_PIN_MAX + 1)
#define GPT_GPIO_BANK4_PIN_MAX	(GPT_GPIO_BANK4_PIN_MIN + \
					GPT_GPIO_BANK4_NGPIO - 1)
#define GPT_GPIO_BANK5_PIN_MIN	(GPT_GPIO_BANK4_PIN_MAX + 1)
#define GPT_GPIO_BANK5_PIN_MAX	(GPT_GPIO_BANK5_PIN_MIN + \
					GPT_GPIO_BANK5_NGPIO - 1)
#define GPT_GPIO_BANK6_PIN_MIN	(GPT_GPIO_BANK5_PIN_MAX + 1)
#define GPT_GPIO_BANK6_PIN_MAX	(GPT_GPIO_BANK6_PIN_MIN + \
					GPT_GPIO_BANK6_NGPIO - 1)
#define GPT_GPIO_BANK7_PIN_MIN	(GPT_GPIO_BANK6_PIN_MAX + 1)
#define GPT_GPIO_BANK7_PIN_MAX	(GPT_GPIO_BANK7_PIN_MIN + \
					GPT_GPIO_BANK7_NGPIO - 1)


#define GPT_MASK_SHIFT			0x10
/* Register offsets for the GPIO device */
/* BANK0 CTRL*/
#define GPT_GPIO_CTL(BANK)		(0xFFC + (0x1000 * BANK))
/* USFR */
#define GPT_GPIO_USFR(BANK)		(0xFF8 + (0x1000 * BANK))
/* Data Register */
#define GPT_GPIO_DATA(BANK)		(0x000 + (0x1000 * BANK))
/* Direction Reg */
#define GPT_GPIO_DIRCTION(BANK)		(0x400 + (0x1000 * BANK))
/* Bit write reg */
#define GPT_GPIO_DATA_BIT_WRITE(BANK)	(0x004 + (0x1000 * BANK))
/* Interrupt type */
#define GPT_GPIO_INTTYPE(BANK)		(0x404 + (0x1000 * BANK))
/* Rising / High - Level reg */
#define GPT_GPIO_R_HL(BANK)		(0x408 + (0x1000 * BANK))
/* Failing /Low - Level reg */
#define GPT_GPIO_F_LL(BANK)		(0x40C + (0X1000 * BANK))
/* Interrupt enable reg */
#define GPT_GPIO_INT_ENABLE(BANK)	(0x410 + (0x1000 * BANK))
/* Direct Interrupt reg */
#define GPT_GPIO_DIRECT_INT(BANK)	(0x414 + (0x1000 * BANK))
/* Interrupt clear reg */
#define GPT_GPIO_INT_CLEAR(BANK)	(0x41C + (0x1000 * BANK))



/* Disable all interrupts mask */
#define GPT_GPIO_IXR_DISABLE_ALL	0x000000FF

/* Mid pin number of a bank */
#define GPT_GPIO_MID_PIN_NUM 4

/* GPIO upper 16 bit mask */
#define GPT_GPIO_UPPER_MASK 0x000000F0


struct gpt_gpio_bank{
	int id;
	int parent_irq;
};

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
	struct gpt_gpio_bank ggpio_bank[GPT_GPIO_MAX_BANK];
};

static struct irq_chip gpt_gpio_irqchip;
int intc_gpio = -1;
void __iomem *gpio_base_addr;

/**
 * gpt_gpio_get_bank_pin - Get the bank number and pin number within that bank
 * for a given pin in the GPIO device
 * @pin_num:	gpio pin number within the device
 * @bank_num:	an output parameter used to return the bank number of the gpio
 *		pin
 * @bank_pin_num: an output parameter used to return pin number within a bank
 *		  for the given gpio pin
 *
 * Returns the bank number and pin offset within the bank.
 */
static inline void gpt_gpio_get_bank_pin(unsigned int pin_num,
					  unsigned int *bank_num,
					  unsigned int *bank_pin_num)
{
	switch (pin_num) {
	case GPT_GPIO_BANK0_PIN_MIN ... GPT_GPIO_BANK0_PIN_MAX:
		*bank_num = 0;
		*bank_pin_num = pin_num;
		break;
	case GPT_GPIO_BANK1_PIN_MIN ... GPT_GPIO_BANK1_PIN_MAX:
		*bank_num = 1;
		*bank_pin_num = pin_num - GPT_GPIO_BANK1_PIN_MIN;
		break;
	case GPT_GPIO_BANK2_PIN_MIN ... GPT_GPIO_BANK2_PIN_MAX:
		*bank_num = 2;
		*bank_pin_num = pin_num - GPT_GPIO_BANK2_PIN_MIN;
		break;
	case GPT_GPIO_BANK3_PIN_MIN ... GPT_GPIO_BANK3_PIN_MAX:
		*bank_num = 3;
		*bank_pin_num = pin_num - GPT_GPIO_BANK3_PIN_MIN;
		break;
	case GPT_GPIO_BANK4_PIN_MIN ... GPT_GPIO_BANK4_PIN_MAX:
		*bank_num = 4;
		*bank_pin_num = pin_num - GPT_GPIO_BANK4_PIN_MIN;
		break;
	case GPT_GPIO_BANK5_PIN_MIN ... GPT_GPIO_BANK5_PIN_MAX:
		*bank_num = 5;
		*bank_pin_num = pin_num - GPT_GPIO_BANK5_PIN_MIN;
		break;
	case GPT_GPIO_BANK6_PIN_MIN ... GPT_GPIO_BANK6_PIN_MAX:
		*bank_num = 6;
		*bank_pin_num = pin_num - GPT_GPIO_BANK6_PIN_MIN;
		break;
	case GPT_GPIO_BANK7_PIN_MIN ... GPT_GPIO_BANK7_PIN_MAX:
		*bank_num = 7;
		*bank_pin_num = pin_num - GPT_GPIO_BANK7_PIN_MIN;
		break;
	default:
		WARN(true, "invalid GPIO pin number: %u", pin_num);
		*bank_num = 0;
		*bank_pin_num = 0;
		break;
	}
}

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
	unsigned int bank_num, bank_pin_num;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);

	gpt_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num);

	data = readl(gpio->base_addr +
			     GPT_GPIO_DATA(bank_num));

	return (data >> bank_pin_num) & 1;
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
	unsigned int reg_offset, bank_num, bank_pin_num;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);
	unsigned int  tmp ;

	gpt_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num);

	reg_offset = GPT_GPIO_DATA(bank_num);
	tmp = readl(gpio->base_addr + reg_offset);

	if(state)
	{
		state = tmp | (BIT(bank_pin_num));
	}else{
		state = tmp & (~BIT(bank_pin_num));
	}

	writel(state , gpio->base_addr + reg_offset);


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
	unsigned int bank_num, bank_pin_num;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);

	gpt_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num);

	/* set the bit in direction mode reg to set the pin as input */

	reg = readl(gpio->base_addr + GPT_GPIO_DIRCTION(bank_num));
	reg |= BIT(bank_pin_num);
	writel(reg, gpio->base_addr + GPT_GPIO_DIRCTION(bank_num));

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
	unsigned int bank_num, bank_pin_num;
	struct gpt_gpio *gpio = container_of(chip, struct gpt_gpio, chip);

	gpt_gpio_get_bank_pin(pin, &bank_num, &bank_pin_num);

	/* clear the GPIO pin as output */
	reg = readl(gpio->base_addr + GPT_GPIO_DIRCTION(bank_num));
	reg &= ~BIT(bank_pin_num);

	writel(reg, gpio->base_addr + GPT_GPIO_DIRCTION(bank_num));

	/* set the state of the pin */
	gpt_gpio_set_value(chip, pin, state);
	return 0;
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
	unsigned int device_pin_num, bank_num, bank_pin_num;
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(irq_data);
	unsigned int tmp ;

	device_pin_num = irq_data->hwirq;
	gpt_gpio_get_bank_pin(device_pin_num, &bank_num, &bank_pin_num);

	/* 0 disable */
	tmp = readl(gpio->base_addr + GPT_GPIO_CTL(bank_num));
	tmp &= ~BIT(bank_pin_num);

	/*enable register*/
	writel(tmp, gpio->base_addr + GPT_GPIO_INT_ENABLE(bank_num));
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
	unsigned int device_pin_num, bank_num, bank_pin_num;
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(irq_data);
	unsigned int tmp ;
	unsigned int clear_bit =0;

	device_pin_num = irq_data->hwirq;
	gpt_gpio_get_bank_pin(device_pin_num, &bank_num, &bank_pin_num);

	/* 1 enable */
	tmp = readl(gpio->base_addr + GPT_GPIO_CTL(bank_num));
	tmp |= BIT(bank_pin_num);

	writel(tmp, gpio->base_addr + GPT_GPIO_INT_ENABLE(bank_num));
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
	unsigned int device_pin_num, bank_num, bank_pin_num;
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(irq_data);

	device_pin_num = irq_data->hwirq;
	gpt_gpio_get_bank_pin(device_pin_num, &bank_num, &bank_pin_num);

	int_type = readl(gpio->base_addr +
				 GPT_GPIO_INTTYPE(bank_num));
	int_rhl = readl(gpio->base_addr +
				GPT_GPIO_R_HL(bank_num));
	int_fll = readl(gpio->base_addr +
				GPT_GPIO_F_LL(bank_num));

	/*
	 * based on the type requested, configure the INT_TYPE, INT_POLARITY
	 * and INT_ANY registers
	 */
	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		int_type |= BIT(bank_pin_num);
		int_rhl |= BIT(bank_pin_num);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		int_type |= BIT(bank_pin_num);
		int_fll |= BIT(bank_pin_num);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		int_type |= BIT(bank_pin_num);
		int_rhl |= BIT(bank_pin_num);
		int_fll |= BIT(bank_pin_num);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		int_type &= ~BIT(bank_pin_num);
		int_rhl |= BIT(bank_pin_num);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		int_type &= ~BIT(bank_pin_num);
		int_fll |= BIT(bank_pin_num);
		break;
	default:
		return -EINVAL;
	}

	SP_PRINTK("%s,int_type = %x\n",__func__,int_type);
	SP_PRINTK("%s,int_rhl = %x\n",__func__,int_rhl);
	SP_PRINTK("%s,int_fll = %x\n",__func__,int_fll);

	writel(int_type,
		       gpio->base_addr + GPT_GPIO_INTTYPE(bank_num));
	writel(int_rhl,
		       gpio->base_addr + GPT_GPIO_R_HL(bank_num));
	writel(int_fll,
		       gpio->base_addr + GPT_GPIO_F_LL(bank_num));

	return 0;
}

static int gpt_gpio_set_wake(struct irq_data *data, unsigned int on)
{
	struct gpt_gpio *gpio = irq_data_get_irq_chip_data(data);

	SP_PRINTK("func:%s\n",__func__);
	irq_set_irq_wake(gpio->irq, on);

	return 0;
}

void gpt_gpio_irq_disable(struct irq_data *data)
{
}

/* irq chip descriptor */
static struct irq_chip gpt_gpio_irqchip = {
	.name		= DRIVER_NAME,
	.irq_enable	= gpt_gpio_irq_enable,
	.irq_disable	= gpt_gpio_irq_disable,
	.irq_mask	= gpt_gpio_irq_mask,
	.irq_unmask	= gpt_gpio_irq_unmask,
	.irq_set_type	= gpt_gpio_set_irq_type,
	.irq_set_wake	= gpt_gpio_set_wake,
	.flags		= IRQCHIP_EOI_THREADED | IRQCHIP_EOI_IF_HANDLED |
			  IRQCHIP_MASK_ON_SUSPEND,
};


static void gpt_gpio_handle_bank_irq(struct gpt_gpio *gpio, 
					unsigned int bank_num,
					unsigned long pending)
{
	struct irq_domain *irqdomain = gpio->chip.irqdomain;
	int offset;

	SP_PRINTK("func:%s\n",__func__);

	if (!pending)
		return;

	for_each_set_bit(offset, &pending, 8) {

		unsigned int gpio_irq;
		gpio_irq = irq_find_mapping(irqdomain, bank_num * 8 + offset);
		generic_handle_irq(gpio_irq);
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
static struct gpt_gpio *g_gpio;
static void gpt_gpio_irqhandler(unsigned int irq, struct irq_desc *desc)
{
	u32 int_sts, int_enb;
	unsigned int bank_num;
	struct gpt_gpio *gpio = irq_get_handler_data(irq);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	SP_PRINTK("TEST func:%s, irq = %d\n",__func__,irq);
	printk("------------[gke]---%s---%d---\n", __func__, __LINE__);
	bank_num = desc->irq_data.hwirq;
	SP_PRINTK("TEST func:%s, bank_num = %d\n",__func__,bank_num);
	chained_irq_enter(irqchip, desc);

	writel(GPIO_INT_CLR, gpio->base_addr +
					GPT_GPIO_INT_CLEAR(bank_num));

	int_sts = readl(gpio->base_addr +
					GPT_GPIO_USFR(bank_num));
	int_enb = readl(gpio->base_addr +
					GPT_GPIO_INT_ENABLE(bank_num));
	gpt_gpio_handle_bank_irq(gpio, bank_num,int_sts & int_enb);

	chained_irq_exit(irqchip, desc);
}


static int gpt_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static void gpt_gpio_free(struct gpio_chip *chip, unsigned offset)
{

}



//irqreturn_t irq_handle_gpio(int irq, struct irq_desc*irq_desc){
irqreturn_t irq_handle_gpio(int irq, void *irq_desc){
	int i = 0;

	struct gpt_gpio *gpio = g_gpio;

	writel(0x40, gpio->base_addr +GPT_GPIO_INT_CLEAR(0) );

	for(i = 0;i<20;i++){
		SP_PRINTK("OK, now i'm in irq_handle_gpio func\n");
	}

	return IRQ_HANDLED;

}


int polaris_request_irq(struct platform_device*pdev){

	int irq = 0 ;
	int ret = 0 ;
	long gpio = 488;
	struct gpio_desc* desc;
	int status;
	int flags = IRQF_TRIGGER_RISING;
	//int flags = IRQF_TRIGGER_FALLING;
	//int flags = IRQF_TRIGGER_LOW;
	//int flags = IRQF_TRIGGER_HIGH;

	desc = gpio_to_desc(gpio);
	if(!desc){
		SP_PRINTK("ERROR in gpio_to_desc");
	}
	//push to high
	gpiod_direction_output(desc,1);

	//set to input
	gpiod_direction_input(desc);

	//rquest irq
	status = gpiod_request(desc, "sys");
	if(status<0){
		SP_PRINTK("error in request gpio");
	}

	//gpio->irq
	irq = gpiod_to_irq(desc);
	SP_PRINTK("gpiod_to_irq irq = %d\n",irq);

	//devm_request_irq
	ret = devm_request_irq(&pdev->dev, irq, irq_handle_gpio, flags, " gpio_g ", NULL);
	if(0 == ret){
		SP_PRINTK("ok in devm__request_irq\n");
	}else{
		SP_PRINTK("error in devm_request_irq\n");
	}

	return  0 ;
}

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
	int ret, bank_num;
	struct gpt_gpio *gpio;
	struct gpio_chip *chip;
	struct device_node *dev = pdev->dev.of_node;
	int ngpio;
	int i,bs;
	int irq_num[8] ={0};
	int nirq;


	SP_PRINTK("func :gpt gpio probe\n");
	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	platform_set_drvdata(pdev, gpio);

	gpio->base_addr = of_iomap(dev,0);
	gpio_base_addr = gpio->base_addr;

	//res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//gpio->base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gpio->base_addr))
		return PTR_ERR(gpio->base_addr);
	SP_PRINTK("gpio->base_addr = %p\n",gpio->base_addr);

	if(of_property_read_u32(dev,"gpt,ngpio",&ngpio))
	{
		dev_err(&pdev->dev,"invalid gpt,ngpio");
	}

	if(of_property_read_u32(dev,"gpt,base",&bs))
	{
		dev_err(&pdev->dev,"invalid gpt,base");
	}

	/*enable all banks*/
	/*unmask all irqs*/
	for(bank_num = 0; bank_num < GPT_GPIO_MAX_BANK; bank_num++)
	{
		writel(GPIO_EN | GPIO_B0_EN | GPIO_COMP_EN, (void*)(gpio->base_addr + GPT_GPIO_CTL(bank_num)));
	 
		if(bank_num == 0){
		  /* config gpio1 input mode */
		  writel(readl((void*)(gpio->base_addr + GPT_GPIO_DIRCTION(bank_num))) | GPIO_INPUT, (void*)(gpio->base_addr + GPT_GPIO_DIRCTION(bank_num)));
		  /* config gpio1 edge interrupt */
		  writel(readl((void*)(gpio->base_addr + GPT_GPIO_INTTYPE(bank_num))) | GPIO_EDGE_INT, (void*)(gpio->base_addr + GPT_GPIO_INTTYPE(bank_num)));
		  /* config gpio1 rising edge interrupt*/
		  writel(readl((void*)(gpio->base_addr + GPT_GPIO_R_HL(bank_num))) | GPIO_RISING_EDGE, (void*)(gpio->base_addr + GPT_GPIO_R_HL(bank_num)));
		  /* config gpio1 interupt enable */
		  writel(readl((void*)(gpio->base_addr + GPT_GPIO_INT_ENABLE(bank_num))) | GPIO_INT_EN, (void*)(gpio->base_addr + GPT_GPIO_INT_ENABLE(bank_num)));
		}
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
	chip->base = -1;
	chip->ngpio = ngpio ;

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

	ret = gpiochip_irqchip_add(chip, &gpt_gpio_irqchip, 0,
				   handle_simple_irq, IRQ_TYPE_NONE);
		if (ret) {
			dev_err(&pdev->dev, "Failed to add irq chip\n");
			goto err_rm_gpiochip;
		}


	nirq = of_irq_count(dev);

	for(i = 0;i < nirq;i++)
	{

		//irq_num[i] vir number
		irq_num[i] = platform_get_irq(pdev, i);
		//irq_num[i] = irq_of_parse_and_map(dev,i);
		SP_PRINTK("irq_num = %x\n",irq_num[i]);

		if (irq_num[i] < 0) {
			dev_err(&pdev->dev, "invalid IRQ\n");
			return irq_num[i];
		}
//
//		irq_set_chained_handler(irq_num[i], gpt_gpio_irqhandler);
//		irq_set_handler_data(irq_num[i],chip);
//
		//it's ok too
		gpiochip_set_chained_irqchip(chip, &gpt_gpio_irqchip, irq_num[i],
				     gpt_gpio_irqhandler);
	}

//	ret = polaris_request_irq(pdev);

	return 0;

err_rm_gpiochip:
	gpiochip_remove(chip);

	return ret;
}

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
	{ .compatible = "gpt-gpio-0.0", },
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
postcore_initcall(gpt_gpio_init);

MODULE_AUTHOR("GPT Inc.");
MODULE_DESCRIPTION("gpt GPIO driver");
MODULE_LICENSE("GPL");
