#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/pinctrl/machine.h>
#include <linux/device.h>
#include <linux/err.h>

enum gpt_pin_num {
	GPT_VCAP1_D0=0, GPT_VCAP1_D1 , GPT_VCAP1_D2 , GPT_VCAP1_D3 , 
	GPT_VCAP1_D4 ,  GPT_VCAP1_D5 , GPT_VCAP1_D6 , GPT_VCAP1_D7 , 
	GPT_VCAP2_D0 ,  GPT_VCAP2_D1 , GPT_VCAP2_D2 , GPT_VCAP2_D3 ,
	GPT_VCAP2_D4 ,  GPT_VCAP2_D5 , GPT_VCAP2_D6 , GPT_VCAP2_D7 ,
	GPT_VOUT_D8  ,  GPT_VOUT_D9  , GPT_VOUT_D10 , GPT_VOUT_D11 ,
	GPT_VOUT_D12 ,  GPT_VOUT_D13 , GPT_VOUT_D14 , GPT_VOUT_D15 ,
	GPT_SPI0_CLK ,  GPT_SPI0_SSN , GPT_SPI0_MOSI, GPT_SPI0_MISO,
	GPT_SPI1_CLK ,  GPT_SPI1_SSN , GPT_SPI1_MOSI, GPT_SPI1_MISO,
	GPT_SPI2_CLK ,  GPT_SPI2_SSN , GPT_SPI2_MOSI, GPT_SPI2_MISO,
	GPT_UART1_TXD,  GPT_UART1_RXD, GPT_UART2_TXD, GPT_UART2_RXD,
	GPT_I2C1_SCL ,  GPT_I2C1_SDA , GPT_I2C2_SCL , GPT_I2C2_SDA ,
	GPT_GPIO1_D0 ,  GPT_GPIO1_D1 , GPT_GPIO1_D2 , GPT_GPIO1_D3 ,
	GPT_GPIO1_D4 ,  GPT_GPIO1_D5 , GPT_GPIO1_D6 , GPT_GPIO1_D7 
};

//#include "pinctrl-gpt.h"
static unsigned int gpt_pins[] = {
	GPT_VCAP1_D0 , GPT_VCAP1_D1 , GPT_VCAP1_D2 , GPT_VCAP1_D3 , 
	GPT_VCAP1_D4 , GPT_VCAP1_D5 , GPT_VCAP1_D6 , GPT_VCAP1_D7 , 
	GPT_VCAP2_D0 , GPT_VCAP2_D1 , GPT_VCAP2_D2 , GPT_VCAP2_D3 ,
	GPT_VCAP2_D4 , GPT_VCAP2_D5 , GPT_VCAP2_D6 , GPT_VCAP2_D7 ,
	GPT_VOUT_D8  , GPT_VOUT_D9  , GPT_VOUT_D10 , GPT_VOUT_D11 ,
	GPT_VOUT_D12 , GPT_VOUT_D13 , GPT_VOUT_D14 , GPT_VOUT_D15 ,
	GPT_SPI0_CLK , GPT_SPI0_SSN , GPT_SPI0_MOSI, GPT_SPI0_MISO,
	GPT_SPI1_CLK , GPT_SPI1_SSN , GPT_SPI1_MOSI, GPT_SPI1_MISO,
	GPT_SPI2_CLK , GPT_SPI2_SSN , GPT_SPI2_MOSI, GPT_SPI2_MISO,
	GPT_UART1_TXD, GPT_UART1_RXD, GPT_UART2_TXD, GPT_UART2_RXD,
	GPT_I2C1_SCL , GPT_I2C1_SDA , GPT_I2C2_SCL , GPT_I2C2_SDA ,
	GPT_GPIO1_D0 , GPT_GPIO1_D1 , GPT_GPIO1_D2 , GPT_GPIO1_D3 ,
	GPT_GPIO1_D4 , GPT_GPIO1_D5 , GPT_GPIO1_D6 , GPT_GPIO1_D7 
};

static const struct pinctrl_pin_desc gpt_desc_pins[] = {
	PINCTRL_PIN(GPT_VCAP1_D0,  "VCAP1_D0 "),
	PINCTRL_PIN(GPT_VCAP1_D1,  "VCAP1_D1 "),
	PINCTRL_PIN(GPT_VCAP1_D2,  "VCAP1_D2 "),
	PINCTRL_PIN(GPT_VCAP1_D3,  "VCAP1_D3 "),
	PINCTRL_PIN(GPT_VCAP1_D4,  "VCAP1_D4 "),
	PINCTRL_PIN(GPT_VCAP1_D5,  "VCAP1_D5 "),
	PINCTRL_PIN(GPT_VCAP1_D6,  "VCAP1_D6 "),
	PINCTRL_PIN(GPT_VCAP1_D7,  "VCAP1_D7 "),

	PINCTRL_PIN(GPT_VCAP2_D0,  "VCAP2_D0 "),
	PINCTRL_PIN(GPT_VCAP2_D1,  "VCAP2_D1 "),
	PINCTRL_PIN(GPT_VCAP2_D2,  "VCAP2_D2 "),
	PINCTRL_PIN(GPT_VCAP2_D3,  "VCAP2_D3 "),
	PINCTRL_PIN(GPT_VCAP2_D4,  "VCAP2_D4 "),
	PINCTRL_PIN(GPT_VCAP2_D5,  "VCAP2_D5 "),
	PINCTRL_PIN(GPT_VCAP2_D6,  "VCAP2_D6 "),
	PINCTRL_PIN(GPT_VCAP2_D7,  "VCAP2_D7 "),

	PINCTRL_PIN(GPT_VOUT_D8,   "VOUT_D8  "),
	PINCTRL_PIN(GPT_VOUT_D9,   "VOUT_D9  "),
	PINCTRL_PIN(GPT_VOUT_D10,  "VOUT_D10 "),
	PINCTRL_PIN(GPT_VOUT_D11,  "VOUT_D11 "),
	PINCTRL_PIN(GPT_VOUT_D12,  "VOUT_D12 "),
	PINCTRL_PIN(GPT_VOUT_D13,  "VOUT_D13 "),
	PINCTRL_PIN(GPT_VOUT_D14,  "VOUT_D14 "),
	PINCTRL_PIN(GPT_VOUT_D15,  "VOUT_D15 "),

	PINCTRL_PIN(GPT_SPI0_CLK,  "SPI0_CLK "),
	PINCTRL_PIN(GPT_SPI0_SSN,  "SPI0_SSN "),
	PINCTRL_PIN(GPT_SPI0_MOSI, "SPI0_MOSI"),
	PINCTRL_PIN(GPT_SPI0_MISO, "SPI0_MISO"),
	PINCTRL_PIN(GPT_SPI1_CLK,  "SPI1_CLK "),
	PINCTRL_PIN(GPT_SPI1_SSN,  "SPI1_SSN "),
	PINCTRL_PIN(GPT_SPI1_MOSI, "SPI1_MOSI"),
	PINCTRL_PIN(GPT_SPI1_MISO, "SPI1_MISO"),
	PINCTRL_PIN(GPT_SPI2_CLK,  "SPI2_CLK "),
	PINCTRL_PIN(GPT_SPI2_SSN,  "SPI2_SSN "),
	PINCTRL_PIN(GPT_SPI2_MOSI, "SPI2_MOSI"),
	PINCTRL_PIN(GPT_SPI2_MISO, "SPI2_MISO"),

	PINCTRL_PIN(GPT_UART1_TXD, "UART1_TXD"),
	PINCTRL_PIN(GPT_UART1_RXD, "UART1_RXD"),
	PINCTRL_PIN(GPT_UART2_TXD, "UART2_TXD"),
	PINCTRL_PIN(GPT_UART2_RXD, "UART2_RXD"),

	PINCTRL_PIN(GPT_I2C1_SCL,  "I2C1_SCL "),
	PINCTRL_PIN(GPT_I2C1_SDA,  "I2C1_SDA "),
	PINCTRL_PIN(GPT_I2C2_SCL,  "I2C2_SCL "),
	PINCTRL_PIN(GPT_I2C2_SDA,  "I2C2_SDA "),

	PINCTRL_PIN(GPT_GPIO1_D0,  "GPIO1_D0 "),
	PINCTRL_PIN(GPT_GPIO1_D1,  "GPIO1_D1 "),
	PINCTRL_PIN(GPT_GPIO1_D2,  "GPIO1_D2 "),
	PINCTRL_PIN(GPT_GPIO1_D3,  "GPIO1_D3 "),
	PINCTRL_PIN(GPT_GPIO1_D4,  "GPIO1_D4 "),
	PINCTRL_PIN(GPT_GPIO1_D5,  "GPIO1_D5 "),
	PINCTRL_PIN(GPT_GPIO1_D6,  "GPIO1_D6 "),
	PINCTRL_PIN(GPT_GPIO1_D7,  "GPIO1_D7 "),
};

/**
 * struct gpt_pin_group: represent group of pins of a pinmux function.
 * @name: name of the pin group, used to lookup the group.
 * @pins: the pins included in this group.
 * @num_pins: number of pins included in this group.
 * @func: the function number to be programmed when selected.
 */
struct gpt_pin_group {
	const char		*name;
	unsigned int		*pins;
	u8			num_pins;
	u8			func;
	u32			mask;
} pin_groups[] = {
	{"VCAP1_D0_7",  &gpt_pins[GPT_VCAP1_D0], 8, 0,        0x7},  //2:0 
	{"VCAP2_D0_7",  &gpt_pins[GPT_VCAP2_D0], 8, 0,       0x38},  //5:3
	{"VOUT_D8_15",  &gpt_pins[GPT_VOUT_D8],  8, 0,      0x1c0},  //8:6
	{"SPI0",        &gpt_pins[GPT_SPI0_CLK], 4, 0,      0xe00},  //11:9
	{"SPI1",        &gpt_pins[GPT_SPI1_CLK], 4, 0,     0x7000},  //14:12
	{"SPI2",        &gpt_pins[GPT_SPI2_CLK], 4, 0,    0x38000},  //17:15
	{"UART1",       &gpt_pins[GPT_UART1_TXD],2, 0,   0x1c0000},  //20:18
	{"UART2",       &gpt_pins[GPT_UART2_TXD],2, 0,   0xe00000},  //23:21
	{"I2C1",        &gpt_pins[GPT_I2C1_SCL], 2, 0,  0x7000000},  //26:24
	{"I2C2",        &gpt_pins[GPT_I2C2_SCL], 2, 0, 0x38000000},  //29:27
};

/**
 * struct gpt_pmx_functions: represent a pingroup function.
 * @name       : name of the pin function, used to lookup the function.
 * @num_groups : number of groups included in @groups.
 * @func       : func value of groups included in @groups.
 * @mask       : mask of io_config register for pins.
 */
struct gpt_pmx_functions {
	const char	*name;
	const char	**groups;
	u8		num_groups;
	u32		func;
	u32		mask;
};

/**
 * struct gpt_pci - pinctrl ctrl info
 *
 * pmx_functions: list of pin groups available to the driver
 * pctl_dev : pinctrl device instance
 * pctl     : pin controller descriptor registered with the pinctrl subsystem
 *nr_functions  : functions count
 */
struct gpt_pci {
	struct pinctrl_dev *pctl_dev;
	struct pinctrl_desc pctl;
	void __iomem *regs;

	struct gpt_pmx_functions *pmx_functions;
	unsigned int nr_functions;
	struct gpt_pin_group *pingroups;
	unsigned int nr_groups;
	struct  gpt_of_pinfunc *pinfuncs;
	unsigned int pinfuncnt;
	struct list_head gpiofuncs;
	
	struct device *dev;
	struct mutex mutex;
};

/* pinctrl info read */
static inline u32 pci_read(struct gpt_pci *pci)
{
	return readl(pci->regs);
}

/* pinctrl info write */
static inline void pci_write(struct gpt_pci *pci, u32 val)
{
	writel(val, pci->regs);
}

static int gpt_pinctrl_get_groups_count(struct pinctrl_dev *pctl_dev)
{

	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	return pci->nr_groups;
}

static const char *gpt_pinctrl_get_group_name(struct pinctrl_dev *pctl_dev,
						 unsigned int group)
{
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	return pci->pingroups[group].name;
}

static int gpt_pinctrl_get_group_pins(struct pinctrl_dev *pctl_dev,
					 unsigned int group,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	*pins     = pci->pingroups[group].pins;
	*num_pins = pci->pingroups[group].num_pins;
	return 0;
}

static void gpt_pinctrl_dt_free_map(struct pinctrl_dev *pctl_dev,
			       struct pinctrl_map *map, unsigned num_maps)
{
	kfree(map);
}

#ifdef CONFIG_DEBUG_FS
static void gpt_pinctrl_pin_dbg_show(struct pinctrl_dev *pctl_dev,
					struct seq_file *s,
					unsigned int offset)
{
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	seq_printf(s, " %s", dev_name(pci->dev));
}
#endif

static int dt_subnode_to_map(struct gpt_pci *pci, struct device_node *np, struct pinctrl_map **map, 
							unsigned *num_maps)
{
	int ret;
	const char *group;
	const char *function;
	struct pinctrl_map *new_map;

	new_map = krealloc(*map, sizeof(*new_map), GFP_KERNEL);
	if (!new_map) {
		dev_err(pci->dev, "krealloc(map) failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_string(np, "group", &group);
	if (ret != 0)
		return -1;
		
	function = np->name;
	*map = new_map;

	(*map)[*num_maps].type = PIN_MAP_TYPE_MUX_GROUP;
	(*map)[*num_maps].data.mux.function = function;
	(*map)[*num_maps].data.mux.group = group;
	dev_dbg(pci->dev, "%s pinctrl function:%s, group :%s\n",__func__,  
		(*map)[*num_maps].data.mux.function, (*map)[*num_maps].data.mux.group);

	(*num_maps)++;
	
	return 0;

}

static int gpt_pinctrl_dt_node_to_map(struct pinctrl_dev *pctl_dev,
					 struct device_node *np_config,
					 struct pinctrl_map **map,
					 unsigned int *num_maps)
{
	int ret;
	struct device_node *np;
	unsigned int reserved_maps;

	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;
	
	if (!of_get_child_count(np_config)) {
		return dt_subnode_to_map(pci, np_config, map, num_maps);
	}

	for_each_child_of_node(np_config, np) {
		ret = dt_subnode_to_map(pci, np, map, num_maps);
		if (ret < 0) {
			gpt_pinctrl_dt_free_map(pctl_dev, *map, *num_maps);
			return ret;
		}
	}

	return 0;
}

static struct pinctrl_ops gpt_pinctrl_ops = {
	.get_groups_count	= gpt_pinctrl_get_groups_count,
	.get_group_name		= gpt_pinctrl_get_group_name,
	.get_group_pins		= gpt_pinctrl_get_group_pins,
#ifdef CONFIG_DEBUG_FS
	.pin_dbg_show		= gpt_pinctrl_pin_dbg_show,
#endif
	.dt_node_to_map		= gpt_pinctrl_dt_node_to_map,
	.dt_free_map		= gpt_pinctrl_dt_free_map,
};

static int gpt_pinctrl_get_funcs_count(struct pinctrl_dev *pctl_dev)
{
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	return pci->nr_functions;
}

static const char *gpt_pinctrl_get_func_name(struct pinctrl_dev *pctl_dev,
						unsigned int function)
{
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	return pci->pmx_functions[function].name;
}

static int gpt_pinctrl_get_func_groups(struct pinctrl_dev *pctl_dev,
					  unsigned int function,
					  const char * const **groups,
					  unsigned int *const num_groups)
{
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);
	*groups = pci->pmx_functions[function].groups;
	*num_groups = pci->pmx_functions[function].num_groups;

	return 0;
};

static int gpt_pinctrl_set_mux(struct pinctrl_dev *pctl_dev,
				  unsigned int function, unsigned int group)
{
	u32 val, func, mask;
	struct gpt_pci *pci = pinctrl_dev_get_drvdata(pctl_dev);

	func = pci->pmx_functions[function].func;
	mask = pci->pingroups[group].mask;

	mutex_lock(&pci->mutex);
	
	val = pci_read(pci);
	val = func | (val & ~mask);
	pci_write(pci, val);
	dev_dbg(pci->dev, "%s function:%d, group:%d, func:0x%x, mask:0x%x, val:0x%x\n", 
		__func__, function, group, func, mask, val);

	pci->pingroups[group].func = func;
	mutex_unlock(&pci->mutex);

	return 0;
}

#if 0
static int gpt_pinctrl_gpio_request_enable(struct pinctrl_dev *pctl_dev,
					      struct pinctrl_gpio_range *range,
					      unsigned int offset)
{
	return 0;
}

static void gpt_pinctrl_gpio_disable_free(struct pinctrl_dev *pctl_dev,
				    struct pinctrl_gpio_range *range,
				     unsigned int offset)
{
}
#endif
static const struct pinmux_ops gpt_pinmux_ops = {
	.get_functions_count = gpt_pinctrl_get_funcs_count,
	.get_function_name = gpt_pinctrl_get_func_name,
	.get_function_groups = gpt_pinctrl_get_func_groups,
	.set_mux = gpt_pinctrl_set_mux,
	//.gpio_request_enable = gpt_pinctrl_gpio_request_enable,
	//.gpio_disable_free = gpt_pinctrl_gpio_disable_free,
};

static int gpt_pinctrl_create_function(struct platform_device *pdev,
				struct device_node *func_np,
				struct gpt_pmx_functions *func)
{
	int npins;
	int ret;
	int i;

	ret = of_property_read_u32(func_np, "mux-value", &func->func);
	func->name = func_np->name;
	
	npins = of_property_count_strings(func_np, "group");
	if (npins < 1) {
		dev_err(&pdev->dev, "invalid pin list in %s node", func->name);
		return -EINVAL;
	}

	func->groups = devm_kzalloc(&pdev->dev, npins * sizeof(char *), GFP_KERNEL);
	if (!func->groups)
		return -ENOMEM;

	for (i = 0; i < npins; ++i) {
		const char *gname;

		ret = of_property_read_string_index(func_np, "group", i, &gname);
		if (ret) {
			dev_err(&pdev->dev, "failed to read pin name %d from %s node\n",
				i, func_np->name);
			return ret;
		}
		func->groups[i] = gname;
	}

	func->num_groups = npins;
	return 1;

}

static int gpt_pinctrl_parse_dt(struct platform_device *pdev, struct gpt_pci *pci)
{
	u32 ret;
	struct device_node *dev_np = pdev->dev.of_node;
	struct device_node *cfg_np;
	struct gpt_pmx_functions *functions, *func;
	int func_cnt = 0;

	pci->pingroups = pin_groups;
	pci->nr_groups = ARRAY_SIZE(pin_groups);
	/*
	 * Iterate over all the child nodes of the pin controller node
	 * and create pin groups and pin function lists.
	 */
	for_each_child_of_node(dev_np, cfg_np) {
		struct device_node *func_np;

		if (!of_get_child_count(cfg_np)) {
			if (!of_find_property(cfg_np,
			    "mux-value", NULL))
				continue;
			++func_cnt;
			continue;
		}

		for_each_child_of_node(cfg_np, func_np) {
			if (!of_find_property(func_np,
			    "mux-value", NULL))
				continue;
			++func_cnt;
		}
	}

	functions = devm_kzalloc(&pdev->dev, func_cnt * sizeof(*functions), GFP_KERNEL);
	if (!functions) {
		dev_err(pci->dev, "failed to allocate memory for function list\n");
		return -EINVAL;
	}

	func = functions;

	func_cnt = 0;
	for_each_child_of_node(dev_np, cfg_np) {
		struct device_node *func_np;
		if (!of_get_child_count(cfg_np)) {
			ret = gpt_pinctrl_create_function(pdev,cfg_np, func);
			if (ret < 0)
				return ret;
			if (ret > 0) {
				++func;
				++func_cnt;
			}
			continue;
		}

		for_each_child_of_node(cfg_np, func_np) {
			ret = gpt_pinctrl_create_function(pdev,func_np, func);
			if (ret < 0)
				return ret;
			if (ret > 0) {
				++func;
				++func_cnt;
			}
			continue;

		}			

	}

	pci->pmx_functions = functions;
	pci->nr_functions = func_cnt;

	return 0;
}

static int gpt_pinctrl_probe(struct platform_device *pdev)
{
	struct gpt_pci *pci;
	struct pinctrl_desc *ctrldesc;
	struct resource *resource;

	pci = devm_kzalloc(&pdev->dev, sizeof(struct gpt_pci), GFP_KERNEL);
	if(!pci)
		return -ENOMEM;

	ctrldesc = &pci->pctl;
	ctrldesc->pins = gpt_desc_pins,
	ctrldesc->npins = ARRAY_SIZE(gpt_desc_pins),
	ctrldesc->name = "gpt-pinctrl";
	ctrldesc->owner = THIS_MODULE;
	ctrldesc->pctlops = &gpt_pinctrl_ops;
	ctrldesc->pmxops = &gpt_pinmux_ops;

	pci->dev = &pdev->dev;
	mutex_init(&pci->mutex);
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pci->regs = ioremap(resource->start, resource_size(resource));

	gpt_pinctrl_parse_dt(pdev,pci);
	pci->pctl_dev = pinctrl_register(ctrldesc, &pdev->dev, pci);
	if(!pci->pctl_dev) {
		dev_err(&pdev->dev, "Couldn`t register pinctrl driver\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, pci);
	dev_info(&pdev->dev, "Pinctrl subsystem successfully registered\n");
	return 0;
}

static int gpt_pinctrl_remove(struct platform_device *pdev)
{
	struct gpt_pci *pci = platform_get_drvdata(pdev);

	if (!pci)
		return 0;

	iounmap(pci->regs);
	pinctrl_unregister(pci->pctl_dev);

	return 0;
}

static const struct of_device_id gpt_pinctrl_of_match[] = {
	{ .compatible = "gpt,gpt_pinctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, gpt_pinctrl_of_match);

static struct platform_driver gpt_pinctrl_pdrv = {
	.driver = {
		.name	= "gpt_pinctrl",
		.of_match_table = of_match_ptr(gpt_pinctrl_of_match),
		.owner = THIS_MODULE,
	},
	.probe	= gpt_pinctrl_probe,
	.remove	= gpt_pinctrl_remove,
};

static int __init gpt_pinctrl_init(void)
{
	return platform_driver_register(&gpt_pinctrl_pdrv);
}
subsys_initcall(gpt_pinctrl_init);

MODULE_DESCRIPTION("GPT pinctrl driver");
MODULE_LICENSE("GPL v2");
