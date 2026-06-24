// SPDX-License-Identifier: GPL-2.0
/*
 * The driver DT usage ought to look something like this.
 * Note that there needs to be added a dt-bindings header file with the pinmux
 * macros used below, but they're just to give you an idea.
 *
 *   pinctrl: pinctrl@31004000 {
 *       compatible = "adi,sc598-pinctrl";
 *       adi,pads-syscon = <&pads_syscon>
 *       #address-cells = <1>;
 *       #size-cells = <1>;
 *       ranges;
 *
 *       gpioa: gpio@31004080 {
 *           reg = <0x31004080 0x40>
 *           gpio-controller;
 *           #gpio-cells = <2>;
 *           gpio-ranges = <&pinctrl 0 0 16>
 *           interrupt-controller;
 *           #interrupt-cells = <2>;
 *           interrupts-extended = <&pint0 16>
 *       };
 *
 *       gpiob: gpio@310040c0 {
 *           reg = <0x310040c0 0x40>;
 *           gpio-controller;
 *           #gpio-cells = <2>;
 *           gpio-ranges = <&pinctrl 0 16 16>
 *           interrupt-controller;
 *           #interrupt-cells = <2>;
 *           interrupts-extended = <&pint0 0>
 *       };
 *
 *       etc...
 *   };
 *
 *   &uart2 {
 *       pinctrl-names = "default";
 *       pinctrl-0 = <&uart2_pins>;
 *   };
 *
 *   &pinctrl {
 *       uart2_pins: uart2 {
 *           pinmux = <ADSP_SC598_PB_11__UART2_RX>, <ADSP_SC598_PB12__UART2_TX>;
 *           bias-pull-up;
 *       };
 *
 *       // configuring just DS for a particular pin
 *       pb05_cfg: pb05-cfg {
 *           pins = "PB_05";
 *           drive-strength = <1>; // or vendor property? how to interprete val?
 *       };
 *   };
 */

#include <linux/bitfield.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"
#include "../pinmux.h"

/* TODO: Add header with mux values, this is for decoding them */
#define ADSP_PINMUX_PIN(v) FIELD_GET(0xffffff00, v)
#define ADSP_PINMUX_ALT(v) FIELD_GET(0x000000ff, v)

#define ADSP_PINS_PER_PORT 16

/* TODO check reg addresses */
#define ADSP_PORT_FER		0x00
#define ADSP_PORT_FER_SET	0x04
#define ADSP_PORT_FER_CLEAR	0x08
#define ADSP_PORT_DATA		0x0c
#define ADSP_PORT_DATA_SET	0x10
#define ADSP_PORT_DATA_CLEAR	0x14
#define ADSP_PORT_DIR		0x18
#define ADSP_PORT_DIR_SET	0x1c
#define ADSP_PORT_DIR_CLEAR	0x20
#define ADSP_PORT_INEN		0x24
#define ADSP_PORT_INEN_SET	0x28
#define ADSP_PORT_INEN_CLEAR	0x2c
#define ADSP_PORT_MUX		0x30
#define ADSP_PORT_MUX_BITS	2
#define ADSP_PORT_MUX_MASK	GENMASK(ADSP_PORT_MUX_BITS - 1, 0)

struct adsp_pinctrl;

struct adsp_port {
	unsigned int index;
	unsigned int ngpio;
	unsigned int pin_base;
	struct adsp_pinctrl *pc;
	struct device_node *np;
	void __iomem *regs;
	struct gpio_chip gc;
	spinlock_t lock;
};

struct adsp_pinctrl {
	struct device *dev;
	const struct adsp_pinctrl_info *info;
	struct regmap *pads;
	struct adsp_port *ports;
	unsigned int nports;
	struct pinctrl_pin_desc *pins;
	unsigned int npins;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pctldesc;
};

enum adsp_pin_bias {
	ADSP_PIN_BIAS_UNKNOWN,
	ADSP_PIN_BIAS_DISABLE,
	ADSP_PIN_BIAS_PULL_UP,
	ADSP_PIN_BIAS_PULL_DOWN,
};

#define ADSP_PIN_NAME_LEN sizeof("PA_00")

static const char * const adsp_functions[] = { "alt0", "alt1", "alt2", "alt3" };

/* Per-SoC info */

struct adsp_pinctrl_info {
	unsigned int max_ports;
	unsigned int port_pue_reg;
	unsigned int port_pud_reg;
	unsigned int port_pde_reg;
};

struct adsp_pinctrl_info sc598_pinctrl_info = {
	.max_ports = 9,
	.port_pue_reg = 0x98, /* starting from 0x31004600 in syscon */
	.port_pde_reg = 0xc4,
};
// sc598-64
/* sc596 sc598 - port i, (i has 7 pins)
           all pads
   sc592 sc594 - ditto
   sc58x       - port g (though some packages have fewer!)
           dai0,1_IE and PCFG0 only
   sc57x       - ditto
           PADS_DAI[n]_PUDDAIx Pull-Up Disable
           PADS_DAI[n]_PUEDAIx Pull-Up Enable
           PADS_PCFG0Peripheral PAD Configuration0 Register
           PADS_PORTS_DSVoltage Domain Control Register
           PADS_PORT[n]_PUDPORTx Pull-Up Disable
           PADS_PORT[n]_PUEPORTx Pull-Up Enable

point to regmap field inits for each SoC type
*/

/* Helpers */

static inline struct adsp_port *
adsp_pinctrl_pin_to_port(struct adsp_pinctrl *pc, unsigned int pin)
{
	return pc->pins[pin].drv_data;
}

static inline unsigned int adsp_pinctrl_pin_to_gpio(struct adsp_pinctrl *pc,
						    unsigned int pin)
{
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	return pin - port->pin_base;
}

/* GPIO ops */

static int adsp_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	return !!(readl(port->regs + ADSP_PORT_DATA) & BIT(gpio));
}

static int adsp_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	/* TODO: route through adsp_gpio_apply() once open-drain lands;
	 * for now a plain push-pull write
	 */
	if (val)
		writel(BIT(gpio), port->regs + ADSP_PORT_DATA_SET);
	else
		writel(BIT(gpio), port->regs + ADSP_PORT_DATA_CLEAR);

	return 0;
}
static int adsp_gpio_get_direction(struct gpio_chip *gc, unsigned int gpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);

	return (readl(port->regs + ADSP_PORT_DIR) & BIT(gpio)) ?
		       GPIO_LINE_DIRECTION_OUT :
		       GPIO_LINE_DIRECTION_IN;
}

static int adsp_gpio_direction_input(struct gpio_chip *gc, unsigned int gpio)
{
	struct adsp_port *port = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	writel(BIT(gpio), port->regs + ADSP_PORT_DIR_CLEAR);
	writel(BIT(gpio), port->regs + ADSP_PORT_INEN_SET);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static int adsp_gpio_direction_output(struct gpio_chip *gc, unsigned int gpio,
				      int val)
{
	struct adsp_port *port = gpiochip_get_data(gc);
	unsigned long flags;
	int ret;

	ret = adsp_gpio_set(gc, gpio, val);
	if (ret)
		return ret;

	spin_lock_irqsave(&port->lock, flags);
	writel(BIT(gpio), port->regs + ADSP_PORT_INEN_CLEAR);
	writel(BIT(gpio), port->regs + ADSP_PORT_DIR_SET);
	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

/* pinconf ops */

static enum adsp_pin_bias adsp_pinconf_get_pin_bias(struct adsp_pinctrl *pc,
						    unsigned int pin)
{
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	const struct adsp_pinctrl_info *info = pc->info;
	struct regmap *pads = pc->pads;
	unsigned int reg_offset = port->index * regmap_get_reg_stride(pads);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);
	unsigned int pue = 0;
	unsigned int pud = 0;
	unsigned int pde = 0;
	bool pullup = false;
	bool pulldn = false;
	unsigned int val;

	if (info->port_pue_reg)
		pue = info->port_pue_reg + reg_offset;
	if (info->port_pud_reg)
		pud = info->port_pud_reg + reg_offset;
	if (info->port_pde_reg)
		pde = info->port_pde_reg + reg_offset;

	if (!pue && !pud && !pde)
		return ADSP_PIN_BIAS_UNKNOWN;

	if (pue) {
		regmap_read(pads, pue, &val);
		pullup = !!(val & BIT(gpio));
	}

	if (pud) {
		/* PUD takes precedence over PUE when it is present */
		regmap_read(pads, pud, &val);
		pullup = !(val & BIT(gpio));
	}

	if (pde) {
		regmap_read(pads, pde, &val);
		pulldn = !!(val & BIT(gpio));
	}

	if (pullup && !pulldn)
		return ADSP_PIN_BIAS_PULL_UP;
	else if (pulldn && !pullup)
		return ADSP_PIN_BIAS_PULL_DOWN;
	else if (!pullup && !pulldn)
		return ADSP_PIN_BIAS_DISABLE;

	/* We should never get here */
	return ADSP_PIN_BIAS_UNKNOWN;
}

static int adsp_pinconf_set_pin_bias(struct adsp_pinctrl *pc, unsigned int pin,
				     enum adsp_pin_bias bias)
{
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	const struct adsp_pinctrl_info *info = pc->info;
	struct regmap *pads = pc->pads;
	unsigned int reg_offset = port->index * regmap_get_reg_stride(pads);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);
	bool pullup = bias == ADSP_PIN_BIAS_PULL_UP;
	bool pulldn = bias == ADSP_PIN_BIAS_PULL_DOWN;
	unsigned int pue = 0;
	unsigned int pud = 0;
	unsigned int pde = 0;

	/*
	 * Not all SoCs in the family support setting the bin bias. The three
	 * cases are:
	 *
	 *  1. No support at all
	 *  2. Pull-up control only
	 *  3. Pull-up and pull-down control
	 *
	 * For pull-up control, sometimes it is split across two registers, PUE
	 * (enable) and PUD (disable), where PUD takes precedence. The driver
	 * assumes that SoC info register addresses are valid when they are
	 * nonzero to determine the level of control available. Accordingly, an
	 * error is only returned when a requested bias configuration cannot be
	 * provided by the hardware.
	 */
	if (info->port_pue_reg)
		pue = info->port_pue_reg + reg_offset;
	if (info->port_pud_reg)
		pud = info->port_pud_reg + reg_offset;
	if (info->port_pde_reg)
		pde = info->port_pde_reg + reg_offset;

	if (!pue && !pud && !pde)
		return -ENOTSUPP;

	/* Disable bias */
	if (!pullup) {
		if (pud)
			regmap_set_bits(pads, pud, BIT(gpio));

		if (pue)
			regmap_clear_bits(pads, pue, BIT(gpio));
	}

	if (!pulldn) {
		if (pde)
			regmap_clear_bits(pads, pde, BIT(gpio));
	}

	/* Enable bias */
	if (pullup) {
		if (pud)
			regmap_clear_bits(pads, pud, BIT(gpio));

		if (pue)
			regmap_set_bits(pads, pue, BIT(gpio));
		else
			return -ENOTSUPP;
	}

	if (pulldn) {
		if (pde)
			regmap_set_bits(pads, pde, BIT(gpio));
		else
			return -ENOTSUPP;
	}

	return 0;
}

static int adsp_pinconf_pin_config_get(struct pinctrl_dev *pctldev,
				       unsigned int pin, unsigned long *config)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg = pinconf_to_config_argument(*config);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN: {
		enum adsp_pin_bias bias = adsp_pinconf_get_pin_bias(pc, pin);

		if (bias == ADSP_PIN_BIAS_DISABLE &&
		    param == PIN_CONFIG_BIAS_DISABLE)
			break;
		else if (bias == ADSP_PIN_BIAS_PULL_UP &&
			 param == PIN_CONFIG_BIAS_PULL_UP)
			break;
		else if (bias == ADSP_PIN_BIAS_PULL_DOWN &&
			 param == PIN_CONFIG_BIAS_PULL_DOWN)
			break;
		else
			return -ENOTSUPP;
	}
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int adsp_pinconf_pin_config_set(struct pinctrl_dev *pctldev,
				       unsigned int pin, unsigned long *configs,
				       unsigned int num_configs)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	int i;
	int ret = 0;

	for (i = 0; i < num_configs; i++) {
		enum pin_config_param param = pinconf_to_config_param(configs[i]);
		u32 arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			ret = adsp_pinconf_set_pin_bias(pc, pin,
							ADSP_PIN_BIAS_DISABLE);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			ret = adsp_pinconf_set_pin_bias(pc, pin,
							ADSP_PIN_BIAS_PULL_UP);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			ret = adsp_pinconf_set_pin_bias(pc, pin,
							ADSP_PIN_BIAS_PULL_DOWN);
			break;
		default:
			ret = -ENOTSUPP;
			goto out;
		}
	}

 out:
	return ret;
}

static const struct pinconf_ops adsp_confops = {
	.is_generic = true,
	.pin_config_get = adsp_pinconf_pin_config_get,
	.pin_config_set = adsp_pinconf_pin_config_set,
};

static int adsp_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			       unsigned int group)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, group);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, group);
	u32 shift = ADSP_PORT_MUX_BITS * gpio;
	u32 mask = ADSP_PORT_MUX_MASK << shift;
	u32 field = func << shift;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* Set alternative mode mux value */
	val = readl(port->regs + ADSP_PORT_MUX);
	val &= ~mask;
	val |= field;
	writel(val, port->regs + ADSP_PORT_MUX);

	/* Enable alternate function on the pin */
	writel(BIT(gpio), port->regs + ADSP_PORT_FER_SET);

	spin_unlock_irqrestore(&port->lock, flags);

	return 0;
}

static int adsp_pinmux_gpio_request_enable(struct pinctrl_dev *pctldev,
					   struct pinctrl_gpio_range *range,
					   unsigned int pin)
{
	struct adsp_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	struct adsp_port *port = adsp_pinctrl_pin_to_port(pc, pin);
	unsigned int gpio = adsp_pinctrl_pin_to_gpio(pc, pin);

	/* Disable alternate function on the pin */
	writel(BIT(gpio), port->regs + ADSP_PORT_FER_CLEAR);

	return 0;
}

static const struct pinmux_ops adsp_pmxops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux = adsp_pinmux_set_mux,
	.gpio_request_enable = adsp_pinmux_gpio_request_enable,
	.strict = true,
};

static int adsp_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				       struct device_node *np,
				       struct pinctrl_map **map,
				       unsigned int *num_maps)
{
	unsigned long *configs = NULL;
	unsigned int num_configs = 0;
	unsigned int reserved = 0;
	int num_mux;
	int i;
	int ret;

	/* If there's no muxing, defer to the generic helper (pinconf only) */
	if (!of_property_present(np, "pinmux"))
		return pinconf_generic_dt_node_to_map(
			pctldev, np, map, num_maps, PIN_MAP_TYPE_CONFIGS_PIN);

	num_mux = of_property_count_u32_elems(np, "pinmux");
	if (num_mux <= 0)
		return num_mux ?: -EINVAL;

	*map = NULL;
	*num_maps = 0;

	ret = pinconf_generic_parse_dt_config(np, pctldev, &configs,
					      &num_configs);
	if (ret)
		return ret;

	ret = pinctrl_utils_reserve_map(pctldev, map, &reserved, num_maps,
					num_configs ? 2 * num_mux : num_mux);
	if (ret)
		goto out;

	for (i = 0; i < num_mux; i++) {
		const char *group;
		const char *func;
		unsigned int val;
		unsigned int pin;
		unsigned int alt;

		ret = of_property_read_u32_index(np, "pinmux", i, &val);
		if (ret)
			goto out_map;

		/* Decompose the macro into pin and alternate mode indices */
		pin = ADSP_PINMUX_PIN(val);
		alt = ADSP_PINMUX_ALT(val);

		if (pin >= pctldev->desc->npins ||
		    alt >= pinmux_generic_get_function_count(pctldev)) {
			ret = -EINVAL;
			goto out_map;
		}

		/* Get the group (single pin) and function (alt mode) strings */
		group = pinctrl_generic_get_group_name(pctldev, pin);
		func = pinmux_generic_get_function_name(pctldev, alt);

		/* Add the muxing map */
		ret = pinctrl_utils_add_map_mux(pctldev, map, &reserved,
						num_maps, group, func);
		if (ret)
			goto out_map;

		/* If there were configs, add their map too */
		if (num_configs) {
			ret = pinctrl_utils_add_map_configs(
				pctldev, map, &reserved, num_maps, group,
				configs, num_configs, PIN_MAP_TYPE_CONFIGS_PIN);
			if (ret)
				goto out_map;
		}
	}

	goto out;

out_map:
	pinctrl_utils_free_map(pctldev, *map, *num_maps);
out:
	kfree(configs);
	return ret;
}

static const struct pinctrl_ops adsp_pctlops = {
	.get_groups_count = pinctrl_generic_get_group_count,
	.get_group_name = pinctrl_generic_get_group_name,
	.get_group_pins = pinctrl_generic_get_group_pins,
	.dt_node_to_map = adsp_pinctrl_dt_node_to_map,
	.dt_free_map = pinctrl_utils_free_map,
};

static int adsp_pinctrl_register_port(struct adsp_port *port)
{
	struct device *dev = port->pc->dev;
	struct gpio_chip *gc = &port->gc;

	port->regs = devm_of_iomap(dev, port->np, 0, NULL);
	if (IS_ERR(port->regs))
		return dev_err_probe(dev, PTR_ERR(port->regs),
				     "%pOF: failed to map regs\n", port->np);

	gc->label = devm_kasprintf(dev, GFP_KERNEL, "adsp-port%c",
				   'a' + port->index);
	if (!gc->label)
		return -ENOMEM;

	gc->parent = dev;
	gc->fwnode = of_fwnode_handle(port->np);
	gc->owner = THIS_MODULE;
	gc->request = gpiochip_generic_request;
	gc->free = gpiochip_generic_free;
	gc->get = adsp_gpio_get;
	gc->set = adsp_gpio_set;
	gc->get_direction = adsp_gpio_get_direction;
	gc->direction_input = adsp_gpio_direction_input;
	gc->direction_output = adsp_gpio_direction_output;
	gc->set_config = gpiochip_generic_config;
	gc->base = -1;
	gc->ngpio = port->ngpio;

	/* TODO: gc->irq.* (hierarchical PINT parent), do during PINT work */

	return devm_gpiochip_add_data(dev, gc, port);
}

static int adsp_pinctrl_register_ports(struct adsp_pinctrl *pc)
{
	unsigned int i;
	int ret;

	for (i = 0; i < pc->nports; i++) {
		ret = adsp_pinctrl_register_port(&pc->ports[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int adsp_pinctrl_add_groups_and_functions(struct adsp_pinctrl *pc)
{
	struct pinctrl_dev *pctldev = pc->pctldev;
	unsigned int i, f;
	int ret;

	/*
	 * Muxing is per-pin. Add a single-pin group per pin, and add the
	 * alternate pin functions without any associated pins. While not all
	 * pins actually support every alternate function, the DT header macros
	 * only expose valid combinations.
	 */

	for (i = 0; i < pc->npins; i++) {
		ret = pinctrl_generic_add_group(pctldev, pc->pins[i].name,
						&pc->pins[i].number, 1, NULL);
		if (ret < 0)
			return ret;
	}

	for (f = 0; f < ARRAY_SIZE(adsp_functions); f++) {
		ret = pinmux_generic_add_function(pctldev, adsp_functions[f],
						  NULL, 0, NULL);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int adsp_pinctrl_register(struct adsp_pinctrl *pc)
{
	struct device *dev = pc->dev;
	int ret;

	pc->pctldesc.name = dev_name(dev);
	pc->pctldesc.pins = pc->pins;
	pc->pctldesc.npins = pc->npins;
	pc->pctldesc.pctlops = &adsp_pctlops;
	pc->pctldesc.pmxops = &adsp_pmxops;
	pc->pctldesc.confops = &adsp_confops;
	pc->pctldesc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &pc->pctldesc, pc,
					     &pc->pctldev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register pinctrl\n");

	return 0;
}

static int adsp_pinctrl_build_pins(struct adsp_pinctrl *pc)
{
	struct device *dev = pc->dev;
	char (*names)[ADSP_PIN_NAME_LEN];
	int i;

	pc->pins = devm_kcalloc(dev, pc->npins, sizeof(*pc->pins), GFP_KERNEL);
	if (!pc->pins)
		return -ENOMEM;

	names = devm_kcalloc(dev, pc->npins, sizeof(*names), GFP_KERNEL);
	if (!names)
		return -ENOMEM;

	for (i = 0; i < pc->npins; i++) {
		unsigned int port = i / ADSP_PINS_PER_PORT;
		unsigned int offset = i % ADSP_PINS_PER_PORT;

		snprintf(names[i], ADSP_PIN_NAME_LEN, "P%c_%02u", 'A' + port,
			 offset); /* PA_00, PB_12, ... */
		pc->pins[i].number = i;
		pc->pins[i].name = names[i];
		pc->pins[i].drv_data = &pc->ports[port];
	}

	return 0;
}

static int adsp_parse_port_gpio_range(struct device_node *child,
				      unsigned int *pin_base,
				      unsigned int *count)
{
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(child, "gpio-ranges", 3, 0,
					       &args);
	if (ret)
		return ret;
	of_node_put(args.np);

	*pin_base = args.args[1];
	*count = args.args[2];

	if (!*count || *pin_base % ADSP_PINS_PER_PORT)
		return -EINVAL;

	return 0;
}

static int adsp_pinctrl_parse_ports(struct adsp_pinctrl *pc)
{
	struct device *dev = pc->dev;
	struct device_node *np = dev->of_node;
	int ret;

	/*
	 * For each PORT, compute its index (PORTA, PORTB, etc.) and count the
	 * number of available pins based on its gpio-ranges property. From
	 * that, derive the total number of pins to expose in the pin
	 * controller: last PORT's pin base + pin count.
	 */

	for_each_child_of_node_scoped(np, child) {
		struct adsp_port *port;
		unsigned int pin_base;
		unsigned int count;
		unsigned int index;

		if (!of_property_present(child, "gpio-controller"))
			continue;

		ret = adsp_parse_port_gpio_range(child, &pin_base, &count);
		if (ret)
			return dev_err_probe(dev, ret,
					     "%pOFn: bad gpio-ranges\n", child);

		index = pin_base / ADSP_PINS_PER_PORT;
		if (index >= pc->nports || index > 'Z' - 'A')
			return dev_err_probe(dev, -EINVAL,
					     "%pOFn: port %u out of range\n",
					     child, index);

		port = &pc->ports[index];
		if (port->ngpio)
			return dev_err_probe(dev, -EINVAL,
					     "%pOFn: duplicate port %u\n",
					     child, index);

		port->index = index;
		port->ngpio = count;
		port->pin_base = pin_base;
		port->pc = pc;
		port->np = of_node_get(child);
		spin_lock_init(&port->lock);
		pc->npins = max(pc->npins, pin_base + count);
	}

	return 0;
}

static unsigned int adsp_pinctrl_count_ports(struct device_node *np)
{
	unsigned int n = 0;

	for_each_child_of_node_scoped(np, child)
		if (of_property_present(child, "gpio-controller"))
			n++;

	return n;
}

static int adsp_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adsp_pinctrl *pc;
	int ret;

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc)
		return -ENOMEM;

	pc->dev = dev;
	pc->info = device_get_match_data(dev);
	platform_set_drvdata(pdev, pc);

	pc->pads = syscon_regmap_lookup_by_phandle(np, "adi,pads-syscon");
	if (IS_ERR(pc->pads))
		return dev_err_probe(dev, PTR_ERR(pc->pads),
				     "missing adi,pads-syscon property\n");

	pc->nports = adsp_pinctrl_count_ports(np);
	if (!pc->nports)
		return dev_err_probe(dev, -EINVAL,
				     "missing gpio-controller child nodes\n");

	pc->ports =
		devm_kcalloc(dev, pc->nports, sizeof(*pc->ports), GFP_KERNEL);
	if (!pc->ports)
		return -ENOMEM;

	ret = adsp_pinctrl_parse_ports(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_build_pins(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_register(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_add_groups_and_functions(pc);
	if (ret)
		return ret;

	ret = adsp_pinctrl_register_ports(pc);
	if (ret)
		return ret;

	ret = pinctrl_enable(pc->pctldev);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id adsp_pinctrl_of_match[] = {
	{ .compatible = "adi,sc598-pinctrl", .data = &sc598_pinctrl_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adsp_pinctrl_of_match);

static struct platform_driver adsp_pinctrl_driver = {
	.driver = {
		.name = "adsp-pinctrl",
		.of_match_table = adsp_pinctrl_of_match,
	},
	.probe = adsp_pinctrl_probe,
};
module_platform_driver(adsp_pinctrl_driver);

MODULE_AUTHOR("Alvin Šipraga <alvin.sipraga@analog.com>");
MODULE_DESCRIPTION("ADI ADSP pinctrl/GPIO driver");
MODULE_LICENSE("GPL");
