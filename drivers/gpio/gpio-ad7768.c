// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7768 GPIO auxiliary driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define AD7768_REG_GPIO_CONTROL		0x0E
#define AD7768_REG_GPIO_WRITE		0x0F
#define AD7768_REG_GPIO_READ		0x10

#define AD7768_GPIO_UGPIO_ENABLE	BIT(7)
#define AD7768_GPIO_INPUT		0x00
#define AD7768_GPIO_OUTPUT(x)		BIT(x)

#define AD7768_NUM_GPIOS		5

struct ad7768_gpio_state {
	struct device *parent;
	struct regmap *regmap;
	struct mutex lock; /* protects regmap accesses */
	struct gpio_chip gc;
};

static int ad7768_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int offset)
{
	struct ad7768_gpio_state *st = gpiochip_get_data(chip);

	PM_RUNTIME_ACQUIRE_IF_ENABLED_AUTOSUSPEND(st->parent, pm);
	int ret = PM_RUNTIME_ACQUIRE_ERR(&pm);

	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	return regmap_update_bits(st->regmap, AD7768_REG_GPIO_CONTROL,
				  BIT(offset), AD7768_GPIO_INPUT);
}

static int ad7768_gpio_get_direction(struct gpio_chip *chip,
				     unsigned int offset)
{
	struct ad7768_gpio_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	PM_RUNTIME_ACQUIRE_IF_ENABLED_AUTOSUSPEND(st->parent, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	scoped_guard(mutex, &st->lock)
		ret = regmap_read(st->regmap, AD7768_REG_GPIO_CONTROL, &val);

	if (ret)
		return ret;

	return !!(val & BIT(offset)) ? GPIO_LINE_DIRECTION_OUT :
				       GPIO_LINE_DIRECTION_IN;
}

static int ad7768_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset, int value)
{
	struct ad7768_gpio_state *st = gpiochip_get_data(chip);
	int ret;

	PM_RUNTIME_ACQUIRE_IF_ENABLED_AUTOSUSPEND(st->parent, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	ret = regmap_update_bits(st->regmap, AD7768_REG_GPIO_CONTROL,
				 BIT(offset), AD7768_GPIO_OUTPUT(offset));
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, AD7768_REG_GPIO_WRITE,
				  BIT(offset), value << offset);
}

static int ad7768_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_gpio_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	PM_RUNTIME_ACQUIRE_IF_ENABLED_AUTOSUSPEND(st->parent, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AD7768_REG_GPIO_CONTROL, &val);
	if (ret)
		return ret;

	if (val & BIT(offset))
		ret = regmap_read(st->regmap, AD7768_REG_GPIO_WRITE, &val);
	else
		ret = regmap_read(st->regmap, AD7768_REG_GPIO_READ, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset));
}

static int ad7768_gpio_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct ad7768_gpio_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	PM_RUNTIME_ACQUIRE_IF_ENABLED_AUTOSUSPEND(st->parent, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, AD7768_REG_GPIO_CONTROL, &val);
	if (ret)
		return ret;

	if (!(val & BIT(offset)))
		return 0;

	return regmap_update_bits(st->regmap, AD7768_REG_GPIO_WRITE,
				  BIT(offset), value << offset);
}

static int ad7768_gpio_probe(struct auxiliary_device *adev,
			     const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	const char *label = dev_get_platdata(dev);
	struct ad7768_gpio_state *st;
	struct gpio_chip *gc;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->parent = dev->parent;
	st->regmap = dev_get_regmap(dev->parent, NULL);
	if (!st->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get regmap\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(st->parent);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, AD7768_REG_GPIO_CONTROL,
				 AD7768_GPIO_UGPIO_ENABLE,
				 AD7768_GPIO_UGPIO_ENABLE);

	pm_runtime_mark_last_busy(st->parent);
	pm_runtime_put_autosuspend(st->parent);

	if (ret < 0)
		return ret;

	gc = &st->gc;
	gc->label = label;
	gc->base = -1;
	gc->ngpio = AD7768_NUM_GPIOS;
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->can_sleep = true;
	gc->get_direction = ad7768_gpio_get_direction;
	gc->direction_input = ad7768_gpio_direction_input;
	gc->direction_output = ad7768_gpio_direction_output;
	gc->get = ad7768_gpio_get;
	gc->set = ad7768_gpio_set;

	return devm_gpiochip_add_data(dev, &st->gc, st);
}

static const struct auxiliary_device_id ad7768_gpio_ids[] = {
	{ .name = "ad7768.gpio" },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, ad7768_gpio_ids);

static struct auxiliary_driver ad7768_gpio_driver = {
	.probe = ad7768_gpio_probe,
	.id_table = ad7768_gpio_ids,
};
module_auxiliary_driver(ad7768_gpio_driver);

MODULE_AUTHOR("Janani Sunil <janani.sunil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 GPIO auxiliary driver");
MODULE_LICENSE("GPL");
