/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smart-core.cn>
 */

#include <linux/pinctrl/pinctrl-sbi.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/bitfield.h>
#include <asm/sbi.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"

/**
 * struct __sbi_driver - internally managed SBI pinctrl driver
 * @drv: platform specific pinctrl driver
 * @kref: reference counter
 * @node: link to the managed pinctrl drivers
 *
 * SBI pin control data.  Used by the SBI driver to register platform
 * provided pins to the Linux pinctrl infrastructure.
 */
struct __sbi_driver {
	struct sbi_pinctrl_driver *drv;
	struct kref kref;
	struct list_head node;
};

struct __sbi_device {
	struct pinctrl_desc desc;
	struct device *dev;
	struct __sbi_driver *drv;
	struct pinctrl_dev *pctl;
	const char *pins[];
};

#define PIN_SBI_DEV_SIZE(nr_pins)	\
	(sizeof(struct __sbi_device) + (nr_pins) * sizeof(const char *))

static LIST_HEAD(sbi_pinctrl_drivers);
static DEFINE_MUTEX(sbi_pinctrl_mutex);

/*
 * SBI clock driver registration
 */
static struct __sbi_driver *sbi_pinctrl_get_driver(unsigned long type)
{
	struct __sbi_driver *driver;

	mutex_lock(&sbi_pinctrl_mutex);
	list_for_each_entry(driver, &sbi_pinctrl_drivers, node) {
		if (driver->drv->type == type) {
			kref_get(&driver->kref);
			mutex_unlock(&sbi_pinctrl_mutex);
			return driver;
		}
	}
	mutex_unlock(&sbi_pinctrl_mutex);
	return NULL;
}

static void sbi_pinctrl_driver_release(struct kref *kref)
{
	struct __sbi_driver *driver =
		container_of(kref, struct __sbi_driver, kref);

	kfree(driver);
}

static void sbi_pinctrl_put_driver(struct __sbi_driver *driver)
{
	kref_put(&driver->kref, sbi_pinctrl_driver_release);
}

/**
 * sbi_pinctrl_register_driver() - register SBI pin controls
 * @drv: platform specific pinctrl driver
 *
 * Register the list of pin controls.
 *
 * Return: 0 upon success or a negative error code upon failure.
 */
int sbi_pinctrl_register_driver(struct sbi_pinctrl_driver *drv)
{
	struct __sbi_driver *driver;

	driver = kzalloc(sizeof(struct __sbi_driver), GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	mutex_lock(&sbi_pinctrl_mutex);
	kref_init(&driver->kref);
	INIT_LIST_HEAD(&driver->node);
	list_add(&driver->node, &sbi_pinctrl_drivers);
	driver->drv = drv;
	mutex_unlock(&sbi_pinctrl_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(sbi_pinctrl_register_driver);

/**
 * sbi_pinctrl_unregister_driver() - unregister SBI pin controls
 * @drv: platform specific pinctrl driver
 *
 * Unregister the list of pin controls.
 *
 * Return: none.
 */
void sbi_pinctrl_unregister_driver(struct sbi_pinctrl_driver *drv)
{
	struct __sbi_driver *driver, *tmp;
	LIST_HEAD(free_list);

	mutex_lock(&sbi_pinctrl_mutex);
	list_for_each_entry_safe(driver, tmp, &sbi_pinctrl_drivers, node) {
		if (driver->drv == drv) {
			list_del_init(&driver->node);
			break;
		}
	}
	mutex_unlock(&sbi_pinctrl_mutex);
	sbi_pinctrl_put_driver(driver);
}
EXPORT_SYMBOL_GPL(sbi_pinctrl_unregister_driver);

static unsigned long sbi_pinctrl_id2pin(struct sbi_pinctrl_driver *drv,
				        unsigned int group)
{
	if (group >= drv->nr_pins)
		return INVALID_SBI_PIN;
	return drv->pins[group].number;
}

static unsigned int sbi_pinctrl_pin2id(struct sbi_pinctrl_driver *drv,
				       unsigned long pinid)
{
	return 0;
}

#define PIN_CONFIG_IO_TYPE		(PIN_CONFIG_END + 1)
#define PIN_CONFIG_PULL_TYPE		(PIN_CONFIG_END + 2)
#define PIN_CONFIG_KEEPER		(PIN_CONFIG_END + 3)

#define PIN_IO_ANALOG			0
#define PIN_IO_DIGITAL			1
#define PIN_PULL_WEAK			0
#define PIN_PULL_MEDIUM			1
#define PIN_PULL_STRONG			2

static const struct pinconf_generic_params sbi_pinpad_custom_params[] = {
	{ "io-type",  PIN_CONFIG_IO_TYPE, 1 },
	{ "pull-type",  PIN_CONFIG_PULL_TYPE, 0 },
	{ "keeper", PIN_CONFIG_KEEPER, 1 },
};

static unsigned long sbi_pinconf_parse_param(struct sbi_pinctrl_driver *drv,
					     unsigned int param,
					     unsigned int arg,
					     unsigned long cfg)
{
	uint32_t pad = sbi_pad_pad(cfg);
	uint8_t drive = sbi_pad_drive(cfg);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		pad &= ~GPIO_PAD_PULL_MASK;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		pad |= GPIO_PAD_PULL_DOWN;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		pad |= GPIO_PAD_PULL_UP;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		pad |= GPIO_PAD_OD;
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		pad |= GPIO_PAD_PP;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		drive = arg;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		drive = GPIO_DRIVE_IN;
		break;
	case PIN_CONFIG_OUTPUT_ENABLE:
	case PIN_CONFIG_OUTPUT:
		if (drive == GPIO_DRIVE_IN)
			drive = drv->drive_def;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
	case PIN_CONFIG_INPUT_SCHMITT:
		drive = GPIO_DRIVE_IN;
		pad |= GPIO_PAD_SCHMITT_TRIG;
		break;
	case PIN_CONFIG_SLEW_RATE:
		pad |= GPIO_PAD_SLEW_RATE;
		break;
	case PIN_CONFIG_IO_TYPE:
		pad &= ~GPIO_PAD_IO_MASK;
		if (arg == PIN_IO_ANALOG)
			pad |= GPIO_PAD_ANALOG_IO;
		if (arg == PIN_IO_DIGITAL)
			pad |= GPIO_PAD_DIGITAL_IO;
		break;
	case PIN_CONFIG_PULL_TYPE:
		pad &= ~GPIO_PAD_PULL_STRENGTH_MASK;
		if (arg == PIN_PULL_WEAK)
			pad |= GPIO_PAD_WEAK_PULL;
		if (arg == PIN_PULL_MEDIUM)
			pad |= GPIO_PAD_MEDIUM_PULL;
		if (arg == PIN_PULL_STRONG)
			pad |= GPIO_PAD_STRONG_PULL;
		break;
	case PIN_CONFIG_KEEPER:
		pad |= GPIO_PAD_KEEPER;
		break;
	}
	return SBI_PAD(pad, drive);
}

static int sbi_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			   unsigned long *configs, unsigned int num_configs)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;
	unsigned int param, arg;
	unsigned long cfg = 0;
	unsigned long pinid;
	int i;

	if (pin >= driver->drv->nr_pins)
		return -EINVAL;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		cfg = sbi_pinconf_parse_param(driver->drv, param, arg, cfg);
	}

	pinid = sbi_pinctrl_id2pin(driver->drv, pin);
	dev_dbg(device->dev, "set pin %s-%u-%lu(%d:%d) pad 0x%08x:%02x\n",
		driver->drv->pins[pin].name, pin, pinid,
		sbi_pin_port(pinid), sbi_pin_pin(pinid),
		(unsigned int)sbi_pad_pad(cfg),
		(unsigned int)sbi_pad_drive(cfg));
	sbi_config_pin_pad(pinid, cfg);
	return 0;
}

static int sbi_pinconf_group_set(struct pinctrl_dev *pctldev,
				 unsigned int group, unsigned long *configs,
				 unsigned int num_configs)
{
	/* Support only one pin per group. */
	return sbi_pinconf_set(pctldev, group, configs, num_configs);
}

static const struct pinconf_ops sbi_pinconf_ops = {
	.is_generic = true,
	.pin_config_set = sbi_pinconf_set,
	.pin_config_group_set = sbi_pinconf_group_set,
};

static int sbi_pinmux_get_function_count(struct pinctrl_dev *pctldev)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;

	return driver->drv->nr_funcs;
}

static const char *sbi_pinmux_get_function_name(struct pinctrl_dev *pctldev,
						 unsigned int selector)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;

	if (selector >= driver->drv->nr_funcs)
		return NULL;

	return driver->drv->funcs[selector];
}

static int sbi_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
					  unsigned int selector,
					  const char *const **groups,
					  unsigned int *const num_groups)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;

	/* Any function can be mapped to any pin */
	*groups = device->pins;
	*num_groups = driver->drv->nr_pins;
	return 0;
}

static int sbi_pinmux_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int function,
			      unsigned int group)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;
	unsigned long pinid;

	if (group >= driver->drv->nr_pins)
		return -EINVAL;

	pinid = sbi_pinctrl_id2pin(driver->drv, group);
	dev_dbg(device->dev, "set pin %s-%u-%lu(%d:%d) mux 0x%08x\n",
		driver->drv->pins[group].name, group, pinid,
		sbi_pin_port(pinid), sbi_pin_pin(pinid), function);
	sbi_config_pin_mux(pinid, function);
	return 0;
}

static const struct pinmux_ops sbi_pinmux_ops = {
	.get_functions_count = sbi_pinmux_get_function_count,
	.get_function_name = sbi_pinmux_get_function_name,
	.get_function_groups = sbi_pinmux_get_function_groups,
	.set_mux = sbi_pinmux_set_mux,
	.strict = true,
};

static int sbi_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;

	return driver->drv->nr_pins;
}

static const char *sbi_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int group)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;

	if (group >= driver->drv->nr_pins)
		return NULL;

	return driver->drv->pins[group].name;
}

static int sbi_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned int group,
				      const unsigned int **pins,
				      unsigned int *npins)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;

	if (group >= driver->drv->nr_pins) {
		*pins = NULL;
		*npins = 0;
		return 0;
	}

	*pins = &driver->drv->pins[group].number;
	*npins = 1;
	return 0;
}

static void sbi_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
				     struct seq_file *s, unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int sbi_pinctrl_dt_subnode_to_map(struct pinctrl_dev *pctldev,
					 struct device_node *np,
					 struct pinctrl_map **map,
					 unsigned int *reserved_maps,
					 unsigned int *num_maps)
{
	struct __sbi_device *device = pinctrl_dev_get_drvdata(pctldev);
	struct __sbi_driver *driver = device->drv;
	struct property *prop;
	const __be32 *p;
	int ret, pinmux_groups;
	u32 pinmux_group;
	unsigned long *configs = NULL;
	unsigned int num_configs = 0;
	unsigned int reserve = 0;

	ret = of_property_count_strings(np, "groups");
	if (!ret)
		return pinconf_generic_dt_subnode_to_map(pctldev, np, map,
						reserved_maps, num_maps,
						PIN_MAP_TYPE_CONFIGS_GROUP);

	pinmux_groups = of_property_count_u32_elems(np, "pinmux");
	if (pinmux_groups <= 0) {
		/* Ignore this node */
		return 0;
	}

	ret = pinconf_generic_parse_dt_config(np, pctldev, &configs,
					      &num_configs);
	if (ret < 0) {
		dev_err(pctldev->dev, "%pOF: could not parse node property\n",
			np);
		return ret;
	}

	reserve = pinmux_groups * (1 + num_configs);
	ret = pinctrl_utils_reserve_map(pctldev, map, reserved_maps, num_maps,
					reserve);
	if (ret < 0)
		goto exit;

	of_property_for_each_u32(np, "pinmux", prop, p, pinmux_group) {
		const char *group_name, *func_name;
		unsigned int group;
		unsigned long pinid = FIELD_GET(SBI_PINCTRL_PIN, pinmux_group);
		unsigned long func = FIELD_GET(SBI_PINCTRL_MUX, pinmux_group);

		group = sbi_pinctrl_pin2id(driver->drv, pinid);
		if (group >= driver->drv->nr_pins ||
		    func >= driver->drv->nr_funcs) {
			ret = -EINVAL;
			goto exit;
		}

		group_name = driver->drv->pins[group].name;
		func_name = driver->drv->funcs[func];

		dev_dbg(pctldev->dev,
			"Pinmux %s: pin %s-%u-%lu(%d:%d) mux %s\n",
			np->name, group_name, group, pinid,
			sbi_pin_port(pinid), sbi_pin_pin(pinid),
			func_name);

		ret = pinctrl_utils_add_map_mux(pctldev, map, reserved_maps,
						num_maps, group_name,
						func_name);
		if (ret < 0) {
			dev_err(pctldev->dev, "%pOF add mux map failed %d\n",
				np, ret);
			goto exit;
		}

		if (num_configs) {
			ret = pinctrl_utils_add_map_configs(pctldev, map,
					reserved_maps, num_maps, group_name,
					configs, num_configs,
					PIN_MAP_TYPE_CONFIGS_PIN);
			if (ret < 0) {
				dev_err(pctldev->dev,
					"%pOF add configs map failed %d\n",
					np, ret);
				goto exit;
			}
		}
	}

	ret = 0;

exit:
	kfree(configs);
	return ret;
}

static int sbi_pinctrl_dt_node_to_map(struct pinctrl_dev *pctldev,
				       struct device_node *np_config,
				       struct pinctrl_map **map,
				       unsigned int *num_maps)
{
	unsigned int reserved_maps;
	struct device_node *np;
	int ret;

	reserved_maps = 0;
	*map = NULL;
	*num_maps = 0;

	ret = sbi_pinctrl_dt_subnode_to_map(pctldev, np_config, map,
					    &reserved_maps, num_maps);
	if (ret < 0)
		goto err;

	for_each_available_child_of_node(np_config, np) {
		ret = sbi_pinctrl_dt_subnode_to_map(pctldev, np, map,
						     &reserved_maps, num_maps);
		if (ret < 0)
			goto err;
	}
	return 0;

err:
	pinctrl_utils_free_map(pctldev, *map, *num_maps);
	return ret;
}


static const struct pinctrl_ops sbi_pinctrl_ops = {
	.get_groups_count = sbi_pinctrl_get_groups_count,
	.get_group_name = sbi_pinctrl_get_group_name,
	.get_group_pins = sbi_pinctrl_get_group_pins,
	.pin_dbg_show = sbi_pinctrl_pin_dbg_show,
	.dt_node_to_map = sbi_pinctrl_dt_node_to_map,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int sbi_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long type;
	struct __sbi_driver *driver;
	struct __sbi_device *device;
	struct pinctrl_desc *pdesc;
	int ret = 0;
	int i;

	type = (unsigned long)of_device_get_match_data(dev);
	driver = sbi_pinctrl_get_driver(type);
	if (!driver) {
		dev_err(dev, "could not find pinctrl type: %lu\n", type);
		return -ENODEV;
	}

	device = devm_kzalloc(dev, PIN_SBI_DEV_SIZE(driver->drv->nr_pins),
			      GFP_KERNEL);
	if (!device) {
		dev_err(dev, "%s: could not alloc device\n",
			driver->drv->name);
		ret = -ENOMEM;
		goto err_exit;
	}

	device->drv = driver;
	device->dev = dev;
	platform_set_drvdata(pdev, device);

	/* Create group name array */
	for (i = 0; i < driver->drv->nr_pins; i++)
		device->pins[i] = driver->drv->pins[i].name;

	pdesc = &device->desc;
	pdesc->name = "sbi-pinctrl";
	pdesc->npins = driver->drv->nr_pins;
	pdesc->pins = driver->drv->pins;

	pdesc->pctlops = &sbi_pinctrl_ops;
	pdesc->pmxops = &sbi_pinmux_ops;
	pdesc->confops = &sbi_pinconf_ops;

#if 0
	pdesc->custom_params = sbi_pinpad_custom_params,
	pdesc->num_custom_params = ARRAY_SIZE(sbi_pinpad_custom_params),
#endif

	device->pctl = pinctrl_register(pdesc, dev, (void *)device);
	if (IS_ERR(device->pctl))
		return PTR_ERR(device->pctl);

err_exit:
	if (ret) {
		if (device)
			devm_kfree(dev, device);
		sbi_pinctrl_put_driver(driver);
	}
	return ret;
}

static const struct of_device_id sbi_pinctrl_of_match[] = {
	{},
};

static struct platform_driver sbi_pinctrl_driver = {
	.probe	= sbi_pinctrl_probe,
	.driver = {
		.name		= "sbi-pinctrl",
		.of_match_table	= sbi_pinctrl_of_match,
	},
};
builtin_platform_driver(sbi_pinctrl_driver);
