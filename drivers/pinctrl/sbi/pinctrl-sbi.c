/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smart-core.cn>
 */

#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/bitfield.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/io.h>

#include <dt-bindings/pinctrl/sbi-pinctrl.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinctrl-utils.h"

/*
 * struct sbi_pin: Kendryte SBI FPIOA memory mapped registers
 * @pins: 48 32-bits IO pin registers
 */
struct sbi_pin {
	u32 pins[48];
	u32 tie_en[8];
	u32 tie_val[8];
};

struct sbi_pin_data {

	struct device *dev;
	struct pinctrl_dev *pctl;

	struct sbi_pin __iomem *pin;
	struct regmap *sysctl_map;
	u32 power_offset;
	struct clk *clk;
	struct clk *pclk;
};

static const struct pinctrl_pin_desc sbi_pins[] = {
};

#define SBI_NPINS ARRAY_SIZE(sbi_pins)

/*
 * Pin groups: each of the 48 programmable pins is a group.
 * To this are added 8 power domain groups, which for the purposes of
 * the pin subsystem, contain no pins. The power domain groups only exist
 * to set the power level. The id should never be used (since there are
 * no pins 48-55).
 */
static const char *const sbi_group_names[] = {
};

#define SBI_NGROUPS	ARRAY_SIZE(sbi_group_names)

enum sbi_pinctrl_mode_id {
	SBI_PC_DEFAULT_DISABLED,
};

#define SBI_PC_DEFAULT(mode) \
	[SBI_PC_DEFAULT_##mode] = SBI_PC_MODE_##mode

static const u32 sbi_pinconf_mode_id_to_mode[] = {
	[SBI_PC_DEFAULT_DISABLED] = 0,
};

#undef DEFAULT

/*
 * Pin functions configuration information.
 */
struct sbi_pcf_info {
	char name[15];
	u8 mode_id;
};

static const struct sbi_pcf_info sbi_pcf_infos[] = {
};

#define PIN_CONFIG_OUTPUT_INVERT	(PIN_CONFIG_END + 1)
#define PIN_CONFIG_INPUT_INVERT		(PIN_CONFIG_END + 2)

static const struct pinconf_generic_params sbi_pinconf_custom_params[] = {
	{ "output-polarity-invert", PIN_CONFIG_OUTPUT_INVERT, 1 },
	{ "input-polarity-invert",  PIN_CONFIG_INPUT_INVERT, 1 },
};

static void sbi_pinmux_set_pin_function(struct pinctrl_dev *pctldev,
					 u32 pin, u32 func)
{
	struct sbi_pin_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	const struct sbi_pcf_info *info = &sbi_pcf_infos[func];
	u32 mode = sbi_pinconf_mode_id_to_mode[info->mode_id];
	u32 val = func | mode;

	dev_dbg(pdata->dev, "set pin %u function %s (%u) -> 0x%08x\n",
		pin, info->name, func, val);

	writel(val, &pdata->pin->pins[pin]);
}

static int sbi_pinconf_set_param(struct pinctrl_dev *pctldev,
				 unsigned int pin,
				 unsigned int param, unsigned int arg)
{
	struct sbi_pin_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	dev_dbg(pdata->dev, "set pin %u param %u, arg 0x%x\n",
		pin, param, arg);

	return 0;
}

static int sbi_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			   unsigned long *configs, unsigned int num_configs)
{
	unsigned int param, arg;
	int i, ret;

	if (WARN_ON(pin >= SBI_NPINS))
		return -EINVAL;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		ret = sbi_pinconf_set_param(pctldev, pin, param, arg);
		if (ret)
			return ret;
	}

	return 0;
}

static int sbi_pinconf_group_set(struct pinctrl_dev *pctldev,
				 unsigned int selector, unsigned long *configs,
				 unsigned int num_configs)
{
	struct sbi_pin_data *pdata = pinctrl_dev_get_drvdata(pctldev);
	unsigned int param, arg;
	u32 bit;
	int i;

	/* Pins should be configured with pinmux, not groups*/
	if (selector < SBI_NPINS)
		return -EINVAL;

	/* Otherwise it's a power domain */
	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		if (param != PIN_CONFIG_POWER_SOURCE)
			return -EINVAL;

		arg = pinconf_to_config_argument(configs[i]);
		bit = BIT(selector - SBI_NPINS);
#if 0
		/* TODO: sbi_pin_xxx */
		regmap_update_bits(pdata->sysctl_map,
				   pdata->power_offset,
				   bit, arg ? bit : 0);
#endif
	}

	return 0;
}

static void sbi_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				 struct seq_file *s, unsigned pin)
{
}

static void sbi_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
				       struct seq_file *s, unsigned selector)
{
}

static void sbi_pinconf_config_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s,
					unsigned long config)
{
	pinconf_generic_dump_config(pctldev, s, config);
}

static const struct pinconf_ops sbi_pinconf_ops = {
#if 0
	.pin_config_get = sbi_pinconf_get,
#endif
	.pin_config_set = sbi_pinconf_set,
#if 0
	.pin_config_group_get = sbi_pinconf_group_get,
#endif
	.pin_config_group_set = sbi_pinconf_group_set,
	.pin_config_dbg_show = sbi_pinconf_dbg_show,
	.pin_config_group_dbg_show = sbi_pinconf_group_dbg_show,
	.pin_config_config_dbg_show = sbi_pinconf_config_dbg_show,
	.is_generic = true,
};

static int sbi_pinmux_get_function_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(sbi_pcf_infos);
}

static const char *sbi_pinmux_get_function_name(struct pinctrl_dev *pctldev,
						 unsigned int selector)
{
	return sbi_pcf_infos[selector].name;
}

static int sbi_pinmux_get_function_groups(struct pinctrl_dev *pctldev,
					   unsigned int selector,
					   const char * const **groups,
					   unsigned int * const num_groups)
{
	/* Any function can be mapped to any pin */
	*groups = sbi_group_names;
	*num_groups = SBI_NPINS;

	return 0;
}

static int sbi_pinmux_set_mux(struct pinctrl_dev *pctldev,
			       unsigned int function,
			       unsigned int group)
{
	/* Can't mux power domains */
	if (group >= SBI_NPINS)
		return -EINVAL;

	sbi_pinmux_set_pin_function(pctldev, group, function);

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
	return SBI_NGROUPS;
}

static const char *sbi_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned int group)
{
	return sbi_group_names[group];
}

static int sbi_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
				       unsigned int group,
				       const unsigned int **pins,
				       unsigned int *npins)
{
	if (group >= SBI_NPINS) {
		*pins = NULL;
		*npins = 0;
		return 0;
	}

	*pins = &sbi_pins[group].number;
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
#if 0
		const char *group_name, *func_name;
		u32 pin = FIELD_GET(SBI_PG_PIN, pinmux_group);
		u32 func = FIELD_GET(SBI_PG_FUNC, pinmux_group);

		if (pin >= SBI_NPINS) {
			ret = -EINVAL;
			goto exit;
		}

		group_name = sbi_group_names[pin];
		func_name = sbi_pcf_infos[func].name;

		dev_dbg(pctldev->dev, "Pinmux %s: pin %u func %s\n",
			np->name, pin, func_name);

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
#endif
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

static struct pinctrl_desc sbi_pinctrl_desc = {
	.name = "sbi-pinctrl",
	.pins = sbi_pins,
	.npins = SBI_NPINS,
	.pctlops = &sbi_pinctrl_ops,
	.pmxops = &sbi_pinmux_ops,
	.confops = &sbi_pinconf_ops,
	.custom_params = sbi_pinconf_custom_params,
	.num_custom_params = ARRAY_SIZE(sbi_pinconf_custom_params),
};

static void sbi_pin_init_ties(struct sbi_pin_data *pdata)
{
	struct sbi_pin __iomem *pin = pdata->pin;
	u32 val;
	int i, j;

	dev_dbg(pdata->dev, "Init pin ties\n");

	/* Init pin functions input ties */
	for (i = 0; i < ARRAY_SIZE(pin->tie_en); i++) {
		val = 0;
		for (j = 0; j < 32; j++) {
			if (sbi_pcf_infos[i * 32 + j].mode_id ==
			    SBI_PC_DEFAULT_DISABLED) {
				dev_dbg(pdata->dev,
					"tie_en function %d (%s)\n",
					i * 32 + j,
					sbi_pcf_infos[i * 32 + j].name);
				val |= BIT(j);
			}
		}

		/* Set value before enable */
		writel(val, &pin->tie_val[i]);
		writel(val, &pin->tie_en[i]);
	}
}

static int sbi_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long type;
	struct device_node *np = dev->of_node;
	struct sbi_pin_data *pdata;
	int ret;

	type = (unsigned long)of_device_get_match_data(dev);
#if 0
	driver = sbi_pinctrl_get_driver(type);
	if (!driver) {
		dev_err(dev, "could not find pinctrl type: %lu\n", type);
		return -ENODEV;
	}
#endif

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = dev;
	platform_set_drvdata(pdev, pdata);

	pdata->pin = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pdata->pin))
		return PTR_ERR(pdata->pin);

	sbi_pin_init_ties(pdata);

	pdata->pctl = pinctrl_register(&sbi_pinctrl_desc, dev, (void *)pdata);
	if (IS_ERR(pdata->pctl))
		return PTR_ERR(pdata->pctl);

	return 0;
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
