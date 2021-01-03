/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smart-core.cn>
 */

#ifndef __LINUX_PINCTRL_SBI_H
#define __LINUX_PINCTRL_SBI_H

#include <linux/types.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <dt-bindings/pinctrl/sbi-pinctrl.h>

#ifdef CONFIG_PINCTRL_SBI
#define SBI_PIN_INIT(pin)	[sbi_##pin] = PINCTRL_PIN((pin), #pin)
#else
#define SBI_PIN_INIT(pin)
#endif

#define SBI_PINCTRL_PIN		GENMASK(15, 0)
#define SBI_PINCTRL_MUX		GENMASK(31, 16)
#define SBI_PINCTRL_PAD		GENMASK(31, 16)

struct sbi_pinctrl_driver {
	unsigned long type;
	const char *name;
	uint8_t drive_def;
	uint8_t pad_mask;
	int nr_funcs;
	const char *const *funcs;
	int nr_pins;
	const struct pinctrl_pin_desc *pins;
};

int sbi_pinctrl_register_driver(struct sbi_pinctrl_driver *drv);
void sbi_pinctrl_unregister_driver(struct sbi_pinctrl_driver *drv);

#endif /* __LINUX_PINCTRL_SBI_H */
