/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smart-core.cn>
 */

#ifndef __LINUX_PIN_SBI_H
#define __LINUX_PIN_SBI_H

#include <linux/types.h>

#ifdef CONFIG_PINCTRL_SBI
#define SBI_PIN_INIT(pin)	\
[sbi_##pin] = {			\
	.name = #pin,		\
	.pinid = pin,		\
}
#else
#define SBI_PIN_INIT(pin)
#endif

struct sbi_pin {
	const char *name;
	unsigned long pinid;
};

struct sbi_pin_driver {
	unsigned long type;
	const char *name;
	int nr_funcs;
	int nr_pins;
	struct sbi_pin *pins;
};

int sbi_pin_register_driver(struct sbi_pin_driver *drv);
void sbi_pin_unregister_driver(struct sbi_pin_driver *drv);

#endif /* __LINUX_PIN_SBI_H */
