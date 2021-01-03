/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smart-core.cn>
 */

#ifndef __LINUX_CLK_SBI_H
#define __LINUX_CLK_SBI_H

#include <linux/types.h>
#include <linux/clk-provider.h>

#ifdef CONFIG_CLK_SBI
#define SBI_CLK_INIT(clk)	\
[sbi_##clk] = {			\
	.name = #clk,		\
	.clkid = clk,		\
}
#else
#define SBI_CLK_INIT(clk)
#endif

/**
 * struct sbi_clk - describes a clock device managed by SBI
 * @name: Clock verbose name
 * @clkid: SBI clock framework clock ID
 * @hw: Linux-private clock data
 *
 * SBI clock data.  Used by the SBI clock driver to register platform
 * provided clocks to the Linux clock infrastructure.
 */
struct sbi_clk {
	const char *name;
	unsigned long clkid;
	struct clk_hw hw;
};

/**
 * struct sbi_clk_driver - describes a clock driver to be registered to
 *                         the SBI
 * @type: Clock driver uniq identifier, maintained by this header file
 * @name: Clock driver verbose name
 * @nr_clks: Number of hardware clocks
 * @clks: Hardware clock array
 *
 * SBI clock driver.  Used by the SBI clock infrastructure to distinguish
 * different platforms.
 */
struct sbi_clk_driver {
	unsigned long type;
	const char *name;
	int nr_clks;
	struct sbi_clk *clks;
};

int sbi_clock_register_driver(struct sbi_clk_driver *drv);
void sbi_clock_unregister_driver(struct sbi_clk_driver *drv);

#endif /* __LINUX_CLK_SBI_H */
