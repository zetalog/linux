/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smarco.cn>
 */

#ifndef __LINUX_CLK_SBI_H
#define __LINUX_CLK_SBI_H

#include <linux/types.h>

#ifdef CONFIG_CLK_SBI
#define SBI_CLK_INIT(clk)	\
[sbi_##clk] = {			\
	.name = #clk,		\
	.clkid = clk,		\
}
#else
#define SBI_CLK_INIT(clk)
#endif

struct sbi_clk {
	const char *name;
	unsigned long clkid;
};

struct sbi_clk_driver {
	unsigned long type;
	const char *name;
	int nr_clks;
	struct sbi_clk *clks;
};

int sbi_clock_register_driver(struct sbi_clk_driver *drv);
void sbi_clock_unregister_driver(struct sbi_clk_driver *drv);

#endif /* __LINUX_CLK_SBI_H */
