/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smarco.cn>
 */

#include <linux/module.h>
#include <linux/clk/clk-sbi.h>
#include <dt-bindings/clock/sbi-clock-duowen.h>

struct sbi_clk duowen_clks[] = {
	SBI_CLK_INIT(soc_clk),
	SBI_CLK_INIT(sysfab_clk),
	SBI_CLK_INIT(cohfab_clk),
	SBI_CLK_INIT(cluster0_clk),
	SBI_CLK_INIT(cluster1_clk),
	SBI_CLK_INIT(cluster2_clk),
	SBI_CLK_INIT(cluster3_clk),
	SBI_CLK_INIT(dma_clk),
	SBI_CLK_INIT(tmr0_clk),
	SBI_CLK_INIT(tmr1_clk),
	SBI_CLK_INIT(tmr2_clk),
	SBI_CLK_INIT(wdt0_clk),
	SBI_CLK_INIT(wdt1_clk),
	SBI_CLK_INIT(tlmm_clk),
	SBI_CLK_INIT(gpio0_clk),
	SBI_CLK_INIT(gpio1_clk),
	SBI_CLK_INIT(gpio2_clk),
	SBI_CLK_INIT(uart0_clk),
	SBI_CLK_INIT(uart1_clk),
	SBI_CLK_INIT(uart2_clk),
	SBI_CLK_INIT(uart3_clk),
	SBI_CLK_INIT(i2c0_clk),
	SBI_CLK_INIT(i2c1_clk),
	SBI_CLK_INIT(i2c2_clk),
	SBI_CLK_INIT(i2c3_clk),
	SBI_CLK_INIT(i2c4_clk),
	SBI_CLK_INIT(i2c5_clk),
	SBI_CLK_INIT(i2c6_clk),
	SBI_CLK_INIT(i2c7_clk),
	SBI_CLK_INIT(i2c8_clk),
	SBI_CLK_INIT(i2c9_clk),
	SBI_CLK_INIT(i2c10_clk),
	SBI_CLK_INIT(i2c11_clk),
	SBI_CLK_INIT(spi0_clk),
	SBI_CLK_INIT(spi1_clk),
	SBI_CLK_INIT(spi2_clk),
	SBI_CLK_INIT(spi3_clk),
	SBI_CLK_INIT(spi4_clk),
	SBI_CLK_INIT(sd_clk),
	SBI_CLK_INIT(tsensor_clk),
	SBI_CLK_INIT(pcie_pclk),
	SBI_CLK_INIT(pcie_aclk),
	SBI_CLK_INIT(pcie_aux_clk),
	SBI_CLK_INIT(pcie_alt_ref_clk),
	SBI_CLK_INIT(eth_alt_ref_clk),
	SBI_CLK_INIT(sgmii_ref_clk),
};

struct sbi_clk_driver duowen_clk_driver = {
	.name = "duowen,crcntl",
	.nr_clks = ARRAY_SIZE(duowen_clks),
	.clks = duowen_clks,
};

static int __init duowen_clk_init(void)
{
	return sbi_clock_register_driver(&duowen_clk_driver);
}
arch_initcall(duowen_clk_init)
