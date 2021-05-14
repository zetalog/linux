// SPDX-License-Identifier: GPL-2.0
// PCIe RC driver for SmartCo Duowen PCIe subsystem

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/regmap.h>

#include "pcie-designware.h"

struct duowen_pcie {
	struct dw_pcie *pci;
	unsigned int atu_offset;
	u64 ctrl_base;
};

static u64 duowen_pcie_cpu_addr(struct dw_pcie *pcie, u64 cpu_addr)
{
	u64 ctrl_base = pcie->ctrl_base;

	return (cpu_addr - ctrl_base);
}

static const struct dw_pcie_ops duowen_pcie_ops = {
	.cpu_addr_fixup = duowen_pcie_cpu_addr,
};

static int duowen_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct duowen_pcie *duowen_pcie;
	struct dw_pcie *pci;
	struct pcie_port *pp;
	struct resource *res;
	int ret;
	u32 atu_offset;
	u64 ctrl_base;

	duowen_pcie = devm_kzalloc(dev, sizeof(*duowen_pcie), GFP_KERNEL);
	if (!duowen_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pp = &pci->pp;

	duowen_pcie->pci = pci;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (!res) {
		dev_err(dev, "Missing *dbi* reg space\n");
		return -ENODEV;
	}

	pci->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	ret = of_property_read_u32(dev->of_node, "atu_offset", &atu_offset);
	if (!ret)
		pci->atu_base = pci->dbi_base + atu_offset;

	ret = of_property_read_u64(dev->of_node, "ctrl_base", &ctrl_base);
	if (!ret)
		pci->ctrl_base = ctrl_base;

	pci->ops = &duowen_pcie_ops;
	ret = dw_pcie_host_init(pp);
	if (ret)
		dev_err(dev, "Failed to initialize host\n");

	return ret;
}

static const struct of_device_id duowen_pcie_of_match[] = {
	{
		.compatible = "smarco,duowen-pcie",
	},
	{},
};

static struct platform_driver duowen_pcie_driver = {
	.driver = {
		.name = "duowen-pcie",
		.of_match_table = duowen_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = duowen_pcie_probe,
};
builtin_platform_driver(duowen_pcie_driver);
