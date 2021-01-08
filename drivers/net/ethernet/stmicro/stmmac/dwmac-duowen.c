/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smarco.cn>
 */

#include <linux/of_net.h>
#include <linux/stmmac.h>
#include "stmmac_platform.h"

struct duowen_dwmac {
	bool has_xpcs;
	bool probe_phy;
};

static int duowen_dwmac_probe(struct platform_device *pdev)
{
	int ret;
	struct duowen_dwmac *mac = NULL;
	struct plat_stmmacenet_data *plat_dat = NULL;
	struct stmmac_resources stmmac_res;

	mac = devm_kzalloc(&pdev->dev, sizeof(*mac), GFP_KERNEL);
	if (!mac)
		return PTR_ERR(mac);

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		goto err_exit;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		ret = PTR_ERR(plat_dat);
		goto err_exit;
	}

	if (plat_dat->mdio_node) {
		mac->has_xpcs = of_property_read_bool(plat_dat->mdio_node,
						      "smarco,has_xpcs");
		mac->probe_phy = of_property_read_bool(plat_dat->mdio_node,
						       "smarco,probe_phy");
		if (mac->has_xpcs)
			dev_info(&pdev->dev, "Found MDIO xpcs\n");
		plat_dat->mdio_bus_data->has_xpcs = mac->has_xpcs;
		plat_dat->mdio_bus_data->probe_phy = mac->probe_phy;
	}

	plat_dat->bsp_priv = mac;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret)
		goto err_exit;

	return 0;

err_exit:
	if (plat_dat)
		stmmac_remove_config_dt(pdev, plat_dat);
	if (mac)
		devm_kfree(&pdev->dev, mac);
	return ret;
}

static const struct of_device_id duowen_dwmac_match[] = {
	{ .compatible = "smarco,duowen-xgmac2" },
	{ }
};
MODULE_DEVICE_TABLE(of, duowen_dwmac_match);

static struct platform_driver duowen_dwmac_driver = {
	.probe  = duowen_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "duowen-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = duowen_dwmac_match,
	},
};
module_platform_driver(duowen_dwmac_driver);

MODULE_DESCRIPTION("SmarCo DUOWEN DWMAC specific glue layer");
MODULE_AUTHOR("Lv Zheng <zhenglv@smart-core.cn>");
MODULE_LICENSE("GPL v2");
