/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  Copyright (C) 2021 CAS SmartCore Co., Ltd.
 *    Author: 2021 Lv Zheng <zhenglv@smarco.cn>
 */

#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/clk/clk-sbi.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <asm/sbi.h>

/**
 * struct __sbi_driver - internally managed SBI clock driver
 * @drv: platform specific clock driver
 * @kref: reference counter
 * @node: link to the managed clock drivers
 *
 * SBI clock data.  Used by the SBI driver to register platform provided
 * clocks to the Linux clock infrastructure.
 */
struct __sbi_driver {
	struct sbi_clk_driver *drv;
	struct kref kref;
	struct list_head node;
};

/**
 * struct __sbi_clock - describes a clock device managed by SBI
 * @hw: Linux-private clock data
 * @clk: platfom specific data associated with this clock (if not NULL)
 *
 * SBI clock data.  Used by the SBI clock driver to register platform
 * provided clocks to the Linux clock infrastructure.
 */
struct __sbi_clock {
	struct clk_hw hw;
	struct sbi_clk *clk;
};

/**
 * struct __sbi_device - per-device-instance data
 * @driver: device specific platform clock driver
 * @clks: platform specific clock data
 * @hw_clks: encapsulates struct clk_hw records
 *
 * SBI clock per-device instance data
 */
struct __sbi_device {
	struct __sbi_driver *driver;
	struct __sbi_clock *clks;
	/* Must be the last member to have hw_clks->hws correctedly
	 * allocated.
	 */
	struct clk_hw_onecell_data hw_clks;
};

#define clk_hw_to_sbi_clock(pwd) container_of(pwd, struct __sbi_clock, hw)

static LIST_HEAD(sbi_clk_drivers);
static DEFINE_MUTEX(sbi_clk_mutex);

/*
 * SBI clock driver registration
 */
static struct __sbi_driver *sbi_clock_get_driver(unsigned long type)
{
	struct __sbi_driver *driver;

	mutex_lock(&sbi_clk_mutex);
	list_for_each_entry(driver, &sbi_clk_drivers, node) {
		if (driver->drv->type == type) {
			kref_get(&driver->kref);
			mutex_unlock(&sbi_clk_mutex);
			return driver;
		}
	}
	mutex_unlock(&sbi_clk_mutex);
	return NULL;
}

static void sbi_clock_driver_release(struct kref *kref)
{
	struct __sbi_driver *driver =
		container_of(kref, struct __sbi_driver, kref);

	kfree(driver);
}

static void sbi_clock_put_driver(struct __sbi_driver *driver)
{
	kref_put(&driver->kref, sbi_clock_driver_release);
}

/**
 * sbi_clock_register_driver() - register SBI clock controls
 * @drv: platform specific clock driver
 *
 * Unregister the list of clock controls.
 *
 * Return: 0 upon success or a negative error code upon failure.
 */
int sbi_clock_register_driver(struct sbi_clk_driver *drv)
{
	struct __sbi_driver *driver;

	driver = kzalloc(sizeof(struct __sbi_driver), GFP_KERNEL);
	if (!driver)
		return -ENOMEM;

	mutex_lock(&sbi_clk_mutex);
	kref_init(&driver->kref);
	INIT_LIST_HEAD(&driver->node);
	list_add(&driver->node, &sbi_clk_drivers);
	driver->drv = drv;
	mutex_unlock(&sbi_clk_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(sbi_clock_register_driver);

/**
 * sbi_clock_unregister_driver() - unregister SBI clock controls
 * @drv: platform specific clock driver
 *
 * Unregister the list of clock controls.
 *
 * Return: none.
 */
void sbi_clock_unregister_driver(struct sbi_clk_driver *drv)
{
	struct __sbi_driver *driver, *tmp;
	LIST_HEAD(free_list);

	mutex_lock(&sbi_clk_mutex);
	list_for_each_entry_safe(driver, tmp, &sbi_clk_drivers, node) {
		if (driver->drv == drv) {
			list_del_init(&driver->node);
			break;
		}
	}
	mutex_unlock(&sbi_clk_mutex);
	sbi_clock_put_driver(driver);
}
EXPORT_SYMBOL_GPL(sbi_clock_unregister_driver);

/*
 * Linux clock framework integration
 *
 * See the Linux clock framework documentation for more information on
 * these functions.
 */
static unsigned long sbi_clock_recalc_rate(struct clk_hw *hw,
					   unsigned long parent_rate)
{
	struct __sbi_clock *clock = clk_hw_to_sbi_clock(hw);

	return sbi_get_clk_freq(clock->clk->clkid);
}

static int sbi_clock_enable(struct clk_hw *hw)
{
	struct __sbi_clock *clock = clk_hw_to_sbi_clock(hw);

	sbi_enable_clk(clock->clk->clkid);
	return 0;
}

static void sbi_clock_disable(struct clk_hw *hw)
{
	struct __sbi_clock *clock = clk_hw_to_sbi_clock(hw);

	sbi_disable_clk(clock->clk->clkid);
}

struct clk_ops sbi_clk_ops = {
	.recalc_rate = sbi_clock_recalc_rate,
	.enable = sbi_clock_enable,
	.disable = sbi_clock_disable,
};

/**
 * __sbi_register_clocks() - register clock controls in the PRCI with Linux
 * @dev: Linux struct device *
 * @device: platform specific device data
 *
 * Register the list of clock controls described in platform specific
 * sbi_clk_driver.
 *
 * Return: 0 upon success or a negative error code upon failure.
 */
static int __sbi_register_clocks(struct device *dev,
				 struct __sbi_device *device)
{
	struct clk_init_data init = { };
	struct sbi_clk *clk;
	struct sbi_clk_driver *drv;
	struct __sbi_clock *clks;
	int i;
	int ret;

	/* Register PLLs */
	drv = device->driver->drv;
	clks = device->clks;
	for (i = 0; i < drv->nr_clks; ++i) {
		clk = &drv->clks[i];

		init.name = clk->name;
		init.parent_names = NULL;
		init.num_parents = 0;
		init.flags = CLK_GET_RATE_NOCACHE | CLK_SET_RATE_NO_REPARENT;
		init.ops = &sbi_clk_ops;
		clks[i].clk = clk;
		clks[i].hw.init = &init;

		ret = devm_clk_hw_register(dev, &clks[i].hw);
		if (ret) {
			dev_warn(dev, "%s: could not register clk_hw: %d\n",
				 clk->name, ret);
			return ret;
		}

		ret = clk_hw_register_clkdev(&clks[i].hw,
					     clk->name, dev_name(dev));
		if (ret) {
			dev_warn(dev, "%s: could not register clkdev: %d\n",
				 clk->name, ret);
			return ret;
		}

		device->hw_clks.hws[i] = &clks[i].hw;
	}
	device->hw_clks.num = i;

	ret = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					  &device->hw_clks);
	if (ret) {
		dev_err(dev, "could not add hw_provider: %d\n", ret);
		return ret;
	}
	return 0;
}

/*
 * Linux device model integration
 *
 * See the Linux device model documentation for more information about
 * these functions.
 */
static int sbi_clock_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned long type;
	struct __sbi_driver *driver;
	struct __sbi_device *device = NULL;
	struct sbi_clk_driver *drv;
	int ret = 0;
	size_t size;

	type = (unsigned long)of_device_get_match_data(dev);
	driver = sbi_clock_get_driver(type);
	if (!driver) {
		dev_err(dev, "could not find clock type: %lu\n", type);
		return -ENODEV;
	}

	drv = driver->drv;

	size = sizeof(struct __sbi_device) +
	       drv->nr_clks * sizeof(struct clk_hw *);
	device = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!device) {
		dev_err(dev, "%s: could not alloc device\n",
			drv->name);
		ret = -ENOMEM;
		goto err_exit;
	}

	size = drv->nr_clks * sizeof(struct __sbi_clock);
	device->clks = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!device) {
		dev_err(dev, "%s: could not alloc %d clocks\n",
			drv->name, drv->nr_clks);
		ret = -ENOMEM;
		goto err_exit;
	}

	device->driver = driver;
	ret = __sbi_register_clocks(dev, device);
	if (ret) {
		dev_err(dev, "%s: could not register clocks: %d\n",
			drv->name, ret);
		goto err_exit;
	}

	dev_info(dev, "SBI clock %s(%lu) probed %d clocks\n",
		 drv->name, drv->type, device->hw_clks.num);

err_exit:
	if (ret) {
		if (device) {
			if (device->clks)
				devm_kfree(dev, device->clks);
			devm_kfree(dev, device);
		}
		sbi_clock_put_driver(driver);
	}
	return ret;
}

static const struct of_device_id sbi_clock_of_match[] = {
	{}
};
MODULE_DEVICE_TABLE(of, sbi_clock_of_match);

static struct platform_driver sbi_clock_driver = {
	.driver	= {
		.name = "sbi-clock",
		.of_match_table = sbi_clock_of_match,
	},
	.probe = sbi_clock_probe,
};

static int __init sbi_clock_init(void)
{
	return platform_driver_register(&sbi_clock_driver);
}
core_initcall(sbi_clock_init);
