/*
 * acpi/early_printk.c - ACPI Boot-Time Debug Ports
 *
 * Copyright (c) 2012, Intel Corporation
 * Author: Lv Zheng <lv.zheng@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define DEBUG
#define pr_fmt(fmt)	"ACPI: " KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/acpi.h>
#include <linux/bitmap.h>
#include <linux/bootmem.h>

#define MAX_ACPI_DBG_PORTS	4

/*
 * ACPI Early Flags:
 *
 * The acpi_early_flags bitmap includes flags for each debug port:
 * 1. enabled bit: "enabled" flags, to disable ports by default.
 *    offset=0, length=MAX_ACPI_DBG_PORTS
 * 2. keep bit: "keep" flags, to keep the early console after the real consoles
 *    are available.
 *    offset=MAX_ACPI_DBG_PORTS, length=MAX_ACPI_DBG_PORTS
 * By combining such small length flags into one bitmap, we can save the memory
 * consumption.
 * Following macros defines the offset of the flags.
 */
#define ACPI_EARLY_ENABLE	0
#define ACPI_EARLY_KEEP		1

static __initdata DECLARE_BITMAP(acpi_early_flags, MAX_ACPI_DBG_PORTS*2);
static __initdata bool acpi_early_enabled;

static inline void acpi_early_set_flag(u8 port, int offset)
{
	BUG_ON(port >= MAX_ACPI_DBG_PORTS);
	set_bit(port + offset*MAX_ACPI_DBG_PORTS, acpi_early_flags);
}

static inline bool acpi_early_test_flag(u8 port, int offset)
{
	BUG_ON(port >= MAX_ACPI_DBG_PORTS);
	return test_bit(port + offset*MAX_ACPI_DBG_PORTS, acpi_early_flags);
}

static int __init acpi_early_console_enable(u8 port, int keep)
{
	if (port >= MAX_ACPI_DBG_PORTS)
		return -ENODEV;

	acpi_early_set_flag(port, ACPI_EARLY_ENABLE);
	if (keep)
		acpi_early_set_flag(port, ACPI_EARLY_KEEP);
	acpi_early_enabled = true;

	pr_debug("DBG LAUNCH - console %d.\n", port);

	return 0;
}

static bool __init acpi_early_console_enabled(u8 port)
{
	return acpi_early_test_flag(port, ACPI_EARLY_ENABLE);
}

bool __init acpi_early_console_keep(struct acpi_debug_port *info)
{
	BUG_ON(!info);
	return acpi_early_test_flag(info->port_index, ACPI_EARLY_KEEP);
}

/* Do not return failures as the caller will not check the returned result
 * in the "START" stage.  Errors are only get reported when the target table
 * does not exist.
 */
static void __init acpi_early_console_start(struct acpi_debug_port *info)
{
	if (!acpi_early_console_enabled(info->port_index))
		return;

	pr_debug("DBG START - console %d(%04x:%04x).\n",
		 info->port_index, info->port_type, info->port_subtype);
	(void)__acpi_early_console_start(info);
}

static int __init acpi_parse_dbg2(struct acpi_table_header *table)
{
	struct acpi_table_dbg2 *dbg2;
	struct acpi_dbg2_device *entry;
	void *tbl_end;
	u32 count = 0;
	u32 max_entries;
	struct acpi_debug_port devinfo;

	dbg2 = (struct acpi_table_dbg2 *)table;
	if (!dbg2) {
		pr_debug("DBG2 not present.\n");
		return -ENODEV;
	}

	tbl_end = (void *)table + table->length;

	entry = (struct acpi_dbg2_device *)((void *)dbg2 + dbg2->info_offset);
	max_entries = min_t(u32, MAX_ACPI_DBG_PORTS, dbg2->info_count);

	while (((void *)entry) + sizeof(struct acpi_dbg2_device) < tbl_end) {
		if (entry->revision != 0) {
			pr_debug("DBG2 revision %d not supported.\n",
				 entry->revision);
			return -ENODEV;
		}
		if (count < max_entries) {
			pr_debug("DBG PROBE - console %d(%04x:%04x).\n",
				 count, entry->port_type, entry->port_subtype);

			devinfo.port_index = (u8)count;
			devinfo.port_type = entry->port_type;
			devinfo.port_subtype = entry->port_subtype;
			devinfo.register_count = entry->register_count;
			devinfo.registers = (struct acpi_generic_address *)
			    ((void *)entry + entry->base_address_offset);
			devinfo.namepath_length = entry->namepath_length;
			devinfo.namepath = (char *)
			    ((void *)entry + entry->namepath_offset);
			devinfo.oem_data_length = entry->oem_data_length;
			devinfo.oem_data = (u8 *)
			    ((void *)entry + entry->oem_data_offset);

			/* Allow the "enabled" bit of the debug ports to be
			 * checked one by one.
			 */
			acpi_early_console_start(&devinfo);
			count++;
		}

		entry = (struct acpi_dbg2_device *)
			((void *)entry + entry->length);
	}

	return 0;
}

int __init acpi_early_console_probe(void)
{
	if (!acpi_early_enabled)
		return -EINVAL;

	if (acpi_table_parse(ACPI_SIG_DBG2, acpi_parse_dbg2) != 0)
		return -ENODEV;

	return 0;
}

int __init acpi_early_console_launch(char *s, int keep)
{
	char *e;
	unsigned long port = 0;

	if (*s)
		port = simple_strtoul(s, &e, 10);

	return acpi_early_console_enable(port, keep);
}
