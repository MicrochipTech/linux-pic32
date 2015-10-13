/*
 *  Joshua Henderson, joshua.henderson@microchip.com
 *  Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 */
#include <linux/init.h>
#include <linux/libfdt.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>

#include <asm/bootinfo.h>
#include <asm/mips-boards/generic.h>
#include <asm/prom.h>
#include <asm/fw/fw.h>

#include <linux/platform_data/sdhci-pic32.h>
#include <linux/platform_data/lcd-pic32.h>

#include "common.h"

const char *get_system_type(void)
{
	return "PIC32MZDA";
}

static ulong get_fdtaddr(void)
{
	ulong ftaddr = 0;

	if ((fw_arg0 == -2) && fw_arg1 && !fw_arg2 && !fw_arg3)
		return (ulong) fw_arg1;

	if (__dtb_start < __dtb_end)
		ftaddr = (ulong) __dtb_start;

	return ftaddr;
}

void __init plat_mem_setup(void)
{
	void *dtb;

	dtb = (void *)get_fdtaddr();
	if (!dtb) {
		pr_err("pic32: no DTB found.\n");
		return;
	}

	/*
	 * Load the builtin device tree. This causes the chosen node to be
	 * parsed resulting in our memory appearing.
	 */
	__dt_setup_arch(dtb);

	pr_info("Found following command lines\n");
	pr_info(" boot_command_line: %s\n", boot_command_line);
	pr_info(" arcs_cmdline     : %s\n", arcs_cmdline);
#ifdef CONFIG_CMDLINE_BOOL
	pr_info(" builtin_cmdline  : %s\n", CONFIG_CMDLINE);
#endif
	if (dtb != __dtb_start)
		strlcpy(arcs_cmdline, boot_command_line, COMMAND_LINE_SIZE);

#ifdef CONFIG_EARLY_PRINTK
	fw_init_early_console(-1);
#endif
	pic32_config_init();
}

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	unflatten_and_copy_device_tree();
}

static struct pic32_sdhci_platform_data sdhci_data = {
	.setup_dma = pic32_set_sdhc_adma_fifo_threshold,
};

static struct pic32_lcd_pdata glcd_data = {
	.enable = pic32_enable_lcd,
	.disable = pic32_disable_lcd,
	.set_mode = pic32_set_lcd_mode,
};

static struct of_dev_auxdata pic32_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("microchip,pic32-sdhci", 0x1f8ec000,
		       "sdhci", &sdhci_data),
	OF_DEV_AUXDATA("microchip,pic32-lcd", 0x1f8ea000, "fb", &glcd_data),
	{ /* sentinel */}
};

static int __init customize_machine(void)
{
	if (!of_have_populated_dt())
		panic("Device tree not present");

	of_platform_populate(NULL, of_default_bus_match_table,
			     pic32_auxdata_lookup, NULL);

	return 0;
}
arch_initcall(customize_machine);
