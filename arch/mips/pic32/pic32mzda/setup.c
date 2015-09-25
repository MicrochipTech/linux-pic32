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

const char *get_system_type(void)
{
	return "PIC32MZDA";
}

static ulong get_fdtaddr_from_env(void)
{
	ulong ftaddr = 0;
	char *p;

	p = fw_getenv("fdtaddr");
	if (p)
		ftaddr = memparse(p, NULL);

	return ftaddr;
}

static ulong get_fdtaddr(void)
{
	ulong ftaddr = 0;

	pr_info("Determined boot loader arguments\n");
	pr_info
	    ("arg0 0x%08lx, arg1 0x%08lx, arg2 0x%08lx, arg3 0x%08lx, builtin_dtb 0x%08lx\n",
	     (ulong) fw_arg0, (ulong) fw_arg1, (ulong) fw_arg2, (ulong) fw_arg3,
	     (ulong) __dtb_start);

	if ((fw_arg0 == -2) && fw_arg1 && !fw_arg2 && !fw_arg3)
		return (ulong) fw_arg1;

	ftaddr = get_fdtaddr_from_env();
	if (ftaddr)
		return ftaddr;

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
}

void __init device_tree_init(void)
{
	if (!initial_boot_params)
		return;

	unflatten_and_copy_device_tree();
}

static int __init customize_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	return 0;
}

arch_initcall(customize_machine);
