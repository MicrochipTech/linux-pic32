/*
 * Joshua Henderson, joshua.henderson@microchip.com
 * Copyright (C) 2014 Microchip Technology Inc.  All rights reserved.
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
#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/bootinfo.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/traps.h>

#include <asm/fw/fw.h>
#include <asm/prom.h>
#include <asm/mips-boards/generic.h>

#include "common.h"

int prom_argc;
int *_prom_argv, *_prom_envp;

#define prom_envp(index) ((char *)(long)_prom_envp[(index)])
#define prom_argv(index) ((char *)(long)_prom_argv[(index)])

char * __init prom_getcmdline(void)
{
	return &(arcs_cmdline[0]);
}

void  __init prom_init_cmdline(void)
{
	char *cp;
	int actr;

	actr = 1;		/* Always ignore argv[0] */

	cp = &(arcs_cmdline[0]);
	while (actr < prom_argc) {
		strcpy(cp, prom_argv(actr));
		cp += strlen(prom_argv(actr));
		*cp++ = ' ';
		actr++;
	}
	if (cp != &(arcs_cmdline[0])) {
		/* get rid of trailing space */
		--cp;
		*cp = '\0';
	}
}

char *prom_getenv(char *envname)
{
	/*
	 * Return a pointer to the given environment variable.
	 * In 64-bit mode: we're using 64-bit pointers, but all pointers
	 * in the PROM structures are only 32-bit, so we need some
	 * workarounds, if we are running in 64-bit mode.
	 */
	int i, index = 0;

	i = strlen(envname);

	while (prom_envp(index)) {
		if (strncmp(envname, prom_envp(index), i) == 0)
			return prom_envp(index + 1);
		index += 2;
	}

	return NULL;
}

void __init prom_init(void)
{
	prom_argc = fw_arg0;
	_prom_argv = (int *)fw_arg1;
	_prom_envp = (int *)fw_arg2;

	fw_init_cmdline();

#ifdef CONFIG_EARLY_PRINTK
	fw_init_early_console(EARLY_CONSOLE_PORT);
#endif
}

void __init prom_free_prom_memory(void)
{
}
