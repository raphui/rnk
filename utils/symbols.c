/*
 * Copyright (C) 2014  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>

#include <symbols.h>

extern struct symbol __rnk_ksym_start[];
extern struct symbol __rnk_ksym_end[];

char *symbol_get_name(unsigned int addr)
{
	char *ret = NULL;
	struct symbol *sym;

	for (sym = __rnk_ksym_start; sym < __rnk_ksym_end; sym++) {
		if (sym->addr == addr) {
			ret = sym->name;
			break;
		}
	}

	return ret;
}

int symbol_get_addr(char *name)
{
	int ret = -ENXIO;
	struct symbol *sym;

	for (sym = __rnk_ksym_start; sym < __rnk_ksym_end; sym++) {
		if (!strcmp(name, sym->name)) {
			ret = sym->addr;
			break;
		}
	}

	return ret;
}
