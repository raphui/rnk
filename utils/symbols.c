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

static struct sym symbols[] = {
/* Generate symbols table is insert here */

};

char *symbol_get_name(unsigned int addr)
{
	int size = sizeof(symbols) / sizeof(struct sym);
	int i = 0;
	char *ret = NULL;

	for (i = 0; i < size; i++) {
		if (symbols[i].addr == addr) {
			ret = symbols[i].name;
			break;
		} else if ((addr > symbols[i].addr) && (addr < (symbols[i].addr + symbols[i].size))) {
			ret = symbols[i].name;
			break;	
		}
	}

	return ret;
}

int symbol_get_addr(char *name)
{
	int size = sizeof(symbols) / sizeof(struct sym);
	int i = 0;
	int ret = -ENXIO;

	for (i = 0; i < size; i++) {
		if (!strcmp(name, symbols[i].name)) {
			ret = symbols[i].addr;
			break;
		}
	}

	return ret;
}
