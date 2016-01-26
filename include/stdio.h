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

#ifndef STDIO_H
#define STDIO_H

#include <stdarg.h>

#define DEBUG	0
#define ERROR	1
#define VERBOSE 0

#define verbose_printk(...) do{ if(VERBOSE){ printk(__VA_ARGS__); } }while(0)
#define debug_printk(...) do{ if(DEBUG){ printk(__VA_ARGS__); } }while(0)
#define error_printk(...) do{ if(ERROR){ printk(__VA_ARGS__); } }while(0)

struct io_operations
{
	void (*write)(unsigned char c);
	int (*write_string)(const char *string);
};

extern struct io_operations io_op;

void printk(char *fmt, ...);
void vprintf(char *fmt, va_list va);

#endif /* STDIO_H */
