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

#include <io.h>

#define MAX_BUFFER	128

static char buffer[MAX_BUFFER];

static inline int put_char(char *buff, char val)
{
	int num = 0;

	*buff = val;

	return num;
}

static inline int put_string(char *buff, char *s)
{

	int num = 0;

	while(*s) {
		*buff++ = *s++;
		num++;
	}


	return num;
}

static inline int put_int(char *buff, unsigned int val)
{

	int num = 0;

	/* Not implemented */


	return num;

}

static inline int put_uint(char *buff, unsigned int val)
{

	int num = 0;

	/* Not implemented */


	return num;

}

static int put_hex(char *buff, unsigned int val)
{
	int num = 0;

	if((val >> 4) > 0) {
		num += put_hex(buff, (val >> 4));
		buff += num;
	}


	if((val & 0xF) < 10)
		put_char(buff, (val & 0xF) + '0');
	else
		put_char(buff, (val & 0xF) - 10 + 'a');

	num++;

	return num;
}

int printk(const char *fmt, ...)
{
	va_list arg;

	char *p = buffer;

	int num = 0;

	while(*fmt) {
		if(*fmt != '%') {
			*p++ = *fmt++;
		} else if(*(fmt + 1) == '%') {
			*p++ = '%';
			fmt += 2;
		} else {
			fmt++;
			
			switch(*fmt) {
				case 'd':
				case 'i':
				case 'u':
				case 'x':
					*p++ = '0';
					*p++ = 'x';
					num = put_hex(p, va_arg(arg, unsigned int));
					break;
				case 's':
					num = put_string(p, va_arg(arg, char *));
					break;
				case 'c':
					num = put_char(p, (char)va_arg(arg, unsigned int));
					break;

				default:
					return -1;
			}

			fmt++;
			p += num;
		}
	}

	va_end(arg);

	*p = '\0';

	return io_op.write(buffer);
}
