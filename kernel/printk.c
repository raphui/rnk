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

#include <printk.h>
#include <errno.h>
#include <console.h>

static void putchar(unsigned char c)
{
	console_write(&c, sizeof(unsigned char));
}

static void puts_x(char *str, int width, const char pad)
{
	while (*str) {
		if (*str == '\n')
			putchar('\r');
		putchar(*(str++));
		--width;
	}

	while (width > 0) {
		putchar(pad);
		--width;
	}
}

#define hexchars(x) \
	(((x) < 10) ? \
		('0' + (x)) : \
	 	('a' + ((x) - 10)))

static int put_hex(const unsigned int val, int width, const char pad)
{
	int i, n = 0;
	int nwidth = 0;

	/* Find width of hexnumber */
	while ((val >> (4 * nwidth)) && ((unsigned) nwidth <  2 * sizeof(val)))
		nwidth++;
	if (nwidth == 0)
		nwidth = 1;

	/* May need to increase number of printed characters */
	if (width == 0 && width < nwidth)
		width = nwidth;

	/* Print number with padding */
	for (i = width - nwidth; i > 0; i--, n++)
		putchar(pad);
	for (i = 4 * (nwidth - 1); i >= 0; i -= 4, n++)
		putchar(hexchars((val >> i) & 0xF));

	return n;
}

static void put_dec(const unsigned int val, const int width, const char pad)
{
	unsigned int divisor;
	int digits;

	/* estimate number of spaces and digits */
	for (divisor = 1, digits = 1; val / divisor >= 10; divisor *= 10, digits++)
		/* */ ;

	/* print spaces */
	for (; digits < width; digits++)
		putchar(pad);

	/* print digits */
	do {
		putchar(((val / divisor) % 10) + '0');
	} while (divisor /= 10);
}


void printk(char *fmt, ...)
{
	va_list va;
	va_start(va, fmt);

	vprintk(fmt, va);
}

void vprintk(char *fmt, va_list va)
{
	int mode = 0;	/* 0: usual char; 1: specifiers */
	int width = 0;
	char pad = ' ';
	int size = 16;

	while (*fmt) {
		if (*fmt == '%') {
			mode = 1;
			pad = ' ';
			width = 0;
			size = 32;

			fmt++;
			continue;
		}

		if (!mode) {
			if (*fmt == '\n')
				putchar('\r');
			putchar(*fmt);
		} else {
			switch (*fmt) {
			case 'c':
				putchar(va_arg(va, unsigned int));
				mode = 0;
				break;
			case 's':
				puts_x(va_arg(va, char *), width, pad);
				mode = 0;
				break;
			case 'l':
			case 'L':
				size = 64;
				break;
			case 'd':
			case 'D':
				put_dec((size == 32) ?
				        va_arg(va, unsigned int) :
				        va_arg(va, unsigned long long),
				        width, pad);
				mode = 0;
				break;
			case 'p':
			case 't':
				size = 32;
				width = 8;
				pad = '0';
			case 'x':
			case 'X':
				put_hex((size == 32) ?
					va_arg(va, unsigned int) :
				        va_arg(va, unsigned long long),
				        width, pad);
				mode = 0;
				break;
			case '%':
				putchar('%');
				mode = 0;
				break;
			case '0':
				if (!width)
					pad = '0';
				break;
			case ' ':
				pad = ' ';
			}

			if (*fmt >= '0' && *fmt <= '9') {
				width = width * 10 + (*fmt - '0');
			}
		}

		fmt++;
	}
}
