/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <string.h>

void memcpy(void *dst, const void *src, unsigned int num)
{
	const unsigned char *s = src;
	unsigned char *d = dst;

	while (num--) {
		*d++ = *s++;
	}
}

void *memset(void *s, int c, unsigned int count)
{
	char *xs = (char *)s;

	while (count--)
		*xs++ = c;

	return s;
}

int strcmp(const char *cs, const char *ct)
{
	char res;

	while (1) {
		if ((res = *cs - *ct++) != 0 || !*cs++)
			break;
	}

	return res;
}
