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

#include <board.h>
#include <spi.h>

int spi_init(struct spi *spi)
{
	return spi_ops.init(spi);
}

int spi_write(struct spi *spi, unsigned char *buff, unsigned int size)
{
	return spi_ops.write(spi, buff, size);
}

int spi_read(struct spi *spi, unsigned char *buff, unsigned int size)
{
	return spi_ops.read(spi, buff, size);
}
