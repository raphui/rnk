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

#ifndef SPI_H
#define SPI_H

struct spi {
	unsigned int num;
	unsigned int base_reg;
	unsigned int rate;	/* current rate */
	unsigned int speed;	/* wanted speed */
	unsigned short mode;
};

void spi_init(struct spi *spi);
unsigned short spi_write(struct spi *spi, unsigned short data);
unsigned short spi_read(struct spi *spi);

#endif /* SPI_H */
