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
#include <ili9341.h>
#include <pio.h>
#include <spi.h>

struct spi spi;

void ili9341_init(void)
{
	pio_set_alternate(GPIOF_BASE, 7, 0x5);
	pio_set_alternate(GPIOF_BASE, 8, 0x5);
	pio_set_alternate(GPIOF_BASE, 9, 0x5);

	spi.num = 5;
	spi.base_reg = SPI5_BASE;
	spi.rate = 0;
	spi.speed = 10000000;
	spi.mode = 1;

	spi_init(&spi);
}

void ili9341_init_lcd(void)
{
	int timeout = 1000000;

	ili9341_send_data(0xC3);
	ili9341_send_data(0x08);
	ili9341_send_data(0x50);
	ili9341_send_command(ILI9341_POWERB);
	ili9341_send_data(0x00);
	ili9341_send_data(0xC1);
	ili9341_send_data(0x30);
	ili9341_send_command(ILI9341_POWER_SEQ);
	ili9341_send_data(0x64);
	ili9341_send_data(0x03);
	ili9341_send_data(0x12);
	ili9341_send_data(0x81);
	ili9341_send_command(ILI9341_DTCA);
	ili9341_send_data(0x85);
	ili9341_send_data(0x00);
	ili9341_send_data(0x78);
	ili9341_send_command(ILI9341_POWERA);
	ili9341_send_data(0x39);
	ili9341_send_data(0x2C);
	ili9341_send_data(0x00);
	ili9341_send_data(0x34);
	ili9341_send_data(0x02);
	ili9341_send_command(ILI9341_PRC);
	ili9341_send_data(0x20);
	ili9341_send_command(ILI9341_DTCB);
	ili9341_send_data(0x00);
	ili9341_send_data(0x00);
	ili9341_send_command(ILI9341_FRC);
	ili9341_send_data(0x00);
	ili9341_send_data(0x1B);
	ili9341_send_command(ILI9341_DFC);
	ili9341_send_data(0x0A);
	ili9341_send_data(0xA2);
	ili9341_send_command(ILI9341_POWER1);
	ili9341_send_data(0x10);
	ili9341_send_command(ILI9341_POWER2);
	ili9341_send_data(0x10);
	ili9341_send_command(ILI9341_VCOM1);
	ili9341_send_data(0x45);
	ili9341_send_data(0x15);
	ili9341_send_command(ILI9341_VCOM2);
	ili9341_send_data(0x90);
	ili9341_send_command(ILI9341_MAC);
	ili9341_send_data(0xC8);
	ili9341_send_command(ILI9341_3GAMMA_EN);
	ili9341_send_data(0x00);
	ili9341_send_command(ILI9341_RGB_INTERFACE);
	ili9341_send_data(0xC2);
	ili9341_send_command(ILI9341_DFC);
	ili9341_send_data(0x0A);
	ili9341_send_data(0xA7);
	ili9341_send_data(0x27);
	ili9341_send_data(0x04);

	ili9341_send_command(ILI9341_COLUMN_ADDR);
	ili9341_send_data(0x00);
	ili9341_send_data(0x00);
	ili9341_send_data(0x00);
	ili9341_send_data(0xEF);

	ili9341_send_command(ILI9341_PAGE_ADDR);
	ili9341_send_data(0x00);
	ili9341_send_data(0x00);
	ili9341_send_data(0x01);
	ili9341_send_data(0x3F);
	ili9341_send_command(ILI9341_INTERFACE);
	ili9341_send_data(0x01);
	ili9341_send_data(0x00);
	ili9341_send_data(0x06);

	ili9341_send_command(ILI9341_GRAM);
	usleep(1000000);
	ili9341_send_command(ILI9341_GAMMA);
	ili9341_send_data(0x01);

	ili9341_send_command(ILI9341_PGAMMA);
	ili9341_send_data(0x0F);
	ili9341_send_data(0x29);
	ili9341_send_data(0x24);
	ili9341_send_data(0x0C);
	ili9341_send_data(0x0E);
	ili9341_send_data(0x09);
	ili9341_send_data(0x4E);
	ili9341_send_data(0x78);
	ili9341_send_data(0x3C);
	ili9341_send_data(0x09);
	ili9341_send_data(0x13);
	ili9341_send_data(0x05);
	ili9341_send_data(0x17);
	ili9341_send_data(0x11);
	ili9341_send_data(0x00);
	ili9341_send_command(ILI9341_NGAMMA);
	ili9341_send_data(0x00);
	ili9341_send_data(0x16);
	ili9341_send_data(0x1B);
	ili9341_send_data(0x04);
	ili9341_send_data(0x11);
	ili9341_send_data(0x07);
	ili9341_send_data(0x31);
	ili9341_send_data(0x33);
	ili9341_send_data(0x42);
	ili9341_send_data(0x05);
	ili9341_send_data(0x0C);
	ili9341_send_data(0x0A);
	ili9341_send_data(0x28);
	ili9341_send_data(0x2F);
	ili9341_send_data(0x0F);

	ili9341_send_command(ILI9341_SLEEP_OUT);
	usleep(1000000);
	ili9341_send_command(ILI9341_DISPLAY_ON);

	ili9341_send_command(ILI9341_GRAM);
}

void ili9341_send_command(unsigned char data)
{
	pio_clear_value(GPIOD_BASE, 13);
	pio_clear_value(GPIOC_BASE, 2);
	spi_write(&spi, data);
	pio_set_value(GPIOC_BASE, 2);
}

void ili9341_send_data(unsigned char data)
{
	pio_set_value(GPIOD_BASE, 13);
	pio_clear_value(GPIOC_BASE, 2);
	spi_write(&spi, data);
	pio_set_value(GPIOC_BASE, 2);
}
