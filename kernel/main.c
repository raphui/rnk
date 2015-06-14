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

#include <board.h>
#include <usart.h>
#include <stdio.h>
#include <scheduler.h>
#include <task.h>
#include <interrupt.h>
#include <pio.h>
#include <utils.h>
#include <mm.h>
#include <mutex.h>
#include <semaphore.h>
#include <queue.h>
#include <arch/svc.h>
#include <time.h>
#include <spi.h>
#include <dma.h>
#include <common.h>

#ifdef STM32_F429
#include <ltdc.h>

struct ltdc ltdc;
#endif /* STM32_F429 */


struct usart usart;
struct spi spi;
struct dma dma;
struct dma_transfer dma_trans;

static int count = 0;

void first_task(void)
{

	printk("starting task A\r\n");
	while (1) {
		mutex_lock(&mutex);
		printk("A");
		mutex_unlock(&mutex);
	}
}

void second_task(void)
{
	printk("starting task B\r\n");
	mutex_lock(&mutex);
	while (1) {
		printk("B");
		if (count++ == 500)
			mutex_unlock(&mutex);
	}
}

void third_task(void)
{
	printk("starting task C\r\n");
	sem_wait(&sem);
	count = 0;
	while (1) {
		printk("C");
		if (count++ == 4000)
			sem_post(&sem);
	}
}

void fourth_task(void)
{
	unsigned int size = sizeof(unsigned int);
	unsigned char *p;
	unsigned int *array = (unsigned int *)kmalloc(size);
	int i = 0;

	printk("starting task D\r\n");
	printk("array (%x)\r\n", array);

	p = (unsigned char *)kmalloc(24);
	*p = 0xea;
	printk("p(%x): %x\r\n", p, *p);
	kfree(p);

	while (1) {
		for (i = 0; i < size; i++) {
			printk("D");
			array[i] = (1 << i);
			printk("%x ", array[i]);
		}

		for (i = 0; i < size; i++) {
			kfree(array);
		}
	}
}

void fifth_task(void)
{
	printk("starting task E\r\n");
	while (1) {
		sem_wait(&sem);
		printk("E");
		sem_post(&sem);
	}
}

void sixth_task(void)
{
	printk("starting task F\r\n");
	pio_set_output(GPIOE_BASE, 6, 0);
	while (1) {
		pio_set_value(GPIOE_BASE, 6);
		usleep(500);
		printk("F");
		pio_clear_value(GPIOE_BASE, 6);
	}
}

void seventh_task(void)
{
	int a = 5;
	count = 0;
	printk("starting task H\r\n");
	printk("#####a(%x): %d\r\n", &a , a);
	while (1) {
		printk("H");
		if (count++ == 4000)
			queue_post(&queue, &a, 0);
	}
}


void eighth_task(void)
{
	int b = 0;
	printk("starting task G\r\n");
	queue_receive(&queue, &b, 10000);
	printk("#####b(%x): %d\r\n", &b, b);
	while (1) {
		printk("G");
	}

}

/* Commands */
#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MAC				0x36
#define ILI9341_PIXEL_FORMAT			0x3A
#define ILI9341_WDB				0x51
#define ILI9341_WCD				0x53
#define ILI9341_RGB_INTERFACE			0xB0
#define ILI9341_FRC				0xB1
#define ILI9341_BPC				0xB5
#define ILI9341_DFC				0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC				0xF7

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
	timeout = 1000000;
	while (timeout--)
		;
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
	timeout = 1000000;
	while (timeout--)
		;
	ili9341_send_command(ILI9341_DISPLAY_ON);

	ili9341_send_command(ILI9341_GRAM);
}

#ifdef STM32_F429
static void ltdc_init(void)
{
#define GPIO_AF_LTDC	((unsigned char)0x0E)
#define GPIO_AF_LCD	((unsigned char)0x09)

	ltdc.hsync = 16;
	ltdc.vsync = 2;
	ltdc.hbp = 40;
	ltdc.hfp = 10;
	ltdc.vbp = 2;
	ltdc.vfp = 4;
	ltdc.width = 240;
	ltdc.height = 320;
	ltdc.bpp = 2;
	ltdc.fb_addr = 0xd0000000;

	lcd_init(&ltdc);
}

#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

#define MAX_DMA_SIZE	0xFFFF

unsigned short color = 0;

void lcd_rgb565_fill(unsigned short rgb)
{
	int size = ltdc.width * ltdc.height * ltdc.bpp;
	int i = 0;
	int num = 0;
	int remain = 0;
	unsigned int *p = ltdc.fb_addr;

	color = rgb;

	dma_trans.src_addr = &color;

	dma_enable(&dma);

	num = size / MAX_DMA_SIZE;
	remain = size % MAX_DMA_SIZE;

	for (i = 0; i < num; i++) {
		dma_trans.dest_addr = p;
		dma_trans.size = MAX_DMA_SIZE;
		dma_transfer(&dma, &dma_trans);
		p += MAX_DMA_SIZE;
	}

	dma_trans.dest_addr = p;
	dma_trans.size = remain;
}
#endif /* STM32_F429 */


void ninth_task(void)
{
	printk("starting task I\r\n");
#ifdef STM32_F429
	pio_set_alternate(GPIOA_BASE, 3, GPIO_AF_LTDC);
	pio_set_alternate(GPIOA_BASE, 4, GPIO_AF_LTDC);
	pio_set_alternate(GPIOA_BASE, 6, GPIO_AF_LTDC);
	pio_set_alternate(GPIOA_BASE, 11, GPIO_AF_LTDC);
	pio_set_alternate(GPIOA_BASE, 12, GPIO_AF_LTDC);

	pio_set_alternate(GPIOB_BASE, 0, GPIO_AF_LCD);
	pio_set_alternate(GPIOB_BASE, 1, GPIO_AF_LCD);
	pio_set_alternate(GPIOB_BASE, 8, GPIO_AF_LTDC);
	pio_set_alternate(GPIOB_BASE, 9, GPIO_AF_LTDC);
	pio_set_alternate(GPIOB_BASE, 10, GPIO_AF_LTDC);
	pio_set_alternate(GPIOB_BASE, 11, GPIO_AF_LTDC);

	pio_set_alternate(GPIOC_BASE, 6, GPIO_AF_LTDC);
	pio_set_alternate(GPIOC_BASE, 7, GPIO_AF_LTDC);
	pio_set_alternate(GPIOC_BASE, 10, GPIO_AF_LTDC);

	pio_set_alternate(GPIOD_BASE, 3, GPIO_AF_LTDC);
	pio_set_alternate(GPIOD_BASE, 6, GPIO_AF_LTDC);

	pio_set_alternate(GPIOF_BASE, 10, GPIO_AF_LTDC);

	pio_set_alternate(GPIOG_BASE, 6, GPIO_AF_LTDC);
	pio_set_alternate(GPIOG_BASE, 7, GPIO_AF_LTDC);
	pio_set_alternate(GPIOG_BASE, 10, GPIO_AF_LCD);
	pio_set_alternate(GPIOG_BASE, 11, GPIO_AF_LTDC);
	pio_set_alternate(GPIOG_BASE, 12, GPIO_AF_LCD);

	pio_set_output(GPIOD_BASE, 13, 0);
	pio_set_output(GPIOD_BASE, 12, 1);
	pio_set_output(GPIOC_BASE, 2, 0);

	pio_set_value(GPIOC_BASE, 2);
	ili9341_init_lcd();
	ltdc_init();
#endif /* STM32_F429 */

	while (1) {
		printk("I");
		lcd_rgb565_fill(BLACK);
		usleep(100000000);
		lcd_rgb565_fill(BLUE);
		usleep(100000000);
		lcd_rgb565_fill(RED);
		usleep(100000000);
		lcd_rgb565_fill(GREEN);
		usleep(100000000);
		lcd_rgb565_fill(CYAN);
		usleep(100000000);
		lcd_rgb565_fill(MAGENTA);
		usleep(100000000);
		lcd_rgb565_fill(YELLOW);
		usleep(100000000);
		lcd_rgb565_fill(WHITE);
	}
}

int main(void)
{

#ifdef STM32_F429
	usart.num = 1;
	usart.base_reg = USART1_BASE;
	usart.baud_rate = 115200;

	usart_init(&usart);

	pio_set_alternate(GPIOA_BASE, 9, 0x7);
	pio_set_alternate(GPIOA_BASE, 10, 0x7);
#else
	usart.num = 3;
	usart.base_reg = USART3_BASE;
	usart.baud_rate = 115200;

	usart_init(&usart);

	pio_set_alternate(GPIOC_BASE, 10, 0x7);
	pio_set_alternate(GPIOC_BASE, 11, 0x7);
#endif /* STM32_F429 */

	pio_set_alternate(GPIOF_BASE, 7, 0x5);
	pio_set_alternate(GPIOF_BASE, 8, 0x5);
	pio_set_alternate(GPIOF_BASE, 9, 0x5);

	spi.num = 5;
	spi.base_reg = SPI5_BASE;
	spi.rate = 0;
	spi.speed = 10000000;
	spi.mode = 1;

	spi_init(&spi);

	dma.num = 2;
	dma.stream_base = DMA2_Stream0_BASE;
	dma.stream_num = 0;
	dma.channel = 0;
	dma.dir = DMA_M_M;
	dma.mdata_size = DATA_SIZE_HALF_WORD;
	dma.pdata_size = DATA_SIZE_HALF_WORD;
	dma.mburst = INCR0;
	dma.pburst = INCR0;
	dma.minc = 1;
	dma.pinc = 0;
	dma.use_fifo = 0;

	dma_init(&dma);

	/* User button */
	pio_set_input(GPIOA_BASE, 0, 0, 0);

	printk("Welcome to rnk\r\n");
	printk("- Initialise heap...\r\n");

	init_heap();

	printk("- Initialise scheduler...\r\n");
	schedule_init();

	init_mutex(&mutex);
	init_semaphore(&sem, 1);
	init_queue(&queue, sizeof(int), 5);
	time_init();

	printk("- Add task to scheduler\r\n");

//	add_task(&first_task, 1);
//	add_task(&second_task, 6);
//	add_task(&third_task, 2);
//	add_task(&fourth_task, 20);
//	add_task(&fifth_task, 1);
//	add_task(&sixth_task, 1);
//	add_task(&seventh_task, 1);
//	add_task(&eighth_task, 1);
	add_task(&ninth_task, 1);

	printk("- Start scheduling...\r\n");
	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
