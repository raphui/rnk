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
#include <thread.h>
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
#include <elfloader.h>
#include <unistd.h>
#include <mtd.h>

#ifdef CONFIG_UNWIND
#include <backtrace.h>
#endif /* CONFIG_UNWIND */

#ifdef CONFIG_STM32F429
#include <ili9341.h>
#include <ltdc.h>

struct ltdc ltdc;
#endif /* CONFIG_STM32F429 */

//#define FAULT

#ifdef CONFIG_INITCALL
#include <init.h>

extern initcall_t __rnk_initcalls_start[], __rnk_initcalls_end[];
extern exitcall_t __rnk_exitcalls_start[], __rnk_exitcalls_end[];
#endif /* CONFIG_INITCALL */

struct usart usart;
struct spi spi;
struct dma dma;
struct dma_transfer dma_trans;

static int count = 0;

void first_thread(void)
{

	printk("starting thread A\r\n");
	while (1) {
		mutex_lock(&mutex);
		printk("A");
		mutex_unlock(&mutex);
	}
}

void second_thread(void)
{
	printk("starting thread B\r\n");
	mutex_lock(&mutex);
	while (1) {
		printk("B");
		if (count++ == 500)
			mutex_unlock(&mutex);
	}
}

void third_thread(void)
{
	printk("starting thread C\r\n");
	sem_wait(&sem);
	count = 0;
	while (1) {
		printk("C");
		if (count++ == 4000)
			sem_post(&sem);
	}
}

void fourth_thread(void)
{
	unsigned int size = sizeof(unsigned int);
	unsigned char *p;
	unsigned int *array = (unsigned int *)kmalloc(size);
	int i = 0;

	printk("starting thread D\r\n");
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

void fifth_thread(void)
{
	printk("starting thread E\r\n");
	while (1) {
		sem_wait(&sem);
		printk("E");
		sem_post(&sem);
	}
}

void sixth_thread(void)
{
	printk("starting thread F\r\n");
	pio_set_output(GPIOE_BASE, 6, 0);
	while (1) {
		pio_set_value(GPIOE_BASE, 6);
		usleep(500);
		printk("F");
		pio_clear_value(GPIOE_BASE, 6);
	}
}

void seventh_thread(void)
{
	int a = 5;
	count = 0;
	printk("starting thread G\r\n");
	printk("#####a(%x): %d\r\n", &a , a);
	while (1) {
		printk("G");
		if (count++ == 4000)
			queue_post(&queue, &a, 0);
	}
}


void eighth_thread(void)
{
	int b = 0;
	printk("starting thread H\r\n");
	queue_receive(&queue, &b, 10000);
	printk("#####b(%x): %d\r\n", &b, b);
	while (1) {
		printk("H");
	}

}

#ifdef CONFIG_STM32F429
#ifdef CONFIG_LCD_SUBSYS
static void ltdc_init(void)
{
	lcd_init_gpio();

	ltdc.hsync = 16;
	ltdc.vsync = 2;
	ltdc.hbp = 40;
	ltdc.hfp = 10;
	ltdc.vbp = 2;
	ltdc.vfp = 4;
	ltdc.width = 240;
	ltdc.height = 320;
	ltdc.bpp = 2;
	ltdc.fb_addr = 0xD0000000;

	lcd_init(&ltdc);
}

#define BLACK		0x0000
#define BLUE		0x001F
#define RED		0xF800
#define GREEN		0x07E0
#define CYAN		0x07FF
#define MAGENTA		0xF81F
#define YELLOW		0xFFE0
#define WHITE		0xFFFF
#define GREY		0x8C51

#define MAX_DMA_SIZE	0xFFFF

#define LCD_DMA_FILL

unsigned short color = 0;

void lcd_rgb565_fill(unsigned short rgb)
{
	int size = ltdc.width * ltdc.height * ltdc.bpp;
	int i = 0;
	int num = 0;
	int remain = 0;
	unsigned int p = ltdc.fb_addr;
	unsigned short *l = ltdc.fb_addr;

	debug_printk("lcd_rgb565_fill\r\n");

#ifdef LCD_DMA_FILL

	color = rgb;

	dma_trans.src_addr = &color;

	num = size / MAX_DMA_SIZE;
	remain = size % MAX_DMA_SIZE;

	debug_printk("transfer: ");

	for (i = 0; i < num; i++) {
		dma_disable(&dma);
		debug_printk("%d ", i);
		dma_trans.dest_addr = p;
		dma_trans.size = MAX_DMA_SIZE;
		dma_transfer(&dma, &dma_trans);
		p += MAX_DMA_SIZE;
		dma_enable(&dma);
		usleep(1000000);
	}

	dma_disable(&dma);
	dma_trans.dest_addr = p;
	dma_trans.size = remain;
	dma_transfer(&dma, &dma_trans);
	dma_enable(&dma);
#else

	for (i = 0; i < size; i++) {
		*l++ = rgb;
	}

#endif /* LCD_DMA_FILL */

	sem_wait(&sem);
}
#endif /* CONFIG_LCD_SUBSYS */
#endif /* CONFIG_STM32F429 */


void ninth_thread(void)
{
	printk("starting thread I\r\n");

#ifdef CONFIG_STM32F429
#if defined(CONFIG_LCD_SUBSYS) && defined(CONFIG_ILI9341)
	ltdc_init();
	ili9341_init();
	ili9341_init_lcd();
#endif /* CONFIG_LCD_SUBSYS && CONFIG_ILI9341 */
#elif defined(CONFIG_STM32F746)
	pio_set_output(GPIOI_BASE, 1, 0);
#endif /* CONFIG_STM32F429 */

	while (1) {
		printk("I");
#ifdef CONFIG_STM32F429
#if defined(CONFIG_LCD_SUBSYS) && defined(CONFIG_ILI9341)
		lcd_rgb565_fill(BLACK);
		sem_wait(&sem);
		lcd_rgb565_fill(BLUE);
		sem_wait(&sem);
		lcd_rgb565_fill(RED);
		sem_wait(&sem);
		lcd_rgb565_fill(GREEN);
		sem_wait(&sem);
		lcd_rgb565_fill(CYAN);
		sem_wait(&sem);
		lcd_rgb565_fill(MAGENTA);
		sem_wait(&sem);
		lcd_rgb565_fill(YELLOW);
		sem_wait(&sem);
		lcd_rgb565_fill(WHITE);
#endif /* CONFIG_LCD_SUBSYS && CONFIG_ILI9341 */
#elif defined(CONFIG_STM32F746)
		pio_toggle_value(GPIOI_BASE, 6);
#endif /* CONFIG_STM32F429 */
	}
}

void tenth_thread(void)
{
	int (*fct)(void) = 0x50002540;
	printk("starting thread J\r\n");

	while (1) {
		fct();
	}
}

void eleventh_thread(void)
{
	int ret;

	printk("starting thread K\r\n");

	ret = elf_exec((char *)0x08050000, 220417, 0x08050000);
	if (ret < 0)
		printk("failed to exec elf\r\n");
	else
		printk("efl execution done\r\n");

	while (1)
		;
//	{
//		printk("K");
//	}
	
}

int main(void)
{
	int fd;
	unsigned char c;

#ifdef CONFIG_INITCALL
	int ret;
	initcall_t *initcall;

	for (initcall = __rnk_initcalls_start; initcall < __rnk_initcalls_end; initcall++) {
		debug_printk("initcall-> %pS\n", *initcall);
		ret = (*initcall)();
		if (ret < 0)
			error_printk("initcall %pS failed: %d\n", *initcall, ret);
	}
#endif /* CONFIG_INITCALL */

#ifndef CONFIG_INITCALL
	init_heap();
#endif /* CONFIG_INITCALL */

#ifndef CONFIG_INITCALL
#ifdef CONFIG_STM32F429
	usart_init(1, USART1_BASE, 115200);
	pio_set_alternate(GPIOA_BASE, 9, 0x7);
	pio_set_alternate(GPIOA_BASE, 10, 0x7);
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
#elif defined (CONFIG_STM32F407)
	usart_init(3, USART3_BASE, 115200);
	pio_set_alternate(GPIOC_BASE, 10, 0x7);
	pio_set_alternate(GPIOC_BASE, 11, 0x7);
#elif defined (CONFIG_STM32F401)
	usart_init(2, USART2_BASE, 115200);
	pio_set_alternate(GPIOA_BASE, 2, 0x7);
	pio_set_alternate(GPIOA_BASE, 3, 0x7);
#endif /* CONFIG_STM32F429 */
#endif /* CONFIG_INITCALL */

	printk("Welcome to rnk\r\n");

	printk("- Initialise scheduler...\r\n");

#ifndef CONFIG_INITCALL
	schedule_init();
	time_init();
#endif /* CONFIG_INITCALL */

	init_mutex(&mutex);
	init_semaphore(&sem, 1);
	init_queue(&queue, sizeof(int), 5);

#ifdef CONFIG_UNWIND
	unwind_init();
#endif /* CONFIG_UNWIND */

	printk("- Add thread to scheduler\r\n");

//	add_thread(&first_thread, 1);
//	add_thread(&second_thread, 6);
//	add_thread(&third_thread, 2);
//	add_thread(&fourth_thread, 20);
//	add_thread(&fifth_thread, 1);
//	add_thread(&sixth_thread, 1);
//	add_thread(&seventh_thread, 1);
//	add_thread(&eighth_thread, 1);
//	add_thread(&ninth_thread, 1);
#ifdef FAULT
	add_thread(&tenth_thread, 1);
#endif /* FAULT */
	add_thread(&eleventh_thread, 2);

	printk("- Start scheduling...\r\n");
	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
