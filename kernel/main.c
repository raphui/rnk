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
#include <aic.h>
#include <utils.h>
#include <mm.h>
#include <mutex.h>
#include <semaphore.h>
#include <queue.h>
#include <arch/svc.h>
#include <time.h>

#ifdef STM32_F429
#include <ltdc.h>

struct ltdc ltdc;
#endif /* STM32_F429 */

struct mutex mutex;
struct semaphore sem;
struct queue queue;

struct usart usart;

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
	while(1) {
		printk("G");
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

#ifdef STM32_F429

#define GPIO_AF_LTDC	((unsigned char)0x0E)
#define GPIO_AF_LCD	((unsigned char)0x09)

//	pio_set_output(GPIOD_BASE, 3, 0);
//	pio_set_output(GPIOC_BASE, 2, 0);

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
	pio_set_alternate(GPIOD_BASE, 10, GPIO_AF_LTDC);

	pio_set_alternate(GPIOF_BASE, 10, GPIO_AF_LTDC);

	pio_set_alternate(GPIOG_BASE, 6, GPIO_AF_LTDC);
	pio_set_alternate(GPIOG_BASE, 7, GPIO_AF_LTDC);
	pio_set_alternate(GPIOG_BASE, 11, GPIO_AF_LTDC);

	pio_set_alternate(GPIOG_BASE, 10, GPIO_AF_LCD);
	pio_set_alternate(GPIOG_BASE, 12, GPIO_AF_LCD);


	ltdc.hsync = 10;
	ltdc.vsync = 2;
	ltdc.hbp = 20;
	ltdc.hfp = 10;
	ltdc.vbp = 2;
	ltdc.vfp = 4;
	ltdc.width = 240;
	ltdc.height = 320;

	lcd_init(&ltdc);
#endif /* STM32_F429 */

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
	add_task(&seventh_task, 1);
	add_task(&eighth_task, 1);

	printk("- Start scheduling...\r\n");
	start_schedule();

	while(1)
		;

	return 0; //Never reach
}
