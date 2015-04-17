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
#include <stdio.h>
#include <utils.h>
#include <interrupt.h>
#include <scheduler.h>
#include <armv7m/system.h>
#include <arch/nvic.h>
#include <time.h>
#include <pio.h>


void hardfault_handler(void)
{
	while (1)
		;
}

void memmanage_handler(void)
{
	while (1)
		;
}

void busfault_handler(void)
{
	while (1)
		;
}

void usagefault_handler(void)
{
	while (1)
		;
}

void systick_handler(void)
{
	unsigned int val = readl(SCB_ICSR);

	system_tick++;

	val |= SCB_ICSR_PENDSVSET;
	writel(SCB_ICSR, val);
}

void pendsv_handler(void)
{
	schedule_task(NULL);
}

void timer2_handler(void)
{
	nvic_clear_interrupt(28);
	debug_printk("timer2 trig\r\n");
//	pio_toggle_value(GPIOE_BASE, 6);
	decrease_task_delay();
}


void i2c1_event_handler(void)
{
}

void i2c1_error_handler(void)
{
}

void dma2_stream0_handler(void)
{
	if (DMA2->LISR & DMA_LISR_TCIF0) {
		debug_printk("transfert complete\r\n");
		DMA2->LIFCR = DMA_LIFCR_CTCIF0;
	}

	if (DMA2->LISR & DMA_LISR_HTIF0) {
		debug_printk("half transfer interrupt\r\n");
		DMA2->LIFCR = DMA_LIFCR_CHTIF0;
	}

	if (DMA2->LISR & DMA_LISR_TEIF0) {
		debug_printk("transfert error\r\n");
		DMA2->LIFCR = DMA_LIFCR_CTEIF0;

	}
	
	if (DMA2->LISR & DMA_LISR_DMEIF0) {
		debug_printk("direct mode error\r\n");
		DMA2->LIFCR = DMA_LIFCR_CDMEIF0;
	}

	if (DMA2->LISR & DMA_LISR_FEIF0) {
		debug_printk("fifo error\r\n");
		DMA2->LIFCR = DMA_LIFCR_CFEIF0;
	}
}
