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
#include <common.h>
#include <queue.h>
#include <symbols.h>
#include <backtrace.h>

static void dump_stack(unsigned int *stack)
{
	volatile unsigned int lr;
	volatile unsigned int pc;

	lr = stack[5];
	pc = stack[6];

#ifdef UNWIND
	/* fp = 0, because we don't care about it */
	unwind_backtrace(0, (unsigned int)stack, lr, pc);
#endif /* UNWIND */

	while (1)
		;
}


void hardfault_handler(void)
{
	asm volatile (
		"tst lr, #4			\n"
		"ite eq				\n"
		"mrseq r0, msp			\n"
		"mrsne r0, psp			\n"
//		"ldr r1, [r0, #24]		\n"
//		"ldr r2, _dump_stack		\n"
		"b dump_stack			\n"
//		"_dump_stack: .word dump_stack	\n"
	);	

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
	nvic_clear_interrupt(TIM2_IRQn);
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
	nvic_clear_interrupt(DMA2_Stream0_IRQn);

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

void dma2_stream1_handler(void)
{
	nvic_clear_interrupt(DMA2_Stream1_IRQn);

	if (DMA2->LISR & DMA_LISR_TCIF1) {
		debug_printk("transfert complete\r\n");
		DMA2->LIFCR = DMA_LIFCR_CTCIF1;
	}

	if (DMA2->LISR & DMA_LISR_HTIF1) {
		debug_printk("half transfer interrupt\r\n");
		DMA2->LIFCR = DMA_LIFCR_CHTIF1;
	}

	if (DMA2->LISR & DMA_LISR_TEIF1) {
		debug_printk("transfert error\r\n");
		DMA2->LIFCR = DMA_LIFCR_CTEIF1;

	}
	
	if (DMA2->LISR & DMA_LISR_DMEIF1) {
		debug_printk("direct mode error\r\n");
		DMA2->LIFCR = DMA_LIFCR_CDMEIF1;
	}

	if (DMA2->LISR & DMA_LISR_FEIF1) {
		debug_printk("fifo error\r\n");
		DMA2->LIFCR = DMA_LIFCR_CFEIF1;
	}
}

void dma2_stream4_handler(void)
{
	nvic_clear_interrupt(DMA2_Stream4_IRQn);

	if (DMA2->HISR & DMA_HISR_TCIF4) {
		printk("transfert complete\r\n");
		DMA2->HIFCR = DMA_HIFCR_CTCIF4;
	}

	if (DMA2->HISR & DMA_HISR_HTIF4) {
		printk("half transfer interrupt\r\n");
		DMA2->HIFCR = DMA_HIFCR_CHTIF4;
	}

	if (DMA2->HISR & DMA_HISR_TEIF4) {
		printk("transfert error\r\n");
		DMA2->HIFCR = DMA_HIFCR_CTEIF4;
	}

	if (DMA2->HISR & DMA_HISR_DMEIF4) {
		printk("direct mode error\r\n");
		DMA2->HIFCR = DMA_HIFCR_CDMEIF4;
	}

	if (DMA2->HISR & DMA_HISR_FEIF4) {
		printk("fifo error\r\n");
		DMA2->HIFCR = DMA_HIFCR_CFEIF4;
	}
}

void spi5_handler(void)
{
	int tmp;

	nvic_clear_interrupt(SPI5_IRQn);
	nvic_disable_interrupt(SPI5_IRQn);

	printk("SR: 0x%x\r\n", SPI5->SR);

	if ((SPI5->SR & SPI_SR_TXE) && !(SPI5->SR & SPI_SR_BSY)) {
		SPI5->CR2 &= ~SPI_CR2_TXDMAEN;
		printk("spi transfer ready\r\n");
		tmp = 1;
		svc_queue_post(&queue, &tmp);
	}

	if (SPI5->SR & SPI_SR_OVR) {
		printk("spi overrun\r\n");
		tmp = SPI5->DR;
	}
}

void exti0_handler(void)
{
	nvic_clear_interrupt(EXTI0_IRQn);
	EXTI->PR |= (1 << 0);

	debug_printk("exti0_handler\r\n");

	sem_post_isr(&sem);
}

void exti1_handler(void)
{
	nvic_clear_interrupt(EXTI1_IRQn);
	EXTI->PR |= (1 << 1);

	debug_printk("exti1_handler\r\n");
}

void exti2_handler(void)
{
	nvic_clear_interrupt(EXTI2_IRQn);
	EXTI->PR |= (1 << 2);

	debug_printk("exti2_handler\r\n");
}
void exti3_handler(void)
{
	nvic_clear_interrupt(EXTI3_IRQn);
	EXTI->PR |= (1 << 3);

	debug_printk("exti3_handler\r\n");
}
void exti4_handler(void)
{
	nvic_clear_interrupt(EXTI4_IRQn);
	EXTI->PR |= (1 << 4);

	debug_printk("exti4_handler\r\n");
}

void exti9_5_handler(void)
{
	nvic_clear_interrupt(EXTI9_5_IRQn);
	EXTI->PR |= (0x3E);

	debug_printk("exti9_5_handler\r\n");
}
void exti15_10_handler(void)
{
	nvic_clear_interrupt(EXTI15_10_IRQn);
	EXTI->PR |= (0xFC00);

	debug_printk("exti15_10_handler\r\n");
}
