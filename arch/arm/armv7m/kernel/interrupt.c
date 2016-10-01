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
#include <arch/system.h>
#include <arch/svc.h>
#include <time.h>
#include <pio.h>
#include <irq.h>
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

#ifdef CONFIG_UNWIND
	/* fp = 0, because we don't care about it */
	unwind_backtrace(0, (unsigned int)stack, lr, pc);
#endif /* CONFIG_UNWIND */

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
	system_tick++;

	arch_request_sched();
}

void pendsv_handler(void)
{
	__disable_it();

	schedule_thread(NULL);

	__enable_it();
}

void svc_handler(unsigned int call, void *arg)
{
	unsigned int svc_number;
	unsigned int *psp = (unsigned int *)call;

	__disable_it();

	svc_number = ((char *)psp[6])[-2];

	debug_printk("svc_handler: got call %d with arg (%x)\r\n", svc_number, arg);

	switch (svc_number) {
		schedule_thread((struct thread *)arg);
		break;
	case SVC_ACQUIRE_MUTEX:
		debug_printk("SVC call ask for acquiring mutex\r\n");
		svc_mutex_lock((struct mutex *)arg);
		break;
	case SVC_RELEASE_MUTEX:
		debug_printk("SVC call ask for releasing mutex\r\n");
		svc_mutex_unlock((struct mutex *)arg);
		break;
	case SVC_WAIT_SEM:
		debug_printk("SVC call ask for wait semaphore\r\n");
		svc_sem_wait((struct semaphore *)arg);
		break;
	case SVC_POST_SEM:
		debug_printk("SVC call ask for post semaphore\r\n");
		svc_sem_post((struct semaphore *)arg);
		break;
	case SVC_USLEEP:
		debug_printk("SVC call ask for usleep\r\n");
		svc_usleep((struct timer *)arg);
		break;
	case SVC_QUEUE_POST:
		debug_printk("SVC call ask for post in queue\r\n");
		svc_queue_post((struct queue *)psp[1], (void *)psp[2]);
		break;
	case SVC_QUEUE_RECEIVE:
		debug_printk("SVC call ask for receive from queue\r\n");
		svc_queue_receive((struct queue *)psp[1], (void *)psp[2]);
		break;
	case SVC_TIMER_ONESHOT:
		debug_printk("SVC call ask for oneshot timer\r\n");
		svc_timer_soft_oneshot((int)psp[1], (void (*)(void *))psp[2], (void *)psp[3]);
		break;
	default:
		debug_printk("Invalid svc call\r\n");
		break;
	}

	__enable_it();
}

#ifndef CONFIG_SW_ISR_TABLE
void timer2_handler(void)
{
	nvic_clear_interrupt(TIM2_IRQn);
	debug_printk("timer2 trig\r\n");
//	pio_toggle_value(GPIOE_BASE, 6);

#ifdef CONFIG_HR_TIMER
	decrease_thread_delay();
#endif /* CONFIG_HR_TIMER */
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

#if defined(CONFIG_STM32F429) || defined(CONFIG_STM32F746)
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
#endif /* CONFIG_STM32F429 */
#endif /* CONFIG_SW_ISR_TABLE */
