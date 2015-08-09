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

#include <stdarg.h>
#include <stdio.h>
#include <stddef.h>

#include <symbols.h>

static struct sym symbols[] = {
/* Generate symbols table is insert here */
	{0x080052d9, "add_task"},
	{0x08004929, "busfault_handler"},
	{0x080002ed, "create_context"},
	{0x08005709, "decrease_task_delay"},
	{0x08002d70, "Default_Handler"},
	{0x080001f4, "disable_psp"},
	{0x080049d5, "dma2_stream0_handler"},
	{0x08004a7d, "dma2_stream1_handler"},
	{0x08004b2d, "dma2_stream4_handler"},
	{0x08003065, "dma_disable"},
	{0x08003049, "dma_enable"},
	{0x08003005, "dma_init"},
	{0x08003025, "dma_transfer"},
	{0x0800480d, "dump_stack"},
	{0x08003add, "eighth_task"},
	{0x080001de, "enable_psp"},
	{0x08004511, "end_task"},
	{0x08006748, "_etext"},
	{0x08004d0d, "exti0_handler"},
	{0x08004e25, "exti15_10_handler"},
	{0x08004d49, "exti1_handler"},
	{0x08004d75, "exti2_handler"},
	{0x08004da1, "exti3_handler"},
	{0x08004dcd, "exti4_handler"},
	{0x08004df9, "exti9_5_handler"},
	{0x08003985, "fifth_task"},
	{0x080054cd, "find_next_task"},
	{0x08006748, "_fini"},
	{0x080053ed, "first_switch_task"},
	{0x08003755, "first_task"},
	{0x08003891, "fourth_task"},
	{0x080054b5, "get_current_task"},
	{0x08000371, "get_user_context"},
	{0x08000000, "g_pfnVectors"},
	{0x08004909, "hardfault_handler"},
	{0x080049c9, "i2c1_error_handler"},
	{0x080049bd, "i2c1_event_handler"},
	{0x080030b5, "ili9341_init"},
	{0x08003175, "ili9341_init_lcd"},
	{0x0800368d, "ili9341_send_command"},
	{0x080036f1, "ili9341_send_data"},
	{0x08005a5d, "init_heap"},
	{0x080041f1, "init_mutex"},
	{0x08004f61, "init_queue"},
	{0x080045c5, "init_semaphore"},
	{0x080005a5, "init_systick"},
	{0x080054f1, "insert_runnable_task"},
	{0x08005c15, "kfree"},
	{0x080059fd, "kmalloc"},
	{0x08003081, "lcd_init"},
	{0x080030a1, "lcd_init_gpio"},
	{0x08003be1, "lcd_rgb565_fill"},
	{0x08000809, "low_level_init"},
	{0x08003ee1, "main"},
	{0x080061dd, "memcpy"},
	{0x08004921, "memmanage_handler"},
	{0x080042d9, "mutex_lock"},
	{0x0800421d, "mutex_lock_isr"},
	{0x08004405, "mutex_unlock"},
	{0x08004301, "mutex_unlock_isr"},
	{0x08003d85, "ninth_task"},
	{0x08000501, "nvic_clear_interrupt"},
	{0x08000401, "nvic_disable_interrupt"},
	{0x08000381, "nvic_enable_interrupt"},
	{0x08000481, "nvic_set_interrupt"},
	{0x08000581, "nvic_set_priority_interrupt"},
	{0x08000621, "__pendsv"},
	{0x0800498d, "pendsv_handler"},
	{0x08002e05, "pio_clear_value"},
	{0x08002e65, "pio_disable_interrupt"},
	{0x08002e45, "pio_enable_interrupt"},
	{0x08002dc1, "pio_set_alternate"},
	{0x08002d99, "pio_set_input"},
	{0x08002d75, "pio_set_output"},
	{0x08002de5, "pio_set_value"},
	{0x08002e25, "pio_toggle_value"},
	{0x08005e51, "printk"},
	{0x0800505d, "queue_post"},
	{0x08005125, "queue_receive"},
	{0x08006239, "readl"},
	{0x08005511, "remove_runnable_task"},
	{0x080001ba, "restore_context"},
	{0x080001ac, "save_context"},
	{0x08000361, "save_user_context"},
	{0x08004489, "schedule"},
	{0x08004469, "schedule_init"},
	{0x08004501, "schedule_isr"},
	{0x080044b9, "schedule_task"},
	{0x080037a5, "second_task"},
	{0x080047e5, "sem_post"},
	{0x08004769, "sem_post_isr"},
	{0x080046c1, "sem_wait"},
	{0x08004659, "sem_wait_isr"},
	{0x08000631, "set_sys_clock"},
	{0x08003a4d, "seventh_task"},
	{0x08006748, "_sidata"},
	{0x080039d5, "sixth_task"},
	{0x08004c31, "spi5_handler"},
	{0x08002f9d, "spi_init"},
	{0x08002fe5, "spi_read"},
	{0x08002fbd, "spi_write"},
	{0x08004479, "start_schedule"},
	{0x08001e1d, "stm32_dma_disable"},
	{0x08001d71, "stm32_dma_enable"},
	{0x08001bfd, "stm32_dma_init"},
	{0x08001ce1, "stm32_dma_transfer"},
	{0x08002bb5, "stm32_exti_disable_falling"},
	{0x08002c59, "stm32_exti_disable_rising"},
	{0x08002a95, "stm32_exti_enable_falling"},
	{0x08002b25, "stm32_exti_enable_rising"},
	{0x080029a5, "stm32_exti_init"},
	{0x080026d5, "stm32_fmc_init"},
	{0x080014e1, "stm32_i2c_init"},
	{0x08002001, "stm32_ltdc_init"},
	{0x080021cd, "stm32_ltdc_init_gpio"},
	{0x08001141, "stm32_pio_clear_value"},
	{0x08001091, "stm32_pio_set_alternate"},
	{0x08001055, "stm32_pio_set_input"},
	{0x08000fd9, "stm32_pio_set_output"},
	{0x08001115, "stm32_pio_set_value"},
	{0x0800116d, "stm32_pio_toggle_value"},
	{0x08000f15, "stm32_rcc_enable_clk"},
	{0x080016e5, "stm32_spi_init"},
	{0x08001955, "stm32_spi_read"},
	{0x080018f1, "stm32_spi_write"},
	{0x08000609, "__svc"},
	{0x0800020d, "svc_handler"},
	{0x08004279, "svc_mutex_lock"},
	{0x08004381, "svc_mutex_unlock"},
	{0x08004fdd, "svc_queue_post"},
	{0x080050c5, "svc_queue_receive"},
	{0x080046e9, "svc_sem_post"},
	{0x080045f1, "svc_sem_wait"},
	{0x08005595, "svc_usleep"},
	{0x08005415, "switch_task"},
	{0x080001c8, "switch_to_privilege_mode"},
	{0x0800617d, "symbol_get_function"},
	{0x08004939, "systick_handler"},
	{0x080052b1, "task_init"},
	{0x08003eb9, "tenth_task"},
	{0x08000000, "_text"},
	{0x08003815, "third_task"},
	{0x0800557d, "time_init"},
	{0x080049a1, "timer2_handler"},
	{0x08002f7d, "timer_clear_it_flags"},
	{0x08002f61, "timer_disable"},
	{0x08002f45, "timer_enable"},
	{0x08002ee1, "timer_init"},
	{0x08002f21, "timer_set_counter"},
	{0x08002f01, "timer_set_rate"},
	{0x08004931, "usagefault_handler"},
	{0x08002e85, "usart_init"},
	{0x08002ea1, "usart_print"},
	{0x08002ec1, "usart_printl"},
	{0x080056a1, "usleep"},
	{0x08005e79, "vprintf"},
	{0x08006219, "writel"},

};

char *symbol_get_function(unsigned int addr)
{
	int size = sizeof(symbols) / sizeof(struct sym);
	int i = 0;
	char *ret = NULL;

	for (i = 0; i < size; i++) {
		if (symbols[i].addr == addr) {
			ret = symbols[i].function;
			break;
		}
	}

	return ret;
}
