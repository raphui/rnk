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
#include <string.h>
#include <errno.h>

#include <symbols.h>

static struct sym symbols[] = {
/* Generate symbols table is insert here */
{0x080051d9, 0x00000112, "add_task"},
{0x08005b41, 0x0000000c, "__aeabi_unwind_cpp_pr0"},
{0x08005b4d, 0x0000000c, "__aeabi_unwind_cpp_pr1"},
{0x08005b59, 0x0000000c, "__aeabi_unwind_cpp_pr2"},
{0x08004829, 0x00000006, "busfault_handler"},
{0x080002ed, 0x00000072, "create_context"},
{0x08005609, 0x000000a0, "decrease_task_delay"},
{0x08002d4c, 0x00000002, "Default_Handler"},
{0x080001f4, 0x00000000, "disable_psp"},
{0x080048d5, 0x000000a8, "dma2_stream0_handler"},
{0x0800497d, 0x000000b0, "dma2_stream1_handler"},
{0x08004a2d, 0x00000102, "dma2_stream4_handler"},
{0x08003041, 0x0000001c, "dma_disable"},
{0x08003025, 0x0000001c, "dma_enable"},
{0x08002fe1, 0x00000020, "dma_init"},
{0x08003001, 0x00000024, "dma_transfer"},
{0x08005b9d, 0x00000028, "dump_backtrace_entry"},
{0x08003ab9, 0x00000064, "eighth_task"},
{0x080001de, 0x00000000, "enable_psp"},
{0x080044e5, 0x0000002a, "end_task"},
{0x08004c0d, 0x0000003c, "exti0_handler"},
{0x08004d25, 0x0000002a, "exti15_10_handler"},
{0x08004c49, 0x0000002a, "exti1_handler"},
{0x08004c75, 0x0000002a, "exti2_handler"},
{0x08004ca1, 0x0000002a, "exti3_handler"},
{0x08004ccd, 0x0000002a, "exti4_handler"},
{0x08004cf9, 0x0000002a, "exti9_5_handler"},
{0x08003961, 0x0000004e, "fifth_task"},
{0x080053cd, 0x00000024, "find_next_task"},
{0x080052ed, 0x00000026, "first_switch_task"},
{0x08003731, 0x0000004e, "first_task"},
{0x0800386d, 0x000000f4, "fourth_task"},
{0x080053b5, 0x00000018, "get_current_task"},
{0x08000371, 0x00000010, "get_user_context"},
{0x08000000, 0x00000000, "g_pfnVectors"},
{0x08004809, 0x00000018, "hardfault_handler"},
{0x080048c9, 0x0000000c, "i2c1_error_handler"},
{0x080048bd, 0x0000000c, "i2c1_event_handler"},
{0x08003091, 0x000000be, "ili9341_init"},
{0x08003151, 0x00000516, "ili9341_init_lcd"},
{0x08003669, 0x00000064, "ili9341_send_command"},
{0x080036cd, 0x00000064, "ili9341_send_data"},
{0x0800595d, 0x00000044, "init_heap"},
{0x080041c5, 0x0000002a, "init_mutex"},
{0x08004e61, 0x0000007a, "init_queue"},
{0x08004599, 0x0000002c, "init_semaphore"},
{0x080005a5, 0x00000064, "init_systick"},
{0x080053f1, 0x00000020, "insert_runnable_task"},
{0x08005b15, 0x0000002a, "kfree"},
{0x080058fd, 0x00000060, "kmalloc"},
{0x0800305d, 0x00000020, "lcd_init"},
{0x0800307d, 0x00000012, "lcd_init_gpio"},
{0x08003bbd, 0x000001a4, "lcd_rgb565_fill"},
{0x08000809, 0x0000055c, "low_level_init"},
{0x08003ebd, 0x00000206, "main"},
{0x080068dd, 0x0000003a, "memcpy"},
{0x08004821, 0x00000006, "memmanage_handler"},
{0x080042ad, 0x00000028, "mutex_lock"},
{0x080041f1, 0x0000005c, "mutex_lock_isr"},
{0x080043d9, 0x00000028, "mutex_unlock"},
{0x080042d5, 0x00000080, "mutex_unlock_isr"},
{0x08003d61, 0x00000132, "ninth_task"},
{0x08000501, 0x00000080, "nvic_clear_interrupt"},
{0x08000401, 0x00000080, "nvic_disable_interrupt"},
{0x08000381, 0x00000080, "nvic_enable_interrupt"},
{0x08000481, 0x00000080, "nvic_set_interrupt"},
{0x08000581, 0x00000022, "nvic_set_priority_interrupt"},
{0x08000621, 0x00000000, "__pendsv"},
{0x0800488d, 0x00000012, "pendsv_handler"},
{0x08002de1, 0x00000020, "pio_clear_value"},
{0x08002e41, 0x00000020, "pio_disable_interrupt"},
{0x08002e21, 0x00000020, "pio_enable_interrupt"},
{0x08002d9d, 0x00000024, "pio_set_alternate"},
{0x08002d75, 0x00000028, "pio_set_input"},
{0x08002d51, 0x00000024, "pio_set_output"},
{0x08002dc1, 0x00000020, "pio_set_value"},
{0x08002e01, 0x00000020, "pio_toggle_value"},
{0x080064d5, 0x00000028, "printk"},
{0x08004f5d, 0x00000068, "queue_post"},
{0x08005025, 0x00000060, "queue_receive"},
{0x08006939, 0x00000020, "readl"},
{0x08005411, 0x0000003c, "remove_runnable_task"},
{0x080001ba, 0x00000000, "restore_context"},
{0x080001ac, 0x00000000, "save_context"},
{0x08000361, 0x00000010, "save_user_context"},
{0x0800445d, 0x00000030, "schedule"},
{0x0800443d, 0x00000010, "schedule_init"},
{0x080044d5, 0x00000010, "schedule_isr"},
{0x0800448d, 0x00000046, "schedule_task"},
{0x08003781, 0x0000006e, "second_task"},
{0x080047b9, 0x00000028, "sem_post"},
{0x0800473d, 0x0000007c, "sem_post_isr"},
{0x08004695, 0x00000028, "sem_wait"},
{0x0800462d, 0x00000066, "sem_wait_isr"},
{0x08000631, 0x000001d8, "set_sys_clock"},
{0x08003a29, 0x0000008e, "seventh_task"},
{0x080039b1, 0x00000076, "sixth_task"},
{0x08004b31, 0x000000dc, "spi5_handler"},
{0x08002f79, 0x00000020, "spi_init"},
{0x08002fc1, 0x00000020, "spi_read"},
{0x08002f99, 0x00000028, "spi_write"},
{0x0800444d, 0x00000010, "start_schedule"},
{0x08001e01, 0x0000006a, "stm32_dma_disable"},
{0x08001d55, 0x000000ac, "stm32_dma_enable"},
{0x08001bfd, 0x000000e4, "stm32_dma_init"},
{0x08001ce1, 0x00000074, "stm32_dma_transfer"},
{0x08002b99, 0x000000a2, "stm32_exti_disable_falling"},
{0x08002c3d, 0x000000a2, "stm32_exti_disable_rising"},
{0x08002a79, 0x00000090, "stm32_exti_enable_falling"},
{0x08002b09, 0x00000090, "stm32_exti_enable_rising"},
{0x08002989, 0x000000f0, "stm32_exti_init"},
{0x080026b9, 0x00000168, "stm32_fmc_init"},
{0x080014e1, 0x00000068, "stm32_i2c_init"},
{0x08001fe5, 0x000001cc, "stm32_ltdc_init"},
{0x080021b1, 0x00000236, "stm32_ltdc_init_gpio"},
{0x08001141, 0x0000002c, "stm32_pio_clear_value"},
{0x08001091, 0x00000082, "stm32_pio_set_alternate"},
{0x08001055, 0x0000003a, "stm32_pio_set_input"},
{0x08000fd9, 0x0000007c, "stm32_pio_set_output"},
{0x08001115, 0x0000002a, "stm32_pio_set_value"},
{0x0800116d, 0x0000002a, "stm32_pio_toggle_value"},
{0x08000f15, 0x00000086, "stm32_rcc_enable_clk"},
{0x080016e5, 0x00000108, "stm32_spi_init"},
{0x08001955, 0x00000048, "stm32_spi_read"},
{0x080018f1, 0x00000062, "stm32_spi_write"},
{0x08000609, 0x00000000, "__svc"},
{0x0800020d, 0x000000e0, "svc_handler"},
{0x0800424d, 0x00000060, "svc_mutex_lock"},
{0x08004355, 0x00000082, "svc_mutex_unlock"},
{0x08004edd, 0x00000080, "svc_queue_post"},
{0x08004fc5, 0x0000005e, "svc_queue_receive"},
{0x080046bd, 0x0000007e, "svc_sem_post"},
{0x080045c5, 0x00000068, "svc_sem_wait"},
{0x08005495, 0x0000010c, "svc_usleep"},
{0x08005315, 0x0000009e, "switch_task"},
{0x080001c8, 0x00000000, "switch_to_privilege_mode"},
{0x08006801, 0x000000dc, "symbol_get_function"},
{0x08004839, 0x00000052, "systick_handler"},
{0x080051b1, 0x00000026, "task_init"},
{0x08003e95, 0x00000028, "tenth_task"},
{0x08000000, 0x00000000, "_text"},
{0x080037f1, 0x0000007a, "third_task"},
{0x0800547d, 0x00000018, "time_init"},
{0x080048a1, 0x0000001c, "timer2_handler"},
{0x08002f59, 0x00000020, "timer_clear_it_flags"},
{0x08002f3d, 0x0000001c, "timer_disable"},
{0x08002f21, 0x0000001c, "timer_enable"},
{0x08002ebd, 0x00000020, "timer_init"},
{0x08002efd, 0x00000024, "timer_set_counter"},
{0x08002edd, 0x00000020, "timer_set_rate"},
{0x080061e1, 0x00000096, "unwind_backtrace"},
{0x08005ff9, 0x000001e6, "unwind_frame"},
{0x08006279, 0x0000004a, "unwind_init"},
{0x08004831, 0x00000006, "usagefault_handler"},
{0x08002e61, 0x0000001c, "usart_init"},
{0x08002e7d, 0x00000020, "usart_print"},
{0x08002e9d, 0x00000020, "usart_printl"},
{0x080055a1, 0x00000068, "usleep"},
{0x080064fd, 0x00000302, "vprintf"},
{0x08006919, 0x0000001e, "writel"},

};

char *symbol_get_name(unsigned int addr)
{
	int size = sizeof(symbols) / sizeof(struct sym);
	int i = 0;
	char *ret = NULL;

	for (i = 0; i < size; i++) {
		if (symbols[i].addr == addr) {
			ret = symbols[i].name;
			break;
		} else if ((addr > symbols[i].addr) && (addr < (symbols[i].addr + symbols[i].size))) {
			ret = symbols[i].name;
			break;	
		}
	}

	return ret;
}

int symbol_get_addr(char *name)
{
	int size = sizeof(symbols) / sizeof(struct sym);
	int i = 0;
	int ret = -ENXIO;

	for (i = 0; i < size; i++) {
		if (!strcmp(name, symbols[i].name)) {
			ret = symbols[i].addr;
			break;
		}
	}

	return ret;
}
