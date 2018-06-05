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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef BOARD_STM32F4XX_H
#define BOARD_STM32F4XX_H

#ifdef CONFIG_STM32F401
#include <stm32f401.h>
#elif defined(CONFIG_STM32F407)
#include <stm32f407.h>
#elif defined(CONFIG_STM32F429)
#include <stm32f429.h>
#endif

#include <pio.h>

/*#define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET 0x00

#ifdef CONFIG_USB_STACK
#define __IO volatile
#define USE_USB_OTG_FS
#endif /* CONFIG_USB_STACK */
 
struct pio_operations pio_ops;

#endif /* BOARD_STM32F4XX_H */
