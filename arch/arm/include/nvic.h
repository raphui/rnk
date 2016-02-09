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

#ifndef NVIC_H
#define NVIC_H

enum
{
	/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
	nonmaskableInt_irq        = -14,    /*!< 2 Non Maskable Interrupt                                          */
	memorymanagement_irq      = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
	busfault_irq              = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
	usagefault_irq		  = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
	svcall_irq	          = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
	debugmonitor_irq          = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
	pendsv_irq                = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
	systick_irq               = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
};

void nvic_enable_interrupt(unsigned int num);
void nvic_disable_interrupt(unsigned int num);
void nvic_set_interrupt(unsigned int num);
void nvic_clear_interrupt(unsigned int num);
void nvic_set_priority_interrupt(unsigned int num, unsigned char priority);

#endif /* NVIC_H */
