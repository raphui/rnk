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

#include <arch/nvic.h>
#include <armv7m/system.h>
#include <printk.h>
#include <utils.h>

void nvic_enable_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ISER0;
			break;
		case 1:
			nvic_reg = NVIC_ISER1;
			break;
		case 2:
			nvic_reg = NVIC_ISER2;
			break;
		case 3:
			nvic_reg = NVIC_ISER3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_disable_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ICER0;
			break;
		case 1:
			nvic_reg = NVIC_ICER1;
			break;
		case 2:
			nvic_reg = NVIC_ICER2;
			break;
		case 3:
			nvic_reg = NVIC_ICER3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_set_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ISPR0;
			break;
		case 1:
			nvic_reg = NVIC_ISPR1;
			break;
		case 2:
			nvic_reg = NVIC_ISPR2;
			break;
		case 3:
			nvic_reg = NVIC_ISPR3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_clear_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ICPR0;
			break;
		case 1:
			nvic_reg = NVIC_ICPR1;
			break;
		case 2:
			nvic_reg = NVIC_ICPR2;
			break;
		case 3:
			nvic_reg = NVIC_ICPR3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_set_priority_interrupt(unsigned int num, unsigned char priority)
{
	if (num < 0)
		writel(SCB_SHPR(num), priority);
	else
		writel(NVIC_IPR(num), priority);
}
