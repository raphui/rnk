/*
 * Copyright (C) 2017  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef AIC_AT91_H
#define AIC_AT91_H

#define NR_AIC_IRQS			128

#define AT91_AIC_SSR			0x0
#define AT91_AIC_INTSEL_MSK		(0x7f << 0)

#define AT91_AIC_SMR			0x4

#define AT91_AIC_SVR			0x8
#define AT91_AIC_IVR			0x10
#define AT91_AIC_FVR			0x14
#define AT91_AIC_ISR			0x18

#define AT91_AIC_IPR0			0x20
#define AT91_AIC_IPR1			0x24
#define AT91_AIC_IPR2			0x28
#define AT91_AIC_IPR3			0x2c
#define AT91_AIC_IMR			0x30
#define AT91_AIC_CISR			0x34

#define AT91_AIC_IECR			0x40
#define AT91_AIC_IDCR			0x44
#define AT91_AIC_ICCR			0x48
#define AT91_AIC_ISCR			0x4c
#define AT91_AIC_EOICR			0x38
#define AT91_AIC_SPU			0x3c
#define AT91_AIC_DCR			0x6c

#define AT91_AIC_FFER			0x50
#define AT91_AIC_FFDR			0x54
#define AT91_AIC_FFSR			0x58


#endif /* AIC_AT91_H */
