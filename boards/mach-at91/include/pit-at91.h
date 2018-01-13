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

#ifndef PIT_AT91_H
#define PIT_AT91_H

/* *** Register offset in AT91S_PITC structure ***/
#define PIT_MR		0x00	/*  Period Interval Mode Register */
#define PIT_SR		0x04	/*  Period Interval Status Register */
#define PIT_PIVR	0x08	/*  Period Interval Value Register */
#define PIT_PIIR	0x0c	/*  Period Interval Image Register */

/* -------- PITC_PIMR : (PITC Offset: 0x0) Periodic Interval Mode Register -------*/
#define AT91C_PIT_PIV		(0xFFFFF << 0)	/*  Periodic Interval Value */
#define AT91C_PIT_PITEN	(0x01 << 24)	/*  Periodic Interval Timer Enabled */
#define AT91C_PIT_PITIEN	(0x01 << 25)	/*  Periodic Interval Timer Interrupt Enable */

/* -------- PITC_PISR : (PITC Offset: 0x4) Periodic Interval Status Register --------*/
#define AT91C_PIT_PITS		(0x01 << 0)	/*  (PITC) Periodic Interval Timer Status */

/* -------- PITC_PIVR : (PITC Offset: 0x8) Periodic Interval Value Register --------*/
/* -------- PITC_PIIR : (PITC Offset: 0xc) Periodic Interval Image Register --------*/
#define AT91C_PIT_CPIV		(0xFFFFF << 0)	/*  (PITC) Current Periodic Interval Value */
#define AT91C_PIT_PICNT	(0xFFF << 20)	/*  (PITC) Periodic Interval Counter */

#endif /* PIT_AT91_H */
