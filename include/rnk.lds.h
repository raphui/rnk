/*
 * Copyright (C) 2016  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef RNK_LDS_H
#define RNK_LDS_H

#define INITCALLS			\
	KEEP(*(.initcall.0))			\
	KEEP(*(.initcall.1))			\
	KEEP(*(.initcall.2))			\
	KEEP(*(.initcall.3))			\
	KEEP(*(.initcall.4))			\
	KEEP(*(.initcall.5))			\
	KEEP(*(.initcall.6))			\
	KEEP(*(.initcall.7))			\
	KEEP(*(.initcall.8))			\
	KEEP(*(.initcall.9))			\
	KEEP(*(.initcall.10))			\
	KEEP(*(.initcall.11))			\
	KEEP(*(.initcall.12))			\
	KEEP(*(.initcall.13))			\
	KEEP(*(.initcall.14))

#define EXITCALLS			\
	KEEP(*(.exitcall.0))			\
	KEEP(*(.exitcall.1))			\
	KEEP(*(.exitcall.2))			\
	KEEP(*(.exitcall.3))			\
	KEEP(*(.exitcall.4))			\
	KEEP(*(.exitcall.5))			\
	KEEP(*(.exitcall.6))

#define KSYM				\
	KEEP(*(.ksym))

#endif /* RNK_LDS_H */
