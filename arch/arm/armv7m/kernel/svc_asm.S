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

.syntax unified
.thumb

.cpu cortex-m4
.arch armv7e-m

.global switch_to_privilege_mode
switch_to_privilege_mode:
	mrs r0, control
	bic r0, r0, #1
	msr control, r0
	dsb
	isb
	bx lr


.global enable_psp
enable_psp:
	mrs r0, control
	orr r0, r0, #2
	msr control, r0
	dsb
	isb
	bx lr

.global disable_psp
disable_psp:
	mrs r0, control
	bic r0, r0, #2
	msr control, r0
	dsb
	isb
	bx lr

.global switch_unprivileged
switch_unprivileged:
	mrs r0, control
	orr r0, r0, #1
	msr control, r0
	dsb
	isb
	bx lr
