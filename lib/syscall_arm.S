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

#include <asm.h>

#define INSTR_CODE_SVC		(0xDF00)
#define INSTR_CODE_BX_LR	(0x4770)

.syntax unified
.thumb

.thumb_func
ENTRY(svc_noarg)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_arg1)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_arg2)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_arg3)
	push {lr}

	bl svc_call

	pop {pc}

.thumb_func
ENTRY(svc_call)
	push {lr}
	svc #0
	pop {pc}
