/*
 * Copyright (C) 2015 Raphaël Poggi <poggi.raph@gmail.com>
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

#include <armv7m/system.h>

#define SYS_CLOCK	168000000
#define SYSTICK_FREQ	100//4000

void init_systick(void) {
    *SYSTICK_RELOAD = SYS_CLOCK / SYSTICK_FREQ;
    *SYSTICK_VAL = 0;
    *SYSTICK_CTL = 0x00000007;

    /* Set PendSV and SVC to lowest priority.
     * This means that both will be deferred
     * until all other exceptions have executed.
     * Additionally, PendSV will not interrupt
     * an SVC. */
    *NVIC_IPR(11) = 0xFF;
    *NVIC_IPR(14) = 0xFF;
}

