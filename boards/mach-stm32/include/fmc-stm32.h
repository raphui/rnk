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

#ifndef FMC_STM32_H
#define FMC_STM32_H

#include <fmc.h>

struct fmc_sdram_cmd_config sdram_cmd_conf;
struct fmc_sdram_timing sdram_timing;
struct fmc_sdram sdram;

extern void stm32_fmc_init(struct fmc_sdram *fmc_sdram);

#endif /* FMC_STM32_H */
