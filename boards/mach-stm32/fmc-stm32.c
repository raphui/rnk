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

#include <board.h>
#include <fmc.h>
#include <utils.h>
#include <stdio.h>

void stm32_fmc_sdram_timing_init(struct fmc_sdram *fmc_sdram)
{
	if (fmc_sdram->num_bank == 1) {
		FMC_Bank5_6->SDCR[fmc_sdram->num_bank - 1] = (fmc_sdram->column_byte)
							| (fmc_sdram->row_byte << 2)
							| (fmc_sdram->data_width << 4)
							| (fmc_sdram->internal_bank_num << 6)
							| (fmc_sdram->cas_latency << 7)
							| (fmc_sdram->write_protection << 9)
							| (fmc_sdram->clk_period << 10)
							| (fmc_sdram->read_burst << 12)
							| (fmc_sdram->read_pipe_delay << 13);
	} else {

	}
}

void stm32_fmc_init(struct fmc_sdram *fmc)
{
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	
}
