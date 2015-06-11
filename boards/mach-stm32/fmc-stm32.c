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

static void stm32_fmc_sdram_timing_init(struct fmc_sdram *fmc_sdram)
{
	if (fmc_sdram->num_bank == 1) {
		FMC_Bank5_6->SDCR[fmc_sdram->num_bank - 1] &= ~(FMC_SDCR1_NC | FMC_SDCR1_NR | FMC_SDCR1_MWID
					| FMC_SDCR1_NB | FMC_SDCR1_CAS | FMC_SDCR1_WP
					| FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE);

		FMC_Bank5_6->SDCR[fmc_sdram->num_bank - 1] = fmc_sdram->column
					| fmc_sdram->row
					| fmc_sdram->data_width
					| fmc_sdram->internal_bank
					| fmc_sdram->cas
					| fmc_sdram->write_protection
					| fmc_sdram->clk_period
					| fmc_sdram->read_burst
					| fmc_sdram->read_pipe_delay;
	} else {
		FMC_Bank5_6->SDCR[0] &= ~(FMC_SDCR1_NC  | FMC_SDCR1_NR | FMC_SDCR1_MWID
					| FMC_SDCR1_NB  | FMC_SDCR1_CAS | FMC_SDCR1_WP
					| FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE);

		FMC_Bank5_6->SDCR[0] |= fmc_sdram->clk_period
					| fmc_sdram->read_burst
					| fmc_sdram->read_pipe_delay;

		FMC_Bank5_6->SDCR[fmc_sdram->num_bank - 1] &= ~(FMC_SDCR1_NC  | FMC_SDCR1_NR | FMC_SDCR1_MWID
					| FMC_SDCR1_NB  | FMC_SDCR1_CAS | FMC_SDCR1_WP
					| FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE);

		FMC_Bank5_6->SDCR[fmc_sdram->num_bank - 1] = fmc_sdram->column
					| fmc_sdram->row
					| fmc_sdram->data_width
					| fmc_sdram->internal_bank
					| fmc_sdram->cas
					| fmc_sdram->write_protection
					| fmc_sdram->clk_period
					| fmc_sdram->read_burst
					| fmc_sdram->read_pipe_delay;
	}

	if (fmc_sdram->num_bank == 1) {
		FMC_Bank5_6->SDTR[fmc_sdram->num_bank - 1] &= ~(FMC_SDTR1_TMRD  | FMC_SDTR1_TXSR | FMC_SDTR1_TRAS
					| FMC_SDTR1_TRC  | FMC_SDTR1_TWR | FMC_SDTR1_TRP
					| FMC_SDTR1_TRCD);

		FMC_Bank5_6->SDTR[fmc_sdram->num_bank - 1] = (fmc_sdram->fmc_sdram_timing->load_to_active_delay - 1)
					| ((fmc_sdram->fmc_sdram_timing->exit_self_refresh_delay - 1) << 4)
					| ((fmc_sdram->fmc_sdram_timing->self_refresh_time - 1) << 8)
					| ((fmc_sdram->fmc_sdram_timing->row_cycle_delay - 1) << 12)
					| ((fmc_sdram->fmc_sdram_timing->write_recovery_time - 1) << 16)
					| ((fmc_sdram->fmc_sdram_timing->rp_delay - 1) << 20)
					| ((fmc_sdram->fmc_sdram_timing->rc_delay - 1) << 24);
	} else {

		FMC_Bank5_6->SDTR[0] &= ~(FMC_SDTR1_TMRD  | FMC_SDTR1_TXSR | FMC_SDTR1_TRAS
					| FMC_SDTR1_TRC  | FMC_SDTR1_TWR | FMC_SDTR1_TRP
					| FMC_SDTR1_TRCD);

		FMC_Bank5_6->SDTR[0] = ((fmc_sdram->fmc_sdram_timing->row_cycle_delay - 1) << 12)
					| ((fmc_sdram->fmc_sdram_timing->rp_delay - 1)<< 20);

		FMC_Bank5_6->SDTR[fmc_sdram->num_bank - 1] &= ~(FMC_SDTR1_TMRD  | FMC_SDTR1_TXSR | FMC_SDTR1_TRAS
					| FMC_SDTR1_TRC  | FMC_SDTR1_TWR | FMC_SDTR1_TRP
					| FMC_SDTR1_TRCD);

		FMC_Bank5_6->SDTR[fmc_sdram->num_bank - 1] = (fmc_sdram->fmc_sdram_timing->load_to_active_delay - 1)
					| ((fmc_sdram->fmc_sdram_timing->exit_self_refresh_delay - 1)<< 4)
					| ((fmc_sdram->fmc_sdram_timing->self_refresh_time - 1) << 8)
					| ((fmc_sdram->fmc_sdram_timing->write_recovery_time - 1) << 16);

	}
}

static void stm32_fmc_sdram_cmd_conf_init(struct fmc_sdram *fmc_sdram)
{
	unsigned int val = fmc_sdram->fmc_sdram_cmd_config->mode;

	val |= fmc_sdram->fmc_sdram_cmd_config->cmd_target;
	val |= ((fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num - 1 )<< 5);
	val |= (fmc_sdram->fmc_sdram_cmd_config->mode << 9);

	FMC_Bank5_6->SDCMR = val;
}

void stm32_fmc_init(struct fmc_sdram *fmc_sdram)
{
	unsigned int timeout = 0xFF;

	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	stm32_fmc_sdram_timing_init(fmc_sdram);
	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	timeout = 100000;
	while (timeout--)
		;

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x2;	/* PALL */

	timeout = 0xFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x3;	/* Auto refresh */
	fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num = 0x8;

	timeout = 0xFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x3;	/* Load */
	fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num = 0x1;
	fmc_sdram->fmc_sdram_cmd_config->mode = 0x0231;

	timeout = 0xFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	FMC_Bank5_6->SDRTR |= (680 < 1);

	timeout = 0xFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;
}
