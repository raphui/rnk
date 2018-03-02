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
#include <init.h>
#include <mm.h>
#include <errno.h>
#include <fdtparse.h>
#include <stdio.h>
#include <string.h>
#include <mach/pio-stm32.h>

#define FMC_MEM_TYPE_SDRAM	0x1

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
	unsigned int val = fmc_sdram->fmc_sdram_cmd_config->cmd_mode;

	val |= fmc_sdram->fmc_sdram_cmd_config->cmd_target;
	val |= ((fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num - 1) << 5);
	val |= (fmc_sdram->fmc_sdram_cmd_config->mode << 9);

	FMC_Bank5_6->SDCMR = val;
}

static int stm32_fmc_sdram_configure(struct fmc_sdram *fmc_sdram)
{
	unsigned int timeout = 0xFFFF;

	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	timeout = 0xFFFF;
	while (timeout--)
		;

	stm32_fmc_sdram_timing_init(fmc_sdram);

	timeout = 0xFFFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x1;	/* Clock enable */
	fmc_sdram->fmc_sdram_cmd_config->cmd_target = 0x10;	/* Bank 1 */
	fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num = 0x1;
	fmc_sdram->fmc_sdram_cmd_config->mode = 0x0;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	timeout = 0xFFFF;
	while (timeout--)
		;

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x2;	/* PALL */

	timeout = 0xFFFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x3;	/* Auto refresh */
	fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num = 0x7;

	timeout = 0xFFFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	fmc_sdram->fmc_sdram_cmd_config->cmd_mode = 0x4;	/* Load */
	fmc_sdram->fmc_sdram_cmd_config->auto_refresh_num = 0x1;
	fmc_sdram->fmc_sdram_cmd_config->mode = fmc_sdram->conf_to_load;

	timeout = 0xFFFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	stm32_fmc_sdram_cmd_conf_init(fmc_sdram);

	FMC_Bank5_6->SDRTR |= (fmc_sdram->refresh_count < 1);

	timeout = 0xFFFF;
	while ((FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) && timeout--)
		;

	return 0;
}

static int stm32_fmc_of_init(struct fmc *fmcdev)
{
	int offset;
	int ret = 0;
	struct fmc_sdram_cmd_config sdram_cmd_conf;
	struct fmc_sdram_timing sdram_timing;
	struct fmc_sdram sdram;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, fmcdev->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, fmcdev->dev.of_compat);
	if (ret < 0)
		goto out;


	ret = fdtparse_get_int(offset, "mem-type", (int *)&fmcdev->mem_type);
	if (ret < 0)
		goto out;

	if (fmcdev->mem_type != FMC_MEM_TYPE_SDRAM) {
		ret = -ENOTSUP;
		goto out;
	}

	ret = fdtparse_get_int(offset, "load_to_active_delay", (int *)&sdram_timing.load_to_active_delay);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "exit_self_refresh_delay", (int *)&sdram_timing.exit_self_refresh_delay);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "self_refresh_time", (int *)&sdram_timing.self_refresh_time);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "row_cycle_delay", (int *)&sdram_timing.row_cycle_delay);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "write_recovery_time", (int *)&sdram_timing.write_recovery_time);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "rp_delay", (int *)&sdram_timing.rp_delay);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "rc_delay", (int *)&sdram_timing.rc_delay);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "num_bank", (int *)&sdram.num_bank);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "column", (int *)&sdram.column);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "row", (int *)&sdram.row);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "data_width", (int *)&sdram.data_width);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "internal_bank", (int *)&sdram.internal_bank);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "cas", (int *)&sdram.cas);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "write_protection", (int *)&sdram.write_protection);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "clk_period", (int *)&sdram.clk_period);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "read_burst", (int *)&sdram.read_burst);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "read_pipe_delay", (int *)&sdram.read_pipe_delay);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "conf_to_load", (int *)&sdram.conf_to_load);
	if (ret < 0)
		goto out;

	ret = fdtparse_get_int(offset, "refresh_count", (int *)&sdram.refresh_count);
	if (ret < 0)
		goto out;

	sdram.fmc_sdram_timing = &sdram_timing;
	sdram.fmc_sdram_cmd_config = &sdram_cmd_conf;

	ret = stm32_pio_of_configure(offset);
	if (ret < 0) {
		error_printk("failed to configure fmc gpio\n");
		goto out;
	}

	ret = stm32_fmc_sdram_configure(&sdram);
	if (ret < 0)
		error_printk("failed to configure fmc sdram conf\n");
out:
	return ret;
}

static int stm32_fmc_init(struct device *dev)
{
	int ret = 0;
	struct fmc *fmcdev = NULL;

	fmcdev = (struct fmc *)kmalloc(sizeof(struct fmc));
	if (!fmcdev) {
		error_printk("cannot allocate stm32 fmc device\n");
		return -ENOMEM;
	}

	memcpy(&fmcdev->dev, dev, sizeof(struct device));

	ret = stm32_fmc_of_init(fmcdev);
	if (ret < 0) {
		error_printk("failed to init stm32 fmc with fdt data\n");
		goto free_fmc;
	}

	return ret;

free_fmc:
	kfree(fmcdev);
	return ret;
}

struct device stm32_fmc_driver = {
	.of_compat = "st,stm32f4xx-fmc",
	.probe = stm32_fmc_init,
};

static int stm32_fmc_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_fmc_driver);
	if (ret < 0)
		error_printk("failed to register stm32_fmc device\n");
	return ret;
}
postarch_initcall(stm32_fmc_register);
