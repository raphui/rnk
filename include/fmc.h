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
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef FMC_H
#define FMC_H

struct fmc_sdram_timing {
	unsigned int load_to_active_delay;
	unsigned int exit_self_refresh_delay;
	unsigned int self_refresh_time;
	unsigned int row_cycle_delay;
	unsigned int write_recovery_time;
	unsigned int rp_delay;
	unsigned int rc_delay;
};

struct fmc_sdram {
	unsigned char num_bank;
	unsigned int column;
	unsigned int row;
	unsigned int data_width;
	unsigned int internal_bank;
	unsigned int cas;
	unsigned int write_protection;
	unsigned int clk_period;
	unsigned int read_burst;
	unsigned int read_pipe_delay;
	struct fmc_sdram_timing *fmc_sdtiming;
};


#endif /* FMC_H */
