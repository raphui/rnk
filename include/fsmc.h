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

#ifndef FSMC_H
#define FSMC_H

struct fsmc_nor_write_timing {
	unsigned char acc_mode:2;
	unsigned char bus_turn:4;
	unsigned char data_phase_duration;
	unsigned char adress_hold_duration:4;
	unsigned char adress_setup_duration:4;
};

struct fsmc_nor_cs_timing {
	unsigned char acc_mode:2;
	unsigned char data_latency:4;
	unsigned char clk_div:4;
	unsigned char bus_turn:4;
	unsigned char data_phase_duration;
	unsigned char adress_hold_duration:4;
	unsigned char adress_setup_duration:4;
};

struct fsmc_nor {
	unsigned char bank:4;
	unsigned char bus_width:2;;
	unsigned char mem_type:2;
	struct fsmc_nor_write_timing fsmc_nor_w;
	struct fsmc_nor_cs_timing fsmc_nor_cs;
};

#endif /* FSMC_H */
