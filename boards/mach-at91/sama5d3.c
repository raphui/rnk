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

#include <board.h>
#include <init.h>

void low_level_init(void)
{

	/*  Configure PLLA = MOSC * (PLL_MULA + 1) / PLL_DIVA */
	at91_pmc_cfg_plla(PLLA_SETTINGS);

	/*  Initialize PLLA charge pump */
	at91_pmc_init_pll(AT91C_PMC_IPLLA_3);

	/*  Switch PCK/MCK on Main clock output */
	at91_pmc_cfg_mck(BOARD_PRESCALER_MAIN_CLOCK);

	/*  Switch PCK/MCK on PLLA output */
	at91_pmc_cfg_mck(BOARD_PRESCALER_PLLA);
}
