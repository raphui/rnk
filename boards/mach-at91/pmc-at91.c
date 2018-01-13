
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
#include <utils.h>

static inline void write_pmc(unsigned int offset, const unsigned int value)
{
	writel(offset + AT91C_BASE_PMC, value);
}

static inline unsigned int read_pmc(unsigned int offset)
{
	return readl(offset + AT91C_BASE_PMC);
}

int at91_pmc_enable_clk(unsigned int periph_id)
{
	int mask = 1 << (periph_id % 32);

	if (periph_id <= 31)
		write_pmc(PMC_PCER, mask);
	else
		write_pmc(PMC_PCER1, mask);
}

int at91_pmc_disable_clk(unsigned int periph_id)
{
	int mask = 1 << (periph_id % 32);

	if (periph_id <= 31)
		write_pmc(PMC_PCDR, mask);
	else
		write_pmc(PMC_PCDR1, mask);
}

int at91_pmc_clk_enabled(unsigned int periph_id)
{
	int ret = 0;

	if (periph_id <= 31) {
		ret = read__pmc(PMC_PCSR);
		ret = (ret >> periph_id) & 1;
	} else {
		ret = read_pmc(PMC_PCSR1);
		ret = (ret >> (periph_id - 32)) & 1;
	}

	return ret;
}

void at91_pmc_init_pll(unsigned int pmc_pllicpr)
{
	write_pmc(PMC_PLLICPR, pmc_pllicpr);
}

int at91_pmc_cfg_plla(unsigned int pmc_pllar)
{
	/*  Always disable PLL before configuring it */
	write_pmc((unsigned int)PMC_PLLAR, 0 | AT91C_CKGR_SRCA);
	write_pmc((unsigned int)PMC_PLLAR, pmc_pllar);

	while (!(read_pmc(PMC_SR) & AT91C_PMC_LOCKA))
		;

	return 0;
}

int at91_pmc_cfg_mck(unsigned int pmc_mckr)
{
	unsigned int tmp;

	/*
	 *	 * Program the PRES field in the PMC_MCKR register
	 *		 */
	tmp = read_pmc(PMC_MCKR);
	tmp &= (~(0x1 << 13));

	tmp &= (~AT91C_PMC_ALT_PRES);
	tmp |= (pmc_mckr & AT91C_PMC_ALT_PRES);

	write_pmc(PMC_MCKR, tmp);

	/*
	 ** Program the MDIV field in the PMC_MCKR register
	 **/
	tmp = read_pmc(PMC_MCKR);
	tmp &= (~AT91C_PMC_MDIV);
	tmp |= (pmc_mckr & AT91C_PMC_MDIV);
	write_pmc(PMC_MCKR, tmp);

	/*
	 ** Program the PLLADIV2 field in the PMC_MCKR register
	 **/
	tmp = read_pmc(PMC_MCKR);
	tmp &= (~AT91C_PMC_PLLADIV2);
	tmp |= (pmc_mckr & AT91C_PMC_PLLADIV2);
	write_pmc(PMC_MCKR, tmp);

	/*
	 ** Program the H32MXDIV field in the PMC_MCKR register
	 **/
	tmp = read_pmc(PMC_MCKR);
	tmp &= (~AT91C_PMC_H32MXDIV);
	tmp |= (pmc_mckr & AT91C_PMC_H32MXDIV);
	write_pmc(PMC_MCKR, tmp);

	/*
	 ** Program the CSS field in the PMC_MCKR register,
	 ** wait for MCKRDY bit to be set in the PMC_SR register
	 **/
	tmp = read_pmc(PMC_MCKR);
	tmp &= (~AT91C_PMC_CSS);
	tmp |= (pmc_mckr & AT91C_PMC_CSS);
	write_pmc(PMC_MCKR, tmp);

	while (!(read_pmc(PMC_SR) & AT91C_PMC_MCKRDY))
		;

	return 0;
}

int at91_pmc_enable_sys_clk(void)
{
	unsigned long tmp;

	/*
	 *	 * Enable the 12MHz oscillator
	 *	 * tST_max = 2ms
	 *	 * Startup Time: 32768 * 2ms / 8 = 8
	 *				 */
	tmp = read_pmc(PMC_MOR);
	tmp &= (~AT91C_CKGR_MOSCXTST);
	tmp &= (~AT91C_CKGR_KEY);
	tmp |= AT91C_CKGR_MOSCXTEN;
	tmp |= AT91_CKGR_MOSCXTST_SET(8);
	tmp |= AT91C_CKGR_PASSWD;
	write_pmc(PMC_MOR, tmp);

	while (!(read_pmc(PMC_SR) & AT91C_PMC_MOSCXTS))
		;

	/*  Switch from internal 12MHz RC to the 12MHz oscillator */
	tmp = read_pmc(PMC_MOR);
	tmp &= (~AT91C_CKGR_MOSCXTBY);
	tmp &= (~AT91C_CKGR_KEY);
	tmp |= AT91C_CKGR_PASSWD;
	write_pmc(PMC_MOR, tmp);

	tmp = read_pmc(PMC_MOR);
	tmp |= AT91C_CKGR_MOSCSEL;
	tmp &= (~AT91C_CKGR_KEY);
	tmp |= AT91C_CKGR_PASSWD;
	write_pmc(PMC_MOR, tmp);

	while (!(read_pmc(PMC_SR) & AT91C_PMC_MOSCSELS))
		;

	/*  After stablization, switch to Main Oscillator */
	if ((read_pmc(PMC_MCKR) & AT91C_PMC_CSS) == AT91C_PMC_CSS_SLOW_CLK) {
		tmp = read_pmc(PMC_MCKR);
		tmp &= (~(0x1 << 13));
		tmp &= ~AT91C_PMC_CSS;
		tmp |= AT91C_PMC_CSS_MAIN_CLK;
		write_pmc(PMC_MCKR, tmp);

		while (!(read_pmc(PMC_SR) & AT91C_PMC_MCKRDY))
			;

		tmp &= ~AT91C_PMC_PRES;
		tmp |= AT91C_PMC_PRES_CLK;
		write_pmc(PMC_MCKR, tmp);

		while (!(read_pmc(PMC_SR) & AT91C_PMC_MCKRDY))
			;
	}

}
arch_initcall(at91_pmc_enable_sys_clk);
