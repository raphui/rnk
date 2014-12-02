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
#include <utils.h>

#define CLOCK_TIMEOUT	0xFFFFFFFF

void low_level_init( void )
{
	unsigned int timeout;

	writel( EEFC0_BASE + EFFC_FMR , E_FMR_FWS( 4 ) );
	writel( EEFC1_BASE + EFFC_FMR , E_FMR_FWS( 4 ) );

	if( !( readl( PMC_BASE + PMC_CKGR_MOR ) & P_CKGR_MOR_MOSCSEL ) )
	{
		writel( PMC_BASE + PMC_CKGR_MOR , P_CKGR_MOR_KEY( 0x37 )
										| P_CKGR_MOR_MOSCXTST( 0x8 )
										| P_CKGR_MOR_MOSCRCEN
										| P_CKGR_MOR_MOSCXTEN );
		timeout = 0;
		while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );
	}

	writel( PMC_BASE + PMC_CKGR_MOR , P_CKGR_MOR_KEY( 0x37 )
										| P_CKGR_MOR_MOSCXTST( 0x8 )
										| P_CKGR_MOR_MOSCRCEN
										| P_CKGR_MOR_MOSCXTEN 
										| P_CKGR_MOR_MOSCSEL );
	timeout = 0;
	while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );

	writel( PMC_BASE + PMC_MCKR , ( readl( PMC_BASE + PMC_MCKR ) & ~( 0x7UL << 0 ) ) 
									| P_MCKR_CSS( 1 ) );
	timeout = 0;
	while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );

	writel( PMC_BASE + PMC_CKGR_PLLAR , P_CKGR_PLLAR_ONE
										| P_CKGR_PLLAR_MULA( 13 )
										| P_CKGR_PLLAR_PLLACOUNT( 2 ) );
	timeout = 0;
	while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );

	writel( PMC_BASE + PMC_CKGR_UCKR , P_CKGR_UCKR_UPLLCOUNT( 3 )
										| P_CKGR_UCKR_UPLLEN );
	timeout = 0;
	while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );
	
	writel( PMC_BASE + PMC_MCKR , ( ( P_MCKR_PRES( 1 ) | P_MCKR_CSS( 2 ) ) & ~( 0x7UL << 0 ) )
									| P_MCKR_CSS( 1 ) );
	timeout = 0;
	while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );


	writel( PMC_BASE + PMC_MCKR , P_MCKR_PRES( 1 )
									| P_MCKR_CSS( 2 ) );
	timeout = 0;
	while( !( readl( PMC_BASE + PMC_SR ) & P_SR_MOSCXTS ) && ( timeout++ < CLOCK_TIMEOUT ) );
}
