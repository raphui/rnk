#include <board.h>
#include <utils.h>

#define CLOCK_TIMEOUT	0xFFFFFFFF

void low_level_init( void )
{
	unsigned int timeout;

	/*TODO: setup EFC */

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
										| P_CKGR_PLLAR_PLLACOUNT( 2 )
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
