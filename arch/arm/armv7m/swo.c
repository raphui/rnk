
#include <armv7m/swo.h>
#include <console.h>

#define ITM_STIM_U32 (*(volatile unsigned int *)0xE0000000)    // Stimulus Port Register word acces
#define ITM_STIM_U8  (*(volatile char *)0xE0000000)    // Stimulus Port Register byte acces
#define ITM_ENA      (*(volatile unsigned int *)0xE0000E00)    // Trace Enable Ports Register
#define ITM_TCR      (*(volatile unsigned int *)0xE0000E80)    // Trace control register

typedef struct
{
	unsigned int DBG_DHCSR;                   /*!< Offset: 0x000 (R/W)  Debug Halting Control and Status Register    */
	unsigned int DBG_DCRSR;                   /*!< Offset: 0x004 ( /W)  Debug Core Register Selector Register        */
	unsigned int DBG_DCRDR;                   /*!< Offset: 0x008 (R/W)  Debug Core Register Data Register            */
	unsigned int DBG_DEMCR;                   /*!< Offset: 0x00C (R/W)  Debug Exception and Monitor Control Register */
} DBG_Type;

/* Debug Halting Control and Status Register */
#define DBG_DHCSR_DBGKEY         (0xFFFF << 16)
#define DBG_DHCSR_S_RESET_ST     (1 << 25)
#define DBG_DHCSR_S_RETIRE_ST    (1 << 24)
#define DBG_DHCSR_S_LOCKUP       (1 << 19)
#define DBG_DHCSR_S_SLEEP        (1 << 18)
#define DBG_DHCSR_S_HALT         (1 << 17)
#define DBG_DHCSR_S_REGRDY       (1 << 16)
#define DBG_DHCSR_C_SNAPSTALL    (1 << 5)
#define DBG_DHCSR_C_MASKINTS     (1 << 3)
#define DBG_DHCSR_C_STEP         (1 << 2)
#define DBG_DHCSR_C_HALT         (1 << 1)
#define DBG_DHCSR_C_DEBUGEN      (1 << 0)

/* Debug Core Register Selector Register */
#define DBG_DCRSR_REGWnR         (1 << 16)
#define DBG_DCRSR_REGSEL         (0x1F << 0)

/* Debug Exception and Monitor Control Register */
#define DBG_DEMCR_TRCENA         (1 << 24)
#define DBG_DEMCR_MON_REQ        (1 << 19)
#define DBG_DEMCR_MON_STEP       (1 << 18)
#define DBG_DEMCR_MON_PEND       (1 << 17)
#define DBG_DEMCR_MON_EN         (1 << 16)
#define DBG_DEMCR_VC_HARDERR     (1 << 10)
#define DBG_DEMCR_VC_INTERR      (1 << 9)
#define DBG_DEMCR_VC_BUSERR      (1 << 8)

#define DBG_DEMCR_VC_STATERR     (1 << 7)
#define DBG_DEMCR_VC_CHKERR      (1 << 6)
#define DBG_DEMCR_VC_NOCPERR     (1 << 5)
#define DBG_DEMCR_VC_MMERR       (1 << 4)
#define DBG_DEMCR_VC_CORERESET   (1 << 0)

#define DBG_BASE      (0xE000EDF0UL)
#define DBG           ((DBG_Type *)	DBG_BASE)

void swo_init(unsigned int masterclock)
{
	unsigned int swo_speed = 2000000;
	unsigned int swo_prescaler = (masterclock / swo_speed) - 1;
	DBG->DBG_DEMCR |= DBG_DEMCR_TRCENA;

	DBG->DBG_DHCSR = 0x00000027; 

	// "Selected PIN Protocol Register": Select which protocol to use 
	// for trace output (2: SWO)
	*((volatile unsigned *) 0xE00400F0) = 0x00000002; 

	//Set TPIU -> Async Clock Prescaler Register [bits 0-12]
	// "Async Clock Prescaler Register". Scale baud rate of asynchronous output
	*((volatile unsigned *) 0xE0040010) = swo_prescaler; 

	//Lock Access Register
	// ITM Lock Access Register, C5ACCE55 enables more write access 
	// to Control Register 0xE00 :: 0xFFC
	*((volatile unsigned *) 0xE0000FB0) = 0xC5ACCE55; 

	*((volatile unsigned *) 0xE0000E80) = 0x0001000D; // ITM Trace Cont Register
	*((volatile unsigned *) 0xE0000E40) = 0x0000000F; // ITM Trace Priv Register
	*((volatile unsigned *) 0xE0000E00) = 0x00000001; // ITM Trace Enab Register 
	// Enabled tracing on stimulus ports. One bit per stimulus port.

	*((volatile unsigned *) 0xE0001000) = 0x400003FE; // DWT_CTRL

	//And this is really tricky! No document for this register at ARM site.
	// Formatter and Flush Control Register
	*((volatile unsigned *) 0xE0040304) = 0x00000100; 
}


static int swo_print(struct device *dev, unsigned char *buff, unsigned int len)
{
	int i;

	if ((ITM_TCR & 1) == 0) {
		return -1;
	}

	if ((ITM_ENA & 1) == 0) {
		return -1;
	}

	for (i = 0; i < len; i++) {

		while ((ITM_STIM_U8 & 1) == 0)
			;

		ITM_STIM_U8 = buff[i];

		while ((ITM_STIM_U8 & 1) == 0)
			;
	}

	return 0;
}

struct io_operations io_op = {
	.write = swo_print,
};
