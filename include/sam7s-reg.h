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

#ifndef SAM7S_REG_H
#define SAM7S_REG_H

/* DBGU */
#define DBGU_CR 	(0x00000000) /* (DBGU_CR) Control Register */
#define DBGU_MR        	(0x00000004) /* (DBGU_MR) Mode Register */
#define DBGU_IER       	(0x00000008) /* (DBGU_IER) Interrupt Enable Register */
#define DBGU_IDR       	(0x0000000C) /* (DBGU_IDR) Interrupt Disable Register */
#define DBGU_IMR       	(0x00000010) /* (DBGU_IMR) Interrupt Mask Register */
#define DBGU_CSR       	(0x00000014) /* (DBGU_CSR) Channel Status Register */
#define DBGU_RHR       	(0x00000018) /* (DBGU_RHR) Receiver Holding Register */
#define DBGU_THR        (0x0000001C) /* (DBGU_THR) Transmitter Holding Register */
#define DBGU_BRGR       (0x00000020) /* (DBGU_BRGR) Baud Rate Generator Register */
#define DBGU_CIDR       (0x00000040) /* (DBGU_CIDR) Chip ID Register */
#define DBGU_EXID       (0x00000044) /* (DBGU_EXID) Chip ID Extension Register */
#define DBGU_FNTR       (0x00000048) /* (DBGU_FNTR) Force NTRST Register */

// -------- DBGU_CR : (DBGU Offset: 0x0) Debug Unit Control Register --------
#define AT91C_US_RSTRX        (0x1 <<  2) // (DBGU) Reset Receiver
#define AT91C_US_RSTTX        (0x1 <<  3) // (DBGU) Reset Transmitter
#define AT91C_US_RXEN         (0x1 <<  4) // (DBGU) Receiver Enable
#define AT91C_US_RXDIS        (0x1 <<  5) // (DBGU) Receiver Disable
#define AT91C_US_TXEN         (0x1 <<  6) // (DBGU) Transmitter Enable
#define AT91C_US_TXDIS        (0x1 <<  7) // (DBGU) Transmitter Disable
#define AT91C_US_RSTSTA       (0x1 <<  8) // (DBGU) Reset Status Bits
// -------- DBGU_MR : (DBGU Offset: 0x4) Debug Unit Mode Register --------
#define AT91C_US_PAR          (0x7 <<  9) // (DBGU) Parity type
#define 	AT91C_US_PAR_EVEN                 (0x0 <<  9) // (DBGU) Even Parity
#define 	AT91C_US_PAR_ODD                  (0x1 <<  9) // (DBGU) Odd Parity
#define 	AT91C_US_PAR_SPACE                (0x2 <<  9) // (DBGU) Parity forced to 0 (Space)
#define 	AT91C_US_PAR_MARK                 (0x3 <<  9) // (DBGU) Parity forced to 1 (Mark)
#define 	AT91C_US_PAR_NONE                 (0x4 <<  9) // (DBGU) No Parity
#define 	AT91C_US_PAR_MULTI_DROP           (0x6 <<  9) // (DBGU) Multi-drop mode
#define AT91C_US_CHMODE       (0x3 << 14) // (DBGU) Channel Mode
#define 	AT91C_US_CHMODE_NORMAL               (0x0 << 14) // (DBGU) Normal Mode: The USART channel operates as an RX/TX USART.
#define 	AT91C_US_CHMODE_AUTO                 (0x1 << 14) // (DBGU) Automatic Echo: Receiver Data Input is connected to the TXD pin.
#define 	AT91C_US_CHMODE_LOCAL                (0x2 << 14) // (DBGU) Local Loopback: Transmitter Output Signal is connected to Receiver Input Signal.
#define 	AT91C_US_CHMODE_REMOTE               (0x3 << 14) // (DBGU) Remote Loopback: RXD pin is internally connected to TXD pin.
// -------- DBGU_IER : (DBGU Offset: 0x8) Debug Unit Interrupt Enable Register --------
#define AT91C_US_RXRDY        (0x1 <<  0) // (DBGU) RXRDY Interrupt
#define AT91C_US_TXRDY        (0x1 <<  1) // (DBGU) TXRDY Interrupt
#define AT91C_US_ENDRX        (0x1 <<  3) // (DBGU) End of Receive Transfer Interrupt
#define AT91C_US_ENDTX        (0x1 <<  4) // (DBGU) End of Transmit Interrupt
#define AT91C_US_OVRE         (0x1 <<  5) // (DBGU) Overrun Interrupt
#define AT91C_US_FRAME        (0x1 <<  6) // (DBGU) Framing Error Interrupt
#define AT91C_US_PARE         (0x1 <<  7) // (DBGU) Parity Error Interrupt
#define AT91C_US_TXEMPTY      (0x1 <<  9) // (DBGU) TXEMPTY Interrupt
#define AT91C_US_TXBUFE       (0x1 << 11) // (DBGU) TXBUFE Interrupt
#define AT91C_US_RXBUFF       (0x1 << 12) // (DBGU) RXBUFF Interrupt
#define AT91C_US_COMM_TX      (0x1 << 30) // (DBGU) COMM_TX Interrupt
#define AT91C_US_COMM_RX      (0x1 << 31) // (DBGU) COMM_RX Interrupt
// -------- DBGU_IDR : (DBGU Offset: 0xc) Debug Unit Interrupt Disable Register --------
// -------- DBGU_IMR : (DBGU Offset: 0x10) Debug Unit Interrupt Mask Register --------
// -------- DBGU_CSR : (DBGU Offset: 0x14) Debug Unit Channel Status Register --------
// -------- DBGU_FNTR : (DBGU Offset: 0x48) Debug Unit FORCE_NTRST Register --------
#define AT91C_US_FORCE_NTRST  (0x1 <<  0) // (DBGU) Force NTRST in JTAG

/* AIC */
#define AIC_SMR         (0x00000000) /* (AIC_SMR) Source Mode Register */
#define AIC_SVR         (0x00000080) /* (AIC_SVR) Source Vector Register */
#define AIC_IVR         (0x00000100) /* (AIC_IVR) IRQ Vector Register */
#define AIC_FVR         (0x00000104) /* (AIC_FVR) FIQ Vector Register */
#define AIC_ISR         (0x00000108) /* (AIC_ISR) Interrupt Status Register */
#define AIC_IPR         (0x0000010C) /* (AIC_IPR) Interrupt Pending Register */
#define AIC_IMR         (0x00000110) /* (AIC_IMR) Interrupt Mask Register */
#define AIC_CISR        (0x00000114) /* (AIC_CISR) Core Interrupt Status Register */
#define AIC_IECR        (0x00000120) /* (AIC_IECR) Interrupt Enable Command Register */
#define AIC_IDCR        (0x00000124) /* (AIC_IDCR) Interrupt Disable Command Register */
#define AIC_ICCR        (0x00000128) /* (AIC_ICCR) Interrupt Clear Command Register */
#define AIC_ISCR        (0x0000012C) /* (AIC_ISCR) Interrupt Set Command Register */
#define AIC_EOICR       (0x00000130) /* (AIC_EOICR) End of Interrupt Command Register */
#define AIC_SPU         (0x00000134) /* (AIC_SPU) Spurious Vector Register */
#define AIC_DCR         (0x00000138) /* (AIC_DCR) Debug Control Register (Protect) */
#define AIC_FFER        (0x00000140) /* (AIC_FFER) Fast Forcing Enable Register */
#define AIC_FFDR        (0x00000144) /* (AIC_FFDR) Fast Forcing Disable Register */
#define AIC_FFSR        (0x00000148) /* (AIC_FFSR) Fast Forcing Status Register */

// -------- AIC_SMR : (AIC Offset: 0x0) Control Register --------
#define AT91C_AIC_PRIOR       (0x7 <<  0) // (AIC) Priority Level
#define 	AT91C_AIC_PRIOR_LOWEST               (0x0) // (AIC) Lowest priority level
#define 	AT91C_AIC_PRIOR_HIGHEST              (0x7) // (AIC) Highest priority level
#define AT91C_AIC_SRCTYPE     (0x3 <<  5) // (AIC) Interrupt Source Type
#define 	AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL       (0x0 <<  5) // (AIC) Internal Sources Code Label High-level Sensitive
#define 	AT91C_AIC_SRCTYPE_EXT_LOW_LEVEL        (0x0 <<  5) // (AIC) External Sources Code Label Low-level Sensitive
#define 	AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE    (0x1 <<  5) // (AIC) Internal Sources Code Label Positive Edge triggered
#define 	AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE    (0x1 <<  5) // (AIC) External Sources Code Label Negative Edge triggered
#define 	AT91C_AIC_SRCTYPE_HIGH_LEVEL           (0x2 <<  5) // (AIC) Internal Or External Sources Code Label High-level Sensitive
#define 	AT91C_AIC_SRCTYPE_POSITIVE_EDGE        (0x3 <<  5) // (AIC) Internal Or External Sources Code Label Positive Edge triggered
// -------- AIC_CISR : (AIC Offset: 0x114) AIC Core Interrupt Status Register --------
#define AT91C_AIC_NFIQ        (0x1 <<  0) // (AIC) NFIQ Status
#define AT91C_AIC_NIRQ        (0x1 <<  1) // (AIC) NIRQ Status
// -------- AIC_DCR : (AIC Offset: 0x138) AIC Debug Control Register (Protect) --------
#define AT91C_AIC_DCR_PROT    (0x1 <<  0) // (AIC) Protection Mode
#define AT91C_AIC_DCR_GMSK    (0x1 <<  1) // (AIC) General Mask

/* PMC */
#define PMC_SCER        (0x00000000) /* (PMC_SCER) System Clock Enable Register */
#define PMC_SCDR        (0x00000004) /* (PMC_SCDR) System Clock Disable Register */
#define PMC_SCSR        (0x00000008) /* (PMC_SCSR) System Clock Status Register */
#define PMC_PCER        (0x00000010) /* (PMC_PCER) Peripheral Clock Enable Register */
#define PMC_PCDR        (0x00000014) /* (PMC_PCDR) Peripheral Clock Disable Register */
#define PMC_PCSR        (0x00000018) /* (PMC_PCSR) Peripheral Clock Status Register */
#define PMC_MCKR        (0x00000030) /* (PMC_MCKR) Master Clock Register */
#define PMC_PCKR        (0x00000040) /* (PMC_PCKR) Programmable Clock Register */
#define PMC_IER         (0x00000060) /* (PMC_IER) Interrupt Enable Register */
#define PMC_IDR         (0x00000064) /* (PMC_IDR) Interrupt Disable Register */
#define PMC_SR          (0x00000068) /* (PMC_SR) Status Register */
#define PMC_IMR         (0x0000006C) /* (PMC_IMR) Interrupt Mask Register */

// -------- PMC_SCER : (PMC Offset: 0x0) System Clock Enable Register --------
#define AT91C_PMC_PCK         (0x1 <<  0) // (PMC) Processor Clock
#define AT91C_PMC_UDP         (0x1 <<  7) // (PMC) USB Device Port Clock
#define AT91C_PMC_PCK0        (0x1 <<  8) // (PMC) Programmable Clock Output
#define AT91C_PMC_PCK1        (0x1 <<  9) // (PMC) Programmable Clock Output
#define AT91C_PMC_PCK2        (0x1 << 10) // (PMC) Programmable Clock Output
// -------- PMC_SCDR : (PMC Offset: 0x4) System Clock Disable Register --------
// -------- PMC_SCSR : (PMC Offset: 0x8) System Clock Status Register --------
// -------- CKGR_MOR : (PMC Offset: 0x20) Main Oscillator Register --------
// -------- CKGR_MCFR : (PMC Offset: 0x24) Main Clock Frequency Register --------
// -------- CKGR_PLLR : (PMC Offset: 0x2c) PLL B Register --------
// -------- PMC_MCKR : (PMC Offset: 0x30) Master Clock Register --------
#define AT91C_PMC_CSS         (0x3 <<  0) // (PMC) Programmable Clock Selection
#define 	AT91C_PMC_CSS_SLOW_CLK             (0x0) // (PMC) Slow Clock is selected
#define 	AT91C_PMC_CSS_MAIN_CLK             (0x1) // (PMC) Main Clock is selected
#define 	AT91C_PMC_CSS_PLL_CLK              (0x3) // (PMC) Clock from PLL is selected
#define AT91C_PMC_PRES        (0x7 <<  2) // (PMC) Programmable Clock Prescaler
#define 	AT91C_PMC_PRES_CLK                  (0x0 <<  2) // (PMC) Selected clock
#define 	AT91C_PMC_PRES_CLK_2                (0x1 <<  2) // (PMC) Selected clock divided by 2
#define 	AT91C_PMC_PRES_CLK_4                (0x2 <<  2) // (PMC) Selected clock divided by 4
#define 	AT91C_PMC_PRES_CLK_8                (0x3 <<  2) // (PMC) Selected clock divided by 8
#define 	AT91C_PMC_PRES_CLK_16               (0x4 <<  2) // (PMC) Selected clock divided by 16
#define 	AT91C_PMC_PRES_CLK_32               (0x5 <<  2) // (PMC) Selected clock divided by 32
#define 	AT91C_PMC_PRES_CLK_64               (0x6 <<  2) // (PMC) Selected clock divided by 64
// -------- PMC_PCKR : (PMC Offset: 0x40) Programmable Clock Register --------
// -------- PMC_IER : (PMC Offset: 0x60) PMC Interrupt Enable Register --------
#define AT91C_PMC_MOSCS       (0x1 <<  0) // (PMC) MOSC Status/Enable/Disable/Mask
#define AT91C_PMC_LOCK        (0x1 <<  2) // (PMC) PLL Status/Enable/Disable/Mask
#define AT91C_PMC_MCKRDY      (0x1 <<  3) // (PMC) MCK_RDY Status/Enable/Disable/Mask
#define AT91C_PMC_PCK0RDY     (0x1 <<  8) // (PMC) PCK0_RDY Status/Enable/Disable/Mask
#define AT91C_PMC_PCK1RDY     (0x1 <<  9) // (PMC) PCK1_RDY Status/Enable/Disable/Mask
#define AT91C_PMC_PCK2RDY     (0x1 << 10) // (PMC) PCK2_RDY Status/Enable/Disable/Mask
// -------- PMC_IDR : (PMC Offset: 0x64) PMC Interrupt Disable Register --------
// -------- PMC_SR : (PMC Offset: 0x68) PMC Status Register --------
// -------- PMC_IMR : (PMC Offset: 0x6c) PMC Interrupt Mask Register --------

/* PIT */
#define PITC_PIMR       (0x00000000) /* (PITC_PIMR) Period Interval Mode Register */
#define PITC_PISR       (0x00000004)/* (PITC_PISR) Period Interval Status Register */
#define PITC_PIVR       (0x00000008)/* (PITC_PIVR) Period Interval Value Register */
#define PITC_PIIR       (0x0000000C)/* (PITC_PIIR) Period Interval Image Register */

// -------- PITC_PIMR : (PITC Offset: 0x0) Periodic Interval Mode Register --------
#define AT91C_PITC_PIV        (0xFFFFF <<  0) // (PITC) Periodic Interval Value
#define AT91C_PITC_PITEN      (0x1 << 24) // (PITC) Periodic Interval Timer Enabled
#define AT91C_PITC_PITIEN     (0x1 << 25) // (PITC) Periodic Interval Timer Interrupt Enable
// -------- PITC_PISR : (PITC Offset: 0x4) Periodic Interval Status Register --------
#define AT91C_PITC_PITS       (0x1 <<  0) // (PITC) Periodic Interval Timer Status
// -------- PITC_PIVR : (PITC Offset: 0x8) Periodic Interval Value Register --------
#define AT91C_PITC_CPIV       (0xFFFFF <<  0) // (PITC) Current Periodic Interval Value
#define AT91C_PITC_PICNT      (0xFFF << 20) // (PITC) Periodic Interval Counter
// -------- PITC_PIIR : (PITC Offset: 0xc) Periodic Interval Image Register --------


/* PDC */
#define PDC_RPR         (  	0x00000000) // (PDC_RPR) Receive Pointer Register
#define PDC_RCR         (  	0x00000004) // (PDC_RCR) Receive Counter Register
#define PDC_TPR         (  	0x00000008) // (PDC_TPR) Transmit Pointer Register
#define PDC_TCR         (  	0x0000000C) // (PDC_TCR) Transmit Counter Register
#define PDC_RNPR        (  	0x00000010) // (PDC_RNPR) Receive Next Pointer Register
#define PDC_RNCR        (  	0x00000014) // (PDC_RNCR) Receive Next Counter Register
#define PDC_TNPR        (  	0x00000018) // (PDC_TNPR) Transmit Next Pointer Register
#define PDC_TNCR        (  	0x0000001C) // (PDC_TNCR) Transmit Next Counter Register
#define PDC_PTCR        (  	0x00000020) // (PDC_PTCR) PDC Transfer Control Register
#define PDC_PTSR        (  	0x00000024) // (PDC_PTSR) PDC Transfer Status Register

// -------- PDC_PTCR : (PDC Offset: 0x20) PDC Transfer Control Register --------
#define AT91C_PDC_RXTEN       (0x1 <<  0) // (PDC) Receiver Transfer Enable
#define AT91C_PDC_RXTDIS      (0x1 <<  1) // (PDC) Receiver Transfer Disable
#define AT91C_PDC_TXTEN       (0x1 <<  8) // (PDC) Transmitter Transfer Enable
#define AT91C_PDC_TXTDIS      (0x1 <<  9) // (PDC) Transmitter Transfer Disable
// -------- PDC_PTSR : (PDC Offset: 0x24) PDC Transfer Status Register --------

/* PIO */
#define PIO_PER         ( 	0x00000000) // (PIO_PER) PIO Enable Register
#define PIO_PDR         (  	0x00000004) // (PIO_PDR) PIO Disable Register
#define PIO_PSR         (  	0x00000008) // (PIO_PSR) PIO Status Register
#define PIO_OER         (  	0x00000010) // (PIO_OER) Output Enable Register
#define PIO_ODR         (  	0x00000014) // (PIO_ODR) Output Disable Registerr
#define PIO_OSR         (  	0x00000018) // (PIO_OSR) Output Status Register
#define PIO_IFER        (  	0x00000020) // (PIO_IFER) Input Filter Enable Register
#define PIO_IFDR        (  	0x00000024) // (PIO_IFDR) Input Filter Disable Register
#define PIO_IFSR        (  	0x00000028) // (PIO_IFSR) Input Filter Status Register
#define PIO_SODR        (  	0x00000030) // (PIO_SODR) Set Output Data Register
#define PIO_CODR        (  	0x00000034) // (PIO_CODR) Clear Output Data Register
#define PIO_ODSR        (  	0x00000038) // (PIO_ODSR) Output Data Status Register
#define PIO_PDSR        (  	0x0000003C) // (PIO_PDSR) Pin Data Status Register
#define PIO_IER         (  	0x00000040) // (PIO_IER) Interrupt Enable Register
#define PIO_IDR         (  	0x00000044) // (PIO_IDR) Interrupt Disable Register
#define PIO_IMR         (  	0x00000048) // (PIO_IMR) Interrupt Mask Register
#define PIO_ISR         (  	0x0000004C) // (PIO_ISR) Interrupt Status Register
#define PIO_MDER        (  	0x00000050) // (PIO_MDER) Multi-driver Enable Register
#define PIO_MDDR        (  	0x00000054) // (PIO_MDDR) Multi-driver Disable Register
#define PIO_MDSR        (  	0x00000058) // (PIO_MDSR) Multi-driver Status Register
#define PIO_PPUDR       (  	0x00000060) // (PIO_PPUDR) Pull-up Disable Register
#define PIO_PPUER       (  	0x00000064) // (PIO_PPUER) Pull-up Enable Register
#define PIO_PPUSR       (  	0x00000068) // (PIO_PPUSR) Pull-up Status Register
#define PIO_ASR         (  	0x00000070) // (PIO_ASR) Select A Register
#define PIO_BSR         (  	0x00000074) // (PIO_BSR) Select B Register
#define PIO_ABSR        (  	0x00000078) // (PIO_ABSR) AB Select Status Register
#define PIO_OWER        (  	0x000000A0) // (PIO_OWER) Output Write Enable Register
#define PIO_OWDR        (  	0x000000A4) // (PIO_OWDR) Output Write Disable Register
#define PIO_OWSR        (  	0x000000A8) // (PIO_OWSR) Output Write Status Register

/* CKGR */
#define CKGR_MOR        (  	0x00000000) // (CKGR_MOR) Main Oscillator Register
#define CKGR_MCFR       (  	0x00000004) // (CKGR_MCFR) Main Clock  Frequency Register
#define CKGR_PLLR       (  	0x0000000C) // (CKGR_PLLR) PLL Register

// -------- CKGR_MOR : (CKGR Offset: 0x0) Main Oscillator Register --------
#define AT91C_CKGR_MOSCEN     (0x1 <<  0) // (CKGR) Main Oscillator Enable
#define AT91C_CKGR_OSCBYPASS  (0x1 <<  1) // (CKGR) Main Oscillator Bypass
#define AT91C_CKGR_OSCOUNT    (0xFF <<  8) // (CKGR) Main Oscillator Start-up Time
// -------- CKGR_MCFR : (CKGR Offset: 0x4) Main Clock Frequency Register --------
#define AT91C_CKGR_MAINF      (0xFFFF <<  0) // (CKGR) Main Clock Frequency
#define AT91C_CKGR_MAINRDY    (0x1 << 16) // (CKGR) Main Clock Ready
// -------- CKGR_PLLR : (CKGR Offset: 0xc) PLL B Register --------
#define AT91C_CKGR_DIV        (0xFF <<  0) // (CKGR) Divider Selected
#define 	AT91C_CKGR_DIV_0                    (0x0) // (CKGR) Divider output is 0
#define 	AT91C_CKGR_DIV_BYPASS               (0x1) // (CKGR) Divider is bypassed
#define AT91C_CKGR_PLLCOUNT   (0x3F <<  8) // (CKGR) PLL Counter
#define AT91C_CKGR_OUT        (0x3 << 14) // (CKGR) PLL Output Frequency Range
#define 	AT91C_CKGR_OUT_0                    (0x0 << 14) // (CKGR) Please refer to the PLL datasheet
#define 	AT91C_CKGR_OUT_1                    (0x1 << 14) // (CKGR) Please refer to the PLL datasheet
#define 	AT91C_CKGR_OUT_2                    (0x2 << 14) // (CKGR) Please refer to the PLL datasheet
#define 	AT91C_CKGR_OUT_3                    (0x3 << 14) // (CKGR) Please refer to the PLL datasheet
#define AT91C_CKGR_MUL        (0x7FF << 16) // (CKGR) PLL Multiplier
#define AT91C_CKGR_USBDIV     (0x3 << 28) // (CKGR) Divider for USB Clocks
#define 	AT91C_CKGR_USBDIV_0                    (0x0 << 28) // (CKGR) Divider output is PLL clock output
#define 	AT91C_CKGR_USBDIV_1                    (0x1 << 28) // (CKGR) Divider output is PLL clock output divided by 2
#define 	AT91C_CKGR_USBDIV_2                    (0x2 << 28) // (CKGR) Divider output is PLL clock output divided by 4


/* RSTC */
#define RSTC_RCR        (  	0x00000000) // (RSTC_RCR) Reset Control Register
#define RSTC_RSR        (  	0x00000004) // (RSTC_RSR) Reset Status Register
#define RSTC_RMR        (  	0x00000008) // (RSTC_RMR) Reset Mode Register

// -------- RSTC_RCR : (RSTC Offset: 0x0) Reset Control Register --------
#define AT91C_RSTC_PROCRST    (0x1 <<  0) // (RSTC) Processor Reset
#define AT91C_RSTC_PERRST     (0x1 <<  2) // (RSTC) Peripheral Reset
#define AT91C_RSTC_EXTRST     (0x1 <<  3) // (RSTC) External Reset
#define AT91C_RSTC_KEY        (0xFF << 24) // (RSTC) Password
// -------- RSTC_RSR : (RSTC Offset: 0x4) Reset Status Register --------
#define AT91C_RSTC_URSTS      (0x1 <<  0) // (RSTC) User Reset Status
#define AT91C_RSTC_BODSTS     (0x1 <<  1) // (RSTC) Brownout Detection Status
#define AT91C_RSTC_RSTTYP     (0x7 <<  8) // (RSTC) Reset Type
#define 	AT91C_RSTC_RSTTYP_POWERUP              (0x0 <<  8) // (RSTC) Power-up Reset. VDDCORE rising.
#define 	AT91C_RSTC_RSTTYP_WAKEUP               (0x1 <<  8) // (RSTC) WakeUp Reset. VDDCORE rising.
#define 	AT91C_RSTC_RSTTYP_WATCHDOG             (0x2 <<  8) // (RSTC) Watchdog Reset. Watchdog overflow occured.
#define 	AT91C_RSTC_RSTTYP_SOFTWARE             (0x3 <<  8) // (RSTC) Software Reset. Processor reset required by the software.
#define 	AT91C_RSTC_RSTTYP_USER                 (0x4 <<  8) // (RSTC) User Reset. NRST pin detected low.
#define 	AT91C_RSTC_RSTTYP_BROWNOUT             (0x5 <<  8) // (RSTC) Brownout Reset occured.
#define AT91C_RSTC_NRSTL      (0x1 << 16) // (RSTC) NRST pin level
#define AT91C_RSTC_SRCMP      (0x1 << 17) // (RSTC) Software Reset Command in Progress.
// -------- RSTC_RMR : (RSTC Offset: 0x8) Reset Mode Register --------
#define AT91C_RSTC_URSTEN     (0x1 <<  0) // (RSTC) User Reset Enable
#define AT91C_RSTC_URSTIEN    (0x1 <<  4) // (RSTC) User Reset Interrupt Enable
#define AT91C_RSTC_ERSTL      (0xF <<  8) // (RSTC) User Reset Length
#define AT91C_RSTC_BODIEN     (0x1 << 16) // (RSTC) Brownout Detection Interrupt Enable

/* RTT */
#define RTTC_RTMR       (  	0x00000000) // (RTTC_RTMR) Real-time Mode Register
#define RTTC_RTAR       (  	0x00000004) // (RTTC_RTAR) Real-time Alarm Register
#define RTTC_RTVR       (  	0x00000008) // (RTTC_RTVR) Real-time Value Register
#define RTTC_RTSR       (  	0x0000000C) // (RTTC_RTSR) Real-time Status Register

// -------- RTTC_RTMR : (RTTC Offset: 0x0) Real-time Mode Register --------
#define AT91C_RTTC_RTPRES     (0xFFFF <<  0) // (RTTC) Real-time Timer Prescaler Value
#define AT91C_RTTC_ALMIEN     (0x1 << 16) // (RTTC) Alarm Interrupt Enable
#define AT91C_RTTC_RTTINCIEN  (0x1 << 17) // (RTTC) Real Time Timer Increment Interrupt Enable
#define AT91C_RTTC_RTTRST     (0x1 << 18) // (RTTC) Real Time Timer Restart
// -------- RTTC_RTAR : (RTTC Offset: 0x4) Real-time Alarm Register --------
#define AT91C_RTTC_ALMV       (0x0 <<  0) // (RTTC) Alarm Value
// -------- RTTC_RTVR : (RTTC Offset: 0x8) Current Real-time Value Register --------
#define AT91C_RTTC_CRTV       (0x0 <<  0) // (RTTC) Current Real-time Value
// -------- RTTC_RTSR : (RTTC Offset: 0xc) Real-time Status Register --------
#define AT91C_RTTC_ALMS       (0x1 <<  0) // (RTTC) Real-time Alarm Status
#define AT91C_RTTC_RTTINC     (0x1 <<  1) // (RTTC) Real-time Timer Increment

/* WDT */
#define WDTC_WDCR       (  	0x00000000) // (WDTC_WDCR) Watchdog Control Register
#define WDTC_WDMR       (  	0x00000004) // (WDTC_WDMR) Watchdog Mode Register
#define WDTC_WDSR       (  	0x00000008) // (WDTC_WDSR) Watchdog Status Register

// -------- WDTC_WDCR : (WDTC Offset: 0x0) Periodic Interval Image Register --------
#define AT91C_WDTC_WDRSTT     (0x1 <<  0) // (WDTC) Watchdog Restart
#define AT91C_WDTC_KEY        (0xFF << 24) // (WDTC) Watchdog KEY Password
// -------- WDTC_WDMR : (WDTC Offset: 0x4) Watchdog Mode Register --------
#define AT91C_WDTC_WDV        (0xFFF <<  0) // (WDTC) Watchdog Timer Restart
#define AT91C_WDTC_WDFIEN     (0x1 << 12) // (WDTC) Watchdog Fault Interrupt Enable
#define AT91C_WDTC_WDRSTEN    (0x1 << 13) // (WDTC) Watchdog Reset Enable
#define AT91C_WDTC_WDRPROC    (0x1 << 14) // (WDTC) Watchdog Timer Restart
#define AT91C_WDTC_WDDIS      (0x1 << 15) // (WDTC) Watchdog Disable
#define AT91C_WDTC_WDD        (0xFFF << 16) // (WDTC) Watchdog Delta Value
#define AT91C_WDTC_WDDBGHLT   (0x1 << 28) // (WDTC) Watchdog Debug Halt
#define AT91C_WDTC_WDIDLEHLT  (0x1 << 29) // (WDTC) Watchdog Idle Halt
// -------- WDTC_WDSR : (WDTC Offset: 0x8) Watchdog Status Register --------
#define AT91C_WDTC_WDUNF      (0x1 <<  0) // (WDTC) Watchdog Underflow
#define AT91C_WDTC_WDERR      (0x1 <<  1) // (WDTC) Watchdog Error

/* VREG */
#define VREG_MR         (  	0x00000000) // (VREG_MR) Voltage Regulator Mode Register

// -------- VREG_MR : (VREG Offset: 0x0) Voltage Regulator Mode Register --------
#define AT91C_VREG_PSTDBY     (0x1 <<  0) // (VREG) Voltage Regulator Power Standby Mode

/* MC */
#define MC_RCR          (  	0x00000000) // (MC_RCR) MC Remap Control Register
#define MC_ASR          (  	0x00000004) // (MC_ASR) MC Abort Status Register
#define MC_AASR         (  	0x00000008) // (MC_AASR) MC Abort Address Status Register
#define MC_FMR          (  	0x00000060) // (MC_FMR) MC Flash Mode Register
#define MC_FCR          (  	0x00000064) // (MC_FCR) MC Flash Command Register
#define MC_FSR          (  	0x00000068) // (MC_FSR) MC Flash Status Register

// -------- MC_RCR : (MC Offset: 0x0) MC Remap Control Register --------
#define AT91C_MC_RCB          (0x1 <<  0) // (MC) Remap Command Bit
// -------- MC_ASR : (MC Offset: 0x4) MC Abort Status Register --------
#define AT91C_MC_UNDADD       (0x1 <<  0) // (MC) Undefined Addess Abort Status
#define AT91C_MC_MISADD       (0x1 <<  1) // (MC) Misaligned Addess Abort Status
#define AT91C_MC_ABTSZ        (0x3 <<  8) // (MC) Abort Size Status
#define 	AT91C_MC_ABTSZ_BYTE                 (0x0 <<  8) // (MC) Byte
#define 	AT91C_MC_ABTSZ_HWORD                (0x1 <<  8) // (MC) Half-word
#define 	AT91C_MC_ABTSZ_WORD                 (0x2 <<  8) // (MC) Word
#define AT91C_MC_ABTTYP       (0x3 << 10) // (MC) Abort Type Status
#define 	AT91C_MC_ABTTYP_DATAR                (0x0 << 10) // (MC) Data Read
#define 	AT91C_MC_ABTTYP_DATAW                (0x1 << 10) // (MC) Data Write
#define 	AT91C_MC_ABTTYP_FETCH                (0x2 << 10) // (MC) Code Fetch
#define AT91C_MC_MST0         (0x1 << 16) // (MC) Master 0 Abort Source
#define AT91C_MC_MST1         (0x1 << 17) // (MC) Master 1 Abort Source
#define AT91C_MC_SVMST0       (0x1 << 24) // (MC) Saved Master 0 Abort Source
#define AT91C_MC_SVMST1       (0x1 << 25) // (MC) Saved Master 1 Abort Source
// -------- MC_FMR : (MC Offset: 0x60) MC Flash Mode Register --------
#define AT91C_MC_FRDY         (0x1 <<  0) // (MC) Flash Ready
#define AT91C_MC_LOCKE        (0x1 <<  2) // (MC) Lock Error
#define AT91C_MC_PROGE        (0x1 <<  3) // (MC) Programming Error
#define AT91C_MC_NEBP         (0x1 <<  7) // (MC) No Erase Before Programming
#define AT91C_MC_FWS          (0x3 <<  8) // (MC) Flash Wait State
#define 	AT91C_MC_FWS_0FWS                 (0x0 <<  8) // (MC) 1 cycle for Read, 2 for Write operations
#define 	AT91C_MC_FWS_1FWS                 (0x1 <<  8) // (MC) 2 cycles for Read, 3 for Write operations
#define 	AT91C_MC_FWS_2FWS                 (0x2 <<  8) // (MC) 3 cycles for Read, 4 for Write operations
#define 	AT91C_MC_FWS_3FWS                 (0x3 <<  8) // (MC) 4 cycles for Read, 4 for Write operations
#define AT91C_MC_FMCN         (0xFF << 16) // (MC) Flash Microsecond Cycle Number
// -------- MC_FCR : (MC Offset: 0x64) MC Flash Command Register --------
#define AT91C_MC_FCMD         (0xF <<  0) // (MC) Flash Command
#define 	AT91C_MC_FCMD_START_PROG           (0x1) // (MC) Starts the programming of th epage specified by PAGEN.
#define 	AT91C_MC_FCMD_LOCK                 (0x2) // (MC) Starts a lock sequence of the sector defined by the bits 4 to 7 of the field PAGEN.
#define 	AT91C_MC_FCMD_PROG_AND_LOCK        (0x3) // (MC) The lock sequence automatically happens after the programming sequence is completed.
#define 	AT91C_MC_FCMD_UNLOCK               (0x4) // (MC) Starts an unlock sequence of the sector defined by the bits 4 to 7 of the field PAGEN.
#define 	AT91C_MC_FCMD_ERASE_ALL            (0x8) // (MC) Starts the erase of the entire flash.If at least a page is locked, the command is cancelled.
#define 	AT91C_MC_FCMD_SET_GP_NVM           (0xB) // (MC) Set General Purpose NVM bits.
#define 	AT91C_MC_FCMD_CLR_GP_NVM           (0xD) // (MC) Clear General Purpose NVM bits.
#define 	AT91C_MC_FCMD_SET_SECURITY         (0xF) // (MC) Set Security Bit.
#define AT91C_MC_PAGEN        (0x3FF <<  8) // (MC) Page Number
#define AT91C_MC_KEY          (0xFF << 24) // (MC) Writing Protect Key
// -------- MC_FSR : (MC Offset: 0x68) MC Flash Command Register --------
#define AT91C_MC_SECURITY     (0x1 <<  4) // (MC) Security Bit Status
#define AT91C_MC_GPNVM0       (0x1 <<  8) // (MC) Sector 0 Lock Status
#define AT91C_MC_GPNVM1       (0x1 <<  9) // (MC) Sector 1 Lock Status
#define AT91C_MC_GPNVM2       (0x1 << 10) // (MC) Sector 2 Lock Status
#define AT91C_MC_GPNVM3       (0x1 << 11) // (MC) Sector 3 Lock Status
#define AT91C_MC_GPNVM4       (0x1 << 12) // (MC) Sector 4 Lock Status
#define AT91C_MC_GPNVM5       (0x1 << 13) // (MC) Sector 5 Lock Status
#define AT91C_MC_GPNVM6       (0x1 << 14) // (MC) Sector 6 Lock Status
#define AT91C_MC_GPNVM7       (0x1 << 15) // (MC) Sector 7 Lock Status
#define AT91C_MC_LOCKS0       (0x1 << 16) // (MC) Sector 0 Lock Status
#define AT91C_MC_LOCKS1       (0x1 << 17) // (MC) Sector 1 Lock Status
#define AT91C_MC_LOCKS2       (0x1 << 18) // (MC) Sector 2 Lock Status
#define AT91C_MC_LOCKS3       (0x1 << 19) // (MC) Sector 3 Lock Status
#define AT91C_MC_LOCKS4       (0x1 << 20) // (MC) Sector 4 Lock Status
#define AT91C_MC_LOCKS5       (0x1 << 21) // (MC) Sector 5 Lock Status
#define AT91C_MC_LOCKS6       (0x1 << 22) // (MC) Sector 6 Lock Status
#define AT91C_MC_LOCKS7       (0x1 << 23) // (MC) Sector 7 Lock Status
#define AT91C_MC_LOCKS8       (0x1 << 24) // (MC) Sector 8 Lock Status
#define AT91C_MC_LOCKS9       (0x1 << 25) // (MC) Sector 9 Lock Status
#define AT91C_MC_LOCKS10      (0x1 << 26) // (MC) Sector 10 Lock Status
#define AT91C_MC_LOCKS11      (0x1 << 27) // (MC) Sector 11 Lock Status
#define AT91C_MC_LOCKS12      (0x1 << 28) // (MC) Sector 12 Lock Status
#define AT91C_MC_LOCKS13      (0x1 << 29) // (MC) Sector 13 Lock Status
#define AT91C_MC_LOCKS14      (0x1 << 30) // (MC) Sector 14 Lock Status
#define AT91C_MC_LOCKS15      (0x1 << 31) // (MC) Sector 15 Lock Status

/* SPI */
#define SPI_CR          (  	0x00000000) // (SPI_CR) Control Register
#define SPI_MR          (  	0x00000004) // (SPI_MR) Mode Register
#define SPI_RDR         (  	0x00000008) // (SPI_RDR) Receive Data Register
#define SPI_TDR         (  	0x0000000C) // (SPI_TDR) Transmit Data Register
#define SPI_SR          (  	0x00000010) // (SPI_SR) Status Register
#define SPI_IER         (  	0x00000014) // (SPI_IER) Interrupt Enable Register
#define SPI_IDR         (  	0x00000018) // (SPI_IDR) Interrupt Disable Register
#define SPI_IMR         (  	0x0000001C) // (SPI_IMR) Interrupt Mask Register
#define SPI_CSR         (  	0x00000030) // (SPI_CSR) Chip Select Register

// -------- SPI_CR : (SPI Offset: 0x0) SPI Control Register --------
#define AT91C_SPI_SPIEN       (0x1 <<  0) // (SPI) SPI Enable
#define AT91C_SPI_SPIDIS      (0x1 <<  1) // (SPI) SPI Disable
#define AT91C_SPI_SWRST       (0x1 <<  7) // (SPI) SPI Software reset
#define AT91C_SPI_LASTXFER    (0x1 << 24) // (SPI) SPI Last Transfer
// -------- SPI_MR : (SPI Offset: 0x4) SPI Mode Register --------
#define AT91C_SPI_MSTR        (0x1 <<  0) // (SPI) Master/Slave Mode
#define AT91C_SPI_PS          (0x1 <<  1) // (SPI) Peripheral Select
#define 	AT91C_SPI_PS_FIXED                (0x0 <<  1) // (SPI) Fixed Peripheral Select
#define 	AT91C_SPI_PS_VARIABLE             (0x1 <<  1) // (SPI) Variable Peripheral Select
#define AT91C_SPI_PCSDEC      (0x1 <<  2) // (SPI) Chip Select Decode
#define AT91C_SPI_FDIV        (0x1 <<  3) // (SPI) Clock Selection
#define AT91C_SPI_MODFDIS     (0x1 <<  4) // (SPI) Mode Fault Detection
#define AT91C_SPI_LLB         (0x1 <<  7) // (SPI) Clock Selection
#define AT91C_SPI_PCS         (0xF << 16) // (SPI) Peripheral Chip Select
#define AT91C_SPI_DLYBCS      (0xFF << 24) // (SPI) Delay Between Chip Selects
// -------- SPI_RDR : (SPI Offset: 0x8) Receive Data Register --------
#define AT91C_SPI_RD          (0xFFFF <<  0) // (SPI) Receive Data
#define AT91C_SPI_RPCS        (0xF << 16) // (SPI) Peripheral Chip Select Status
// -------- SPI_TDR : (SPI Offset: 0xc) Transmit Data Register --------
#define AT91C_SPI_TD          (0xFFFF <<  0) // (SPI) Transmit Data
#define AT91C_SPI_TPCS        (0xF << 16) // (SPI) Peripheral Chip Select Status
// -------- SPI_SR : (SPI Offset: 0x10) Status Register --------
#define AT91C_SPI_RDRF        (0x1 <<  0) // (SPI) Receive Data Register Full
#define AT91C_SPI_TDRE        (0x1 <<  1) // (SPI) Transmit Data Register Empty
#define AT91C_SPI_MODF        (0x1 <<  2) // (SPI) Mode Fault Error
#define AT91C_SPI_OVRES       (0x1 <<  3) // (SPI) Overrun Error Status
#define AT91C_SPI_ENDRX       (0x1 <<  4) // (SPI) End of Receiver Transfer
#define AT91C_SPI_ENDTX       (0x1 <<  5) // (SPI) End of Receiver Transfer
#define AT91C_SPI_RXBUFF      (0x1 <<  6) // (SPI) RXBUFF Interrupt
#define AT91C_SPI_TXBUFE      (0x1 <<  7) // (SPI) TXBUFE Interrupt
#define AT91C_SPI_NSSR        (0x1 <<  8) // (SPI) NSSR Interrupt
#define AT91C_SPI_TXEMPTY     (0x1 <<  9) // (SPI) TXEMPTY Interrupt
#define AT91C_SPI_SPIENS      (0x1 << 16) // (SPI) Enable Status
// -------- SPI_IER : (SPI Offset: 0x14) Interrupt Enable Register --------
// -------- SPI_IDR : (SPI Offset: 0x18) Interrupt Disable Register --------
// -------- SPI_IMR : (SPI Offset: 0x1c) Interrupt Mask Register --------
// -------- SPI_CSR : (SPI Offset: 0x30) Chip Select Register --------
#define AT91C_SPI_CPOL        (0x1 <<  0) // (SPI) Clock Polarity
#define AT91C_SPI_NCPHA       (0x1 <<  1) // (SPI) Clock Phase
#define AT91C_SPI_CSAAT       (0x1 <<  3) // (SPI) Chip Select Active After Transfer
#define AT91C_SPI_BITS        (0xF <<  4) // (SPI) Bits Per Transfer
#define 	AT91C_SPI_BITS_8                    (0x0 <<  4) // (SPI) 8 Bits Per transfer
#define 	AT91C_SPI_BITS_9                    (0x1 <<  4) // (SPI) 9 Bits Per transfer
#define 	AT91C_SPI_BITS_10                   (0x2 <<  4) // (SPI) 10 Bits Per transfer
#define 	AT91C_SPI_BITS_11                   (0x3 <<  4) // (SPI) 11 Bits Per transfer
#define 	AT91C_SPI_BITS_12                   (0x4 <<  4) // (SPI) 12 Bits Per transfer
#define 	AT91C_SPI_BITS_13                   (0x5 <<  4) // (SPI) 13 Bits Per transfer
#define 	AT91C_SPI_BITS_14                   (0x6 <<  4) // (SPI) 14 Bits Per transfer
#define 	AT91C_SPI_BITS_15                   (0x7 <<  4) // (SPI) 15 Bits Per transfer
#define 	AT91C_SPI_BITS_16                   (0x8 <<  4) // (SPI) 16 Bits Per transfer
#define AT91C_SPI_SCBR        (0xFF <<  8) // (SPI) Serial Clock Baud Rate
#define AT91C_SPI_DLYBS       (0xFF << 16) // (SPI) Delay Before SPCK
#define AT91C_SPI_DLYBCT      (0xFF << 24) // (SPI) Delay Between Consecutive Transfers

/* ADC */
#define ADC_CR          (  	0x00000000) // (ADC_CR) ADC Control Register
#define ADC_MR          (  	0x00000004) // (ADC_MR) ADC Mode Register
#define ADC_CHER        (  	0x00000010) // (ADC_CHER) ADC Channel Enable Register
#define ADC_CHDR        (  	0x00000014) // (ADC_CHDR) ADC Channel Disable Register
#define ADC_CHSR        (  	0x00000018) // (ADC_CHSR) ADC Channel Status Register
#define ADC_SR          (  	0x0000001C) // (ADC_SR) ADC Status Register
#define ADC_LCDR        (  	0x00000020) // (ADC_LCDR) ADC Last Converted Data Register
#define ADC_IER         (  	0x00000024) // (ADC_IER) ADC Interrupt Enable Register
#define ADC_IDR         (  	0x00000028) // (ADC_IDR) ADC Interrupt Disable Register
#define ADC_IMR         (  	0x0000002C) // (ADC_IMR) ADC Interrupt Mask Register
#define ADC_CDR0        (  	0x00000030) // (ADC_CDR0) ADC Channel Data Register 0
#define ADC_CDR1        (  	0x00000034) // (ADC_CDR1) ADC Channel Data Register 1
#define ADC_CDR2        (  	0x00000038) // (ADC_CDR2) ADC Channel Data Register 2
#define ADC_CDR3        (  	0x0000003C) // (ADC_CDR3) ADC Channel Data Register 3
#define ADC_CDR4        (  	0x00000040) // (ADC_CDR4) ADC Channel Data Register 4
#define ADC_CDR5        (  	0x00000044) // (ADC_CDR5) ADC Channel Data Register 5
#define ADC_CDR6        (  	0x00000048) // (ADC_CDR6) ADC Channel Data Register 6
#define ADC_CDR7        (  	0x0000004C) // (ADC_CDR7) ADC Channel Data Register 7

// -------- ADC_CR : (ADC Offset: 0x0) ADC Control Register --------
#define AT91C_ADC_SWRST       (0x1 <<  0) // (ADC) Software Reset
#define AT91C_ADC_START       (0x1 <<  1) // (ADC) Start Conversion
// -------- ADC_MR : (ADC Offset: 0x4) ADC Mode Register --------
#define AT91C_ADC_TRGEN       (0x1 <<  0) // (ADC) Trigger Enable
#define 	AT91C_ADC_TRGEN_DIS                  (0x0) // (ADC) Hradware triggers are disabled. Starting a conversion is only possible by software
#define 	AT91C_ADC_TRGEN_EN                   (0x1) // (ADC) Hardware trigger selected by TRGSEL field is enabled.
#define AT91C_ADC_TRGSEL      (0x7 <<  1) // (ADC) Trigger Selection
#define 	AT91C_ADC_TRGSEL_TIOA0                (0x0 <<  1) // (ADC) Selected TRGSEL = TIAO0
#define 	AT91C_ADC_TRGSEL_TIOA1                (0x1 <<  1) // (ADC) Selected TRGSEL = TIAO1
#define 	AT91C_ADC_TRGSEL_TIOA2                (0x2 <<  1) // (ADC) Selected TRGSEL = TIAO2
#define 	AT91C_ADC_TRGSEL_TIOA3                (0x3 <<  1) // (ADC) Selected TRGSEL = TIAO3
#define 	AT91C_ADC_TRGSEL_TIOA4                (0x4 <<  1) // (ADC) Selected TRGSEL = TIAO4
#define 	AT91C_ADC_TRGSEL_TIOA5                (0x5 <<  1) // (ADC) Selected TRGSEL = TIAO5
#define 	AT91C_ADC_TRGSEL_EXT                  (0x6 <<  1) // (ADC) Selected TRGSEL = External Trigger
#define AT91C_ADC_LOWRES      (0x1 <<  4) // (ADC) Resolution.
#define 	AT91C_ADC_LOWRES_10_BIT               (0x0 <<  4) // (ADC) 10-bit resolution
#define 	AT91C_ADC_LOWRES_8_BIT                (0x1 <<  4) // (ADC) 8-bit resolution
#define AT91C_ADC_SLEEP       (0x1 <<  5) // (ADC) Sleep Mode
#define 	AT91C_ADC_SLEEP_NORMAL_MODE          (0x0 <<  5) // (ADC) Normal Mode
#define 	AT91C_ADC_SLEEP_MODE                 (0x1 <<  5) // (ADC) Sleep Mode
#define AT91C_ADC_PRESCAL     (0x3F <<  8) // (ADC) Prescaler rate selection
#define AT91C_ADC_STARTUP     (0x1F << 16) // (ADC) Startup Time
#define AT91C_ADC_SHTIM       (0xF << 24) // (ADC) Sample & Hold Time
// -------- 	ADC_CHER : (ADC Offset: 0x10) ADC Channel Enable Register --------
#define AT91C_ADC_CH0         (0x1 <<  0) // (ADC) Channel 0
#define AT91C_ADC_CH1         (0x1 <<  1) // (ADC) Channel 1
#define AT91C_ADC_CH2         (0x1 <<  2) // (ADC) Channel 2
#define AT91C_ADC_CH3         (0x1 <<  3) // (ADC) Channel 3
#define AT91C_ADC_CH4         (0x1 <<  4) // (ADC) Channel 4
#define AT91C_ADC_CH5         (0x1 <<  5) // (ADC) Channel 5
#define AT91C_ADC_CH6         (0x1 <<  6) // (ADC) Channel 6
#define AT91C_ADC_CH7         (0x1 <<  7) // (ADC) Channel 7
// -------- 	ADC_CHDR : (ADC Offset: 0x14) ADC Channel Disable Register --------
// -------- 	ADC_CHSR : (ADC Offset: 0x18) ADC Channel Status Register --------
// -------- ADC_SR : (ADC Offset: 0x1c) ADC Status Register --------
#define AT91C_ADC_EOC0        (0x1 <<  0) // (ADC) End of Conversion
#define AT91C_ADC_EOC1        (0x1 <<  1) // (ADC) End of Conversion
#define AT91C_ADC_EOC2        (0x1 <<  2) // (ADC) End of Conversion
#define AT91C_ADC_EOC3        (0x1 <<  3) // (ADC) End of Conversion
#define AT91C_ADC_EOC4        (0x1 <<  4) // (ADC) End of Conversion
#define AT91C_ADC_EOC5        (0x1 <<  5) // (ADC) End of Conversion
#define AT91C_ADC_EOC6        (0x1 <<  6) // (ADC) End of Conversion
#define AT91C_ADC_EOC7        (0x1 <<  7) // (ADC) End of Conversion
#define AT91C_ADC_OVRE0       (0x1 <<  8) // (ADC) Overrun Error
#define AT91C_ADC_OVRE1       (0x1 <<  9) // (ADC) Overrun Error
#define AT91C_ADC_OVRE2       (0x1 << 10) // (ADC) Overrun Error
#define AT91C_ADC_OVRE3       (0x1 << 11) // (ADC) Overrun Error
#define AT91C_ADC_OVRE4       (0x1 << 12) // (ADC) Overrun Error
#define AT91C_ADC_OVRE5       (0x1 << 13) // (ADC) Overrun Error
#define AT91C_ADC_OVRE6       (0x1 << 14) // (ADC) Overrun Error
#define AT91C_ADC_OVRE7       (0x1 << 15) // (ADC) Overrun Error
#define AT91C_ADC_DRDY        (0x1 << 16) // (ADC) Data Ready
#define AT91C_ADC_GOVRE       (0x1 << 17) // (ADC) General Overrun
#define AT91C_ADC_ENDRX       (0x1 << 18) // (ADC) End of Receiver Transfer
#define AT91C_ADC_RXBUFF      (0x1 << 19) // (ADC) RXBUFF Interrupt
// -------- ADC_LCDR : (ADC Offset: 0x20) ADC Last Converted Data Register --------
#define AT91C_ADC_LDATA       (0x3FF <<  0) // (ADC) Last Data Converted
// -------- ADC_IER : (ADC Offset: 0x24) ADC Interrupt Enable Register --------
// -------- ADC_IDR : (ADC Offset: 0x28) ADC Interrupt Disable Register --------
// -------- ADC_IMR : (ADC Offset: 0x2c) ADC Interrupt Mask Register --------
// -------- ADC_CDR0 : (ADC Offset: 0x30) ADC Channel Data Register 0 --------
#define AT91C_ADC_DATA        (0x3FF <<  0) // (ADC) Converted Data
// -------- ADC_CDR1 : (ADC Offset: 0x34) ADC Channel Data Register 1 --------
// -------- ADC_CDR2 : (ADC Offset: 0x38) ADC Channel Data Register 2 --------
// -------- ADC_CDR3 : (ADC Offset: 0x3c) ADC Channel Data Register 3 --------
// -------- ADC_CDR4 : (ADC Offset: 0x40) ADC Channel Data Register 4 --------
// -------- ADC_CDR5 : (ADC Offset: 0x44) ADC Channel Data Register 5 --------
// -------- ADC_CDR6 : (ADC Offset: 0x48) ADC Channel Data Register 6 --------
// -------- ADC_CDR7 : (ADC Offset: 0x4c) ADC Channel Data Register 7 --------

/* SSC */
#define SSC_CR          (  	0x00000000) // (SSC_CR) Control Register
#define SSC_CMR         (  	0x00000004) // (SSC_CMR) Clock Mode Register
#define SSC_RCMR        (  	0x00000010) // (SSC_RCMR) Receive Clock ModeRegister
#define SSC_RFMR        (  	0x00000014) // (SSC_RFMR) Receive Frame Mode Register
#define SSC_TCMR        (  	0x00000018) // (SSC_TCMR) Transmit Clock Mode Register
#define SSC_TFMR        (  	0x0000001C) // (SSC_TFMR) Transmit Frame Mode Register
#define SSC_RHR         (  	0x00000020) // (SSC_RHR) Receive Holding Register
#define SSC_THR         (  	0x00000024) // (SSC_THR) Transmit Holding Register
#define SSC_RSHR        (  	0x00000030) // (SSC_RSHR) Receive Sync Holding Register
#define SSC_TSHR        (  	0x00000034) // (SSC_TSHR) Transmit Sync Holding Register
#define SSC_SR          (  	0x00000040) // (SSC_SR) Status Register
#define SSC_IER         (  	0x00000044) // (SSC_IER) Interrupt Enable Register
#define SSC_IDR         (  	0x00000048) // (SSC_IDR) Interrupt Disable Register
#define SSC_IMR         (  	0x0000004C) // (SSC_IMR) Interrupt Mask Register

// -------- SSC_CR : (SSC Offset: 0x0) SSC Control Register --------
#define AT91C_SSC_RXEN        (0x1 <<  0) // (SSC) Receive Enable
#define AT91C_SSC_RXDIS       (0x1 <<  1) // (SSC) Receive Disable
#define AT91C_SSC_TXEN        (0x1 <<  8) // (SSC) Transmit Enable
#define AT91C_SSC_TXDIS       (0x1 <<  9) // (SSC) Transmit Disable
#define AT91C_SSC_SWRST       (0x1 << 15) // (SSC) Software Reset
// -------- SSC_RCMR : (SSC Offset: 0x10) SSC Receive Clock Mode Register --------
#define AT91C_SSC_CKS         (0x3 <<  0) // (SSC) Receive/Transmit Clock Selection
#define 	AT91C_SSC_CKS_DIV                  (0x0) // (SSC) Divided Clock
#define 	AT91C_SSC_CKS_TK                   (0x1) // (SSC) TK Clock signal
#define 	AT91C_SSC_CKS_RK                   (0x2) // (SSC) RK pin
#define AT91C_SSC_CKO         (0x7 <<  2) // (SSC) Receive/Transmit Clock Output Mode Selection
#define 	AT91C_SSC_CKO_NONE                 (0x0 <<  2) // (SSC) Receive/Transmit Clock Output Mode: None RK pin: Input-only
#define 	AT91C_SSC_CKO_CONTINOUS            (0x1 <<  2) // (SSC) Continuous Receive/Transmit Clock RK pin: Output
#define 	AT91C_SSC_CKO_DATA_TX              (0x2 <<  2) // (SSC) Receive/Transmit Clock only during data transfers RK pin: Output
#define AT91C_SSC_CKI         (0x1 <<  5) // (SSC) Receive/Transmit Clock Inversion
#define AT91C_SSC_START       (0xF <<  8) // (SSC) Receive/Transmit Start Selection
#define 	AT91C_SSC_START_CONTINOUS            (0x0 <<  8) // (SSC) Continuous, as soon as the receiver is enabled, and immediately after the end of transfer of the previous data.
#define 	AT91C_SSC_START_TX                   (0x1 <<  8) // (SSC) Transmit/Receive start
#define 	AT91C_SSC_START_LOW_RF               (0x2 <<  8) // (SSC) Detection of a low level on RF input
#define 	AT91C_SSC_START_HIGH_RF              (0x3 <<  8) // (SSC) Detection of a high level on RF input
#define 	AT91C_SSC_START_FALL_RF              (0x4 <<  8) // (SSC) Detection of a falling edge on RF input
#define 	AT91C_SSC_START_RISE_RF              (0x5 <<  8) // (SSC) Detection of a rising edge on RF input
#define 	AT91C_SSC_START_LEVEL_RF             (0x6 <<  8) // (SSC) Detection of any level change on RF input
#define 	AT91C_SSC_START_EDGE_RF              (0x7 <<  8) // (SSC) Detection of any edge on RF input
#define 	AT91C_SSC_START_0                    (0x8 <<  8) // (SSC) Compare 0
#define AT91C_SSC_STTDLY      (0xFF << 16) // (SSC) Receive/Transmit Start Delay
#define AT91C_SSC_PERIOD      (0xFF << 24) // (SSC) Receive/Transmit Period Divider Selection
// -------- SSC_RFMR : (SSC Offset: 0x14) SSC Receive Frame Mode Register --------
#define AT91C_SSC_DATLEN      (0x1F <<  0) // (SSC) Data Length
#define AT91C_SSC_LOOP        (0x1 <<  5) // (SSC) Loop Mode
#define AT91C_SSC_MSBF        (0x1 <<  7) // (SSC) Most Significant Bit First
#define AT91C_SSC_DATNB       (0xF <<  8) // (SSC) Data Number per Frame
#define AT91C_SSC_FSLEN       (0xF << 16) // (SSC) Receive/Transmit Frame Sync length
#define AT91C_SSC_FSOS        (0x7 << 20) // (SSC) Receive/Transmit Frame Sync Output Selection
#define 	AT91C_SSC_FSOS_NONE                 (0x0 << 20) // (SSC) Selected Receive/Transmit Frame Sync Signal: None RK pin Input-only
#define 	AT91C_SSC_FSOS_NEGATIVE             (0x1 << 20) // (SSC) Selected Receive/Transmit Frame Sync Signal: Negative Pulse
#define 	AT91C_SSC_FSOS_POSITIVE             (0x2 << 20) // (SSC) Selected Receive/Transmit Frame Sync Signal: Positive Pulse
#define 	AT91C_SSC_FSOS_LOW                  (0x3 << 20) // (SSC) Selected Receive/Transmit Frame Sync Signal: Driver Low during data transfer
#define 	AT91C_SSC_FSOS_HIGH                 (0x4 << 20) // (SSC) Selected Receive/Transmit Frame Sync Signal: Driver High during data transfer
#define 	AT91C_SSC_FSOS_TOGGLE               (0x5 << 20) // (SSC) Selected Receive/Transmit Frame Sync Signal: Toggling at each start of data transfer
#define AT91C_SSC_FSEDGE      (0x1 << 24) // (SSC) Frame Sync Edge Detection
// -------- SSC_TCMR : (SSC Offset: 0x18) SSC Transmit Clock Mode Register --------
// -------- SSC_TFMR : (SSC Offset: 0x1c) SSC Transmit Frame Mode Register --------
#define AT91C_SSC_DATDEF      (0x1 <<  5) // (SSC) Data Default Value
#define AT91C_SSC_FSDEN       (0x1 << 23) // (SSC) Frame Sync Data Enable
// -------- SSC_SR : (SSC Offset: 0x40) SSC Status Register --------
#define AT91C_SSC_TXRDY       (0x1 <<  0) // (SSC) Transmit Ready
#define AT91C_SSC_TXEMPTY     (0x1 <<  1) // (SSC) Transmit Empty
#define AT91C_SSC_ENDTX       (0x1 <<  2) // (SSC) End Of Transmission
#define AT91C_SSC_TXBUFE      (0x1 <<  3) // (SSC) Transmit Buffer Empty
#define AT91C_SSC_RXRDY       (0x1 <<  4) // (SSC) Receive Ready
#define AT91C_SSC_OVRUN       (0x1 <<  5) // (SSC) Receive Overrun
#define AT91C_SSC_ENDRX       (0x1 <<  6) // (SSC) End of Reception
#define AT91C_SSC_RXBUFF      (0x1 <<  7) // (SSC) Receive Buffer Full
#define AT91C_SSC_TXSYN       (0x1 << 10) // (SSC) Transmit Sync
#define AT91C_SSC_RXSYN       (0x1 << 11) // (SSC) Receive Sync
#define AT91C_SSC_TXENA       (0x1 << 16) // (SSC) Transmit Enable
#define AT91C_SSC_RXENA       (0x1 << 17) // (SSC) Receive Enable
// -------- SSC_IER : (SSC Offset: 0x44) SSC Interrupt Enable Register --------
// -------- SSC_IDR : (SSC Offset: 0x48) SSC Interrupt Disable Register --------
// -------- SSC_IMR : (SSC Offset: 0x4c) SSC Interrupt Mask Register --------

/* USART */
#define US_CR           (  	0x00000000) // (US_CR) Control Register
#define US_MR           (  	0x00000004) // (US_MR) Mode Register
#define US_IER          (  	0x00000008) // (US_IER) Interrupt Enable Register
#define US_IDR          (  	0x0000000C) // (US_IDR) Interrupt Disable Register
#define US_IMR          (  	0x00000010) // (US_IMR) Interrupt Mask Register
#define US_CSR          (  	0x00000014) // (US_CSR) Channel Status Register
#define US_RHR          (  	0x00000018) // (US_RHR) Receiver Holding Register
#define US_THR          (  	0x0000001C) // (US_THR) Transmitter Holding Register
#define US_BRGR         (  	0x00000020) // (US_BRGR) Baud Rate Generator Register
#define US_RTOR         (  	0x00000024) // (US_RTOR) Receiver Time-out Register
#define US_TTGR         (  	0x00000028) // (US_TTGR) Transmitter Time-guard Register
#define US_FIDI         (  	0x00000040) // (US_FIDI) FI_DI_Ratio Register
#define US_NER          (  	0x00000044) // (US_NER) Nb Errors Register
#define US_IF           (  	0x0000004C) // (US_IF) IRDA_FILTER Register

// -------- US_CR : (USART Offset: 0x0) Debug Unit Control Register --------
#define AT91C_US_STTBRK       (0x1 <<  9) // (USART) Start Break
#define AT91C_US_STPBRK       (0x1 << 10) // (USART) Stop Break
#define AT91C_US_STTTO        (0x1 << 11) // (USART) Start Time-out
#define AT91C_US_SENDA        (0x1 << 12) // (USART) Send Address
#define AT91C_US_RSTIT        (0x1 << 13) // (USART) Reset Iterations
#define AT91C_US_RSTNACK      (0x1 << 14) // (USART) Reset Non Acknowledge
#define AT91C_US_RETTO        (0x1 << 15) // (USART) Rearm Time-out
#define AT91C_US_DTREN        (0x1 << 16) // (USART) Data Terminal ready Enable
#define AT91C_US_DTRDIS       (0x1 << 17) // (USART) Data Terminal ready Disable
#define AT91C_US_RTSEN        (0x1 << 18) // (USART) Request to Send enable
#define AT91C_US_RTSDIS       (0x1 << 19) // (USART) Request to Send Disable
// -------- US_MR : (USART Offset: 0x4) Debug Unit Mode Register --------
#define AT91C_US_USMODE       (0xF <<  0) // (USART) Usart mode
#define 	AT91C_US_USMODE_NORMAL               (0x0) // (USART) Normal
#define 	AT91C_US_USMODE_RS485                (0x1) // (USART) RS485
#define 	AT91C_US_USMODE_HWHSH                (0x2) // (USART) Hardware Handshaking
#define 	AT91C_US_USMODE_MODEM                (0x3) // (USART) Modem
#define 	AT91C_US_USMODE_ISO7816_0            (0x4) // (USART) ISO7816 protocol: T = 0
#define 	AT91C_US_USMODE_ISO7816_1            (0x6) // (USART) ISO7816 protocol: T = 1
#define 	AT91C_US_USMODE_IRDA                 (0x8) // (USART) IrDA
#define 	AT91C_US_USMODE_SWHSH                (0xC) // (USART) Software Handshaking
#define AT91C_US_CLKS         (0x3 <<  4) // (USART) Clock Selection (Baud Rate generator Input Clock
#define 	AT91C_US_CLKS_CLOCK                (0x0 <<  4) // (USART) Clock
#define 	AT91C_US_CLKS_FDIV1                (0x1 <<  4) // (USART) fdiv1
#define 	AT91C_US_CLKS_SLOW                 (0x2 <<  4) // (USART) slow_clock (ARM)
#define 	AT91C_US_CLKS_EXT                  (0x3 <<  4) // (USART) External (SCK)
#define AT91C_US_CHRL         (0x3 <<  6) // (USART) Clock Selection (Baud Rate generator Input Clock
#define 	AT91C_US_CHRL_5_BITS               (0x0 <<  6) // (USART) Character Length: 5 bits
#define 	AT91C_US_CHRL_6_BITS               (0x1 <<  6) // (USART) Character Length: 6 bits
#define 	AT91C_US_CHRL_7_BITS               (0x2 <<  6) // (USART) Character Length: 7 bits
#define 	AT91C_US_CHRL_8_BITS               (0x3 <<  6) // (USART) Character Length: 8 bits
#define AT91C_US_SYNC         (0x1 <<  8) // (USART) Synchronous Mode Select
#define AT91C_US_NBSTOP       (0x3 << 12) // (USART) Number of Stop bits
#define 	AT91C_US_NBSTOP_1_BIT                (0x0 << 12) // (USART) 1 stop bit
#define 	AT91C_US_NBSTOP_15_BIT               (0x1 << 12) // (USART) Asynchronous (SYNC=0) 2 stop bits Synchronous (SYNC=1) 2 stop bits
#define 	AT91C_US_NBSTOP_2_BIT                (0x2 << 12) // (USART) 2 stop bits
#define AT91C_US_MSBF         (0x1 << 16) // (USART) Bit Order
#define AT91C_US_MODE9        (0x1 << 17) // (USART) 9-bit Character length
#define AT91C_US_CKLO         (0x1 << 18) // (USART) Clock Output Select
#define AT91C_US_OVER         (0x1 << 19) // (USART) Over Sampling Mode
#define AT91C_US_INACK        (0x1 << 20) // (USART) Inhibit Non Acknowledge
#define AT91C_US_DSNACK       (0x1 << 21) // (USART) Disable Successive NACK
#define AT91C_US_MAX_ITER     (0x1 << 24) // (USART) Number of Repetitions
#define AT91C_US_FILTER       (0x1 << 28) // (USART) Receive Line Filter
// -------- US_IER : (USART Offset: 0x8) Debug Unit Interrupt Enable Register --------
#define AT91C_US_RXBRK        (0x1 <<  2) // (USART) Break Received/End of Break
#define AT91C_US_TIMEOUT      (0x1 <<  8) // (USART) Receiver Time-out
#define AT91C_US_ITERATION    (0x1 << 10) // (USART) Max number of Repetitions Reached
#define AT91C_US_NACK         (0x1 << 13) // (USART) Non Acknowledge
#define AT91C_US_RIIC         (0x1 << 16) // (USART) Ring INdicator Input Change Flag
#define AT91C_US_DSRIC        (0x1 << 17) // (USART) Data Set Ready Input Change Flag
#define AT91C_US_DCDIC        (0x1 << 18) // (USART) Data Carrier Flag
#define AT91C_US_CTSIC        (0x1 << 19) // (USART) Clear To Send Input Change Flag
// -------- US_IDR : (USART Offset: 0xc) Debug Unit Interrupt Disable Register --------
// -------- US_IMR : (USART Offset: 0x10) Debug Unit Interrupt Mask Register --------
// -------- US_CSR : (USART Offset: 0x14) Debug Unit Channel Status Register --------
#define AT91C_US_RI           (0x1 << 20) // (USART) Image of RI Input
#define AT91C_US_DSR          (0x1 << 21) // (USART) Image of DSR Input
#define AT91C_US_DCD          (0x1 << 22) // (USART) Image of DCD Input
#define AT91C_US_CTS          (0x1 << 23) // (USART) Image of CTS Input

/* TWI */
#define TWI_CR          (  	0x00000000) // (TWI_CR) Control Register
#define TWI_MMR         (  	0x00000004) // (TWI_MMR) Master Mode Register
#define TWI_IADR        (  	0x0000000C) // (TWI_IADR) Internal Address Register
#define TWI_CWGR        (  	0x00000010) // (TWI_CWGR) Clock Waveform Generator Register
#define TWI_SR          (  	0x00000020) // (TWI_SR) Status Register
#define TWI_IER         (  	0x00000024) // (TWI_IER) Interrupt Enable Register
#define TWI_IDR         (  	0x00000028) // (TWI_IDR) Interrupt Disable Register
#define TWI_IMR         (  	0x0000002C) // (TWI_IMR) Interrupt Mask Register
#define TWI_RHR         (  	0x00000030) // (TWI_RHR) Receive Holding Register
#define TWI_THR         (  	0x00000034) // (TWI_THR) Transmit Holding Register

// -------- TWI_CR : (TWI Offset: 0x0) TWI Control Register --------
#define AT91C_TWI_START       (0x1 <<  0) // (TWI) Send a START Condition
#define AT91C_TWI_STOP        (0x1 <<  1) // (TWI) Send a STOP Condition
#define AT91C_TWI_MSEN        (0x1 <<  2) // (TWI) TWI Master Transfer Enabled
#define AT91C_TWI_MSDIS       (0x1 <<  3) // (TWI) TWI Master Transfer Disabled
#define AT91C_TWI_SWRST       (0x1 <<  7) // (TWI) Software Reset
// -------- TWI_MMR : (TWI Offset: 0x4) TWI Master Mode Register --------
#define AT91C_TWI_IADRSZ      (0x3 <<  8) // (TWI) Internal Device Address Size
#define 	AT91C_TWI_IADRSZ_NO                   (0x0 <<  8) // (TWI) No internal device address
#define 	AT91C_TWI_IADRSZ_1_BYTE               (0x1 <<  8) // (TWI) One-byte internal device address
#define 	AT91C_TWI_IADRSZ_2_BYTE               (0x2 <<  8) // (TWI) Two-byte internal device address
#define 	AT91C_TWI_IADRSZ_3_BYTE               (0x3 <<  8) // (TWI) Three-byte internal device address
#define AT91C_TWI_MREAD       (0x1 << 12) // (TWI) Master Read Direction
#define AT91C_TWI_DADR        (0x7F << 16) // (TWI) Device Address
// -------- TWI_CWGR : (TWI Offset: 0x10) TWI Clock Waveform Generator Register --------
#define AT91C_TWI_CLDIV       (0xFF <<  0) // (TWI) Clock Low Divider
#define AT91C_TWI_CHDIV       (0xFF <<  8) // (TWI) Clock High Divider
#define AT91C_TWI_CKDIV       (0x7 << 16) // (TWI) Clock Divider
// -------- TWI_SR : (TWI Offset: 0x20) TWI Status Register --------
#define AT91C_TWI_TXCOMP      (0x1 <<  0) // (TWI) Transmission Completed
#define AT91C_TWI_RXRDY       (0x1 <<  1) // (TWI) Receive holding register ReaDY
#define AT91C_TWI_TXRDY       (0x1 <<  2) // (TWI) Transmit holding register ReaDY
#define AT91C_TWI_OVRE        (0x1 <<  6) // (TWI) Overrun Error
#define AT91C_TWI_UNRE        (0x1 <<  7) // (TWI) Underrun Error
#define AT91C_TWI_NACK        (0x1 <<  8) // (TWI) Not Acknowledged
#define AT91C_TWI_ENDRX       (0x1 << 12) // (TWI)
#define AT91C_TWI_ENDTX       (0x1 << 13) // (TWI)
#define AT91C_TWI_RXBUFF      (0x1 << 14) // (TWI)
#define AT91C_TWI_TXBUFE      (0x1 << 15) // (TWI)
// -------- TWI_IER : (TWI Offset: 0x24) TWI Interrupt Enable Register --------
// -------- TWI_IDR : (TWI Offset: 0x28) TWI Interrupt Disable Register --------
// -------- TWI_IMR : (TWI Offset: 0x2c) TWI Interrupt Mask Register --------

/* TC */
#define TC_CCR          (  	0x00000000) // (TC_CCR) Channel Control Register
#define TC_CMR          (  	0x00000004) // (TC_CMR) Channel Mode Register (Capture Mode / Waveform Mode)
#define TC_CV           (  	0x00000010) // (TC_CV) Counter Value
#define TC_RA           (  	0x00000014) // (TC_RA) Register A
#define TC_RB           (  	0x00000018) // (TC_RB) Register B
#define TC_RC           (  	0x0000001C) // (TC_RC) Register C
#define TC_SR           (  	0x00000020) // (TC_SR) Status Register
#define TC_IER          (  	0x00000024) // (TC_IER) Interrupt Enable Register
#define TC_IDR          (  	0x00000028) // (TC_IDR) Interrupt Disable Register
#define TC_IMR          (  	0x0000002C) // (TC_IMR) Interrupt Mask Register

// -------- TC_CCR : (TC Offset: 0x0) TC Channel Control Register --------
#define AT91C_TC_CLKEN        (0x1 <<  0) // (TC) Counter Clock Enable Command
#define AT91C_TC_CLKDIS       (0x1 <<  1) // (TC) Counter Clock Disable Command
#define AT91C_TC_SWTRG        (0x1 <<  2) // (TC) Software Trigger Command
// -------- TC_CMR : (TC Offset: 0x4) TC Channel Mode Register: Capture Mode / Waveform Mode --------
#define AT91C_TC_CLKS         (0x7 <<  0) // (TC) Clock Selection
#define 	AT91C_TC_CLKS_TIMER_DIV1_CLOCK     (0x0) // (TC) Clock selected: TIMER_DIV1_CLOCK
#define 	AT91C_TC_CLKS_TIMER_DIV2_CLOCK     (0x1) // (TC) Clock selected: TIMER_DIV2_CLOCK
#define 	AT91C_TC_CLKS_TIMER_DIV3_CLOCK     (0x2) // (TC) Clock selected: TIMER_DIV3_CLOCK
#define 	AT91C_TC_CLKS_TIMER_DIV4_CLOCK     (0x3) // (TC) Clock selected: TIMER_DIV4_CLOCK
#define 	AT91C_TC_CLKS_TIMER_DIV5_CLOCK     (0x4) // (TC) Clock selected: TIMER_DIV5_CLOCK
#define 	AT91C_TC_CLKS_XC0                  (0x5) // (TC) Clock selected: XC0
#define 	AT91C_TC_CLKS_XC1                  (0x6) // (TC) Clock selected: XC1
#define 	AT91C_TC_CLKS_XC2                  (0x7) // (TC) Clock selected: XC2
#define AT91C_TC_CLKI         (0x1 <<  3) // (TC) Clock Invert
#define AT91C_TC_BURST        (0x3 <<  4) // (TC) Burst Signal Selection
#define 	AT91C_TC_BURST_NONE                 (0x0 <<  4) // (TC) The clock is not gated by an external signal
#define 	AT91C_TC_BURST_XC0                  (0x1 <<  4) // (TC) XC0 is ANDed with the selected clock
#define 	AT91C_TC_BURST_XC1                  (0x2 <<  4) // (TC) XC1 is ANDed with the selected clock
#define 	AT91C_TC_BURST_XC2                  (0x3 <<  4) // (TC) XC2 is ANDed with the selected clock
#define AT91C_TC_CPCSTOP      (0x1 <<  6) // (TC) Counter Clock Stopped with RC Compare
#define AT91C_TC_LDBSTOP      (0x1 <<  6) // (TC) Counter Clock Stopped with RB Loading
#define AT91C_TC_CPCDIS       (0x1 <<  7) // (TC) Counter Clock Disable with RC Compare
#define AT91C_TC_LDBDIS       (0x1 <<  7) // (TC) Counter Clock Disabled with RB Loading
#define AT91C_TC_ETRGEDG      (0x3 <<  8) // (TC) External Trigger Edge Selection
#define 	AT91C_TC_ETRGEDG_NONE                 (0x0 <<  8) // (TC) Edge: None
#define 	AT91C_TC_ETRGEDG_RISING               (0x1 <<  8) // (TC) Edge: rising edge
#define 	AT91C_TC_ETRGEDG_FALLING              (0x2 <<  8) // (TC) Edge: falling edge
#define 	AT91C_TC_ETRGEDG_BOTH                 (0x3 <<  8) // (TC) Edge: each edge
#define AT91C_TC_EEVTEDG      (0x3 <<  8) // (TC) External Event Edge Selection
#define 	AT91C_TC_EEVTEDG_NONE                 (0x0 <<  8) // (TC) Edge: None
#define 	AT91C_TC_EEVTEDG_RISING               (0x1 <<  8) // (TC) Edge: rising edge
#define 	AT91C_TC_EEVTEDG_FALLING              (0x2 <<  8) // (TC) Edge: falling edge
#define 	AT91C_TC_EEVTEDG_BOTH                 (0x3 <<  8) // (TC) Edge: each edge
#define AT91C_TC_EEVT         (0x3 << 10) // (TC) External Event  Selection
#define 	AT91C_TC_EEVT_TIOB                 (0x0 << 10) // (TC) Signal selected as external event: TIOB TIOB direction: input
#define 	AT91C_TC_EEVT_XC0                  (0x1 << 10) // (TC) Signal selected as external event: XC0 TIOB direction: output
#define 	AT91C_TC_EEVT_XC1                  (0x2 << 10) // (TC) Signal selected as external event: XC1 TIOB direction: output
#define 	AT91C_TC_EEVT_XC2                  (0x3 << 10) // (TC) Signal selected as external event: XC2 TIOB direction: output
#define AT91C_TC_ABETRG       (0x1 << 10) // (TC) TIOA or TIOB External Trigger Selection
#define AT91C_TC_ENETRG       (0x1 << 12) // (TC) External Event Trigger enable
#define AT91C_TC_WAVESEL      (0x3 << 13) // (TC) Waveform  Selection
#define 	AT91C_TC_WAVESEL_UP                   (0x0 << 13) // (TC) UP mode without atomatic trigger on RC Compare
#define 	AT91C_TC_WAVESEL_UPDOWN               (0x1 << 13) // (TC) UPDOWN mode without automatic trigger on RC Compare
#define 	AT91C_TC_WAVESEL_UP_AUTO              (0x2 << 13) // (TC) UP mode with automatic trigger on RC Compare
#define 	AT91C_TC_WAVESEL_UPDOWN_AUTO          (0x3 << 13) // (TC) UPDOWN mode with automatic trigger on RC Compare
#define AT91C_TC_CPCTRG       (0x1 << 14) // (TC) RC Compare Trigger Enable
#define AT91C_TC_WAVE         (0x1 << 15) // (TC)
#define AT91C_TC_ACPA         (0x3 << 16) // (TC) RA Compare Effect on TIOA
#define 	AT91C_TC_ACPA_NONE                 (0x0 << 16) // (TC) Effect: none
#define 	AT91C_TC_ACPA_SET                  (0x1 << 16) // (TC) Effect: set
#define 	AT91C_TC_ACPA_CLEAR                (0x2 << 16) // (TC) Effect: clear
#define 	AT91C_TC_ACPA_TOGGLE               (0x3 << 16) // (TC) Effect: toggle
#define AT91C_TC_LDRA         (0x3 << 16) // (TC) RA Loading Selection
#define 	AT91C_TC_LDRA_NONE                 (0x0 << 16) // (TC) Edge: None
#define 	AT91C_TC_LDRA_RISING               (0x1 << 16) // (TC) Edge: rising edge of TIOA
#define 	AT91C_TC_LDRA_FALLING              (0x2 << 16) // (TC) Edge: falling edge of TIOA
#define 	AT91C_TC_LDRA_BOTH                 (0x3 << 16) // (TC) Edge: each edge of TIOA
#define AT91C_TC_ACPC         (0x3 << 18) // (TC) RC Compare Effect on TIOA
#define 	AT91C_TC_ACPC_NONE                 (0x0 << 18) // (TC) Effect: none
#define 	AT91C_TC_ACPC_SET                  (0x1 << 18) // (TC) Effect: set
#define 	AT91C_TC_ACPC_CLEAR                (0x2 << 18) // (TC) Effect: clear
#define 	AT91C_TC_ACPC_TOGGLE               (0x3 << 18) // (TC) Effect: toggle
#define AT91C_TC_LDRB         (0x3 << 18) // (TC) RB Loading Selection
#define 	AT91C_TC_LDRB_NONE                 (0x0 << 18) // (TC) Edge: None
#define 	AT91C_TC_LDRB_RISING               (0x1 << 18) // (TC) Edge: rising edge of TIOA
#define 	AT91C_TC_LDRB_FALLING              (0x2 << 18) // (TC) Edge: falling edge of TIOA
#define 	AT91C_TC_LDRB_BOTH                 (0x3 << 18) // (TC) Edge: each edge of TIOA
#define AT91C_TC_AEEVT        (0x3 << 20) // (TC) External Event Effect on TIOA
#define 	AT91C_TC_AEEVT_NONE                 (0x0 << 20) // (TC) Effect: none
#define 	AT91C_TC_AEEVT_SET                  (0x1 << 20) // (TC) Effect: set
#define 	AT91C_TC_AEEVT_CLEAR                (0x2 << 20) // (TC) Effect: clear
#define 	AT91C_TC_AEEVT_TOGGLE               (0x3 << 20) // (TC) Effect: toggle
#define AT91C_TC_ASWTRG       (0x3 << 22) // (TC) Software Trigger Effect on TIOA
#define 	AT91C_TC_ASWTRG_NONE                 (0x0 << 22) // (TC) Effect: none
#define 	AT91C_TC_ASWTRG_SET                  (0x1 << 22) // (TC) Effect: set
#define 	AT91C_TC_ASWTRG_CLEAR                (0x2 << 22) // (TC) Effect: clear
#define 	AT91C_TC_ASWTRG_TOGGLE               (0x3 << 22) // (TC) Effect: toggle
#define AT91C_TC_BCPB         (0x3 << 24) // (TC) RB Compare Effect on TIOB
#define 	AT91C_TC_BCPB_NONE                 (0x0 << 24) // (TC) Effect: none
#define 	AT91C_TC_BCPB_SET                  (0x1 << 24) // (TC) Effect: set
#define 	AT91C_TC_BCPB_CLEAR                (0x2 << 24) // (TC) Effect: clear
#define 	AT91C_TC_BCPB_TOGGLE               (0x3 << 24) // (TC) Effect: toggle
#define AT91C_TC_BCPC         (0x3 << 26) // (TC) RC Compare Effect on TIOB
#define 	AT91C_TC_BCPC_NONE                 (0x0 << 26) // (TC) Effect: none
#define 	AT91C_TC_BCPC_SET                  (0x1 << 26) // (TC) Effect: set
#define 	AT91C_TC_BCPC_CLEAR                (0x2 << 26) // (TC) Effect: clear
#define 	AT91C_TC_BCPC_TOGGLE               (0x3 << 26) // (TC) Effect: toggle
#define AT91C_TC_BEEVT        (0x3 << 28) // (TC) External Event Effect on TIOB
#define 	AT91C_TC_BEEVT_NONE                 (0x0 << 28) // (TC) Effect: none
#define 	AT91C_TC_BEEVT_SET                  (0x1 << 28) // (TC) Effect: set
#define 	AT91C_TC_BEEVT_CLEAR                (0x2 << 28) // (TC) Effect: clear
#define 	AT91C_TC_BEEVT_TOGGLE               (0x3 << 28) // (TC) Effect: toggle
#define AT91C_TC_BSWTRG       (0x3 << 30) // (TC) Software Trigger Effect on TIOB
#define 	AT91C_TC_BSWTRG_NONE                 (0x0 << 30) // (TC) Effect: none
#define 	AT91C_TC_BSWTRG_SET                  (0x1 << 30) // (TC) Effect: set
#define 	AT91C_TC_BSWTRG_CLEAR                (0x2 << 30) // (TC) Effect: clear
#define 	AT91C_TC_BSWTRG_TOGGLE               (0x3 << 30) // (TC) Effect: toggle
// -------- TC_SR : (TC Offset: 0x20) TC Channel Status Register --------
#define AT91C_TC_COVFS        (0x1 <<  0) // (TC) Counter Overflow
#define AT91C_TC_LOVRS        (0x1 <<  1) // (TC) Load Overrun
#define AT91C_TC_CPAS         (0x1 <<  2) // (TC) RA Compare
#define AT91C_TC_CPBS         (0x1 <<  3) // (TC) RB Compare
#define AT91C_TC_CPCS         (0x1 <<  4) // (TC) RC Compare
#define AT91C_TC_LDRAS        (0x1 <<  5) // (TC) RA Loading
#define AT91C_TC_LDRBS        (0x1 <<  6) // (TC) RB Loading
#define AT91C_TC_ETRGS        (0x1 <<  7) // (TC) External Trigger
#define AT91C_TC_CLKSTA       (0x1 << 16) // (TC) Clock Enabling
#define AT91C_TC_MTIOA        (0x1 << 17) // (TC) TIOA Mirror
#define AT91C_TC_MTIOB        (0x1 << 18) // (TC) TIOA Mirror
// -------- TC_IER : (TC Offset: 0x24) TC Channel Interrupt Enable Register --------
// -------- TC_IDR : (TC Offset: 0x28) TC Channel Interrupt Disable Register --------
// -------- TC_IMR : (TC Offset: 0x2c) TC Channel Interrupt Mask Register --------

/* TCB */
#define TCB_BCR         (  	0x000000C0) // (TCB_BCR) TC Block Control Register
#define TCB_BMR         (  	0x000000C4) // (TCB_BMR) TC Block Mode Register

// -------- TCB_BCR : (TCB Offset: 0xc0) TC Block Control Register --------
#define AT91C_TCB_SYNC        (0x1 <<  0) // (TCB) Synchro Command
// -------- TCB_BMR : (TCB Offset: 0xc4) TC Block Mode Register --------
#define AT91C_TCB_TC0XC0S     (0x3 <<  0) // (TCB) External Clock Signal 0 Selection
#define 	AT91C_TCB_TC0XC0S_TCLK0                (0x0) // (TCB) TCLK0 connected to XC0
#define 	AT91C_TCB_TC0XC0S_NONE                 (0x1) // (TCB) None signal connected to XC0
#define 	AT91C_TCB_TC0XC0S_TIOA1                (0x2) // (TCB) TIOA1 connected to XC0
#define 	AT91C_TCB_TC0XC0S_TIOA2                (0x3) // (TCB) TIOA2 connected to XC0
#define AT91C_TCB_TC1XC1S     (0x3 <<  2) // (TCB) External Clock Signal 1 Selection
#define 	AT91C_TCB_TC1XC1S_TCLK1                (0x0 <<  2) // (TCB) TCLK1 connected to XC1
#define 	AT91C_TCB_TC1XC1S_NONE                 (0x1 <<  2) // (TCB) None signal connected to XC1
#define 	AT91C_TCB_TC1XC1S_TIOA0                (0x2 <<  2) // (TCB) TIOA0 connected to XC1
#define 	AT91C_TCB_TC1XC1S_TIOA2                (0x3 <<  2) // (TCB) TIOA2 connected to XC1
#define AT91C_TCB_TC2XC2S     (0x3 <<  4) // (TCB) External Clock Signal 2 Selection
#define 	AT91C_TCB_TC2XC2S_TCLK2                (0x0 <<  4) // (TCB) TCLK2 connected to XC2
#define 	AT91C_TCB_TC2XC2S_NONE                 (0x1 <<  4) // (TCB) None signal connected to XC2
#define 	AT91C_TCB_TC2XC2S_TIOA0                (0x2 <<  4) // (TCB) TIOA0 connected to XC2
#define 	AT91C_TCB_TC2XC2S_TIOA1                (0x3 <<  4) // (TCB) TIOA2 connected to XC2

/* PWM Channel */
#define PWMC_CMR        (  	0x00000000) // (PWMC_CMR) Channel Mode Register
#define PWMC_CDTYR      (  	0x00000004) // (PWMC_CDTYR) Channel Duty Cycle Register
#define PWMC_CPRDR      (  	0x00000008) // (PWMC_CPRDR) Channel Period Register
#define PWMC_CCNTR      (  	0x0000000C) // (PWMC_CCNTR) Channel Counter Register
#define PWMC_CUPDR      (  	0x00000010) // (PWMC_CUPDR) Channel Update Register
#define Reserved        (  	0x00000014) // (Reserved) Reserved

// -------- PWMC_CMR : (PWMC_CH Offset: 0x0) PWMC Channel Mode Register --------
#define AT91C_PWMC_CPRE       (0xF <<  0) // (PWMC_CH) Channel Pre-scaler : PWMC_CLKx
#define 	AT91C_PWMC_CPRE_MCK                  (0x0) // (PWMC_CH)
#define 	AT91C_PWMC_CPRE_MCKA                 (0xB) // (PWMC_CH)
#define 	AT91C_PWMC_CPRE_MCKB                 (0xC) // (PWMC_CH)
#define AT91C_PWMC_CALG       (0x1 <<  8) // (PWMC_CH) Channel Alignment
#define AT91C_PWMC_CPOL       (0x1 <<  9) // (PWMC_CH) Channel Polarity
#define AT91C_PWMC_CPD        (0x1 << 10) // (PWMC_CH) Channel Update Period
// -------- PWMC_CDTYR : (PWMC_CH Offset: 0x4) PWMC Channel Duty Cycle Register --------
#define AT91C_PWMC_CDTY       (0x0 <<  0) // (PWMC_CH) Channel Duty Cycle
// -------- PWMC_CPRDR : (PWMC_CH Offset: 0x8) PWMC Channel Period Register --------
#define AT91C_PWMC_CPRD       (0x0 <<  0) // (PWMC_CH) Channel Period
// -------- PWMC_CCNTR : (PWMC_CH Offset: 0xc) PWMC Channel Counter Register --------
#define AT91C_PWMC_CCNT       (0x0 <<  0) // (PWMC_CH) Channel Counter
// -------- PWMC_CUPDR : (PWMC_CH Offset: 0x10) PWMC Channel Update Register --------
#define AT91C_PWMC_CUPD       (0x0 <<  0) // (PWMC_CH) Channel Update

/* PWM */
#define PWMC_MR         (  	0x00000000) // (PWMC_MR) PWMC Mode Register
#define PWMC_ENA        (  	0x00000004) // (PWMC_ENA) PWMC Enable Register
#define PWMC_DIS        (  	0x00000008) // (PWMC_DIS) PWMC Disable Register
#define PWMC_SR         (  	0x0000000C) // (PWMC_SR) PWMC Status Register
#define PWMC_IER        (  	0x00000010) // (PWMC_IER) PWMC Interrupt Enable Register
#define PWMC_IDR        (  	0x00000014) // (PWMC_IDR) PWMC Interrupt Disable Register
#define PWMC_IMR        (  	0x00000018) // (PWMC_IMR) PWMC Interrupt Mask Register
#define PWMC_ISR        (  	0x0000001C) // (PWMC_ISR) PWMC Interrupt Status Register
#define PWMC_VR         (  	0x000000FC) // (PWMC_VR) PWMC Version Register

// -------- PWMC_MR : (PWMC Offset: 0x0) PWMC Mode Register --------
#define AT91C_PWMC_DIVA       (0xFF <<  0) // (PWMC) CLKA divide factor.
#define AT91C_PWMC_PREA       (0xF <<  8) // (PWMC) Divider Input Clock Prescaler A
#define 	AT91C_PWMC_PREA_MCK                  (0x0 <<  8) // (PWMC)
#define AT91C_PWMC_DIVB       (0xFF << 16) // (PWMC) CLKB divide factor.
#define AT91C_PWMC_PREB       (0xF << 24) // (PWMC) Divider Input Clock Prescaler B
#define 	AT91C_PWMC_PREB_MCK                  (0x0 << 24) // (PWMC)
// -------- PWMC_ENA : (PWMC Offset: 0x4) PWMC Enable Register --------
#define AT91C_PWMC_CHID0      (0x1 <<  0) // (PWMC) Channel ID 0
#define AT91C_PWMC_CHID1      (0x1 <<  1) // (PWMC) Channel ID 1
#define AT91C_PWMC_CHID2      (0x1 <<  2) // (PWMC) Channel ID 2
#define AT91C_PWMC_CHID3      (0x1 <<  3) // (PWMC) Channel ID 3
// -------- PWMC_DIS : (PWMC Offset: 0x8) PWMC Disable Register --------
// -------- PWMC_SR : (PWMC Offset: 0xc) PWMC Status Register --------
// -------- PWMC_IER : (PWMC Offset: 0x10) PWMC Interrupt Enable Register --------
// -------- PWMC_IDR : (PWMC Offset: 0x14) PWMC Interrupt Disable Register --------
// -------- PWMC_IMR : (PWMC Offset: 0x18) PWMC Interrupt Mask Register --------
// -------- PWMC_ISR : (PWMC Offset: 0x1c) PWMC Interrupt Status Register --------

/* UDP */
#define UDP_FRM_NUM     (  	0x00000000) // (UDP_FRM_NUM) Frame Number Register
#define UDP_GLBSTATE    (  	0x00000004) // (UDP_GLBSTATE) Global State Register
#define UDP_FADDR       (  	0x00000008) // (UDP_FADDR) Function Address Register
#define UDP_IER         (  	0x00000010) // (UDP_IER) Interrupt Enable Register
#define UDP_IDR         (  	0x00000014) // (UDP_IDR) Interrupt Disable Register
#define UDP_IMR         (  	0x00000018) // (UDP_IMR) Interrupt Mask Register
#define UDP_ISR         (  	0x0000001C) // (UDP_ISR) Interrupt Status Register
#define UDP_ICR         (  	0x00000020) // (UDP_ICR) Interrupt Clear Register
#define UDP_RSTEP       (  	0x00000028) // (UDP_RSTEP) Reset Endpoint Register
#define UDP_CSR         (  	0x00000030) // (UDP_CSR) Endpoint Control and Status Register
#define UDP_FDR         (  	0x00000050) // (UDP_FDR) Endpoint FIFO Data Register
#define UDP_TXVC        (  	0x00000074) // (UDP_TXVC) Transceiver Control Register

// -------- UDP_FRM_NUM : (UDP Offset: 0x0) USB Frame Number Register --------
#define AT91C_UDP_FRM_NUM     (0x7FF <<  0) // (UDP) Frame Number as Defined in the Packet Field Formats
#define AT91C_UDP_FRM_ERR     (0x1 << 16) // (UDP) Frame Error
#define AT91C_UDP_FRM_OK      (0x1 << 17) // (UDP) Frame OK
// -------- UDP_GLB_STATE : (UDP Offset: 0x4) USB Global State Register --------
#define AT91C_UDP_FADDEN      (0x1 <<  0) // (UDP) Function Address Enable
#define AT91C_UDP_CONFG       (0x1 <<  1) // (UDP) Configured
#define AT91C_UDP_ESR         (0x1 <<  2) // (UDP) Enable Send Resume
#define AT91C_UDP_RSMINPR     (0x1 <<  3) // (UDP) A Resume Has Been Sent to the Host
#define AT91C_UDP_RMWUPE      (0x1 <<  4) // (UDP) Remote Wake Up Enable
// -------- UDP_FADDR : (UDP Offset: 0x8) USB Function Address Register --------
#define AT91C_UDP_FADD        (0xFF <<  0) // (UDP) Function Address Value
#define AT91C_UDP_FEN         (0x1 <<  8) // (UDP) Function Enable
// -------- UDP_IER : (UDP Offset: 0x10) USB Interrupt Enable Register --------
#define AT91C_UDP_EPINT0      (0x1 <<  0) // (UDP) Endpoint 0 Interrupt
#define AT91C_UDP_EPINT1      (0x1 <<  1) // (UDP) Endpoint 0 Interrupt
#define AT91C_UDP_EPINT2      (0x1 <<  2) // (UDP) Endpoint 2 Interrupt
#define AT91C_UDP_EPINT3      (0x1 <<  3) // (UDP) Endpoint 3 Interrupt
#define AT91C_UDP_RXSUSP      (0x1 <<  8) // (UDP) USB Suspend Interrupt
#define AT91C_UDP_RXRSM       (0x1 <<  9) // (UDP) USB Resume Interrupt
#define AT91C_UDP_EXTRSM      (0x1 << 10) // (UDP) USB External Resume Interrupt
#define AT91C_UDP_SOFINT      (0x1 << 11) // (UDP) USB Start Of frame Interrupt
#define AT91C_UDP_WAKEUP      (0x1 << 13) // (UDP) USB Resume Interrupt
// -------- UDP_IDR : (UDP Offset: 0x14) USB Interrupt Disable Register --------
// -------- UDP_IMR : (UDP Offset: 0x18) USB Interrupt Mask Register --------
// -------- UDP_ISR : (UDP Offset: 0x1c) USB Interrupt Status Register --------
#define AT91C_UDP_ENDBUSRES   (0x1 << 12) // (UDP) USB End Of Bus Reset Interrupt
// -------- UDP_ICR : (UDP Offset: 0x20) USB Interrupt Clear Register --------
// -------- UDP_RST_EP : (UDP Offset: 0x28) USB Reset Endpoint Register --------
#define AT91C_UDP_EP0         (0x1 <<  0) // (UDP) Reset Endpoint 0
#define AT91C_UDP_EP1         (0x1 <<  1) // (UDP) Reset Endpoint 1
#define AT91C_UDP_EP2         (0x1 <<  2) // (UDP) Reset Endpoint 2
#define AT91C_UDP_EP3         (0x1 <<  3) // (UDP) Reset Endpoint 3
// -------- UDP_CSR : (UDP Offset: 0x30) USB Endpoint Control and Status Register --------
#define AT91C_UDP_TXCOMP      (0x1 <<  0) // (UDP) Generates an IN packet with data previously written in the DPR
#define AT91C_UDP_RX_DATA_BK0 (0x1 <<  1) // (UDP) Receive Data Bank 0
#define AT91C_UDP_RXSETUP     (0x1 <<  2) // (UDP) Sends STALL to the Host (Control endpoints)
#define AT91C_UDP_ISOERROR    (0x1 <<  3) // (UDP) Isochronous error (Isochronous endpoints)
#define AT91C_UDP_STALLSENT   (0x1 <<  3) // (UDP) Stall sent (Control, bulk, interrupt endpoints)
#define AT91C_UDP_TXPKTRDY    (0x1 <<  4) // (UDP) Transmit Packet Ready
#define AT91C_UDP_FORCESTALL  (0x1 <<  5) // (UDP) Force Stall (used by Control, Bulk and Isochronous endpoints).
#define AT91C_UDP_RX_DATA_BK1 (0x1 <<  6) // (UDP) Receive Data Bank 1 (only used by endpoints with ping-pong attributes).
#define AT91C_UDP_DIR         (0x1 <<  7) // (UDP) Transfer Direction
#define AT91C_UDP_EPTYPE      (0x7 <<  8) // (UDP) Endpoint type
#define 	AT91C_UDP_EPTYPE_CTRL                 (0x0 <<  8) // (UDP) Control
#define 	AT91C_UDP_EPTYPE_ISO_OUT              (0x1 <<  8) // (UDP) Isochronous OUT
#define 	AT91C_UDP_EPTYPE_BULK_OUT             (0x2 <<  8) // (UDP) Bulk OUT
#define 	AT91C_UDP_EPTYPE_INT_OUT              (0x3 <<  8) // (UDP) Interrupt OUT
#define 	AT91C_UDP_EPTYPE_ISO_IN               (0x5 <<  8) // (UDP) Isochronous IN
#define 	AT91C_UDP_EPTYPE_BULK_IN              (0x6 <<  8) // (UDP) Bulk IN
#define 	AT91C_UDP_EPTYPE_INT_IN               (0x7 <<  8) // (UDP) Interrupt IN
#define AT91C_UDP_DTGLE       (0x1 << 11) // (UDP) Data Toggle
#define AT91C_UDP_EPEDS       (0x1 << 15) // (UDP) Endpoint Enable Disable
#define AT91C_UDP_RXBYTECNT   (0x7FF << 16) // (UDP) Number Of Bytes Available in the FIFO
// -------- UDP_TXVC : (UDP Offset: 0x74) Transceiver Control Register --------
#define AT91C_UDP_TXVDIS      (0x1 <<  8) // (UDP)

// *****************************************************************************
//               PIO DEFINITIONS FOR AT91SAM7S256
// *****************************************************************************
#define AT91C_PIO_PA0        (1 <<  0) // Pin Controlled by PA0
#define AT91C_PA0_PWM0     (AT91C_PIO_PA0) //  PWM Channel 0
#define AT91C_PA0_TIOA0    (AT91C_PIO_PA0) //  Timer Counter 0 Multipurpose Timer I/O Pin A
#define AT91C_PIO_PA1        (1 <<  1) // Pin Controlled by PA1
#define AT91C_PA1_PWM1     (AT91C_PIO_PA1) //  PWM Channel 1
#define AT91C_PA1_TIOB0    (AT91C_PIO_PA1) //  Timer Counter 0 Multipurpose Timer I/O Pin B
#define AT91C_PIO_PA10       (1 << 10) // Pin Controlled by PA10
#define AT91C_PA10_DTXD     (AT91C_PIO_PA10) //  DBGU Debug Transmit Data
#define AT91C_PA10_NPCS2    (AT91C_PIO_PA10) //  SPI Peripheral Chip Select 2
#define AT91C_PIO_PA11       (1 << 11) // Pin Controlled by PA11
#define AT91C_PA11_NPCS0    (AT91C_PIO_PA11) //  SPI Peripheral Chip Select 0
#define AT91C_PA11_PWM0     (AT91C_PIO_PA11) //  PWM Channel 0
#define AT91C_PIO_PA12       (1 << 12) // Pin Controlled by PA12
#define AT91C_PA12_MISO     (AT91C_PIO_PA12) //  SPI Master In Slave
#define AT91C_PA12_PWM1     (AT91C_PIO_PA12) //  PWM Channel 1
#define AT91C_PIO_PA13       (1 << 13) // Pin Controlled by PA13
#define AT91C_PA13_MOSI     (AT91C_PIO_PA13) //  SPI Master Out Slave
#define AT91C_PA13_PWM2     (AT91C_PIO_PA13) //  PWM Channel 2
#define AT91C_PIO_PA14       (1 << 14) // Pin Controlled by PA14
#define AT91C_PA14_SPCK     (AT91C_PIO_PA14) //  SPI Serial Clock
#define AT91C_PA14_PWM3     (AT91C_PIO_PA14) //  PWM Channel 3
#define AT91C_PIO_PA15       (1 << 15) // Pin Controlled by PA15
#define AT91C_PA15_TF       (AT91C_PIO_PA15) //  SSC Transmit Frame Sync
#define AT91C_PA15_TIOA1    (AT91C_PIO_PA15) //  Timer Counter 1 Multipurpose Timer I/O Pin A
#define AT91C_PIO_PA16       (1 << 16) // Pin Controlled by PA16
#define AT91C_PA16_TK       (AT91C_PIO_PA16) //  SSC Transmit Clock
#define AT91C_PA16_TIOB1    (AT91C_PIO_PA16) //  Timer Counter 1 Multipurpose Timer I/O Pin B
#define AT91C_PIO_PA17       (1 << 17) // Pin Controlled by PA17
#define AT91C_PA17_TD       (AT91C_PIO_PA17) //  SSC Transmit data
#define AT91C_PA17_PCK1     (AT91C_PIO_PA17) //  PMC Programmable Clock Output 1
#define AT91C_PIO_PA18       (1 << 18) // Pin Controlled by PA18
#define AT91C_PA18_RD       (AT91C_PIO_PA18) //  SSC Receive Data
#define AT91C_PA18_PCK2     (AT91C_PIO_PA18) //  PMC Programmable Clock Output 2
#define AT91C_PIO_PA19       (1 << 19) // Pin Controlled by PA19
#define AT91C_PA19_RK       (AT91C_PIO_PA19) //  SSC Receive Clock
#define AT91C_PA19_FIQ      (AT91C_PIO_PA19) //  AIC Fast Interrupt Input
#define AT91C_PIO_PA2        (1 <<  2) // Pin Controlled by PA2
#define AT91C_PA2_PWM2     (AT91C_PIO_PA2) //  PWM Channel 2
#define AT91C_PA2_SCK0     (AT91C_PIO_PA2) //  USART 0 Serial Clock
#define AT91C_PIO_PA20       (1 << 20) // Pin Controlled by PA20
#define AT91C_PA20_RF       (AT91C_PIO_PA20) //  SSC Receive Frame Sync
#define AT91C_PA20_IRQ0     (AT91C_PIO_PA20) //  External Interrupt 0
#define AT91C_PIO_PA21       (1 << 21) // Pin Controlled by PA21
#define AT91C_PA21_RXD1     (AT91C_PIO_PA21) //  USART 1 Receive Data
#define AT91C_PA21_PCK1     (AT91C_PIO_PA21) //  PMC Programmable Clock Output 1
#define AT91C_PIO_PA22       (1 << 22) // Pin Controlled by PA22
#define AT91C_PA22_TXD1     (AT91C_PIO_PA22) //  USART 1 Transmit Data
#define AT91C_PA22_NPCS3    (AT91C_PIO_PA22) //  SPI Peripheral Chip Select 3
#define AT91C_PIO_PA23       (1 << 23) // Pin Controlled by PA23
#define AT91C_PA23_SCK1     (AT91C_PIO_PA23) //  USART 1 Serial Clock
#define AT91C_PA23_PWM0     (AT91C_PIO_PA23) //  PWM Channel 0
#define AT91C_PIO_PA24       (1 << 24) // Pin Controlled by PA24
#define AT91C_PA24_RTS1     (AT91C_PIO_PA24) //  USART 1 Ready To Send
#define AT91C_PA24_PWM1     (AT91C_PIO_PA24) //  PWM Channel 1
#define AT91C_PIO_PA25       (1 << 25) // Pin Controlled by PA25
#define AT91C_PA25_CTS1     (AT91C_PIO_PA25) //  USART 1 Clear To Send
#define AT91C_PA25_PWM2     (AT91C_PIO_PA25) //  PWM Channel 2
#define AT91C_PIO_PA26       (1 << 26) // Pin Controlled by PA26
#define AT91C_PA26_DCD1     (AT91C_PIO_PA26) //  USART 1 Data Carrier Detect
#define AT91C_PA26_TIOA2    (AT91C_PIO_PA26) //  Timer Counter 2 Multipurpose Timer I/O Pin A
#define AT91C_PIO_PA27       (1 << 27) // Pin Controlled by PA27
#define AT91C_PA27_DTR1     (AT91C_PIO_PA27) //  USART 1 Data Terminal ready
#define AT91C_PA27_TIOB2    (AT91C_PIO_PA27) //  Timer Counter 2 Multipurpose Timer I/O Pin B
#define AT91C_PIO_PA28       (1 << 28) // Pin Controlled by PA28
#define AT91C_PA28_DSR1     (AT91C_PIO_PA28) //  USART 1 Data Set ready
#define AT91C_PA28_TCLK1    (AT91C_PIO_PA28) //  Timer Counter 1 external clock input
#define AT91C_PIO_PA29       (1 << 29) // Pin Controlled by PA29
#define AT91C_PA29_RI1      (AT91C_PIO_PA29) //  USART 1 Ring Indicator
#define AT91C_PA29_TCLK2    (AT91C_PIO_PA29) //  Timer Counter 2 external clock input
#define AT91C_PIO_PA3        (1 <<  3) // Pin Controlled by PA3
#define AT91C_PA3_TWD      (AT91C_PIO_PA3) //  TWI Two-wire Serial Data
#define AT91C_PA3_NPCS3    (AT91C_PIO_PA3) //  SPI Peripheral Chip Select 3
#define AT91C_PIO_PA30       (1 << 30) // Pin Controlled by PA30
#define AT91C_PA30_IRQ1     (AT91C_PIO_PA30) //  External Interrupt 1
#define AT91C_PA30_NPCS2    (AT91C_PIO_PA30) //  SPI Peripheral Chip Select 2
#define AT91C_PIO_PA31       (1 << 31) // Pin Controlled by PA31
#define AT91C_PA31_NPCS1    (AT91C_PIO_PA31) //  SPI Peripheral Chip Select 1
#define AT91C_PA31_PCK2     (AT91C_PIO_PA31) //  PMC Programmable Clock Output 2
#define AT91C_PIO_PA4        (1 <<  4) // Pin Controlled by PA4
#define AT91C_PA4_TWCK     (AT91C_PIO_PA4) //  TWI Two-wire Serial Clock
#define AT91C_PA4_TCLK0    (AT91C_PIO_PA4) //  Timer Counter 0 external clock input
#define AT91C_PIO_PA5        (1 <<  5) // Pin Controlled by PA5
#define AT91C_PA5_RXD0     (AT91C_PIO_PA5) //  USART 0 Receive Data
#define AT91C_PA5_NPCS3    (AT91C_PIO_PA5) //  SPI Peripheral Chip Select 3
#define AT91C_PIO_PA6        (1 <<  6) // Pin Controlled by PA6
#define AT91C_PA6_TXD0     (AT91C_PIO_PA6) //  USART 0 Transmit Data
#define AT91C_PA6_PCK0     (AT91C_PIO_PA6) //  PMC Programmable Clock Output 0
#define AT91C_PIO_PA7        (1 <<  7) // Pin Controlled by PA7
#define AT91C_PA7_RTS0     (AT91C_PIO_PA7) //  USART 0 Ready To Send
#define AT91C_PA7_PWM3     (AT91C_PIO_PA7) //  PWM Channel 3
#define AT91C_PIO_PA8        (1 <<  8) // Pin Controlled by PA8
#define AT91C_PA8_CTS0     (AT91C_PIO_PA8) //  USART 0 Clear To Send
#define AT91C_PA8_ADTRG    (AT91C_PIO_PA8) //  ADC External Trigger
#define AT91C_PIO_PA9        (1 <<  9) // Pin Controlled by PA9
#define AT91C_PA9_DRXD     (AT91C_PIO_PA9) //  DBGU Debug Receive Data
#define AT91C_PA9_NPCS1    (AT91C_PIO_PA9) //  SPI Peripheral Chip Select 1

// *****************************************************************************
//               PERIPHERAL ID DEFINITIONS FOR AT91SAM7S256
// *****************************************************************************
#define AT91C_ID_FIQ    ( 0) // Advanced Interrupt Controller (FIQ)
#define AT91C_ID_SYS    ( 1) // System Peripheral
#define AT91C_ID_PIOA   ( 2) // Parallel IO Controller
#define AT91C_ID_3_Reserved ( 3) // Reserved
#define AT91C_ID_ADC    ( 4) // Analog-to-Digital Converter
#define AT91C_ID_SPI    ( 5) // Serial Peripheral Interface
#define AT91C_ID_US0    ( 6) // USART 0
#define AT91C_ID_US1    ( 7) // USART 1
#define AT91C_ID_SSC    ( 8) // Serial Synchronous Controller
#define AT91C_ID_TWI    ( 9) // Two-Wire Interface
#define AT91C_ID_PWMC   (10) // PWM Controller
#define AT91C_ID_UDP    (11) // USB Device Port
#define AT91C_ID_TC0    (12) // Timer Counter 0
#define AT91C_ID_TC1    (13) // Timer Counter 1
#define AT91C_ID_TC2    (14) // Timer Counter 2
#define AT91C_ID_15_Reserved (15) // Reserved
#define AT91C_ID_16_Reserved (16) // Reserved
#define AT91C_ID_17_Reserved (17) // Reserved
#define AT91C_ID_18_Reserved (18) // Reserved
#define AT91C_ID_19_Reserved (19) // Reserved
#define AT91C_ID_20_Reserved (20) // Reserved
#define AT91C_ID_21_Reserved (21) // Reserved
#define AT91C_ID_22_Reserved (22) // Reserved
#define AT91C_ID_23_Reserved (23) // Reserved
#define AT91C_ID_24_Reserved (24) // Reserved
#define AT91C_ID_25_Reserved (25) // Reserved
#define AT91C_ID_26_Reserved (26) // Reserved
#define AT91C_ID_27_Reserved (27) // Reserved
#define AT91C_ID_28_Reserved (28) // Reserved
#define AT91C_ID_29_Reserved (29) // Reserved
#define AT91C_ID_IRQ0   (30) // Advanced Interrupt Controller (IRQ0)
#define AT91C_ID_IRQ1   (31) // Advanced Interrupt Controller (IRQ1)
#define AT91C_ALL_INT   (0xC0007FF7) // ALL VALID INTERRUPTS

// *****************************************************************************
//               BASE ADDRESS DEFINITIONS FOR AT91SAM7S256
// *****************************************************************************
#define AT91C_BASE_SYS		0xFFFFF000 // (SYS) Base Address
#define AT91C_BASE_AIC		0xFFFFF000 // (AIC) Base Address
#define AT91C_BASE_PDC_DBGU	0xFFFFF300 // (PDC_DBGU) Base Address
#define AT91C_BASE_DBGU		0xFFFFF200 // (DBGU) Base Address
#define AT91C_BASE_PIOA		0xFFFFF400 // (PIOA) Base Address
#define AT91C_BASE_CKGR      	0xFFFFFC20 // (CKGR) Base Address
#define AT91C_BASE_PMC		0xFFFFFC00 // (PMC) Base Address
#define AT91C_BASE_RSTC      	0xFFFFFD00 // (RSTC) Base Address
#define AT91C_BASE_RTTC      	0xFFFFFD20 // (RTTC) Base Address
#define AT91C_BASE_PITC      	0xFFFFFD30 // (PITC) Base Address
#define AT91C_BASE_WDTC      	0xFFFFFD40 // (WDTC) Base Address
#define AT91C_BASE_VREG      	0xFFFFFD60 // (VREG) Base Address
#define AT91C_BASE_MC        	0xFFFFFF00 // (MC) Base Address
#define AT91C_BASE_PDC_SPI    	0xFFFE0100 // (PDC_SPI) Base Address
#define AT91C_BASE_SPI        	0xFFFE0000 // (SPI) Base Address
#define AT91C_BASE_PDC_ADC   	0xFFFD8100 // (PDC_ADC) Base Address
#define AT91C_BASE_ADC        	0xFFFD8000 // (ADC) Base Address
#define AT91C_BASE_PDC_SSC   	0xFFFD4100 // (PDC_SSC) Base Address
#define AT91C_BASE_SSC       	0xFFFD4000 // (SSC) Base Address
#define AT91C_BASE_PDC_US1    	0xFFFC4100 // (PDC_US1) Base Address
#define AT91C_BASE_US1        	0xFFFC4000 // (US1) Base Address
#define AT91C_BASE_PDC_US0   	0xFFFC0100 // (PDC_US0) Base Address
#define AT91C_BASE_US0       	0xFFFC0000 // (US0) Base Address
#define AT91C_BASE_TWI       	0xFFFB8000 // (TWI) Base Address
#define AT91C_BASE_TC0       	0xFFFA0000 // (TC0) Base Address
#define AT91C_BASE_TC1       	0xFFFA0040 // (TC1) Base Address
#define AT91C_BASE_TC2        	0xFFFA0080 // (TC2) Base Address
#define AT91C_BASE_TCB        	0xFFFA0000 // (TCB) Base Address
#define AT91C_BASE_PWMC_CH3   	0xFFFCC260 // (PWMC_CH3) Base Address
#define AT91C_BASE_PWMC_CH2   	0xFFFCC240 // (PWMC_CH2) Base Address
#define AT91C_BASE_PWMC_CH1   	0xFFFCC220 // (PWMC_CH1) Base Address
#define AT91C_BASE_PWMC_CH0   	0xFFFCC200 // (PWMC_CH0) Base Address
#define AT91C_BASE_PWMC       	0xFFFCC000 // (PWMC) Base Address
#define AT91C_BASE_UDP        	0xFFFB0000 // (UDP) Base Address

// *****************************************************************************
//               MEMORY MAPPING DEFINITIONS FOR AT91SAM7S256
// *****************************************************************************
// ISRAM
#define AT91C_ISRAM	 (0x00200000) // Internal SRAM base address
#define AT91C_ISRAM_SIZE	 (0x00010000) // Internal SRAM size in byte (64 Kbytes)
// IFLASH
#define AT91C_IFLASH	 (0x00100000) // Internal FLASH base address
#define AT91C_IFLASH_SIZE	 (0x00040000) // Internal FLASH size in byte (256 Kbytes)
#define AT91C_IFLASH_PAGE_SIZE	 (256) // Internal FLASH Page Size: 256 bytes
#define AT91C_IFLASH_LOCK_REGION_SIZE	 (16384) // Internal FLASH Lock Region Size: 16 Kbytes
#define AT91C_IFLASH_NB_OF_PAGES	 (1024) // Internal FLASH Number of Pages: 1024 bytes
#define AT91C_IFLASH_NB_OF_LOCK_BITS	 (16) // Internal FLASH Number of Lock Bits: 16 bytes

#endif /* SAM7S_REG_H */
