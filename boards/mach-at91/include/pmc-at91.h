/*
 * Copyright (C) 2017  Raphaël Poggi <poggi.raph@gmail.com>
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

#ifndef PMC_AT91_H
#define PMC_AT91_H

/* Register offset in AT91C_PMC structure */
#define PMC_SCER	0x00	/*  System Clock Enable Register */
#define PMC_SCDR	0x04	/*  System Clock Disable Register */
#define PMC_SCSR	0x08	/*  System Clock Status Register */
/*  0x0C Reversed */
#define PMC_PCER	0x10	/*  Peripheral Clock Enable Register  (0:31 PERI_ID) */
#define PMC_PCDR	0x14	/*  Peripheral Clock Disable Register (0:31 PERI_ID) */
#define PMC_PCSR	0x18	/*  Peripheral Clock Status Register  (0:31 PERI_ID) */
#define PMC_UCKR	0x1C	/*  UTMI Clock Configuration Register */
#define PMC_MOR		0x20	/*  Main Oscillator Register */
#define PMC_MCFR	0x24	/*  Main Clock  Frequency Register */
#define PMC_PLLAR	0x28	/*  PLL A Register */
#define PMC_PLLBR	0x2C	/*  PLL B Register */
#define PMC_MCKR	0x30	/*  Master Clock Register */
/*  0x34 Reserved */
#define PMC_USB		0x38	/*  USB clock register */
#define PMC_SMD		0x3C	/*  SMD clock register */
#define PMC_PCKR	0x40	/*  Programmable Clock 0 Register */
#define PMC_PCKR1	0x44	/*  Programmable Clock 1 Register */
#define PMC_PCKR2	0x48	/*  Programmable Clock 2 Register */
/*  0x48 ~ 0x5C */
#define PMC_IER		0x60	/*  Interrupt Enable Register */
#define PMC_IDR		0x64	/*  Interrupt Disable Register */
#define PMC_SR		0x68	/*  Status Register */
#define PMC_IMR		0x6C	/*  Interrupt Mask Register */
#define PMC_FSMR	0x70	/*  Fast Startup Mode Register */
#define PMC_FSPR	0x74	/*  Fast Startup Polarity Register */
#define PMC_FOCR	0x78	/*  Fault Output Clear Register */
/*  0x7C Reserved */
#define PMC_PLLICPR	0x80	/*  PLL Charge Pump Current Register */
/*  0x84 ~ 0xE0 Reserved */
#define PMC_WPMR	0xE4	/*  Write Protect Mode Register */
#define PMC_WPSR	0xE8	/*  Write Protect Status Register */
/*  0xEC ~ 0xF8 */
#define PMC_VERSION	0xFC	/*  Version Register */
#define PMC_PCER1	0x100	/*  Peripheral Clock Enable Register1  32:63 PERI_ID */
#define PMC_PCDR1	0x104	/*  Peripheral Clock Disable Register1 32:63 PERI_ID */
#define PMC_PCSR1	0x108	/*  Peripheral Clock Status Register1  32:63 PERI_ID */
#define PMC_PCR		0x10C	/*  Peripheral Control Register */
#define PMC_OCR		0x110	/*  Oscillator Calibration Register */

/*  -------- PMC_SCER : (PMC Offset: 0x0) System Clock Enable Register --------*/ 
#define AT91C_PMC_DDR		(0x1UL <<  2)
#define AT91C_PMC_LCDCK		(0x1UL <<  3)
#define AT91C_PMC_SMDCK		(0x1UL <<  4)
#define AT91C_PMC_UHP		(0x1UL <<  6)
#define AT91C_PMC_UDP		(0x1UL <<  7)
#define AT91C_PMC_PCK0		(0x1UL <<  8)
#define AT91C_PMC_PCK1		(0x1UL <<  9)
#define AT91C_PMC_PCK2		(0x1UL << 10)

#define AVAILABLE_SYS_CLK (AT91C_PMC_DDR \
					 | AT91C_PMC_LCDCK \
					 | AT91C_PMC_SMDCK \
					 | AT91C_PMC_UHP \
					 | AT91C_PMC_UDP \
					 | AT91C_PMC_PCK0 \
					 | AT91C_PMC_PCK1 \
					 | AT91C_PMC_PCK2)

/*  -------- PMC_SCDR : (PMC Offset: 0x4) System Clock Disable Register --------*/ 
/*  -------- PMC_SCSR : (PMC Offset: 0x8) System Clock Status Register ---------*/
#define AT91C_PMC_PCK		(0x1UL <<  0)

/* -- PMC_UCKR : (PMC Offset: 0x1c) UTMI Clock Configuration Register ------*/
#define AT91C_CKGR_UOSCEN	(0x1UL <<  0)
#define AT91C_CKGR_OSCBYPASS	(0x1UL <<  1)
#define AT91C_CKGR_OSCOUNT	(0xFFUL << 8)
#define AT91C_CKGR_UPLLEN	(0x1UL << 16)
#define		AT91C_CKGR_UPLLEN_DISABLED	(0x0UL << 16)
#define		AT91C_CKGR_UPLLEN_ENABLED	(0x1UL << 16)
#define AT91C_CKGR_UPLLCOUNT	(0xFUL << 20)
#define		AT91C_CKGR_UPLLCOUNT_DEFAULT	(0x1UL << 20)
#define AT91C_CKGR_BIASEN	(0x1UL << 24)
#define		AT91C_CKGR_BIASEN_DISABLED	(0x0UL << 24)
#define		AT91C_CKGR_BIASEN_ENABLED	(0x1UL << 24)
#define AT91C_CKGR_BIASCOUNT	(0xFUL << 28)

/*  -------- CKGR_MOR : (PMC Offset: 0x20) Main Oscillator Register --------*/ 
#define AT91C_CKGR_MOSCXTEN	(0x1UL <<  0)
#define AT91C_CKGR_MOSCXTBY	(0x1UL <<  1)
#define AT91C_CKGR_WAITMODE	(0x1UL <<  2)
#define AT91C_CKGR_MOSCRCEN	(0x1UL <<  3)
#define AT91C_CKGR_MOSCXTST	(0xFFUL <<  8)
#define		AT91_CKGR_MOSCXTST_SET(value)	(value << 8)
#define AT91C_CKGR_KEY		(0xFFUL << 16)
#define		AT91C_CKGR_PASSWD		(0x37 << 16)
#define AT91C_CKGR_MOSCSEL	(0x1UL << 24)
#define AT91C_CKGR_CFDEN	(0x1UL << 25)

/*  -------- CKGR_MCFR : (PMC Offset: 0x24) Main Clock Frequency Register --------*/ 
#define AT91C_CKGR_MAINF	(0xFFFFUL <<  0)
#define AT91C_CKGR_MAINRDY	(0x1UL << 16)
#define AT91C_CKGR_RCMEAS	(0x1UL << 20)
#define AT91C_CKGR_CCSS         (0x1UL << 24)
#define		AT91C_CKGR_CCSS_RC_OSC		(0x0UL << 24)
#define		AT91C_CKGR_CCSS_XTAL_OSC	(0x1UL << 24)


/*  -------- CKGR_PLLAR : (PMC Offset: 0x28) PLL A Register --------*/ 
#define AT91C_CKGR_DIVA		(0xff << 0)
#define		AT91C_CKGR_DIVA_MSK		0xff
#define		AT91C_CKGR_DIVA_OFFSET		0
#define		AT91C_CKGR_DIVA_0		0x0
#define		AT91C_CKGR_DIVA_BYPASS		0x1
#define AT91C_CKGR_PLLACOUNT	(0x3FUL <<  8)
#define AT91C_CKGR_OUTA		(0x3UL << 14)
#define		AT91C_CKGR_OUTA_0		(0x0UL << 14)
#define		AT91C_CKGR_OUTA_1		(0x1UL << 14)
#define		AT91C_CKGR_OUTA_2		(0x2UL << 14)
#define		AT91C_CKGR_OUTA_3		(0x3UL << 14)
#define AT91C_CKGR_MULA		(0xff << 16)
#define	AT91C_CKGR_ALT_MULA	(0x7f << 18)
#define		AT91C_CKGR_ALT_MULA_MSK		0x7f
#define		AT91C_CKGR_ALT_MULA_OFFSET	18
#define AT91C_CKGR_SRCA		(0x1UL << 29)

/*  -------- CKGR_PLLBR : (PMC Offset: 0x2c) PLL B Register --------*/
#define AT91C_CKGR_DIVB		(0xFFUL <<  0)
#define		AT91C_CKGR_DIVB_0		(0x0UL)
#define		AT91C_CKGR_DIVB_BYPASS		(0x1UL)
#define AT91C_CKGR_PLLBCOUNT	(0x3FUL <<  8)
#define AT91C_CKGR_OUTB		(0x3UL << 14)
#define		AT91C_CKGR_OUTB_0		(0x0UL << 14)
#define		AT91C_CKGR_OUTB_1		(0x1UL << 14)
#define		AT91C_CKGR_OUTB_2		(0x2UL << 14)
#define		AT91C_CKGR_OUTB_3		(0x3UL << 14)
#define AT91C_CKGR_MULB		(0x7FFUL << 16)

/*  -------- PMC_MCKR : (PMC Offset: 0x30) Master Clock Register --------*/ 
#define AT91C_PMC_CSS		(0x7UL <<  0)
#define		AT91C_PMC_CSS_SLOW_CLK		(0x0UL)
#define		AT91C_PMC_CSS_MAIN_CLK		(0x1UL)
#define		AT91C_PMC_CSS_PLLA_CLK		(0x2UL)
#define		AT91C_PMC_CSS_UPLL_CLK		(0x3UL)
#define		AT91C_PMC_CSS_SYS_CLK		(0x4UL)
#define	AT91C_PMC_PRES		(0xFUL << 2)
#define		AT91C_PMC_PRES_CLK		(0x0UL << 2)
#define		AT91C_PMC_PRES_CLK_2		(0x1UL << 2)
#define		AT91C_PMC_PRES_CLK_4		(0x2UL << 2)
#define		AT91C_PMC_PRES_CLK_8		(0x3UL << 2)
#define		AT91C_PMC_PRES_CLK_16		(0x4UL << 2)
#define		AT91C_PMC_PRES_CLK_32		(0x5UL << 2)
#define		AT91C_PMC_PRES_CLK_64		(0x6UL << 2)
#define		AT91C_PMC_PRES_CLK_3		(0x7UL << 2)
#define		AT91C_PMC_PRES_CLK_1_5		(0x8UL << 2)
/*  Master/Processor Clock Prescaler [alternate location ] */
#define	AT91C_PMC_ALT_PRES	(0xFUL << 4)
#define		AT91C_PMC_PRES_ALT_CLK		(0x0UL << 4)
#define		AT91C_PMC_PRES_ALT_CLK_2	(0x1UL << 4)
#define		AT91C_PMC_PRES_ALT_CLK_4	(0x2UL << 4)
#define		AT91C_PMC_PRES_ALT_CLK_8	(0x3UL << 4)
#define		AT91C_PMC_PRES_ALT_CLK_16	(0x4UL << 4)
#define		AT91C_PMC_PRES_ALT_CLK_32	(0x5UL << 4)
#define		AT91C_PMC_PRES_ALT_CLK_64	(0x6UL << 4)
#define AT91C_PMC_MDIV		(0x3UL <<  8)
#define		AT91C_PMC_MDIV_1		(0x0UL <<  8)
#define		AT91C_PMC_MDIV_2		(0x1UL <<  8)
#define		AT91C_PMC_MDIV_4		(0x2UL <<  8)
#define		AT91C_PMC_MDIV_3		(0x3UL <<  8)
#define AT91C_PMC_PLLADIV2	(0x1UL << 12)
#define		AT91C_PMC_PLLADIV2_1		(0x0UL << 12)
#define		AT91C_PMC_PLLADIV2_2		(0x1UL << 12)
#define AT91C_PMC_H32MXDIV	(0x1UL << 24)
#define		AT91C_PMC_H32MXDIV_H32MXDIV1	(0x00UL << 24)
#define		AT91C_PMC_H32MXDIV_H32MXDIV2	(0x01UL << 24)

/*  -------- PMC_USB : (PMC Offset: 0x38) USB Clock Register --------*/ 
#define AT91C_PMC_USBS		(0x1UL <<  0)
#define		AT91C_PMC_USBS_USB_PLLA		(0x0UL)
#define		AT91C_PMC_USBS_USB_UPLL		(0x1UL)
#define AT91C_PMC_USBDIV	(0xFUL <<  8)

/*  -------- PMC_SMD : (PMC Offset: 0x3c) SMD Clock Register --------*/ 
#define AT91C_PMC_SMDS		(0x1UL <<  0)
#define		AT91C_PMC_SMDS_SMD_PLLA		(0x0UL)
#define		AT91C_PMC_SMDS_SMD_PLLB		(0x1UL)
#define AT91C_PMC_SMDDIV	(0x1FUL <<  8)
#define AT91C_PMC_SMDDIV_(x)	((x & 0x1F) << 8)

/*  -------- PMC_PCKR : (PMC Offset: 0x40) Programmable Clock Register --------*/ 
#define AT91C_PMC_SLCKMCK	(0x1UL <<  8)
#define		AT91C_PMC_SLCKMCK_SLCK                 (0x0UL <<  8)
#define		AT91C_PMC_SLCKMCK_MCK                  (0x1UL <<  8)

/*  -------- PMC_IER : (PMC Offset: 0x60) PMC Interrupt Enable Register --------*/ 
#define AT91C_PMC_MOSCXTS	(0x1UL << 0)
#define AT91C_PMC_LOCKA		(0x1UL << 1)
#define AT91C_PMC_LOCKB		(0x1UL << 2)
#define AT91C_PMC_MCKRDY	(0x1UL << 3)
#define AT91C_PMC_LOCKU		(0x1UL << 6)
#define AT91C_PMC_PCKRDY0	(0x1UL << 8)
#define AT91C_PMC_PCKRDY1	(0x1UL << 9)
#define AT91C_PMC_MOSCSELS	(0x1UL << 16)
#define AT91C_PMC_MOSCRCS	(0x1UL << 17)
#define AT91C_PMC_CFDEV		(0x1UL << 18)
#define AT91C_PMC_GCKRDY	(0x1UL << 24)
/*  -------- PMC_IDR : (PMC Offset: 0x64) PMC Interrupt Disable Register --------*/ 
/*  -------- PMC_SR : (PMC Offset: 0x68) PMC Status Register --------*/ 
#define AT91C_PMC_CFDS		(0x1UL << 19)
#define AT91C_PMC_FOS		(0x1UL << 20)
/*  -------- PMC_IMR : (PMC Offset: 0x6c) PMC Interrupt Mask Register --------*/ 

/*  -------- PMC_FSMR : (PMC Offset: 0x70) Fast Startup Mode Register --------*/ 
#define AT91C_PMC_FSTT		(0xFFFFUL << 0)
#define AT91C_PMC_RTTAL		(0x1UL << 16)
#define AT91C_PMC_RTCAL		(0x1UL << 17)
#define AT91C_PMC_USBAL		(0x1UL << 18)
#define AT91C_PMC_LPM		(0x1UL << 20)

/*  -------- PMC_FSPR : (PMC Offset: 0x74) Fast Startup Polarity Register --------*/ 
#define AT91C_PMC_FSTP		(0xFFFFUL <<  0)

/*  -------- PMC_FOCR : (PMC Offset: 0x78) Fault Output Clear Register --------*/ 
#define AT91C_PMC_FOCLR		(0x1UL << 0)

/*  -------- PMC_PLLICPR : (PMC Offset: 0x80) PLL Charge Pump Current Register --------*/ 
#define AT91C_PMC_ICPPLLA	(0xFUL <<  0)
#define		AT91C_PMC_ICPPLLA_0		(0x0UL << 0)
#define		AT91C_PMC_ICPPLLA_1		(0x1UL << 0)
#define AT91C_PMC_REALLOCK	(0x1UL <<  7)
#define AT91C_PMC_IPLLA		(0xFUL <<  8)
#define		AT91C_PMC_IPLLA_0		(0x0UL <<  8)
#define		AT91C_PMC_IPLLA_1		(0x1UL <<  8)
#define		AT91C_PMC_IPLLA_2		(0x2UL <<  8)
#define		AT91C_PMC_IPLLA_3		(0x3UL <<  8)

/*  -------- PMC_FEATURES : (PMC Offset: 0xf8)   --------*/ 
#define AT91C_PMC_CFGAHBCLK	(0x1UL <<  0)
#define		AT91C_PMC_CFGAHBCLK_0		(0x0UL << 0)
#define		AT91C_PMC_CFGAHBCLK_1		(0x1UL << 0)
#define AT91C_PMC_HCLKEN	(0x1UL <<  1)
#define		AT91C_PMC_HCLKEN_0		(0x0UL <<  1)
#define		AT91C_PMC_HCLKEN_1		(0x1UL <<  1)
#define AT91C_PMC_PERMCLK	(0x1UL <<  2)
#define		AT91C_PMC_PERMCLK_0		(0x0UL <<  2)
#define		AT91C_PMC_PERMCLK_1		(0x1UL <<  2)
#define AT91C_PMC_CORE2		(0x1UL <<  3)
#define		AT91C_PMC_CORE2_0		(0x0UL <<  3)
#define		AT91C_PMC_CORE2_1		(0x1UL <<  3)
#define AT91C_PMC_USBDEVCK	(0x1UL <<  4)
#define		AT91C_PMC_USBDEVCK_0		(0x0UL <<  4)
#define		AT91C_PMC_USBDEVCK_1		(0x1UL <<  4)
#define AT91C_PMC_USBHOSTCK	(0x1UL <<  5)
#define		AT91C_PMC_USBHOSTCK_0		(0x0UL <<  5)
#define		AT91C_PMC_USBHOSTCK_1		(0x1UL <<  5)
#define AT91C_PMC_USBOTGCK	(0x1UL <<  6)
#define		AT91C_PMC_USBOTGCK_0		(0x0UL <<  6)
#define		AT91C_PMC_USBOTGCK_1		(0x1UL <<  6)
#define AT91C_PMC_UHSYNRST	(0x1UL <<  7)
#define		AT91C_PMC_UHSYNRST_0		(0x0UL <<  7)
#define		AT91C_PMC_UHSYNRST_1		(0x1UL <<  7)
#define AT91C_PMC_UOSYNRST	(0x1UL <<  8)
#define		AT91C_PMC_UOSYNRST_0		(0x0UL <<  8)
#define		AT91C_PMC_UOSYNRST_1		(0x1UL <<  8)
#define AT91C_PMC_PLLENPOL	(0x1UL <<  9)
#define		AT91C_PMC_PLLENPOL_0		(0x0UL <<  9)
#define		AT91C_PMC_PLLENPOL_1		(0x1UL <<  9)
#define AT91C_PMC_BIASREG	(0x1UL << 10)
#define		AT91C_PMC_BIASREG_0		(0x0UL << 10)
#define		AT91C_PMC_BIASREG_1		(0x1UL << 10)
#define AT91C_PMC_OUTPLL	(0x1UL << 11)
#define		AT91C_PMC_OUTPLL_0		(0x0UL << 11)
#define		AT91C_PMC_OUTPLL_1		(0x1UL << 11)
#define AT91C_PMC_OUTCURR	(0x1UL << 12)
#define		AT91C_PMC_OUTCURR_0		(0x0UL << 12)
#define		AT91C_PMC_OUTCURR_1		(0x1UL << 12)
#define AT91C_PMC_FWUP		(0x1UL << 13)
#define		AT91C_PMC_FWUP_0		(0x0UL << 13)
#define		AT91C_PMC_FWUP_1		(0x1UL << 13)
#define AT91C_PMC_SELMAINCLK	(0x1UL << 14)
#define		AT91C_PMC_SELMAINCLK_0		(0x0UL << 14)
#define		AT91C_PMC_SELMAINCLK_1		(0x1UL << 14)
#define AT91C_PMC_RSTCLKM	(0x1UL << 15)
#define		AT91C_PMC_RSTCLKM_0		(0x0UL << 15)
#define		AT91C_PMC_RSTCLKM_1		(0x1UL << 15)
#define AT91C_PMC_NB_PERIPH_CLOCK	(0xFFUL << 16)

/*  -------- PMC_VERSION : (PMC Offset: 0xfc)   --------*/ 
#define AT91C_PMC_Version	(0x0FFFUL << 0)
#define		AT91C_PMC_Version_0		(0x0UL)
#define		AT91C_PMC_Version_1		(0x1UL)
#define AT91C_PMC_MFN		(0x7UL << 16)
#define		AT91C_PMC_MFN_0		(0x0UL << 16)
#define		AT91C_PMC_MFN_1		(0x1UL << 16)

/*  -------- PMC_PCR : (PMC Offset: 0x10c) Peripheral Control Register --------*/ 
#define	AT91C_PMC_PID		(0x3f << 0)
#define	AT91C_PMC_GCKCSS	(0x7 << 8)
#define		AT91C_PMC_GCKCSS_SLOW_CLK	(0x0 << 8)
#define		AT91C_PMC_GCKCSS_MAIN_CLK	(0x1 << 8)
#define		AT91C_PMC_GCKCSS_PLLA_CLK	(0x2 << 8)
#define		AT91C_PMC_GCKCSS_UPLL_CLK	(0x3 << 8)
#define		AT91C_PMC_GCKCSS_MCK_CLK	(0x4 << 8)
#define		AT91C_PMC_GCKCSS_AUDIO_CLK	(0x5 << 8)
#define	AT91C_PMC_CMD		(0x1 << 12)
#define	AT91C_PMC_DIV		(0x3 << 16)
#define	AT91C_PMC_GCKDIV	(0xff << 20)
#define		AT91C_PMC_GCKDIV_MSK		0xff
#define		AT91C_PMC_GCKDIV_OFFSET		20
#define		AT91C_PMC_GCKDIV_(x)		(((x) & AT91C_PMC_GCKDIV_MSK) << AT91C_PMC_GCKDIV_OFFSET)
#define	AT91C_PMC_EN		(0x1 << 28)
#define	AT91C_PMC_GCKEN		(0x1 << 29)

#endif /* PMC_AT91_H */
