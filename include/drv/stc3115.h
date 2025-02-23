#ifndef STC3115_H
#define STC3115_H
#define STC3115_SLAVE_ADDRESS            0xE0    /* STC31xx 8-bit address byte			*/

/* STC3115 registers define ------------------------------------------------------------ */
#define STC3115_REG_MODE                 0x00    /* Mode Register             			*/
#define STC3115_REG_CTRL                 0x01    /* Control and Status Register 		*/
#define STC3115_REG_SOC                  0x02    /* SOC Data (2 bytes) 					*/
#define STC3115_REG_COUNTER              0x04    /* Number of Conversion (2 bytes) 		*/
#define STC3115_REG_CURRENT              0x06    /* Battery Current (2 bytes) 			*/
#define STC3115_REG_VOLTAGE              0x08    /* Battery Voltage (2 bytes) 			*/
#define STC3115_REG_TEMPERATURE          0x0A    /* Temperature               			*/
#define STC3115_REG_CC_ADJ_HIGH          0x0B    /* CC adjustement     					*/
#define STC3115_REG_CC_ADJ_LOW           0x19    /* CC adjustement     					*/
#define STC3115_REG_VM_ADJ_HIGH          0x0C    /* VM adjustement     					*/
#define STC3115_REG_VM_ADJ_LOW           0x1A    /* VM adjustement     					*/
#define STC3115_REG_OCV                  0x0D    /* Battery OCV (2 bytes) 				*/
#define STC3115_REG_CC_CNF               0x0F    /* Coulomb Counter CC configuration (2 bytes) */
#define STC3115_REG_VM_CNF               0x11    /* Voltage Mode VM configuration (2 bytes)    */
#define STC3115_REG_ALARM_SOC            0x13    /* SOC alarm level         			*/
#define STC3115_REG_ALARM_VOLTAGE        0x14    /* Low voltage alarm level 			*/
#define STC3115_REG_CURRENT_THRES        0x15    /* Current threshold for relaxation 	*/
#define STC3115_REG_RELAX_COUNT          0x16    /* Voltage relaxation counter   		*/
#define STC3115_REG_RELAX_MAX            0x17    /* Voltage relaxation max count 		*/
#define STC3115_REG_ID			 0x18
#define STC3115_REG_RAM     		 0x20    /* General Purpose RAM Registers 		*/
#define STC3115_REG_OCVTAB               0x30	 /* OCV OFFSET table registers			*/

/* STC3115 STC3115_REG_MODE Bit mask definition ------------------------------------ */
#define STC3115_VMODE   	0x01	 	/* Voltage mode bit mask     				*/
#define STC3115_CLR_VM_ADJ	0x02  		/* Clear VM ADJ register bit mask 			*/
#define STC3115_CLR_CC_ADJ	0x04  		/* Clear CC ADJ register bit mask 			*/
#define STC3115_ALM_ENA		0x08	 	/* Alarm enable bit mask     				*/
#define STC3115_GG_RUN		0x10	 	/* Alarm enable bit mask     				*/
#define STC3115_FORCE_CC	0x20	 	/* Force CC bit mask     					*/
#define STC3115_FORCE_VM	0x40	 	/* Force VM bit mask     					*/
#define STC3115_REGMODE_DEFAULT_STANDBY   	0x09   /* GG_RUN=0 (Standby mode)		*/

/* STC3115 STC3115_REG_CTRL Bit mask definition ------------------------------------ */
//ALM TBD
#define STC3115_GG_RST		0x02		/* Convertion counter reset					*/
#define STC3115_GG_VM		0x04		/* STC3115 active mode: cc=0, VM=1			*/
#define STC3115_BATFAIL		0x08		/* Battery presence state					*/
#define STC3115_PORDET		0x10	 	/* W = soft reset, R = POR detect			*/
#define STC3115_ALM_SOC		0x20	 	/* Low SOC alarm event						*/
#define STC3115_ALM_VOLT	0x40	 	/* Low voltage alarm event					*/

/*STC3115 General purpose define ---------------------------------------------------------- */
#define STC3115_ID		0x14    	/* STC3115 ID 										*/
#define STC3115_RAM_SIZE	16      	/* Total RAM size of STC3115 in bytes 				*/
#define STC3115_OCVTAB_SIZE     16      	/* OCVTAB size of STC3115 in bytes 					*/
#define VCOUNT			4       	/* counter value for 1st current/temp measurements	*/
#define VM_MODE 		1		/* Voltage Mode */
#define CC_MODE 		0		/* Coulomb Counter Mode */
#define MIXED_MODE		0		/* Mixed Mode (Voltage + Current) */
#define MAX_HRSOC          	51200  	/* 100% in 1/512% units	*/
#define MAX_SOC            	1000   	/* 100% in 0.1% units */
#define STC3115_OK 		0
#define VOLTAGE_FACTOR 		9011      	/* LSB=2.20mV ~9011/4096 - convert to mV */
#define CURRENT_FACTOR		24084		/* LSB=5.88uV/R= ~24084/R/4096 - convert to mA */
#define VOLTAGE_SECURITY_RANGE	200

#define RAM_TESTWORD	0x53A9		/* STC3115 RAM test word */
#define STC3115_UNINIT    0             /* Gas gauge Not Initialiezd state */
#define STC3115_INIT     'I'		/* Gas gauge Init states */
#define STC3115_RUNNING  'R'		/* Gas gauge Running states */
#define STC3115_POWERDN  'D'		/* Gas gauge Stop states */

#define APP_EOC_CURRENT       75   		/* end charge current in mA                 */
#define APP_CUTOFF_VOLTAGE	  3000   	/* application cut-off voltage in mV      	*/

#endif /* STC3115_H */
