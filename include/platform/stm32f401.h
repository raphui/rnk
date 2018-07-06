#ifndef STM32F401_H
#define STM32F401_H


/** @addtogroup Configuration_section_for_CMSIS
 * @{
 */

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
 */
#define __CM4_REV                 0x0001  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1       /*!< FPU present                                   */

/**
 * @}
 */

/** @addtogroup Peripheral_interrupt_number_definition
 * @{
 */

/**
 * @brief STM32F4XX Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section 
 */
typedef enum
{
	/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
	NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
	MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
	BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
	UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
	SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
	DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
	PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
	SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
	/******  STM32 specific Interrupt Numbers **********************************************************************/
	WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
	PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
	TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
	RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
	FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
	RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
	EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
	EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
	EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
	EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
	EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
	DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
	DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
	DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
	DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
	DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
	DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
	DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
	ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
	CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
	CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
	CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
	CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
	EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
	TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
	TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
	TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
	TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
	TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
	TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
	I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
	I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
	I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
	I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
	SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
	SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
	USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
	USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
	EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
	RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
	OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
	DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
	SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
	TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
	SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
	DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
	DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
	DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
	DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
	DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
	OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
	DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
	DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
	DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
	USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
	I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
	I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
	FPU_IRQn                    = 81,      /*!< FPU global interrupt                                             */
	SPI4_IRQn                   = 84       /*!< SPI4 global Interrupt                                            */
} IRQn_Type;

/**
 * @}
 */


/** @addtogroup Peripheral_registers_structures
 * @{
 */   

/** 
 * @brief Analog to Digital Converter  
 */

typedef struct
{
	volatile unsigned int SR;     /*!< ADC status register,                         Address offset: 0x00 */
	volatile unsigned int CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */      
	volatile unsigned int CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
	volatile unsigned int SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
	volatile unsigned int SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
	volatile unsigned int JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
	volatile unsigned int JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
	volatile unsigned int JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
	volatile unsigned int JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
	volatile unsigned int HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
	volatile unsigned int LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
	volatile unsigned int SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
	volatile unsigned int SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
	volatile unsigned int SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
	volatile unsigned int JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
	volatile unsigned int JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
	volatile unsigned int JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
	volatile unsigned int JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
	volatile unsigned int JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
	volatile unsigned int DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
	volatile unsigned int CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
	volatile unsigned int CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
	volatile unsigned int CDR;    /*!< ADC common regular data register for dual
			       AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;


/** 
 * @brief Controller Area Network TxMailBox 
 */

typedef struct
{
	volatile unsigned int TIR;  /*!< CAN TX mailbox identifier register */
	volatile unsigned int TDTR; /*!< CAN mailbox data length control and time stamp register */
	volatile unsigned int TDLR; /*!< CAN mailbox data low register */
	volatile unsigned int TDHR; /*!< CAN mailbox data high register */
} CAN_TxMailBox_TypeDef;

/** 
 * @brief Controller Area Network FIFOMailBox 
 */

typedef struct
{
	volatile unsigned int RIR;  /*!< CAN receive FIFO mailbox identifier register */
	volatile unsigned int RDTR; /*!< CAN receive FIFO mailbox data length control and time stamp register */
	volatile unsigned int RDLR; /*!< CAN receive FIFO mailbox data low register */
	volatile unsigned int RDHR; /*!< CAN receive FIFO mailbox data high register */
} CAN_FIFOMailBox_TypeDef;

/** 
 * @brief Controller Area Network FilterRegister 
 */

typedef struct
{
	volatile unsigned int FR1; /*!< CAN Filter bank register 1 */
	volatile unsigned int FR2; /*!< CAN Filter bank register 1 */
} CAN_FilterRegister_TypeDef;

/** 
 * @brief Controller Area Network 
 */

typedef struct
{
	volatile unsigned int              MCR;                 /*!< CAN master control register,         Address offset: 0x00          */
	volatile unsigned int              MSR;                 /*!< CAN master status register,          Address offset: 0x04          */
	volatile unsigned int              TSR;                 /*!< CAN transmit status register,        Address offset: 0x08          */
	volatile unsigned int              RF0R;                /*!< CAN receive FIFO 0 register,         Address offset: 0x0C          */
	volatile unsigned int              RF1R;                /*!< CAN receive FIFO 1 register,         Address offset: 0x10          */
	volatile unsigned int              IER;                 /*!< CAN interrupt enable register,       Address offset: 0x14          */
	volatile unsigned int              ESR;                 /*!< CAN error status register,           Address offset: 0x18          */
	volatile unsigned int              BTR;                 /*!< CAN bit timing register,             Address offset: 0x1C          */
	volatile unsigned int                   RESERVED0[88];       /*!< Reserved, 0x020 - 0x17F                                            */
	CAN_TxMailBox_TypeDef      sTxMailBox[3];       /*!< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
	CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];     /*!< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
	volatile unsigned int                   RESERVED1[12];       /*!< Reserved, 0x1D0 - 0x1FF                                            */
	volatile unsigned int              FMR;                 /*!< CAN filter master register,          Address offset: 0x200         */
	volatile unsigned int              FM1R;                /*!< CAN filter mode register,            Address offset: 0x204         */
	volatile unsigned int                   RESERVED2;           /*!< Reserved, 0x208                                                    */
	volatile unsigned int              FS1R;                /*!< CAN filter scale register,           Address offset: 0x20C         */
	volatile unsigned int                   RESERVED3;           /*!< Reserved, 0x210                                                    */
	volatile unsigned int              FFA1R;               /*!< CAN filter FIFO assignment register, Address offset: 0x214         */
	volatile unsigned int                   RESERVED4;           /*!< Reserved, 0x218                                                    */
	volatile unsigned int              FA1R;                /*!< CAN filter activation register,      Address offset: 0x21C         */
	volatile unsigned int                   RESERVED5[8];        /*!< Reserved, 0x220-0x23F                                              */ 
	CAN_FilterRegister_TypeDef sFilterRegister[28]; /*!< CAN Filter Register,                 Address offset: 0x240-0x31C   */
} CAN_TypeDef;

/** 
 * @brief CRC calculation unit 
 */

typedef struct
{
	volatile unsigned int DR;         /*!< CRC Data register,             Address offset: 0x00 */
	volatile unsigned char  IDR;        /*!< CRC Independent data register, Address offset: 0x04 */
	volatile unsigned char       RESERVED0;  /*!< Reserved, 0x05                                      */
	volatile unsigned short      RESERVED1;  /*!< Reserved, 0x06                                      */
	volatile unsigned int CR;         /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;

/** 
 * @brief Digital to Analog Converter
 */

typedef struct
{
	volatile unsigned int CR;       /*!< DAC control register,                                    Address offset: 0x00 */
	volatile unsigned int SWTRIGR;  /*!< DAC software trigger register,                           Address offset: 0x04 */
	volatile unsigned int DHR12R1;  /*!< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
	volatile unsigned int DHR12L1;  /*!< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
	volatile unsigned int DHR8R1;   /*!< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
	volatile unsigned int DHR12R2;  /*!< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
	volatile unsigned int DHR12L2;  /*!< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
	volatile unsigned int DHR8R2;   /*!< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
	volatile unsigned int DHR12RD;  /*!< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
	volatile unsigned int DHR12LD;  /*!< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
	volatile unsigned int DHR8RD;   /*!< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
	volatile unsigned int DOR1;     /*!< DAC channel1 data output register,                       Address offset: 0x2C */
	volatile unsigned int DOR2;     /*!< DAC channel2 data output register,                       Address offset: 0x30 */
	volatile unsigned int SR;       /*!< DAC status register,                                     Address offset: 0x34 */
} DAC_TypeDef;

/** 
 * @brief Debug MCU
 */

typedef struct
{
	volatile unsigned int IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
	volatile unsigned int CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
	volatile unsigned int APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
	volatile unsigned int APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;

/** 
 * @brief DCMI
 */

typedef struct
{
	volatile unsigned int CR;       /*!< DCMI control register 1,                       Address offset: 0x00 */
	volatile unsigned int SR;       /*!< DCMI status register,                          Address offset: 0x04 */
	volatile unsigned int RISR;     /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
	volatile unsigned int IER;      /*!< DCMI interrupt enable register,                Address offset: 0x0C */
	volatile unsigned int MISR;     /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
	volatile unsigned int ICR;      /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
	volatile unsigned int ESCR;     /*!< DCMI embedded synchronization code register,   Address offset: 0x18 */
	volatile unsigned int ESUR;     /*!< DCMI embedded synchronization unmask register, Address offset: 0x1C */
	volatile unsigned int CWSTRTR;  /*!< DCMI crop window start,                        Address offset: 0x20 */
	volatile unsigned int CWSIZER;  /*!< DCMI crop window size,                         Address offset: 0x24 */
	volatile unsigned int DR;       /*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;

/** 
 * @brief DMA Controller
 */

typedef struct
{
	volatile unsigned int CR;     /*!< DMA stream x configuration register      */
	volatile unsigned int NDTR;   /*!< DMA stream x number of data register     */
	volatile unsigned int PAR;    /*!< DMA stream x peripheral address register */
	volatile unsigned int M0AR;   /*!< DMA stream x memory 0 address register   */
	volatile unsigned int M1AR;   /*!< DMA stream x memory 1 address register   */
	volatile unsigned int FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct
{
	volatile unsigned int LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
	volatile unsigned int HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
	volatile unsigned int LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
	volatile unsigned int HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;


/** 
 * @brief Ethernet MAC
 */

typedef struct
{
	volatile unsigned int MACCR;
	volatile unsigned int MACFFR;
	volatile unsigned int MACHTHR;
	volatile unsigned int MACHTLR;
	volatile unsigned int MACMIIAR;
	volatile unsigned int MACMIIDR;
	volatile unsigned int MACFCR;
	volatile unsigned int MACVLANTR;             /*    8 */
	volatile unsigned int      RESERVED0[2];
	volatile unsigned int MACRWUFFR;             /*   11 */
	volatile unsigned int MACPMTCSR;
	volatile unsigned int      RESERVED1[2];
	volatile unsigned int MACSR;                 /*   15 */
	volatile unsigned int MACIMR;
	volatile unsigned int MACA0HR;
	volatile unsigned int MACA0LR;
	volatile unsigned int MACA1HR;
	volatile unsigned int MACA1LR;
	volatile unsigned int MACA2HR;
	volatile unsigned int MACA2LR;
	volatile unsigned int MACA3HR;
	volatile unsigned int MACA3LR;               /*   24 */
	volatile unsigned int      RESERVED2[40];
	volatile unsigned int MMCCR;                 /*   65 */
	volatile unsigned int MMCRIR;
	volatile unsigned int MMCTIR;
	volatile unsigned int MMCRIMR;
	volatile unsigned int MMCTIMR;               /*   69 */
	volatile unsigned int      RESERVED3[14];
	volatile unsigned int MMCTGFSCCR;            /*   84 */
	volatile unsigned int MMCTGFMSCCR;
	volatile unsigned int      RESERVED4[5];
	volatile unsigned int MMCTGFCR;
	volatile unsigned int      RESERVED5[10];
	volatile unsigned int MMCRFCECR;
	volatile unsigned int MMCRFAECR;
	volatile unsigned int      RESERVED6[10];
	volatile unsigned int MMCRGUFCR;
	volatile unsigned int      RESERVED7[334];
	volatile unsigned int PTPTSCR;
	volatile unsigned int PTPSSIR;
	volatile unsigned int PTPTSHR;
	volatile unsigned int PTPTSLR;
	volatile unsigned int PTPTSHUR;
	volatile unsigned int PTPTSLUR;
	volatile unsigned int PTPTSAR;
	volatile unsigned int PTPTTHR;
	volatile unsigned int PTPTTLR;
	volatile unsigned int RESERVED8;
	volatile unsigned int PTPTSSR;
	volatile unsigned int      RESERVED9[565];
	volatile unsigned int DMABMR;
	volatile unsigned int DMATPDR;
	volatile unsigned int DMARPDR;
	volatile unsigned int DMARDLAR;
	volatile unsigned int DMATDLAR;
	volatile unsigned int DMASR;
	volatile unsigned int DMAOMR;
	volatile unsigned int DMAIER;
	volatile unsigned int DMAMFBOCR;
	volatile unsigned int DMARSWTR;
	volatile unsigned int      RESERVED10[8];
	volatile unsigned int DMACHTDR;
	volatile unsigned int DMACHRDR;
	volatile unsigned int DMACHTBAR;
	volatile unsigned int DMACHRBAR;
} ETH_TypeDef;

/** 
 * @brief External Interrupt/Event Controller
 */

typedef struct
{
	volatile unsigned int IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
	volatile unsigned int EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
	volatile unsigned int RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
	volatile unsigned int FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
	volatile unsigned int SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
	volatile unsigned int PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

/** 
 * @brief FLASH Registers
 */

typedef struct
{
	volatile unsigned int ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
	volatile unsigned int KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
	volatile unsigned int OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
	volatile unsigned int SR;       /*!< FLASH status register,           Address offset: 0x0C */
	volatile unsigned int CR;       /*!< FLASH control register,          Address offset: 0x10 */
	volatile unsigned int OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
	volatile unsigned int OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;


/** 
 * @brief Flexible Static Memory Controller
 */

typedef struct
{
	volatile unsigned int BTCR[8];    /*!< NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR), Address offset: 0x00-1C */   
} FSMC_Bank1_TypeDef; 

/** 
 * @brief Flexible Static Memory Controller Bank1E
 */

typedef struct
{
	volatile unsigned int BWTR[7];    /*!< NOR/PSRAM write timing registers, Address offset: 0x104-0x11C */
} FSMC_Bank1E_TypeDef;

/** 
 * @brief Flexible Static Memory Controller Bank2
 */

typedef struct
{
	volatile unsigned int PCR2;       /*!< NAND Flash control register 2,                       Address offset: 0x60 */
	volatile unsigned int SR2;        /*!< NAND Flash FIFO status and interrupt register 2,     Address offset: 0x64 */
	volatile unsigned int PMEM2;      /*!< NAND Flash Common memory space timing register 2,    Address offset: 0x68 */
	volatile unsigned int PATT2;      /*!< NAND Flash Attribute memory space timing register 2, Address offset: 0x6C */
	volatile unsigned int      RESERVED0;  /*!< Reserved, 0x70                                                            */
	volatile unsigned int ECCR2;      /*!< NAND Flash ECC result registers 2,                   Address offset: 0x74 */
	volatile unsigned int      RESERVED1;  /*!< Reserved, 0x78                                                            */
	volatile unsigned int      RESERVED2;  /*!< Reserved, 0x7C                                                            */
	volatile unsigned int PCR3;       /*!< NAND Flash control register 3,                       Address offset: 0x80 */
	volatile unsigned int SR3;        /*!< NAND Flash FIFO status and interrupt register 3,     Address offset: 0x84 */
	volatile unsigned int PMEM3;      /*!< NAND Flash Common memory space timing register 3,    Address offset: 0x88 */
	volatile unsigned int PATT3;      /*!< NAND Flash Attribute memory space timing register 3, Address offset: 0x8C */
	volatile unsigned int      RESERVED3;  /*!< Reserved, 0x90                                                            */
	volatile unsigned int ECCR3;      /*!< NAND Flash ECC result registers 3,                   Address offset: 0x94 */
} FSMC_Bank2_3_TypeDef;

/** 
 * @brief Flexible Static Memory Controller Bank4
 */

typedef struct
{
	volatile unsigned int PCR4;       /*!< PC Card  control register 4,                       Address offset: 0xA0 */
	volatile unsigned int SR4;        /*!< PC Card  FIFO status and interrupt register 4,     Address offset: 0xA4 */
	volatile unsigned int PMEM4;      /*!< PC Card  Common memory space timing register 4,    Address offset: 0xA8 */
	volatile unsigned int PATT4;      /*!< PC Card  Attribute memory space timing register 4, Address offset: 0xAC */
	volatile unsigned int PIO4;       /*!< PC Card  I/O space timing register 4,              Address offset: 0xB0 */
} FSMC_Bank4_TypeDef; 


/** 
 * @brief General Purpose I/O
 */

typedef struct
{
	volatile unsigned int MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	volatile unsigned int OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	volatile unsigned int OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	volatile unsigned int PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	volatile unsigned int IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	volatile unsigned int ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	volatile unsigned short BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
	volatile unsigned short BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
	volatile unsigned int LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	volatile unsigned int AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/** 
 * @brief System configuration controller
 */

typedef struct
{
	volatile unsigned int MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
	volatile unsigned int PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
	volatile unsigned int EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
	volatile unsigned int      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */ 
	volatile unsigned int CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

/** 
 * @brief Inter-integrated Circuit Interface
 */

typedef struct
{
	volatile unsigned int CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
	volatile unsigned int CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
	volatile unsigned int OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
	volatile unsigned int OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
	volatile unsigned int DR;         /*!< I2C Data register,          Address offset: 0x10 */
	volatile unsigned int SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
	volatile unsigned int SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
	volatile unsigned int CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
	volatile unsigned int TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
	volatile unsigned int FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;

/** 
 * @brief Independent WATCHDOG
 */

typedef struct
{
	volatile unsigned int KR;   /*!< IWDG Key register,       Address offset: 0x00 */
	volatile unsigned int PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
	volatile unsigned int RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
	volatile unsigned int SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

/** 
 * @brief Power Control
 */

typedef struct
{
	volatile unsigned int CR;   /*!< PWR power control register,        Address offset: 0x00 */
	volatile unsigned int CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;

/** 
 * @brief Reset and Clock Control
 */

typedef struct
{
	volatile unsigned int CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	volatile unsigned int PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	volatile unsigned int CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	volatile unsigned int CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	volatile unsigned int AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	volatile unsigned int AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	volatile unsigned int AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	volatile unsigned int      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	volatile unsigned int APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	volatile unsigned int APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	volatile unsigned int      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	volatile unsigned int AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	volatile unsigned int AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	volatile unsigned int AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	volatile unsigned int      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
	volatile unsigned int APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	volatile unsigned int APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	volatile unsigned int      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	volatile unsigned int AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	volatile unsigned int AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	volatile unsigned int AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	volatile unsigned int      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
	volatile unsigned int APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	volatile unsigned int APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	volatile unsigned int      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	volatile unsigned int BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	volatile unsigned int CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	volatile unsigned int      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	volatile unsigned int SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	volatile unsigned int PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */

} RCC_TypeDef;

/** 
 * @brief Real-Time Clock
 */

typedef struct
{
	volatile unsigned int TR;      /*!< RTC time register,                                        Address offset: 0x00 */
	volatile unsigned int DR;      /*!< RTC date register,                                        Address offset: 0x04 */
	volatile unsigned int CR;      /*!< RTC control register,                                     Address offset: 0x08 */
	volatile unsigned int ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
	volatile unsigned int PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
	volatile unsigned int WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
	volatile unsigned int CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
	volatile unsigned int ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
	volatile unsigned int ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
	volatile unsigned int WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
	volatile unsigned int SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
	volatile unsigned int SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
	volatile unsigned int TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
	volatile unsigned int TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
	volatile unsigned int TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
	volatile unsigned int CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
	volatile unsigned int TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
	volatile unsigned int ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
	volatile unsigned int ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
	volatile unsigned int RESERVED7;    /*!< Reserved, 0x4C                                                                 */
	volatile unsigned int BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
	volatile unsigned int BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
	volatile unsigned int BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
	volatile unsigned int BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
	volatile unsigned int BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
	volatile unsigned int BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
	volatile unsigned int BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
	volatile unsigned int BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
	volatile unsigned int BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
	volatile unsigned int BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
	volatile unsigned int BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
	volatile unsigned int BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
	volatile unsigned int BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
	volatile unsigned int BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
	volatile unsigned int BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
	volatile unsigned int BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
	volatile unsigned int BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
	volatile unsigned int BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
	volatile unsigned int BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
	volatile unsigned int BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;


/** 
 * @brief SD host Interface
 */

typedef struct
{
	volatile unsigned int POWER;          /*!< SDIO power control register,    Address offset: 0x00 */
	volatile unsigned int CLKCR;          /*!< SDI clock control register,     Address offset: 0x04 */
	volatile unsigned int ARG;            /*!< SDIO argument register,         Address offset: 0x08 */
	volatile unsigned int CMD;            /*!< SDIO command register,          Address offset: 0x0C */
	volatile unsigned int  RESPCMD;        /*!< SDIO command response register, Address offset: 0x10 */
	volatile unsigned int  RESP1;          /*!< SDIO response 1 register,       Address offset: 0x14 */
	volatile unsigned int  RESP2;          /*!< SDIO response 2 register,       Address offset: 0x18 */
	volatile unsigned int  RESP3;          /*!< SDIO response 3 register,       Address offset: 0x1C */
	volatile unsigned int  RESP4;          /*!< SDIO response 4 register,       Address offset: 0x20 */
	volatile unsigned int DTIMER;         /*!< SDIO data timer register,       Address offset: 0x24 */
	volatile unsigned int DLEN;           /*!< SDIO data length register,      Address offset: 0x28 */
	volatile unsigned int DCTRL;          /*!< SDIO data control register,     Address offset: 0x2C */
	volatile unsigned int  DCOUNT;         /*!< SDIO data counter register,     Address offset: 0x30 */
	volatile unsigned int  STA;            /*!< SDIO status register,           Address offset: 0x34 */
	volatile unsigned int ICR;            /*!< SDIO interrupt clear register,  Address offset: 0x38 */
	volatile unsigned int MASK;           /*!< SDIO mask register,             Address offset: 0x3C */
	volatile unsigned int      RESERVED0[2];   /*!< Reserved, 0x40-0x44                                  */
	volatile unsigned int  FIFOCNT;        /*!< SDIO FIFO counter register,     Address offset: 0x48 */
	volatile unsigned int      RESERVED1[13];  /*!< Reserved, 0x4C-0x7C                                  */
	volatile unsigned int FIFO;           /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;

/** 
 * @brief Serial Peripheral Interface
 */

typedef struct
{
	volatile unsigned int CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
	volatile unsigned int CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
	volatile unsigned int SR;         /*!< SPI status register,                                Address offset: 0x08 */
	volatile unsigned int DR;         /*!< SPI data register,                                  Address offset: 0x0C */
	volatile unsigned int CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
	volatile unsigned int RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
	volatile unsigned int TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
	volatile unsigned int I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
	volatile unsigned int I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;

/** 
 * @brief TIM
 */

typedef struct
{
	volatile unsigned int CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
	volatile unsigned int CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
	volatile unsigned int SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
	volatile unsigned int DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
	volatile unsigned int SR;          /*!< TIM status register,                 Address offset: 0x10 */
	volatile unsigned int EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
	volatile unsigned int CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
	volatile unsigned int CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
	volatile unsigned int CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
	volatile unsigned int CNT;         /*!< TIM counter register,                Address offset: 0x24 */
	volatile unsigned int PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
	volatile unsigned int ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
	volatile unsigned int RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
	volatile unsigned int CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
	volatile unsigned int CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
	volatile unsigned int CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
	volatile unsigned int CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
	volatile unsigned int BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
	volatile unsigned int DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
	volatile unsigned int DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
	volatile unsigned int OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

/** 
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */

typedef struct
{
	volatile unsigned int SR;         /*!< USART Status register,                   Address offset: 0x00 */
	volatile unsigned int DR;         /*!< USART Data register,                     Address offset: 0x04 */
	volatile unsigned int BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
	volatile unsigned int CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
	volatile unsigned int CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
	volatile unsigned int CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
	volatile unsigned int GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;

/** 
 * @brief Window WATCHDOG
 */

typedef struct
{
	volatile unsigned int CR;   /*!< WWDG Control register,       Address offset: 0x00 */
	volatile unsigned int CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
	volatile unsigned int SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;

/** 
 * @brief RNG
 */

typedef struct 
{
	volatile unsigned int CR;  /*!< RNG control register, Address offset: 0x00 */
	volatile unsigned int SR;  /*!< RNG status register,  Address offset: 0x04 */
	volatile unsigned int DR;  /*!< RNG data register,    Address offset: 0x08 */
} RNG_TypeDef;



/** 
 * @brief __USB_OTG_Core_register
 */
typedef struct
{
	volatile unsigned int GOTGCTL;      /*!<  USB_OTG Control and Status Register    000h*/
	volatile unsigned int GOTGINT;      /*!<  USB_OTG Interrupt Register             004h*/
	volatile unsigned int GAHBCFG;      /*!<  Core AHB Configuration Register    008h*/
	volatile unsigned int GUSBCFG;      /*!<  Core USB Configuration Register    00Ch*/
	volatile unsigned int GRSTCTL;      /*!<  Core Reset Register                010h*/
	volatile unsigned int GINTSTS;      /*!<  Core Interrupt Register            014h*/
	volatile unsigned int GINTMSK;      /*!<  Core Interrupt Mask Register       018h*/
	volatile unsigned int GRXSTSR;      /*!<  Receive Sts Q Read Register        01Ch*/
	volatile unsigned int GRXSTSP;      /*!<  Receive Sts Q Read & POP Register  020h*/
	volatile unsigned int GRXFSIZ;      /* Receive FIFO Size Register         024h*/
	volatile unsigned int DIEPTXF0_HNPTXFSIZ;   /*!<  EP0 / Non Periodic Tx FIFO Size Register 028h*/
	volatile unsigned int HNPTXSTS;     /*!<  Non Periodic Tx FIFO/Queue Sts reg 02Ch*/
	volatile unsigned int Reserved30[2];     /* Reserved                           030h*/
	volatile unsigned int GCCFG;        /* General Purpose IO Register        038h*/
	volatile unsigned int CID;          /* User ID Register                   03Ch*/
	volatile unsigned int  Reserved40[48];   /* Reserved                      040h-0FFh*/
	volatile unsigned int HPTXFSIZ; /* Host Periodic Tx FIFO Size Reg     100h*/
	volatile unsigned int DIEPTXF[0x0F];/* dev Periodic Transmit FIFO */
}
USB_OTG_GlobalTypeDef;



/** 
 * @brief __device_Registers
 */
typedef struct 
{
	volatile unsigned int DCFG;         /* dev Configuration Register   800h*/
	volatile unsigned int DCTL;         /* dev Control Register         804h*/
	volatile unsigned int DSTS;         /* dev Status Register (RO)     808h*/
	volatile unsigned int Reserved0C;           /* Reserved                     80Ch*/
	volatile unsigned int DIEPMSK;   /* dev IN Endpoint Mask         810h*/
	volatile unsigned int DOEPMSK;  /* dev OUT Endpoint Mask        814h*/
	volatile unsigned int DAINT;     /* dev All Endpoints Itr Reg    818h*/
	volatile unsigned int DAINTMSK; /* dev All Endpoints Itr Mask   81Ch*/
	volatile unsigned int  Reserved20;          /* Reserved                     820h*/
	volatile unsigned int Reserved9;       /* Reserved                     824h*/
	volatile unsigned int DVBUSDIS;    /* dev VBUS discharge Register  828h*/
	volatile unsigned int DVBUSPULSE;  /* dev VBUS Pulse Register      82Ch*/
	volatile unsigned int DTHRCTL;     /* dev thr                      830h*/
	volatile unsigned int DIEPEMPMSK; /* dev empty msk             834h*/
	volatile unsigned int DEACHINT;    /* dedicated EP interrupt       838h*/
	volatile unsigned int DEACHMSK;    /* dedicated EP msk             83Ch*/  
	volatile unsigned int Reserved40;      /* dedicated EP mask           840h*/
	volatile unsigned int DINEP1MSK;  /* dedicated EP mask           844h*/
	volatile unsigned int  Reserved44[15];      /* Reserved                 844-87Ch*/
	volatile unsigned int DOUTEP1MSK; /* dedicated EP msk            884h*/   
}
USB_OTG_DeviceTypeDef;


/** 
 * @brief N_Endpoint-Specific_Register
 */
typedef struct 
{
	volatile unsigned int DIEPCTL; /* dev IN Endpoint Control Reg 900h + (ep_num * 20h) + 00h*/
	volatile unsigned int Reserved04;             /* Reserved                       900h + (ep_num * 20h) + 04h*/
	volatile unsigned int DIEPINT; /* dev IN Endpoint Itr Reg     900h + (ep_num * 20h) + 08h*/
	volatile unsigned int Reserved0C;             /* Reserved                       900h + (ep_num * 20h) + 0Ch*/
	volatile unsigned int DIEPTSIZ; /* IN Endpoint Txfer Size   900h + (ep_num * 20h) + 10h*/
	volatile unsigned int DIEPDMA; /* IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h*/
	volatile unsigned int DTXFSTS;/*IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h*/
	volatile unsigned int Reserved18;             /* Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch*/
}
USB_OTG_INEndpointTypeDef;


/** 
 * @brief __OUT_Endpoint-Specific_Registers
 */
typedef struct 
{
	volatile unsigned int DOEPCTL;       /* dev OUT Endpoint Control Reg  B00h + (ep_num * 20h) + 00h*/
	volatile unsigned int Reserved04;         /* Reserved                      B00h + (ep_num * 20h) + 04h*/
	volatile unsigned int DOEPINT;       /* dev OUT Endpoint Itr Reg      B00h + (ep_num * 20h) + 08h*/
	volatile unsigned int Reserved0C;         /* Reserved                      B00h + (ep_num * 20h) + 0Ch*/
	volatile unsigned int DOEPTSIZ;      /* dev OUT Endpoint Txfer Size   B00h + (ep_num * 20h) + 10h*/
	volatile unsigned int DOEPDMA;       /* dev OUT Endpoint DMA Address  B00h + (ep_num * 20h) + 14h*/
	volatile unsigned int Reserved18[2];      /* Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch*/
}
USB_OTG_OUTEndpointTypeDef;


/** 
 * @brief __Host_Mode_Register_Structures
 */
typedef struct 
{
	volatile unsigned int HCFG;             /* Host Configuration Register    400h*/
	volatile unsigned int HFIR;      /* Host Frame Interval Register   404h*/
	volatile unsigned int HFNUM;         /* Host Frame Nbr/Frame Remaining 408h*/
	volatile unsigned int Reserved40C;                   /* Reserved                       40Ch*/
	volatile unsigned int HPTXSTS;   /* Host Periodic Tx FIFO/ Queue Status 410h*/
	volatile unsigned int HAINT;   /* Host All Channels Interrupt Register 414h*/
	volatile unsigned int HAINTMSK;   /* Host All Channels Interrupt Mask 418h*/
}
USB_OTG_HostTypeDef;


/** 
 * @brief __Host_Channel_Specific_Registers
 */
typedef struct
{
	volatile unsigned int HCCHAR;
	volatile unsigned int HCSPLT;
	volatile unsigned int HCINT;
	volatile unsigned int HCINTMSK;
	volatile unsigned int HCTSIZ;
	volatile unsigned int HCDMA;
	volatile unsigned int Reserved[2];
}
USB_OTG_HostChannelTypeDef;


/** 
 * @brief Peripheral_memory_map
 */
#define FLASH_BASE            ((volatile unsigned int)0x08000000) /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       ((volatile unsigned int)0x10000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            ((volatile unsigned int)0x20000000) /*!< SRAM1(112 KB) base address in the alias region                             */
#define SRAM2_BASE            ((volatile unsigned int)0x2001C000) /*!< SRAM2(16 KB) base address in the alias region                              */
#define SRAM3_BASE            ((volatile unsigned int)0x20020000) /*!< SRAM3(64 KB) base address in the alias region                              */
#define PERIPH_BASE           ((volatile unsigned int)0x40000000) /*!< Peripheral base address in the alias region                                */
#define BKPSRAM_BASE          ((volatile unsigned int)0x40024000) /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define FSMC_R_BASE           ((volatile unsigned int)0xA0000000) /*!< FSMC registers base address                                                */
#define CCMDATARAM_BB_BASE    ((volatile unsigned int)0x12000000) /*!< CCM(core coupled memory) data RAM(64 KB) base address in the bit-band region  */
#define SRAM1_BB_BASE         ((volatile unsigned int)0x22000000) /*!< SRAM1(112 KB) base address in the bit-band region                             */
#define SRAM2_BB_BASE         ((volatile unsigned int)0x2201C000) /*!< SRAM2(16 KB) base address in the bit-band region                              */
#define SRAM3_BB_BASE         ((volatile unsigned int)0x22020000) /*!< SRAM3(64 KB) base address in the bit-band region                              */
#define PERIPH_BB_BASE        ((volatile unsigned int)0x42000000) /*!< Peripheral base address in the bit-band region                                */
#define BKPSRAM_BB_BASE       ((volatile unsigned int)0x42024000) /*!< Backup SRAM(4 KB) base address in the bit-band region                         */
#define FLASH_END             ((volatile unsigned int)0x080FFFFF) /*!< FLASH end address */
#define CCMDATARAM_END        ((volatile unsigned int)0x1000FFFF) /*!< CCM data RAM end address */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE


/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2100)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x2200)
#define ADC_BASE              (APB2PERIPH_BASE + 0x2300)
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8)
#define ETH_BASE              (AHB1PERIPH_BASE + 0x8000)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000)

/*!< AHB2 peripherals */
#define DCMI_BASE             (AHB2PERIPH_BASE + 0x50000)
#define RNG_BASE              (AHB2PERIPH_BASE + 0x60800)

/*!< FSMC Bankx registers base address */
#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000)
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104)
#define FSMC_Bank2_3_R_BASE   (FSMC_R_BASE + 0x0060)
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0)

/* Debug MCU registers base address */
#define DBGMCU_BASE           ((volatile unsigned int )0xE0042000)

/*!< USB registers base address */
#define USB_OTG_HS_PERIPH_BASE               ((volatile unsigned int )0x40040000)
#define USB_OTG_FS_PERIPH_BASE               ((volatile unsigned int )0x50000000)

#define USB_OTG_GLOBAL_BASE                  ((volatile unsigned int )0x000)
#define USB_OTG_DEVICE_BASE                  ((volatile unsigned int )0x800)
#define USB_OTG_IN_ENDPOINT_BASE             ((volatile unsigned int )0x900)
#define USB_OTG_OUT_ENDPOINT_BASE            ((volatile unsigned int )0xB00)
#define USB_OTG_EP_REG_SIZE                  ((volatile unsigned int )0x20)
#define USB_OTG_HOST_BASE                    ((volatile unsigned int )0x400)
#define USB_OTG_HOST_PORT_BASE               ((volatile unsigned int )0x440)
#define USB_OTG_HOST_CHANNEL_BASE            ((volatile unsigned int )0x500)
#define USB_OTG_HOST_CHANNEL_SIZE            ((volatile unsigned int )0x20)
#define USB_OTG_PCGCCTL_BASE                 ((volatile unsigned int )0xE00)
#define USB_OTG_FIFO_BASE                    ((volatile unsigned int )0x1000)
#define USB_OTG_FIFO_SIZE                    ((volatile unsigned int )0x1000)

/**
 * @}
 */

/** @addtogroup Peripheral_declaration
 * @{
 */  
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART6              ((USART_TypeDef *) USART6_BASE)
#define ADC                 ((ADC_Common_TypeDef *) ADC_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE) 
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
#define ETH                 ((ETH_TypeDef *) ETH_BASE)  
#define DCMI                ((DCMI_TypeDef *) DCMI_BASE)
#define RNG                 ((RNG_TypeDef *) RNG_BASE)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2_3        ((FSMC_Bank2_3_TypeDef *) FSMC_Bank2_3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

#define USB_OTG_FS          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)
#define USB_OTG_HS          ((USB_OTG_GlobalTypeDef *) USB_OTG_HS_PERIPH_BASE)

/**
 * @}
 */

/** @addtogroup Exported_constants
 * @{
 */

/** @addtogroup Peripheral_Registers_Bits_Definition
 * @{
 */

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((volatile unsigned int)0x00000001)       /*!<Analog watchdog flag */
#define  ADC_SR_EOC                          ((volatile unsigned int)0x00000002)       /*!<End of conversion */
#define  ADC_SR_JEOC                         ((volatile unsigned int)0x00000004)       /*!<Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((volatile unsigned int)0x00000008)       /*!<Injected channel Start flag */
#define  ADC_SR_STRT                         ((volatile unsigned int)0x00000010)       /*!<Regular channel Start flag */
#define  ADC_SR_OVR                          ((volatile unsigned int)0x00000020)       /*!<Overrun flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((volatile unsigned int)0x0000001F)        /*!<AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  ADC_CR1_EOCIE                       ((volatile unsigned int)0x00000020)        /*!<Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((volatile unsigned int)0x00000040)        /*!<AAnalog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((volatile unsigned int)0x00000080)        /*!<Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((volatile unsigned int)0x00000100)        /*!<Scan mode */
#define  ADC_CR1_AWDSGL                      ((volatile unsigned int)0x00000200)        /*!<Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((volatile unsigned int)0x00000400)        /*!<Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((volatile unsigned int)0x00000800)        /*!<Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((volatile unsigned int)0x00001000)        /*!<Discontinuous mode on injected channels */
#define  ADC_CR1_DISCNUM                     ((volatile unsigned int)0x0000E000)        /*!<DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((volatile unsigned int)0x00002000)        /*!<Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((volatile unsigned int)0x00004000)        /*!<Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((volatile unsigned int)0x00008000)        /*!<Bit 2 */
#define  ADC_CR1_JAWDEN                      ((volatile unsigned int)0x00400000)        /*!<Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((volatile unsigned int)0x00800000)        /*!<Analog watchdog enable on regular channels */
#define  ADC_CR1_RES                         ((volatile unsigned int)0x03000000)        /*!<RES[2:0] bits (Resolution) */
#define  ADC_CR1_RES_0                       ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  ADC_CR1_RES_1                       ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  ADC_CR1_OVRIE                       ((volatile unsigned int)0x04000000)         /*!<overrun interrupt enable */

/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((volatile unsigned int)0x00000001)        /*!<A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((volatile unsigned int)0x00000002)        /*!<Continuous Conversion */
#define  ADC_CR2_DMA                         ((volatile unsigned int)0x00000100)        /*!<Direct Memory access mode */
#define  ADC_CR2_DDS                         ((volatile unsigned int)0x00000200)        /*!<DMA disable selection (Single ADC) */
#define  ADC_CR2_EOCS                        ((volatile unsigned int)0x00000400)        /*!<End of conversion selection */
#define  ADC_CR2_ALIGN                       ((volatile unsigned int)0x00000800)        /*!<Data Alignment */
#define  ADC_CR2_JEXTSEL                     ((volatile unsigned int)0x000F0000)        /*!<JEXTSEL[3:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  ADC_CR2_JEXTSEL_3                   ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  ADC_CR2_JEXTEN                      ((volatile unsigned int)0x00300000)        /*!<JEXTEN[1:0] bits (External Trigger Conversion mode for injected channelsp) */
#define  ADC_CR2_JEXTEN_0                    ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  ADC_CR2_JEXTEN_1                    ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  ADC_CR2_JSWSTART                    ((volatile unsigned int)0x00400000)        /*!<Start Conversion of injected channels */
#define  ADC_CR2_EXTSEL                      ((volatile unsigned int)0x0F000000)        /*!<EXTSEL[3:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  ADC_CR2_EXTSEL_3                    ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  ADC_CR2_EXTEN                       ((volatile unsigned int)0x30000000)        /*!<EXTEN[1:0] bits (External Trigger Conversion mode for regular channelsp) */
#define  ADC_CR2_EXTEN_0                     ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  ADC_CR2_EXTEN_1                     ((volatile unsigned int)0x20000000)        /*!<Bit 1 */
#define  ADC_CR2_SWSTART                     ((volatile unsigned int)0x40000000)        /*!<Start Conversion of regular channels */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((volatile unsigned int)0x00000007)        /*!<SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP11                     ((volatile unsigned int)0x00000038)        /*!<SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((volatile unsigned int)0x00000008)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((volatile unsigned int)0x00000010)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((volatile unsigned int)0x00000020)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP12                     ((volatile unsigned int)0x000001C0)        /*!<SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((volatile unsigned int)0x00000040)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((volatile unsigned int)0x00000080)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((volatile unsigned int)0x00000100)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP13                     ((volatile unsigned int)0x00000E00)        /*!<SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((volatile unsigned int)0x00000200)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((volatile unsigned int)0x00000400)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((volatile unsigned int)0x00000800)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP14                     ((volatile unsigned int)0x00007000)        /*!<SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((volatile unsigned int)0x00001000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((volatile unsigned int)0x00002000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((volatile unsigned int)0x00004000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP15                     ((volatile unsigned int)0x00038000)        /*!<SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((volatile unsigned int)0x00008000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((volatile unsigned int)0x00010000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((volatile unsigned int)0x00020000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP16                     ((volatile unsigned int)0x001C0000)        /*!<SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((volatile unsigned int)0x00040000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((volatile unsigned int)0x00080000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((volatile unsigned int)0x00100000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP17                     ((volatile unsigned int)0x00E00000)        /*!<SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((volatile unsigned int)0x00200000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((volatile unsigned int)0x00400000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((volatile unsigned int)0x00800000)        /*!<Bit 2 */
#define  ADC_SMPR1_SMP18                     ((volatile unsigned int)0x07000000)        /*!<SMP18[2:0] bits (Channel 18 Sample time selection) */
#define  ADC_SMPR1_SMP18_0                   ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  ADC_SMPR1_SMP18_1                   ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  ADC_SMPR1_SMP18_2                   ((volatile unsigned int)0x04000000)        /*!<Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((volatile unsigned int)0x00000007)        /*!<SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP1                      ((volatile unsigned int)0x00000038)        /*!<SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((volatile unsigned int)0x00000008)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP1_1                    ((volatile unsigned int)0x00000010)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP1_2                    ((volatile unsigned int)0x00000020)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP2                      ((volatile unsigned int)0x000001C0)        /*!<SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPR2_SMP2_0                    ((volatile unsigned int)0x00000040)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP2_1                    ((volatile unsigned int)0x00000080)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP2_2                    ((volatile unsigned int)0x00000100)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP3                      ((volatile unsigned int)0x00000E00)        /*!<SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPR2_SMP3_0                    ((volatile unsigned int)0x00000200)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP3_1                    ((volatile unsigned int)0x00000400)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP3_2                    ((volatile unsigned int)0x00000800)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP4                      ((volatile unsigned int)0x00007000)        /*!<SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPR2_SMP4_0                    ((volatile unsigned int)0x00001000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP4_1                    ((volatile unsigned int)0x00002000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP4_2                    ((volatile unsigned int)0x00004000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP5                      ((volatile unsigned int)0x00038000)        /*!<SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPR2_SMP5_0                    ((volatile unsigned int)0x00008000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP5_1                    ((volatile unsigned int)0x00010000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP5_2                    ((volatile unsigned int)0x00020000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP6                      ((volatile unsigned int)0x001C0000)        /*!<SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPR2_SMP6_0                    ((volatile unsigned int)0x00040000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP6_1                    ((volatile unsigned int)0x00080000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP6_2                    ((volatile unsigned int)0x00100000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP7                      ((volatile unsigned int)0x00E00000)        /*!<SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPR2_SMP7_0                    ((volatile unsigned int)0x00200000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP7_1                    ((volatile unsigned int)0x00400000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP7_2                    ((volatile unsigned int)0x00800000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP8                      ((volatile unsigned int)0x07000000)        /*!<SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPR2_SMP8_0                    ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP8_1                    ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP8_2                    ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  ADC_SMPR2_SMP9                      ((volatile unsigned int)0x38000000)        /*!<SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPR2_SMP9_0                    ((volatile unsigned int)0x08000000)        /*!<Bit 0 */
#define  ADC_SMPR2_SMP9_1                    ((volatile unsigned int)0x10000000)        /*!<Bit 1 */
#define  ADC_SMPR2_SMP9_2                    ((volatile unsigned int)0x20000000)        /*!<Bit 2 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define  ADC_JOFR1_JOFFSET1                  ((volatile unsigned int)0x0FFF)            /*!<Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define  ADC_JOFR2_JOFFSET2                  ((volatile unsigned int)0x0FFF)            /*!<Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define  ADC_JOFR3_JOFFSET3                  ((volatile unsigned int)0x0FFF)            /*!<Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define  ADC_JOFR4_JOFFSET4                  ((volatile unsigned int)0x0FFF)            /*!<Data offset for injected channel 4 */

/*******************  Bit definition for ADC_HTR register  ********************/
#define  ADC_HTR_HT                          ((volatile unsigned int)0x0FFF)            /*!<Analog watchdog high threshold */

/*******************  Bit definition for ADC_LTR register  ********************/
#define  ADC_LTR_LT                          ((volatile unsigned int)0x0FFF)            /*!<Analog watchdog low threshold */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define  ADC_SQR1_SQ13                       ((volatile unsigned int)0x0000001F)        /*!<SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_SQR1_SQ13_0                     ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_SQR1_SQ13_1                     ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_SQR1_SQ13_2                     ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_SQR1_SQ13_3                     ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  ADC_SQR1_SQ13_4                     ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  ADC_SQR1_SQ14                       ((volatile unsigned int)0x000003E0)        /*!<SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_SQR1_SQ14_0                     ((volatile unsigned int)0x00000020)        /*!<Bit 0 */
#define  ADC_SQR1_SQ14_1                     ((volatile unsigned int)0x00000040)        /*!<Bit 1 */
#define  ADC_SQR1_SQ14_2                     ((volatile unsigned int)0x00000080)        /*!<Bit 2 */
#define  ADC_SQR1_SQ14_3                     ((volatile unsigned int)0x00000100)        /*!<Bit 3 */
#define  ADC_SQR1_SQ14_4                     ((volatile unsigned int)0x00000200)        /*!<Bit 4 */
#define  ADC_SQR1_SQ15                       ((volatile unsigned int)0x00007C00)        /*!<SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_SQR1_SQ15_0                     ((volatile unsigned int)0x00000400)        /*!<Bit 0 */
#define  ADC_SQR1_SQ15_1                     ((volatile unsigned int)0x00000800)        /*!<Bit 1 */
#define  ADC_SQR1_SQ15_2                     ((volatile unsigned int)0x00001000)        /*!<Bit 2 */
#define  ADC_SQR1_SQ15_3                     ((volatile unsigned int)0x00002000)        /*!<Bit 3 */
#define  ADC_SQR1_SQ15_4                     ((volatile unsigned int)0x00004000)        /*!<Bit 4 */
#define  ADC_SQR1_SQ16                       ((volatile unsigned int)0x000F8000)        /*!<SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_SQR1_SQ16_0                     ((volatile unsigned int)0x00008000)        /*!<Bit 0 */
#define  ADC_SQR1_SQ16_1                     ((volatile unsigned int)0x00010000)        /*!<Bit 1 */
#define  ADC_SQR1_SQ16_2                     ((volatile unsigned int)0x00020000)        /*!<Bit 2 */
#define  ADC_SQR1_SQ16_3                     ((volatile unsigned int)0x00040000)        /*!<Bit 3 */
#define  ADC_SQR1_SQ16_4                     ((volatile unsigned int)0x00080000)        /*!<Bit 4 */
#define  ADC_SQR1_L                          ((volatile unsigned int)0x00F00000)        /*!<L[3:0] bits (Regular channel sequence length) */
#define  ADC_SQR1_L_0                        ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  ADC_SQR1_L_1                        ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  ADC_SQR1_L_2                        ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  ADC_SQR1_L_3                        ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define  ADC_SQR2_SQ7                        ((volatile unsigned int)0x0000001F)        /*!<SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_SQR2_SQ7_0                      ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_SQR2_SQ7_1                      ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_SQR2_SQ7_2                      ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_SQR2_SQ7_3                      ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  ADC_SQR2_SQ7_4                      ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  ADC_SQR2_SQ8                        ((volatile unsigned int)0x000003E0)        /*!<SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_SQR2_SQ8_0                      ((volatile unsigned int)0x00000020)        /*!<Bit 0 */
#define  ADC_SQR2_SQ8_1                      ((volatile unsigned int)0x00000040)        /*!<Bit 1 */
#define  ADC_SQR2_SQ8_2                      ((volatile unsigned int)0x00000080)        /*!<Bit 2 */
#define  ADC_SQR2_SQ8_3                      ((volatile unsigned int)0x00000100)        /*!<Bit 3 */
#define  ADC_SQR2_SQ8_4                      ((volatile unsigned int)0x00000200)        /*!<Bit 4 */
#define  ADC_SQR2_SQ9                        ((volatile unsigned int)0x00007C00)        /*!<SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_SQR2_SQ9_0                      ((volatile unsigned int)0x00000400)        /*!<Bit 0 */
#define  ADC_SQR2_SQ9_1                      ((volatile unsigned int)0x00000800)        /*!<Bit 1 */
#define  ADC_SQR2_SQ9_2                      ((volatile unsigned int)0x00001000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ9_3                      ((volatile unsigned int)0x00002000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ9_4                      ((volatile unsigned int)0x00004000)        /*!<Bit 4 */
#define  ADC_SQR2_SQ10                       ((volatile unsigned int)0x000F8000)        /*!<SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_SQR2_SQ10_0                     ((volatile unsigned int)0x00008000)        /*!<Bit 0 */
#define  ADC_SQR2_SQ10_1                     ((volatile unsigned int)0x00010000)        /*!<Bit 1 */
#define  ADC_SQR2_SQ10_2                     ((volatile unsigned int)0x00020000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ10_3                     ((volatile unsigned int)0x00040000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ10_4                     ((volatile unsigned int)0x00080000)        /*!<Bit 4 */
#define  ADC_SQR2_SQ11                       ((volatile unsigned int)0x01F00000)        /*!<SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_SQR2_SQ11_0                     ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  ADC_SQR2_SQ11_1                     ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  ADC_SQR2_SQ11_2                     ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ11_3                     ((volatile unsigned int)0x00800000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ11_4                     ((volatile unsigned int)0x01000000)        /*!<Bit 4 */
#define  ADC_SQR2_SQ12                       ((volatile unsigned int)0x3E000000)        /*!<SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_SQR2_SQ12_0                     ((volatile unsigned int)0x02000000)        /*!<Bit 0 */
#define  ADC_SQR2_SQ12_1                     ((volatile unsigned int)0x04000000)        /*!<Bit 1 */
#define  ADC_SQR2_SQ12_2                     ((volatile unsigned int)0x08000000)        /*!<Bit 2 */
#define  ADC_SQR2_SQ12_3                     ((volatile unsigned int)0x10000000)        /*!<Bit 3 */
#define  ADC_SQR2_SQ12_4                     ((volatile unsigned int)0x20000000)        /*!<Bit 4 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define  ADC_SQR3_SQ1                        ((volatile unsigned int)0x0000001F)        /*!<SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_SQR3_SQ1_0                      ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_SQR3_SQ1_1                      ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_SQR3_SQ1_2                      ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_SQR3_SQ1_3                      ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  ADC_SQR3_SQ1_4                      ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  ADC_SQR3_SQ2                        ((volatile unsigned int)0x000003E0)        /*!<SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_SQR3_SQ2_0                      ((volatile unsigned int)0x00000020)        /*!<Bit 0 */
#define  ADC_SQR3_SQ2_1                      ((volatile unsigned int)0x00000040)        /*!<Bit 1 */
#define  ADC_SQR3_SQ2_2                      ((volatile unsigned int)0x00000080)        /*!<Bit 2 */
#define  ADC_SQR3_SQ2_3                      ((volatile unsigned int)0x00000100)        /*!<Bit 3 */
#define  ADC_SQR3_SQ2_4                      ((volatile unsigned int)0x00000200)        /*!<Bit 4 */
#define  ADC_SQR3_SQ3                        ((volatile unsigned int)0x00007C00)        /*!<SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_SQR3_SQ3_0                      ((volatile unsigned int)0x00000400)        /*!<Bit 0 */
#define  ADC_SQR3_SQ3_1                      ((volatile unsigned int)0x00000800)        /*!<Bit 1 */
#define  ADC_SQR3_SQ3_2                      ((volatile unsigned int)0x00001000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ3_3                      ((volatile unsigned int)0x00002000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ3_4                      ((volatile unsigned int)0x00004000)        /*!<Bit 4 */
#define  ADC_SQR3_SQ4                        ((volatile unsigned int)0x000F8000)        /*!<SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_SQR3_SQ4_0                      ((volatile unsigned int)0x00008000)        /*!<Bit 0 */
#define  ADC_SQR3_SQ4_1                      ((volatile unsigned int)0x00010000)        /*!<Bit 1 */
#define  ADC_SQR3_SQ4_2                      ((volatile unsigned int)0x00020000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ4_3                      ((volatile unsigned int)0x00040000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ4_4                      ((volatile unsigned int)0x00080000)        /*!<Bit 4 */
#define  ADC_SQR3_SQ5                        ((volatile unsigned int)0x01F00000)        /*!<SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_SQR3_SQ5_0                      ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  ADC_SQR3_SQ5_1                      ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  ADC_SQR3_SQ5_2                      ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ5_3                      ((volatile unsigned int)0x00800000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ5_4                      ((volatile unsigned int)0x01000000)        /*!<Bit 4 */
#define  ADC_SQR3_SQ6                        ((volatile unsigned int)0x3E000000)        /*!<SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_SQR3_SQ6_0                      ((volatile unsigned int)0x02000000)        /*!<Bit 0 */
#define  ADC_SQR3_SQ6_1                      ((volatile unsigned int)0x04000000)        /*!<Bit 1 */
#define  ADC_SQR3_SQ6_2                      ((volatile unsigned int)0x08000000)        /*!<Bit 2 */
#define  ADC_SQR3_SQ6_3                      ((volatile unsigned int)0x10000000)        /*!<Bit 3 */
#define  ADC_SQR3_SQ6_4                      ((volatile unsigned int)0x20000000)        /*!<Bit 4 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define  ADC_JSQR_JSQ1                       ((volatile unsigned int)0x0000001F)        /*!<JSQ1[4:0] bits (1st conversion in injected sequence) */  
#define  ADC_JSQR_JSQ1_0                     ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ1_1                     ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ1_2                     ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ1_3                     ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ1_4                     ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ2                       ((volatile unsigned int)0x000003E0)        /*!<JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQR_JSQ2_0                     ((volatile unsigned int)0x00000020)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ2_1                     ((volatile unsigned int)0x00000040)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ2_2                     ((volatile unsigned int)0x00000080)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ2_3                     ((volatile unsigned int)0x00000100)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ2_4                     ((volatile unsigned int)0x00000200)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ3                       ((volatile unsigned int)0x00007C00)        /*!<JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQR_JSQ3_0                     ((volatile unsigned int)0x00000400)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ3_1                     ((volatile unsigned int)0x00000800)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ3_2                     ((volatile unsigned int)0x00001000)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ3_3                     ((volatile unsigned int)0x00002000)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ3_4                     ((volatile unsigned int)0x00004000)        /*!<Bit 4 */
#define  ADC_JSQR_JSQ4                       ((volatile unsigned int)0x000F8000)        /*!<JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQR_JSQ4_0                     ((volatile unsigned int)0x00008000)        /*!<Bit 0 */
#define  ADC_JSQR_JSQ4_1                     ((volatile unsigned int)0x00010000)        /*!<Bit 1 */
#define  ADC_JSQR_JSQ4_2                     ((volatile unsigned int)0x00020000)        /*!<Bit 2 */
#define  ADC_JSQR_JSQ4_3                     ((volatile unsigned int)0x00040000)        /*!<Bit 3 */
#define  ADC_JSQR_JSQ4_4                     ((volatile unsigned int)0x00080000)        /*!<Bit 4 */
#define  ADC_JSQR_JL                         ((volatile unsigned int)0x00300000)        /*!<JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQR_JL_0                       ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  ADC_JSQR_JL_1                       ((volatile unsigned int)0x00200000)        /*!<Bit 1 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define  ADC_JDR1_JDATA                      ((volatile unsigned int)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define  ADC_JDR2_JDATA                      ((volatile unsigned int)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define  ADC_JDR3_JDATA                      ((volatile unsigned int)0xFFFF)            /*!<Injected data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define  ADC_JDR4_JDATA                      ((volatile unsigned int)0xFFFF)            /*!<Injected data */

/********************  Bit definition for ADC_DR register  ********************/
#define  ADC_DR_DATA                         ((volatile unsigned int)0x0000FFFF)        /*!<Regular data */
#define  ADC_DR_ADC2DATA                     ((volatile unsigned int)0xFFFF0000)        /*!<ADC2 data */

/*******************  Bit definition for ADC_CSR register  ********************/
#define  ADC_CSR_AWD1                        ((volatile unsigned int)0x00000001)        /*!<ADC1 Analog watchdog flag */
#define  ADC_CSR_EOC1                        ((volatile unsigned int)0x00000002)        /*!<ADC1 End of conversion */
#define  ADC_CSR_JEOC1                       ((volatile unsigned int)0x00000004)        /*!<ADC1 Injected channel end of conversion */
#define  ADC_CSR_JSTRT1                      ((volatile unsigned int)0x00000008)        /*!<ADC1 Injected channel Start flag */
#define  ADC_CSR_STRT1                       ((volatile unsigned int)0x00000010)        /*!<ADC1 Regular channel Start flag */
#define  ADC_CSR_DOVR1                       ((volatile unsigned int)0x00000020)        /*!<ADC1 DMA overrun  flag */
#define  ADC_CSR_AWD2                        ((volatile unsigned int)0x00000100)        /*!<ADC2 Analog watchdog flag */
#define  ADC_CSR_EOC2                        ((volatile unsigned int)0x00000200)        /*!<ADC2 End of conversion */
#define  ADC_CSR_JEOC2                       ((volatile unsigned int)0x00000400)        /*!<ADC2 Injected channel end of conversion */
#define  ADC_CSR_JSTRT2                      ((volatile unsigned int)0x00000800)        /*!<ADC2 Injected channel Start flag */
#define  ADC_CSR_STRT2                       ((volatile unsigned int)0x00001000)        /*!<ADC2 Regular channel Start flag */
#define  ADC_CSR_DOVR2                       ((volatile unsigned int)0x00002000)        /*!<ADC2 DMA overrun  flag */
#define  ADC_CSR_AWD3                        ((volatile unsigned int)0x00010000)        /*!<ADC3 Analog watchdog flag */
#define  ADC_CSR_EOC3                        ((volatile unsigned int)0x00020000)        /*!<ADC3 End of conversion */
#define  ADC_CSR_JEOC3                       ((volatile unsigned int)0x00040000)        /*!<ADC3 Injected channel end of conversion */
#define  ADC_CSR_JSTRT3                      ((volatile unsigned int)0x00080000)        /*!<ADC3 Injected channel Start flag */
#define  ADC_CSR_STRT3                       ((volatile unsigned int)0x00100000)        /*!<ADC3 Regular channel Start flag */
#define  ADC_CSR_DOVR3                       ((volatile unsigned int)0x00200000)        /*!<ADC3 DMA overrun  flag */

/*******************  Bit definition for ADC_CCR register  ********************/
#define  ADC_CCR_MULTI                       ((volatile unsigned int)0x0000001F)        /*!<MULTI[4:0] bits (Multi-ADC mode selection) */  
#define  ADC_CCR_MULTI_0                     ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  ADC_CCR_MULTI_1                     ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  ADC_CCR_MULTI_2                     ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  ADC_CCR_MULTI_3                     ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  ADC_CCR_MULTI_4                     ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  ADC_CCR_DELAY                       ((volatile unsigned int)0x00000F00)        /*!<DELAY[3:0] bits (Delay between 2 sampling phases) */  
#define  ADC_CCR_DELAY_0                     ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  ADC_CCR_DELAY_1                     ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  ADC_CCR_DELAY_2                     ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  ADC_CCR_DELAY_3                     ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  ADC_CCR_DDS                         ((volatile unsigned int)0x00002000)        /*!<DMA disable selection (Multi-ADC mode) */
#define  ADC_CCR_DMA                         ((volatile unsigned int)0x0000C000)        /*!<DMA[1:0] bits (Direct Memory Access mode for multimode) */  
#define  ADC_CCR_DMA_0                       ((volatile unsigned int)0x00004000)        /*!<Bit 0 */
#define  ADC_CCR_DMA_1                       ((volatile unsigned int)0x00008000)        /*!<Bit 1 */
#define  ADC_CCR_ADCPRE                      ((volatile unsigned int)0x00030000)        /*!<ADCPRE[1:0] bits (ADC prescaler) */  
#define  ADC_CCR_ADCPRE_0                    ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  ADC_CCR_ADCPRE_1                    ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  ADC_CCR_VBATE                       ((volatile unsigned int)0x00400000)        /*!<VBAT Enable */
#define  ADC_CCR_TSVREFE                     ((volatile unsigned int)0x00800000)        /*!<Temperature Sensor and VREFINT Enable */

/*******************  Bit definition for ADC_CDR register  ********************/
#define  ADC_CDR_DATA1                      ((volatile unsigned int)0x0000FFFF)         /*!<1st data of a pair of regular conversions */
#define  ADC_CDR_DATA2                      ((volatile unsigned int)0xFFFF0000)         /*!<2nd data of a pair of regular conversions */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/
/*!<CAN control and status registers */
/*******************  Bit definition for CAN_MCR register  ********************/
#define  CAN_MCR_INRQ                        ((volatile unsigned int)0x00000001)        /*!<Initialization Request */
#define  CAN_MCR_SLEEP                       ((volatile unsigned int)0x00000002)        /*!<Sleep Mode Request */
#define  CAN_MCR_TXFP                        ((volatile unsigned int)0x00000004)        /*!<Transmit FIFO Priority */
#define  CAN_MCR_RFLM                        ((volatile unsigned int)0x00000008)        /*!<Receive FIFO Locked Mode */
#define  CAN_MCR_NART                        ((volatile unsigned int)0x00000010)        /*!<No Automatic Retransmission */
#define  CAN_MCR_AWUM                        ((volatile unsigned int)0x00000020)        /*!<Automatic Wakeup Mode */
#define  CAN_MCR_ABOM                        ((volatile unsigned int)0x00000040)        /*!<Automatic Bus-Off Management */
#define  CAN_MCR_TTCM                        ((volatile unsigned int)0x00000080)        /*!<Time Triggered Communication Mode */
#define  CAN_MCR_RESET                       ((volatile unsigned int)0x00008000)        /*!<bxCAN software master reset */
#define  CAN_MCR_DBF                         ((volatile unsigned int)0x00010000)        /*!<bxCAN Debug freeze */
/*******************  Bit definition for CAN_MSR register  ********************/
#define  CAN_MSR_INAK                        ((volatile unsigned int)0x0001)            /*!<Initialization Acknowledge */
#define  CAN_MSR_SLAK                        ((volatile unsigned int)0x0002)            /*!<Sleep Acknowledge */
#define  CAN_MSR_ERRI                        ((volatile unsigned int)0x0004)            /*!<Error Interrupt */
#define  CAN_MSR_WKUI                        ((volatile unsigned int)0x0008)            /*!<Wakeup Interrupt */
#define  CAN_MSR_SLAKI                       ((volatile unsigned int)0x0010)            /*!<Sleep Acknowledge Interrupt */
#define  CAN_MSR_TXM                         ((volatile unsigned int)0x0100)            /*!<Transmit Mode */
#define  CAN_MSR_RXM                         ((volatile unsigned int)0x0200)            /*!<Receive Mode */
#define  CAN_MSR_SAMP                        ((volatile unsigned int)0x0400)            /*!<Last Sample Point */
#define  CAN_MSR_RX                          ((volatile unsigned int)0x0800)            /*!<CAN Rx Signal */

/*******************  Bit definition for CAN_TSR register  ********************/
#define  CAN_TSR_RQCP0                       ((volatile unsigned int)0x00000001)        /*!<Request Completed Mailbox0 */
#define  CAN_TSR_TXOK0                       ((volatile unsigned int)0x00000002)        /*!<Transmission OK of Mailbox0 */
#define  CAN_TSR_ALST0                       ((volatile unsigned int)0x00000004)        /*!<Arbitration Lost for Mailbox0 */
#define  CAN_TSR_TERR0                       ((volatile unsigned int)0x00000008)        /*!<Transmission Error of Mailbox0 */
#define  CAN_TSR_ABRQ0                       ((volatile unsigned int)0x00000080)        /*!<Abort Request for Mailbox0 */
#define  CAN_TSR_RQCP1                       ((volatile unsigned int)0x00000100)        /*!<Request Completed Mailbox1 */
#define  CAN_TSR_TXOK1                       ((volatile unsigned int)0x00000200)        /*!<Transmission OK of Mailbox1 */
#define  CAN_TSR_ALST1                       ((volatile unsigned int)0x00000400)        /*!<Arbitration Lost for Mailbox1 */
#define  CAN_TSR_TERR1                       ((volatile unsigned int)0x00000800)        /*!<Transmission Error of Mailbox1 */
#define  CAN_TSR_ABRQ1                       ((volatile unsigned int)0x00008000)        /*!<Abort Request for Mailbox 1 */
#define  CAN_TSR_RQCP2                       ((volatile unsigned int)0x00010000)        /*!<Request Completed Mailbox2 */
#define  CAN_TSR_TXOK2                       ((volatile unsigned int)0x00020000)        /*!<Transmission OK of Mailbox 2 */
#define  CAN_TSR_ALST2                       ((volatile unsigned int)0x00040000)        /*!<Arbitration Lost for mailbox 2 */
#define  CAN_TSR_TERR2                       ((volatile unsigned int)0x00080000)        /*!<Transmission Error of Mailbox 2 */
#define  CAN_TSR_ABRQ2                       ((volatile unsigned int)0x00800000)        /*!<Abort Request for Mailbox 2 */
#define  CAN_TSR_CODE                        ((volatile unsigned int)0x03000000)        /*!<Mailbox Code */

#define  CAN_TSR_TME                         ((volatile unsigned int)0x1C000000)        /*!<TME[2:0] bits */
#define  CAN_TSR_TME0                        ((volatile unsigned int)0x04000000)        /*!<Transmit Mailbox 0 Empty */
#define  CAN_TSR_TME1                        ((volatile unsigned int)0x08000000)        /*!<Transmit Mailbox 1 Empty */
#define  CAN_TSR_TME2                        ((volatile unsigned int)0x10000000)        /*!<Transmit Mailbox 2 Empty */

#define  CAN_TSR_LOW                         ((volatile unsigned int)0xE0000000)        /*!<LOW[2:0] bits */
#define  CAN_TSR_LOW0                        ((volatile unsigned int)0x20000000)        /*!<Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSR_LOW1                        ((volatile unsigned int)0x40000000)        /*!<Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSR_LOW2                        ((volatile unsigned int)0x80000000)        /*!<Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0R register  *******************/
#define  CAN_RF0R_FMP0                       ((volatile unsigned int)0x03)               /*!<FIFO 0 Message Pending */
#define  CAN_RF0R_FULL0                      ((volatile unsigned int)0x08)               /*!<FIFO 0 Full */
#define  CAN_RF0R_FOVR0                      ((volatile unsigned int)0x10)               /*!<FIFO 0 Overrun */
#define  CAN_RF0R_RFOM0                      ((volatile unsigned int)0x20)               /*!<Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1R register  *******************/
#define  CAN_RF1R_FMP1                       ((volatile unsigned int)0x03)               /*!<FIFO 1 Message Pending */
#define  CAN_RF1R_FULL1                      ((volatile unsigned int)0x08)               /*!<FIFO 1 Full */
#define  CAN_RF1R_FOVR1                      ((volatile unsigned int)0x10)               /*!<FIFO 1 Overrun */
#define  CAN_RF1R_RFOM1                      ((volatile unsigned int)0x20)               /*!<Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_IER register  *******************/
#define  CAN_IER_TMEIE                       ((volatile unsigned int)0x00000001)        /*!<Transmit Mailbox Empty Interrupt Enable */
#define  CAN_IER_FMPIE0                      ((volatile unsigned int)0x00000002)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE0                       ((volatile unsigned int)0x00000004)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE0                      ((volatile unsigned int)0x00000008)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_FMPIE1                      ((volatile unsigned int)0x00000010)        /*!<FIFO Message Pending Interrupt Enable */
#define  CAN_IER_FFIE1                       ((volatile unsigned int)0x00000020)        /*!<FIFO Full Interrupt Enable */
#define  CAN_IER_FOVIE1                      ((volatile unsigned int)0x00000040)        /*!<FIFO Overrun Interrupt Enable */
#define  CAN_IER_EWGIE                       ((volatile unsigned int)0x00000100)        /*!<Error Warning Interrupt Enable */
#define  CAN_IER_EPVIE                       ((volatile unsigned int)0x00000200)        /*!<Error Passive Interrupt Enable */
#define  CAN_IER_BOFIE                       ((volatile unsigned int)0x00000400)        /*!<Bus-Off Interrupt Enable */
#define  CAN_IER_LECIE                       ((volatile unsigned int)0x00000800)        /*!<Last Error Code Interrupt Enable */
#define  CAN_IER_ERRIE                       ((volatile unsigned int)0x00008000)        /*!<Error Interrupt Enable */
#define  CAN_IER_WKUIE                       ((volatile unsigned int)0x00010000)        /*!<Wakeup Interrupt Enable */
#define  CAN_IER_SLKIE                       ((volatile unsigned int)0x00020000)        /*!<Sleep Interrupt Enable */
#define  CAN_IER_EWGIE                       ((volatile unsigned int)0x00000100)        /*!<Error warning interrupt enable */
#define  CAN_IER_EPVIE                       ((volatile unsigned int)0x00000200)        /*!<Error passive interrupt enable */
#define  CAN_IER_BOFIE                       ((volatile unsigned int)0x00000400)        /*!<Bus-off interrupt enable */
#define  CAN_IER_LECIE                       ((volatile unsigned int)0x00000800)        /*!<Last error code interrupt enable */
#define  CAN_IER_ERRIE                       ((volatile unsigned int)0x00008000)        /*!<Error interrupt enable */


/********************  Bit definition for CAN_ESR register  *******************/
#define  CAN_ESR_EWGF                        ((volatile unsigned int)0x00000001)        /*!<Error Warning Flag */
#define  CAN_ESR_EPVF                        ((volatile unsigned int)0x00000002)        /*!<Error Passive Flag */
#define  CAN_ESR_BOFF                        ((volatile unsigned int)0x00000004)        /*!<Bus-Off Flag */

#define  CAN_ESR_LEC                         ((volatile unsigned int)0x00000070)        /*!<LEC[2:0] bits (Last Error Code) */
#define  CAN_ESR_LEC_0                       ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  CAN_ESR_LEC_1                       ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  CAN_ESR_LEC_2                       ((volatile unsigned int)0x00000040)        /*!<Bit 2 */

#define  CAN_ESR_TEC                         ((volatile unsigned int)0x00FF0000)        /*!<Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESR_REC                         ((volatile unsigned int)0xFF000000)        /*!<Receive Error Counter */

/*******************  Bit definition for CAN_BTR register  ********************/
#define  CAN_BTR_BRP                         ((volatile unsigned int)0x000003FF)        /*!<Baud Rate Prescaler */
#define  CAN_BTR_TS1                         ((volatile unsigned int)0x000F0000)        /*!<Time Segment 1 */
#define  CAN_BTR_TS1_0                       ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  CAN_BTR_TS1_1                       ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  CAN_BTR_TS1_2                       ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  CAN_BTR_TS1_3                       ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  CAN_BTR_TS2                         ((volatile unsigned int)0x00700000)        /*!<Time Segment 2 */
#define  CAN_BTR_TS2_0                       ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  CAN_BTR_TS2_1                       ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  CAN_BTR_TS2_2                       ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  CAN_BTR_SJW                         ((volatile unsigned int)0x03000000)        /*!<Resynchronization Jump Width */
#define  CAN_BTR_SJW_0                       ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  CAN_BTR_SJW_1                       ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  CAN_BTR_LBKM                        ((volatile unsigned int)0x40000000)        /*!<Loop Back Mode (Debug) */
#define  CAN_BTR_SILM                        ((volatile unsigned int)0x80000000)        /*!<Silent Mode */


/*!<Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TI0R_TXRQ                       ((volatile unsigned int)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI0R_RTR                        ((volatile unsigned int)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI0R_IDE                        ((volatile unsigned int)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI0R_EXID                       ((volatile unsigned int)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_TI0R_STID                       ((volatile unsigned int)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0R register  *******************/
#define  CAN_TDT0R_DLC                       ((volatile unsigned int)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT0R_TGT                       ((volatile unsigned int)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT0R_TIME                      ((volatile unsigned int)0xFFFF0000)        /*!<Message Time Stamp */

/******************  Bit definition for CAN_TDL0R register  *******************/
#define  CAN_TDL0R_DATA0                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL0R_DATA1                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL0R_DATA2                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL0R_DATA3                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 3 */

/******************  Bit definition for CAN_TDH0R register  *******************/
#define  CAN_TDH0R_DATA4                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH0R_DATA5                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH0R_DATA6                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH0R_DATA7                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI1R register  *******************/
#define  CAN_TI1R_TXRQ                       ((volatile unsigned int)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI1R_RTR                        ((volatile unsigned int)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI1R_IDE                        ((volatile unsigned int)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI1R_EXID                       ((volatile unsigned int)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_TI1R_STID                       ((volatile unsigned int)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1R register  ******************/
#define  CAN_TDT1R_DLC                       ((volatile unsigned int)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT1R_TGT                       ((volatile unsigned int)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT1R_TIME                      ((volatile unsigned int)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL1R register  ******************/
#define  CAN_TDL1R_DATA0                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL1R_DATA1                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL1R_DATA2                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL1R_DATA3                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH1R register  ******************/
#define  CAN_TDH1R_DATA4                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH1R_DATA5                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH1R_DATA6                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH1R_DATA7                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_TI2R register  *******************/
#define  CAN_TI2R_TXRQ                       ((volatile unsigned int)0x00000001)        /*!<Transmit Mailbox Request */
#define  CAN_TI2R_RTR                        ((volatile unsigned int)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_TI2R_IDE                        ((volatile unsigned int)0x00000004)        /*!<Identifier Extension */
#define  CAN_TI2R_EXID                       ((volatile unsigned int)0x001FFFF8)        /*!<Extended identifier */
#define  CAN_TI2R_STID                       ((volatile unsigned int)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2R register  ******************/  
#define  CAN_TDT2R_DLC                       ((volatile unsigned int)0x0000000F)        /*!<Data Length Code */
#define  CAN_TDT2R_TGT                       ((volatile unsigned int)0x00000100)        /*!<Transmit Global Time */
#define  CAN_TDT2R_TIME                      ((volatile unsigned int)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_TDL2R register  ******************/
#define  CAN_TDL2R_DATA0                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 0 */
#define  CAN_TDL2R_DATA1                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_TDL2R_DATA2                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_TDL2R_DATA3                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_TDH2R register  ******************/
#define  CAN_TDH2R_DATA4                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 4 */
#define  CAN_TDH2R_DATA5                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_TDH2R_DATA6                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_TDH2R_DATA7                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI0R register  *******************/
#define  CAN_RI0R_RTR                        ((volatile unsigned int)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_RI0R_IDE                        ((volatile unsigned int)0x00000004)        /*!<Identifier Extension */
#define  CAN_RI0R_EXID                       ((volatile unsigned int)0x001FFFF8)        /*!<Extended Identifier */
#define  CAN_RI0R_STID                       ((volatile unsigned int)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0R register  ******************/
#define  CAN_RDT0R_DLC                       ((volatile unsigned int)0x0000000F)        /*!<Data Length Code */
#define  CAN_RDT0R_FMI                       ((volatile unsigned int)0x0000FF00)        /*!<Filter Match Index */
#define  CAN_RDT0R_TIME                      ((volatile unsigned int)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL0R register  ******************/
#define  CAN_RDL0R_DATA0                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 0 */
#define  CAN_RDL0R_DATA1                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_RDL0R_DATA2                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_RDL0R_DATA3                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH0R register  ******************/
#define  CAN_RDH0R_DATA4                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 4 */
#define  CAN_RDH0R_DATA5                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_RDH0R_DATA6                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_RDH0R_DATA7                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 7 */

/*******************  Bit definition for CAN_RI1R register  *******************/
#define  CAN_RI1R_RTR                        ((volatile unsigned int)0x00000002)        /*!<Remote Transmission Request */
#define  CAN_RI1R_IDE                        ((volatile unsigned int)0x00000004)        /*!<Identifier Extension */
#define  CAN_RI1R_EXID                       ((volatile unsigned int)0x001FFFF8)        /*!<Extended identifier */
#define  CAN_RI1R_STID                       ((volatile unsigned int)0xFFE00000)        /*!<Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1R register  ******************/
#define  CAN_RDT1R_DLC                       ((volatile unsigned int)0x0000000F)        /*!<Data Length Code */
#define  CAN_RDT1R_FMI                       ((volatile unsigned int)0x0000FF00)        /*!<Filter Match Index */
#define  CAN_RDT1R_TIME                      ((volatile unsigned int)0xFFFF0000)        /*!<Message Time Stamp */

/*******************  Bit definition for CAN_RDL1R register  ******************/
#define  CAN_RDL1R_DATA0                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 0 */
#define  CAN_RDL1R_DATA1                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 1 */
#define  CAN_RDL1R_DATA2                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 2 */
#define  CAN_RDL1R_DATA3                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 3 */

/*******************  Bit definition for CAN_RDH1R register  ******************/
#define  CAN_RDH1R_DATA4                     ((volatile unsigned int)0x000000FF)        /*!<Data byte 4 */
#define  CAN_RDH1R_DATA5                     ((volatile unsigned int)0x0000FF00)        /*!<Data byte 5 */
#define  CAN_RDH1R_DATA6                     ((volatile unsigned int)0x00FF0000)        /*!<Data byte 6 */
#define  CAN_RDH1R_DATA7                     ((volatile unsigned int)0xFF000000)        /*!<Data byte 7 */

/*!<CAN filter registers */
/*******************  Bit definition for CAN_FMR register  ********************/
#define  CAN_FMR_FINIT                       ((volatile unsigned int)0x01)               /*!<Filter Init Mode */
#define  CAN_FMR_CAN2SB                      ((volatile unsigned int)0x00003F00)        /*!<CAN2 start bank */

/*******************  Bit definition for CAN_FM1R register  *******************/
#define  CAN_FM1R_FBM                        ((volatile unsigned int)0x3FFF)            /*!<Filter Mode */
#define  CAN_FM1R_FBM0                       ((volatile unsigned int)0x0001)            /*!<Filter Init Mode bit 0 */
#define  CAN_FM1R_FBM1                       ((volatile unsigned int)0x0002)            /*!<Filter Init Mode bit 1 */
#define  CAN_FM1R_FBM2                       ((volatile unsigned int)0x0004)            /*!<Filter Init Mode bit 2 */
#define  CAN_FM1R_FBM3                       ((volatile unsigned int)0x0008)            /*!<Filter Init Mode bit 3 */
#define  CAN_FM1R_FBM4                       ((volatile unsigned int)0x0010)            /*!<Filter Init Mode bit 4 */
#define  CAN_FM1R_FBM5                       ((volatile unsigned int)0x0020)            /*!<Filter Init Mode bit 5 */
#define  CAN_FM1R_FBM6                       ((volatile unsigned int)0x0040)            /*!<Filter Init Mode bit 6 */
#define  CAN_FM1R_FBM7                       ((volatile unsigned int)0x0080)            /*!<Filter Init Mode bit 7 */
#define  CAN_FM1R_FBM8                       ((volatile unsigned int)0x0100)            /*!<Filter Init Mode bit 8 */
#define  CAN_FM1R_FBM9                       ((volatile unsigned int)0x0200)            /*!<Filter Init Mode bit 9 */
#define  CAN_FM1R_FBM10                      ((volatile unsigned int)0x0400)            /*!<Filter Init Mode bit 10 */
#define  CAN_FM1R_FBM11                      ((volatile unsigned int)0x0800)            /*!<Filter Init Mode bit 11 */
#define  CAN_FM1R_FBM12                      ((volatile unsigned int)0x1000)            /*!<Filter Init Mode bit 12 */
#define  CAN_FM1R_FBM13                      ((volatile unsigned int)0x2000)            /*!<Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1R register  *******************/
#define  CAN_FS1R_FSC                        ((volatile unsigned int)0x3FFF)            /*!<Filter Scale Configuration */
#define  CAN_FS1R_FSC0                       ((volatile unsigned int)0x0001)            /*!<Filter Scale Configuration bit 0 */
#define  CAN_FS1R_FSC1                       ((volatile unsigned int)0x0002)            /*!<Filter Scale Configuration bit 1 */
#define  CAN_FS1R_FSC2                       ((volatile unsigned int)0x0004)            /*!<Filter Scale Configuration bit 2 */
#define  CAN_FS1R_FSC3                       ((volatile unsigned int)0x0008)            /*!<Filter Scale Configuration bit 3 */
#define  CAN_FS1R_FSC4                       ((volatile unsigned int)0x0010)            /*!<Filter Scale Configuration bit 4 */
#define  CAN_FS1R_FSC5                       ((volatile unsigned int)0x0020)            /*!<Filter Scale Configuration bit 5 */
#define  CAN_FS1R_FSC6                       ((volatile unsigned int)0x0040)            /*!<Filter Scale Configuration bit 6 */
#define  CAN_FS1R_FSC7                       ((volatile unsigned int)0x0080)            /*!<Filter Scale Configuration bit 7 */
#define  CAN_FS1R_FSC8                       ((volatile unsigned int)0x0100)            /*!<Filter Scale Configuration bit 8 */
#define  CAN_FS1R_FSC9                       ((volatile unsigned int)0x0200)            /*!<Filter Scale Configuration bit 9 */
#define  CAN_FS1R_FSC10                      ((volatile unsigned int)0x0400)            /*!<Filter Scale Configuration bit 10 */
#define  CAN_FS1R_FSC11                      ((volatile unsigned int)0x0800)            /*!<Filter Scale Configuration bit 11 */
#define  CAN_FS1R_FSC12                      ((volatile unsigned int)0x1000)            /*!<Filter Scale Configuration bit 12 */
#define  CAN_FS1R_FSC13                      ((volatile unsigned int)0x2000)            /*!<Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1R register  *******************/
#define  CAN_FFA1R_FFA                       ((volatile unsigned int)0x3FFF)            /*!<Filter FIFO Assignment */
#define  CAN_FFA1R_FFA0                      ((volatile unsigned int)0x0001)            /*!<Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1R_FFA1                      ((volatile unsigned int)0x0002)            /*!<Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1R_FFA2                      ((volatile unsigned int)0x0004)            /*!<Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1R_FFA3                      ((volatile unsigned int)0x0008)            /*!<Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1R_FFA4                      ((volatile unsigned int)0x0010)            /*!<Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1R_FFA5                      ((volatile unsigned int)0x0020)            /*!<Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1R_FFA6                      ((volatile unsigned int)0x0040)            /*!<Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1R_FFA7                      ((volatile unsigned int)0x0080)            /*!<Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1R_FFA8                      ((volatile unsigned int)0x0100)            /*!<Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1R_FFA9                      ((volatile unsigned int)0x0200)            /*!<Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1R_FFA10                     ((volatile unsigned int)0x0400)            /*!<Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1R_FFA11                     ((volatile unsigned int)0x0800)            /*!<Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1R_FFA12                     ((volatile unsigned int)0x1000)            /*!<Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1R_FFA13                     ((volatile unsigned int)0x2000)            /*!<Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1R register  *******************/
#define  CAN_FA1R_FACT                       ((volatile unsigned int)0x3FFF)            /*!<Filter Active */
#define  CAN_FA1R_FACT0                      ((volatile unsigned int)0x0001)            /*!<Filter 0 Active */
#define  CAN_FA1R_FACT1                      ((volatile unsigned int)0x0002)            /*!<Filter 1 Active */
#define  CAN_FA1R_FACT2                      ((volatile unsigned int)0x0004)            /*!<Filter 2 Active */
#define  CAN_FA1R_FACT3                      ((volatile unsigned int)0x0008)            /*!<Filter 3 Active */
#define  CAN_FA1R_FACT4                      ((volatile unsigned int)0x0010)            /*!<Filter 4 Active */
#define  CAN_FA1R_FACT5                      ((volatile unsigned int)0x0020)            /*!<Filter 5 Active */
#define  CAN_FA1R_FACT6                      ((volatile unsigned int)0x0040)            /*!<Filter 6 Active */
#define  CAN_FA1R_FACT7                      ((volatile unsigned int)0x0080)            /*!<Filter 7 Active */
#define  CAN_FA1R_FACT8                      ((volatile unsigned int)0x0100)            /*!<Filter 8 Active */
#define  CAN_FA1R_FACT9                      ((volatile unsigned int)0x0200)            /*!<Filter 9 Active */
#define  CAN_FA1R_FACT10                     ((volatile unsigned int)0x0400)            /*!<Filter 10 Active */
#define  CAN_FA1R_FACT11                     ((volatile unsigned int)0x0800)            /*!<Filter 11 Active */
#define  CAN_FA1R_FACT12                     ((volatile unsigned int)0x1000)            /*!<Filter 12 Active */
#define  CAN_FA1R_FACT13                     ((volatile unsigned int)0x2000)            /*!<Filter 13 Active */

/*******************  Bit definition for CAN_F0R1 register  *******************/
#define  CAN_F0R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F0R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F0R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F0R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F0R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F0R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F0R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F0R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F0R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F0R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F0R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F0R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F0R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F0R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F0R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F0R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F0R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F0R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F0R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F0R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F0R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F0R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F0R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F0R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F0R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F0R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F0R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F0R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F0R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F0R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F0R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F0R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R1 register  *******************/
#define  CAN_F1R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F1R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F1R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F1R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F1R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F1R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F1R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F1R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F1R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F1R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F1R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F1R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F1R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F1R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F1R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F1R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F1R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F1R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F1R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F1R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F1R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F1R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F1R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F1R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F1R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F1R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F1R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F1R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F1R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F1R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F1R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F1R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R1 register  *******************/
#define  CAN_F2R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F2R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F2R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F2R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F2R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F2R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F2R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F2R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F2R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F2R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F2R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F2R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F2R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F2R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F2R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F2R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F2R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F2R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F2R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F2R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F2R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F2R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F2R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F2R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F2R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F2R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F2R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F2R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F2R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F2R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F2R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F2R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R1 register  *******************/
#define  CAN_F3R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F3R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F3R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F3R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F3R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F3R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F3R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F3R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F3R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F3R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F3R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F3R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F3R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F3R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F3R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F3R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F3R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F3R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F3R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F3R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F3R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F3R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F3R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F3R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F3R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F3R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F3R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F3R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F3R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F3R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F3R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F3R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R1 register  *******************/
#define  CAN_F4R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F4R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F4R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F4R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F4R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F4R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F4R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F4R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F4R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F4R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F4R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F4R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F4R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F4R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F4R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F4R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F4R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F4R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F4R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F4R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F4R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F4R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F4R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F4R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F4R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F4R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F4R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F4R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F4R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F4R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F4R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F4R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R1 register  *******************/
#define  CAN_F5R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F5R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F5R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F5R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F5R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F5R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F5R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F5R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F5R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F5R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F5R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F5R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F5R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F5R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F5R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F5R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F5R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F5R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F5R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F5R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F5R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F5R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F5R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F5R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F5R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F5R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F5R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F5R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F5R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F5R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F5R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F5R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R1 register  *******************/
#define  CAN_F6R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F6R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F6R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F6R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F6R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F6R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F6R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F6R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F6R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F6R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F6R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F6R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F6R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F6R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F6R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F6R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F6R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F6R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F6R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F6R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F6R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F6R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F6R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F6R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F6R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F6R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F6R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F6R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F6R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F6R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F6R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F6R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R1 register  *******************/
#define  CAN_F7R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F7R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F7R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F7R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F7R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F7R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F7R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F7R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F7R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F7R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F7R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F7R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F7R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F7R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F7R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F7R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F7R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F7R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F7R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F7R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F7R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F7R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F7R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F7R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F7R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F7R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F7R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F7R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F7R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F7R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F7R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F7R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R1 register  *******************/
#define  CAN_F8R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F8R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F8R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F8R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F8R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F8R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F8R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F8R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F8R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F8R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F8R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F8R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F8R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F8R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F8R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F8R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F8R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F8R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F8R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F8R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F8R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F8R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F8R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F8R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F8R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F8R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F8R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F8R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F8R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F8R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F8R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F8R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R1 register  *******************/
#define  CAN_F9R1_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F9R1_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F9R1_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F9R1_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F9R1_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F9R1_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F9R1_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F9R1_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F9R1_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F9R1_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F9R1_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F9R1_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F9R1_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F9R1_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F9R1_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F9R1_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F9R1_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F9R1_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F9R1_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F9R1_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F9R1_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F9R1_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F9R1_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F9R1_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F9R1_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F9R1_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F9R1_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F9R1_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F9R1_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F9R1_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F9R1_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F9R1_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R1 register  ******************/
#define  CAN_F10R1_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F10R1_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F10R1_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F10R1_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F10R1_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F10R1_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F10R1_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F10R1_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F10R1_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F10R1_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F10R1_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F10R1_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F10R1_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F10R1_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F10R1_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F10R1_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F10R1_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F10R1_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F10R1_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F10R1_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F10R1_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F10R1_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F10R1_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F10R1_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F10R1_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F10R1_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F10R1_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F10R1_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F10R1_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F10R1_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F10R1_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F10R1_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R1 register  ******************/
#define  CAN_F11R1_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F11R1_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F11R1_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F11R1_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F11R1_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F11R1_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F11R1_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F11R1_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F11R1_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F11R1_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F11R1_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F11R1_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F11R1_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F11R1_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F11R1_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F11R1_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F11R1_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F11R1_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F11R1_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F11R1_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F11R1_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F11R1_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F11R1_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F11R1_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F11R1_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F11R1_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F11R1_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F11R1_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F11R1_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F11R1_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F11R1_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F11R1_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R1 register  ******************/
#define  CAN_F12R1_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F12R1_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F12R1_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F12R1_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F12R1_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F12R1_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F12R1_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F12R1_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F12R1_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F12R1_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F12R1_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F12R1_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F12R1_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F12R1_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F12R1_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F12R1_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F12R1_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F12R1_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F12R1_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F12R1_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F12R1_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F12R1_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F12R1_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F12R1_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F12R1_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F12R1_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F12R1_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F12R1_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F12R1_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F12R1_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F12R1_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F12R1_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R1 register  ******************/
#define  CAN_F13R1_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F13R1_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F13R1_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F13R1_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F13R1_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F13R1_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F13R1_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F13R1_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F13R1_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F13R1_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F13R1_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F13R1_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F13R1_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F13R1_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F13R1_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F13R1_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F13R1_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F13R1_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F13R1_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F13R1_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F13R1_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F13R1_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F13R1_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F13R1_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F13R1_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F13R1_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F13R1_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F13R1_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F13R1_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F13R1_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F13R1_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F13R1_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F0R2 register  *******************/
#define  CAN_F0R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F0R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F0R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F0R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F0R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F0R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F0R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F0R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F0R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F0R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F0R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F0R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F0R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F0R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F0R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F0R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F0R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F0R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F0R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F0R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F0R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F0R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F0R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F0R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F0R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F0R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F0R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F0R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F0R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F0R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F0R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F0R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F1R2 register  *******************/
#define  CAN_F1R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F1R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F1R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F1R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F1R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F1R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F1R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F1R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F1R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F1R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F1R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F1R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F1R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F1R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F1R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F1R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F1R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F1R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F1R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F1R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F1R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F1R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F1R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F1R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F1R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F1R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F1R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F1R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F1R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F1R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F1R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F1R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F2R2 register  *******************/
#define  CAN_F2R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F2R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F2R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F2R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F2R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F2R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F2R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F2R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F2R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F2R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F2R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F2R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F2R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F2R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F2R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F2R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F2R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F2R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F2R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F2R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F2R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F2R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F2R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F2R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F2R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F2R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F2R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F2R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F2R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F2R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F2R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F2R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F3R2 register  *******************/
#define  CAN_F3R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F3R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F3R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F3R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F3R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F3R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F3R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F3R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F3R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F3R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F3R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F3R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F3R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F3R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F3R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F3R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F3R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F3R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F3R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F3R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F3R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F3R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F3R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F3R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F3R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F3R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F3R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F3R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F3R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F3R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F3R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F3R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F4R2 register  *******************/
#define  CAN_F4R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F4R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F4R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F4R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F4R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F4R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F4R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F4R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F4R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F4R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F4R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F4R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F4R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F4R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F4R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F4R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F4R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F4R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F4R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F4R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F4R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F4R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F4R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F4R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F4R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F4R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F4R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F4R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F4R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F4R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F4R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F4R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F5R2 register  *******************/
#define  CAN_F5R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F5R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F5R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F5R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F5R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F5R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F5R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F5R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F5R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F5R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F5R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F5R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F5R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F5R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F5R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F5R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F5R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F5R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F5R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F5R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F5R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F5R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F5R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F5R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F5R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F5R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F5R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F5R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F5R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F5R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F5R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F5R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F6R2 register  *******************/
#define  CAN_F6R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F6R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F6R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F6R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F6R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F6R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F6R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F6R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F6R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F6R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F6R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F6R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F6R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F6R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F6R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F6R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F6R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F6R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F6R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F6R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F6R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F6R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F6R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F6R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F6R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F6R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F6R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F6R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F6R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F6R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F6R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F6R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F7R2 register  *******************/
#define  CAN_F7R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F7R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F7R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F7R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F7R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F7R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F7R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F7R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F7R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F7R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F7R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F7R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F7R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F7R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F7R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F7R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F7R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F7R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F7R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F7R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F7R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F7R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F7R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F7R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F7R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F7R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F7R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F7R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F7R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F7R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F7R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F7R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F8R2 register  *******************/
#define  CAN_F8R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F8R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F8R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F8R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F8R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F8R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F8R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F8R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F8R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F8R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F8R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F8R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F8R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F8R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F8R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F8R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F8R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F8R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F8R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F8R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F8R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F8R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F8R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F8R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F8R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F8R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F8R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F8R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F8R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F8R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F8R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F8R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F9R2 register  *******************/
#define  CAN_F9R2_FB0                        ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F9R2_FB1                        ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F9R2_FB2                        ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F9R2_FB3                        ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F9R2_FB4                        ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F9R2_FB5                        ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F9R2_FB6                        ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F9R2_FB7                        ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F9R2_FB8                        ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F9R2_FB9                        ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F9R2_FB10                       ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F9R2_FB11                       ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F9R2_FB12                       ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F9R2_FB13                       ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F9R2_FB14                       ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F9R2_FB15                       ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F9R2_FB16                       ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F9R2_FB17                       ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F9R2_FB18                       ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F9R2_FB19                       ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F9R2_FB20                       ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F9R2_FB21                       ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F9R2_FB22                       ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F9R2_FB23                       ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F9R2_FB24                       ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F9R2_FB25                       ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F9R2_FB26                       ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F9R2_FB27                       ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F9R2_FB28                       ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F9R2_FB29                       ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F9R2_FB30                       ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F9R2_FB31                       ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F10R2 register  ******************/
#define  CAN_F10R2_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F10R2_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F10R2_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F10R2_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F10R2_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F10R2_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F10R2_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F10R2_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F10R2_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F10R2_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F10R2_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F10R2_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F10R2_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F10R2_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F10R2_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F10R2_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F10R2_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F10R2_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F10R2_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F10R2_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F10R2_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F10R2_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F10R2_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F10R2_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F10R2_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F10R2_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F10R2_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F10R2_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F10R2_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F10R2_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F10R2_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F10R2_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F11R2 register  ******************/
#define  CAN_F11R2_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F11R2_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F11R2_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F11R2_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F11R2_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F11R2_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F11R2_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F11R2_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F11R2_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F11R2_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F11R2_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F11R2_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F11R2_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F11R2_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F11R2_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F11R2_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F11R2_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F11R2_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F11R2_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F11R2_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F11R2_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F11R2_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F11R2_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F11R2_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F11R2_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F11R2_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F11R2_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F11R2_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F11R2_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F11R2_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F11R2_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F11R2_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F12R2 register  ******************/
#define  CAN_F12R2_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F12R2_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F12R2_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F12R2_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F12R2_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F12R2_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F12R2_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F12R2_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F12R2_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F12R2_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F12R2_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F12R2_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F12R2_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F12R2_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F12R2_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F12R2_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F12R2_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F12R2_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F12R2_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F12R2_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F12R2_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F12R2_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F12R2_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F12R2_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F12R2_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F12R2_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F12R2_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F12R2_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F12R2_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F12R2_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F12R2_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F12R2_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/*******************  Bit definition for CAN_F13R2 register  ******************/
#define  CAN_F13R2_FB0                       ((volatile unsigned int)0x00000001)        /*!<Filter bit 0 */
#define  CAN_F13R2_FB1                       ((volatile unsigned int)0x00000002)        /*!<Filter bit 1 */
#define  CAN_F13R2_FB2                       ((volatile unsigned int)0x00000004)        /*!<Filter bit 2 */
#define  CAN_F13R2_FB3                       ((volatile unsigned int)0x00000008)        /*!<Filter bit 3 */
#define  CAN_F13R2_FB4                       ((volatile unsigned int)0x00000010)        /*!<Filter bit 4 */
#define  CAN_F13R2_FB5                       ((volatile unsigned int)0x00000020)        /*!<Filter bit 5 */
#define  CAN_F13R2_FB6                       ((volatile unsigned int)0x00000040)        /*!<Filter bit 6 */
#define  CAN_F13R2_FB7                       ((volatile unsigned int)0x00000080)        /*!<Filter bit 7 */
#define  CAN_F13R2_FB8                       ((volatile unsigned int)0x00000100)        /*!<Filter bit 8 */
#define  CAN_F13R2_FB9                       ((volatile unsigned int)0x00000200)        /*!<Filter bit 9 */
#define  CAN_F13R2_FB10                      ((volatile unsigned int)0x00000400)        /*!<Filter bit 10 */
#define  CAN_F13R2_FB11                      ((volatile unsigned int)0x00000800)        /*!<Filter bit 11 */
#define  CAN_F13R2_FB12                      ((volatile unsigned int)0x00001000)        /*!<Filter bit 12 */
#define  CAN_F13R2_FB13                      ((volatile unsigned int)0x00002000)        /*!<Filter bit 13 */
#define  CAN_F13R2_FB14                      ((volatile unsigned int)0x00004000)        /*!<Filter bit 14 */
#define  CAN_F13R2_FB15                      ((volatile unsigned int)0x00008000)        /*!<Filter bit 15 */
#define  CAN_F13R2_FB16                      ((volatile unsigned int)0x00010000)        /*!<Filter bit 16 */
#define  CAN_F13R2_FB17                      ((volatile unsigned int)0x00020000)        /*!<Filter bit 17 */
#define  CAN_F13R2_FB18                      ((volatile unsigned int)0x00040000)        /*!<Filter bit 18 */
#define  CAN_F13R2_FB19                      ((volatile unsigned int)0x00080000)        /*!<Filter bit 19 */
#define  CAN_F13R2_FB20                      ((volatile unsigned int)0x00100000)        /*!<Filter bit 20 */
#define  CAN_F13R2_FB21                      ((volatile unsigned int)0x00200000)        /*!<Filter bit 21 */
#define  CAN_F13R2_FB22                      ((volatile unsigned int)0x00400000)        /*!<Filter bit 22 */
#define  CAN_F13R2_FB23                      ((volatile unsigned int)0x00800000)        /*!<Filter bit 23 */
#define  CAN_F13R2_FB24                      ((volatile unsigned int)0x01000000)        /*!<Filter bit 24 */
#define  CAN_F13R2_FB25                      ((volatile unsigned int)0x02000000)        /*!<Filter bit 25 */
#define  CAN_F13R2_FB26                      ((volatile unsigned int)0x04000000)        /*!<Filter bit 26 */
#define  CAN_F13R2_FB27                      ((volatile unsigned int)0x08000000)        /*!<Filter bit 27 */
#define  CAN_F13R2_FB28                      ((volatile unsigned int)0x10000000)        /*!<Filter bit 28 */
#define  CAN_F13R2_FB29                      ((volatile unsigned int)0x20000000)        /*!<Filter bit 29 */
#define  CAN_F13R2_FB30                      ((volatile unsigned int)0x40000000)        /*!<Filter bit 30 */
#define  CAN_F13R2_FB31                      ((volatile unsigned int)0x80000000)        /*!<Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((volatile unsigned int)0xFFFFFFFF) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((volatile unsigned int)0xFF)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((volatile unsigned int)0x01)        /*!< RESET bit */


/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DAC_CR register  ********************/
#define  DAC_CR_EN1                          ((volatile unsigned int)0x00000001)        /*!<DAC channel1 enable */
#define  DAC_CR_BOFF1                        ((volatile unsigned int)0x00000002)        /*!<DAC channel1 output buffer disable */
#define  DAC_CR_TEN1                         ((volatile unsigned int)0x00000004)        /*!<DAC channel1 Trigger enable */

#define  DAC_CR_TSEL1                        ((volatile unsigned int)0x00000038)        /*!<TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CR_TSEL1_0                      ((volatile unsigned int)0x00000008)        /*!<Bit 0 */
#define  DAC_CR_TSEL1_1                      ((volatile unsigned int)0x00000010)        /*!<Bit 1 */
#define  DAC_CR_TSEL1_2                      ((volatile unsigned int)0x00000020)        /*!<Bit 2 */

#define  DAC_CR_WAVE1                        ((volatile unsigned int)0x000000C0)        /*!<WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE1_0                      ((volatile unsigned int)0x00000040)        /*!<Bit 0 */
#define  DAC_CR_WAVE1_1                      ((volatile unsigned int)0x00000080)        /*!<Bit 1 */

#define  DAC_CR_MAMP1                        ((volatile unsigned int)0x00000F00)        /*!<MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CR_MAMP1_0                      ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  DAC_CR_MAMP1_1                      ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  DAC_CR_MAMP1_2                      ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  DAC_CR_MAMP1_3                      ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  DAC_CR_DMAEN1                       ((volatile unsigned int)0x00001000)        /*!<DAC channel1 DMA enable */
#define  DAC_CR_EN2                          ((volatile unsigned int)0x00010000)        /*!<DAC channel2 enable */
#define  DAC_CR_BOFF2                        ((volatile unsigned int)0x00020000)        /*!<DAC channel2 output buffer disable */
#define  DAC_CR_TEN2                         ((volatile unsigned int)0x00040000)        /*!<DAC channel2 Trigger enable */

#define  DAC_CR_TSEL2                        ((volatile unsigned int)0x00380000)        /*!<TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CR_TSEL2_0                      ((volatile unsigned int)0x00080000)        /*!<Bit 0 */
#define  DAC_CR_TSEL2_1                      ((volatile unsigned int)0x00100000)        /*!<Bit 1 */
#define  DAC_CR_TSEL2_2                      ((volatile unsigned int)0x00200000)        /*!<Bit 2 */

#define  DAC_CR_WAVE2                        ((volatile unsigned int)0x00C00000)        /*!<WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CR_WAVE2_0                      ((volatile unsigned int)0x00400000)        /*!<Bit 0 */
#define  DAC_CR_WAVE2_1                      ((volatile unsigned int)0x00800000)        /*!<Bit 1 */

#define  DAC_CR_MAMP2                        ((volatile unsigned int)0x0F000000)        /*!<MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CR_MAMP2_0                      ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  DAC_CR_MAMP2_1                      ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  DAC_CR_MAMP2_2                      ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  DAC_CR_MAMP2_3                      ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  DAC_CR_DMAEN2                       ((volatile unsigned int)0x10000000)        /*!<DAC channel2 DMA enabled */

/*****************  Bit definition for DAC_SWTRIGR register  ******************/
#define  DAC_SWTRIGR_SWTRIG1                 ((volatile unsigned int)0x01)               /*!<DAC channel1 software trigger */
#define  DAC_SWTRIGR_SWTRIG2                 ((volatile unsigned int)0x02)               /*!<DAC channel2 software trigger */

/*****************  Bit definition for DAC_DHR12R1 register  ******************/
#define  DAC_DHR12R1_DACC1DHR                ((volatile unsigned int)0x0FFF)            /*!<DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L1 register  ******************/
#define  DAC_DHR12L1_DACC1DHR                ((volatile unsigned int)0xFFF0)            /*!<DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R1 register  ******************/
#define  DAC_DHR8R1_DACC1DHR                 ((volatile unsigned int)0xFF)               /*!<DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12R2 register  ******************/
#define  DAC_DHR12R2_DACC2DHR                ((volatile unsigned int)0x0FFF)            /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12L2 register  ******************/
#define  DAC_DHR12L2_DACC2DHR                ((volatile unsigned int)0xFFF0)            /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8R2 register  ******************/
#define  DAC_DHR8R2_DACC2DHR                 ((volatile unsigned int)0xFF)               /*!<DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12RD register  ******************/
#define  DAC_DHR12RD_DACC1DHR                ((volatile unsigned int)0x00000FFF)        /*!<DAC channel1 12-bit Right aligned data */
#define  DAC_DHR12RD_DACC2DHR                ((volatile unsigned int)0x0FFF0000)        /*!<DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_DHR12LD register  ******************/
#define  DAC_DHR12LD_DACC1DHR                ((volatile unsigned int)0x0000FFF0)        /*!<DAC channel1 12-bit Left aligned data */
#define  DAC_DHR12LD_DACC2DHR                ((volatile unsigned int)0xFFF00000)        /*!<DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_DHR8RD register  ******************/
#define  DAC_DHR8RD_DACC1DHR                 ((volatile unsigned int)0x00FF)            /*!<DAC channel1 8-bit Right aligned data */
#define  DAC_DHR8RD_DACC2DHR                 ((volatile unsigned int)0xFF00)            /*!<DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_DOR1 register  *******************/
#define  DAC_DOR1_DACC1DOR                   ((volatile unsigned int)0x0FFF)            /*!<DAC channel1 data output */

/*******************  Bit definition for DAC_DOR2 register  *******************/
#define  DAC_DOR2_DACC2DOR                   ((volatile unsigned int)0x0FFF)            /*!<DAC channel2 data output */

/********************  Bit definition for DAC_SR register  ********************/
#define  DAC_SR_DMAUDR1                      ((volatile unsigned int)0x00002000)        /*!<DAC channel1 DMA underrun flag */
#define  DAC_SR_DMAUDR2                      ((volatile unsigned int)0x20000000)        /*!<DAC channel2 DMA underrun flag */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                                    DCMI                                    */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DCMI_CR register  ******************/
#define DCMI_CR_CAPTURE                      ((volatile unsigned int)0x00000001)
#define DCMI_CR_CM                           ((volatile unsigned int)0x00000002)
#define DCMI_CR_CROP                         ((volatile unsigned int)0x00000004)
#define DCMI_CR_JPEG                         ((volatile unsigned int)0x00000008)
#define DCMI_CR_ESS                          ((volatile unsigned int)0x00000010)
#define DCMI_CR_PCKPOL                       ((volatile unsigned int)0x00000020)
#define DCMI_CR_HSPOL                        ((volatile unsigned int)0x00000040)
#define DCMI_CR_VSPOL                        ((volatile unsigned int)0x00000080)
#define DCMI_CR_FCRC_0                       ((volatile unsigned int)0x00000100)
#define DCMI_CR_FCRC_1                       ((volatile unsigned int)0x00000200)
#define DCMI_CR_EDM_0                        ((volatile unsigned int)0x00000400)
#define DCMI_CR_EDM_1                        ((volatile unsigned int)0x00000800)
#define DCMI_CR_CRE                          ((volatile unsigned int)0x00001000)
#define DCMI_CR_ENABLE                       ((volatile unsigned int)0x00004000)

/********************  Bits definition for DCMI_SR register  ******************/
#define DCMI_SR_HSYNC                        ((volatile unsigned int)0x00000001)
#define DCMI_SR_VSYNC                        ((volatile unsigned int)0x00000002)
#define DCMI_SR_FNE                          ((volatile unsigned int)0x00000004)

/********************  Bits definition for DCMI_RISR register  ****************/
#define DCMI_RISR_FRAME_RIS                  ((volatile unsigned int)0x00000001)
#define DCMI_RISR_OVF_RIS                    ((volatile unsigned int)0x00000002)
#define DCMI_RISR_ERR_RIS                    ((volatile unsigned int)0x00000004)
#define DCMI_RISR_VSYNC_RIS                  ((volatile unsigned int)0x00000008)
#define DCMI_RISR_LINE_RIS                   ((volatile unsigned int)0x00000010)

/********************  Bits definition for DCMI_IER register  *****************/
#define DCMI_IER_FRAME_IE                    ((volatile unsigned int)0x00000001)
#define DCMI_IER_OVF_IE                      ((volatile unsigned int)0x00000002)
#define DCMI_IER_ERR_IE                      ((volatile unsigned int)0x00000004)
#define DCMI_IER_VSYNC_IE                    ((volatile unsigned int)0x00000008)
#define DCMI_IER_LINE_IE                     ((volatile unsigned int)0x00000010)

/********************  Bits definition for DCMI_MISR register  ****************/
#define DCMI_MISR_FRAME_MIS                  ((volatile unsigned int)0x00000001)
#define DCMI_MISR_OVF_MIS                    ((volatile unsigned int)0x00000002)
#define DCMI_MISR_ERR_MIS                    ((volatile unsigned int)0x00000004)
#define DCMI_MISR_VSYNC_MIS                  ((volatile unsigned int)0x00000008)
#define DCMI_MISR_LINE_MIS                   ((volatile unsigned int)0x00000010)

/********************  Bits definition for DCMI_ICR register  *****************/
#define DCMI_ICR_FRAME_ISC                   ((volatile unsigned int)0x00000001)
#define DCMI_ICR_OVF_ISC                     ((volatile unsigned int)0x00000002)
#define DCMI_ICR_ERR_ISC                     ((volatile unsigned int)0x00000004)
#define DCMI_ICR_VSYNC_ISC                   ((volatile unsigned int)0x00000008)
#define DCMI_ICR_LINE_ISC                    ((volatile unsigned int)0x00000010)

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DMA_SxCR register  *****************/ 
#define DMA_SxCR_CHSEL                       ((volatile unsigned int)0x0E000000)
#define DMA_SxCR_CHSEL_0                     ((volatile unsigned int)0x02000000)
#define DMA_SxCR_CHSEL_1                     ((volatile unsigned int)0x04000000)
#define DMA_SxCR_CHSEL_2                     ((volatile unsigned int)0x08000000) 
#define DMA_SxCR_MBURST                      ((volatile unsigned int)0x01800000)
#define DMA_SxCR_MBURST_0                    ((volatile unsigned int)0x00800000)
#define DMA_SxCR_MBURST_1                    ((volatile unsigned int)0x01000000)
#define DMA_SxCR_PBURST                      ((volatile unsigned int)0x00600000)
#define DMA_SxCR_PBURST_0                    ((volatile unsigned int)0x00200000)
#define DMA_SxCR_PBURST_1                    ((volatile unsigned int)0x00400000)
#define DMA_SxCR_ACK                         ((volatile unsigned int)0x00100000)
#define DMA_SxCR_CT                          ((volatile unsigned int)0x00080000)  
#define DMA_SxCR_DBM                         ((volatile unsigned int)0x00040000)
#define DMA_SxCR_PL                          ((volatile unsigned int)0x00030000)
#define DMA_SxCR_PL_0                        ((volatile unsigned int)0x00010000)
#define DMA_SxCR_PL_1                        ((volatile unsigned int)0x00020000)
#define DMA_SxCR_PINCOS                      ((volatile unsigned int)0x00008000)
#define DMA_SxCR_MSIZE                       ((volatile unsigned int)0x00006000)
#define DMA_SxCR_MSIZE_0                     ((volatile unsigned int)0x00002000)
#define DMA_SxCR_MSIZE_1                     ((volatile unsigned int)0x00004000)
#define DMA_SxCR_PSIZE                       ((volatile unsigned int)0x00001800)
#define DMA_SxCR_PSIZE_0                     ((volatile unsigned int)0x00000800)
#define DMA_SxCR_PSIZE_1                     ((volatile unsigned int)0x00001000)
#define DMA_SxCR_MINC                        ((volatile unsigned int)0x00000400)
#define DMA_SxCR_PINC                        ((volatile unsigned int)0x00000200)
#define DMA_SxCR_CIRC                        ((volatile unsigned int)0x00000100)
#define DMA_SxCR_DIR                         ((volatile unsigned int)0x000000C0)
#define DMA_SxCR_DIR_0                       ((volatile unsigned int)0x00000040)
#define DMA_SxCR_DIR_1                       ((volatile unsigned int)0x00000080)
#define DMA_SxCR_PFCTRL                      ((volatile unsigned int)0x00000020)
#define DMA_SxCR_TCIE                        ((volatile unsigned int)0x00000010)
#define DMA_SxCR_HTIE                        ((volatile unsigned int)0x00000008)
#define DMA_SxCR_TEIE                        ((volatile unsigned int)0x00000004)
#define DMA_SxCR_DMEIE                       ((volatile unsigned int)0x00000002)
#define DMA_SxCR_EN                          ((volatile unsigned int)0x00000001)

/********************  Bits definition for DMA_SxCNDTR register  **************/
#define DMA_SxNDT                            ((volatile unsigned int)0x0000FFFF)
#define DMA_SxNDT_0                          ((volatile unsigned int)0x00000001)
#define DMA_SxNDT_1                          ((volatile unsigned int)0x00000002)
#define DMA_SxNDT_2                          ((volatile unsigned int)0x00000004)
#define DMA_SxNDT_3                          ((volatile unsigned int)0x00000008)
#define DMA_SxNDT_4                          ((volatile unsigned int)0x00000010)
#define DMA_SxNDT_5                          ((volatile unsigned int)0x00000020)
#define DMA_SxNDT_6                          ((volatile unsigned int)0x00000040)
#define DMA_SxNDT_7                          ((volatile unsigned int)0x00000080)
#define DMA_SxNDT_8                          ((volatile unsigned int)0x00000100)
#define DMA_SxNDT_9                          ((volatile unsigned int)0x00000200)
#define DMA_SxNDT_10                         ((volatile unsigned int)0x00000400)
#define DMA_SxNDT_11                         ((volatile unsigned int)0x00000800)
#define DMA_SxNDT_12                         ((volatile unsigned int)0x00001000)
#define DMA_SxNDT_13                         ((volatile unsigned int)0x00002000)
#define DMA_SxNDT_14                         ((volatile unsigned int)0x00004000)
#define DMA_SxNDT_15                         ((volatile unsigned int)0x00008000)

/********************  Bits definition for DMA_SxFCR register  ****************/ 
#define DMA_SxFCR_FEIE                       ((volatile unsigned int)0x00000080)
#define DMA_SxFCR_FS                         ((volatile unsigned int)0x00000038)
#define DMA_SxFCR_FS_0                       ((volatile unsigned int)0x00000008)
#define DMA_SxFCR_FS_1                       ((volatile unsigned int)0x00000010)
#define DMA_SxFCR_FS_2                       ((volatile unsigned int)0x00000020)
#define DMA_SxFCR_DMDIS                      ((volatile unsigned int)0x00000004)
#define DMA_SxFCR_FTH                        ((volatile unsigned int)0x00000003)
#define DMA_SxFCR_FTH_0                      ((volatile unsigned int)0x00000001)
#define DMA_SxFCR_FTH_1                      ((volatile unsigned int)0x00000002)

/********************  Bits definition for DMA_LISR register  *****************/ 
#define DMA_LISR_TCIF3                       ((volatile unsigned int)0x08000000)
#define DMA_LISR_HTIF3                       ((volatile unsigned int)0x04000000)
#define DMA_LISR_TEIF3                       ((volatile unsigned int)0x02000000)
#define DMA_LISR_DMEIF3                      ((volatile unsigned int)0x01000000)
#define DMA_LISR_FEIF3                       ((volatile unsigned int)0x00400000)
#define DMA_LISR_TCIF2                       ((volatile unsigned int)0x00200000)
#define DMA_LISR_HTIF2                       ((volatile unsigned int)0x00100000)
#define DMA_LISR_TEIF2                       ((volatile unsigned int)0x00080000)
#define DMA_LISR_DMEIF2                      ((volatile unsigned int)0x00040000)
#define DMA_LISR_FEIF2                       ((volatile unsigned int)0x00010000)
#define DMA_LISR_TCIF1                       ((volatile unsigned int)0x00000800)
#define DMA_LISR_HTIF1                       ((volatile unsigned int)0x00000400)
#define DMA_LISR_TEIF1                       ((volatile unsigned int)0x00000200)
#define DMA_LISR_DMEIF1                      ((volatile unsigned int)0x00000100)
#define DMA_LISR_FEIF1                       ((volatile unsigned int)0x00000040)
#define DMA_LISR_TCIF0                       ((volatile unsigned int)0x00000020)
#define DMA_LISR_HTIF0                       ((volatile unsigned int)0x00000010)
#define DMA_LISR_TEIF0                       ((volatile unsigned int)0x00000008)
#define DMA_LISR_DMEIF0                      ((volatile unsigned int)0x00000004)
#define DMA_LISR_FEIF0                       ((volatile unsigned int)0x00000001)

/********************  Bits definition for DMA_HISR register  *****************/ 
#define DMA_HISR_TCIF7                       ((volatile unsigned int)0x08000000)
#define DMA_HISR_HTIF7                       ((volatile unsigned int)0x04000000)
#define DMA_HISR_TEIF7                       ((volatile unsigned int)0x02000000)
#define DMA_HISR_DMEIF7                      ((volatile unsigned int)0x01000000)
#define DMA_HISR_FEIF7                       ((volatile unsigned int)0x00400000)
#define DMA_HISR_TCIF6                       ((volatile unsigned int)0x00200000)
#define DMA_HISR_HTIF6                       ((volatile unsigned int)0x00100000)
#define DMA_HISR_TEIF6                       ((volatile unsigned int)0x00080000)
#define DMA_HISR_DMEIF6                      ((volatile unsigned int)0x00040000)
#define DMA_HISR_FEIF6                       ((volatile unsigned int)0x00010000)
#define DMA_HISR_TCIF5                       ((volatile unsigned int)0x00000800)
#define DMA_HISR_HTIF5                       ((volatile unsigned int)0x00000400)
#define DMA_HISR_TEIF5                       ((volatile unsigned int)0x00000200)
#define DMA_HISR_DMEIF5                      ((volatile unsigned int)0x00000100)
#define DMA_HISR_FEIF5                       ((volatile unsigned int)0x00000040)
#define DMA_HISR_TCIF4                       ((volatile unsigned int)0x00000020)
#define DMA_HISR_HTIF4                       ((volatile unsigned int)0x00000010)
#define DMA_HISR_TEIF4                       ((volatile unsigned int)0x00000008)
#define DMA_HISR_DMEIF4                      ((volatile unsigned int)0x00000004)
#define DMA_HISR_FEIF4                       ((volatile unsigned int)0x00000001)

/********************  Bits definition for DMA_LIFCR register  ****************/ 
#define DMA_LIFCR_CTCIF3                     ((volatile unsigned int)0x08000000)
#define DMA_LIFCR_CHTIF3                     ((volatile unsigned int)0x04000000)
#define DMA_LIFCR_CTEIF3                     ((volatile unsigned int)0x02000000)
#define DMA_LIFCR_CDMEIF3                    ((volatile unsigned int)0x01000000)
#define DMA_LIFCR_CFEIF3                     ((volatile unsigned int)0x00400000)
#define DMA_LIFCR_CTCIF2                     ((volatile unsigned int)0x00200000)
#define DMA_LIFCR_CHTIF2                     ((volatile unsigned int)0x00100000)
#define DMA_LIFCR_CTEIF2                     ((volatile unsigned int)0x00080000)
#define DMA_LIFCR_CDMEIF2                    ((volatile unsigned int)0x00040000)
#define DMA_LIFCR_CFEIF2                     ((volatile unsigned int)0x00010000)
#define DMA_LIFCR_CTCIF1                     ((volatile unsigned int)0x00000800)
#define DMA_LIFCR_CHTIF1                     ((volatile unsigned int)0x00000400)
#define DMA_LIFCR_CTEIF1                     ((volatile unsigned int)0x00000200)
#define DMA_LIFCR_CDMEIF1                    ((volatile unsigned int)0x00000100)
#define DMA_LIFCR_CFEIF1                     ((volatile unsigned int)0x00000040)
#define DMA_LIFCR_CTCIF0                     ((volatile unsigned int)0x00000020)
#define DMA_LIFCR_CHTIF0                     ((volatile unsigned int)0x00000010)
#define DMA_LIFCR_CTEIF0                     ((volatile unsigned int)0x00000008)
#define DMA_LIFCR_CDMEIF0                    ((volatile unsigned int)0x00000004)
#define DMA_LIFCR_CFEIF0                     ((volatile unsigned int)0x00000001)

/********************  Bits definition for DMA_HIFCR  register  ****************/ 
#define DMA_HIFCR_CTCIF7                     ((volatile unsigned int)0x08000000)
#define DMA_HIFCR_CHTIF7                     ((volatile unsigned int)0x04000000)
#define DMA_HIFCR_CTEIF7                     ((volatile unsigned int)0x02000000)
#define DMA_HIFCR_CDMEIF7                    ((volatile unsigned int)0x01000000)
#define DMA_HIFCR_CFEIF7                     ((volatile unsigned int)0x00400000)
#define DMA_HIFCR_CTCIF6                     ((volatile unsigned int)0x00200000)
#define DMA_HIFCR_CHTIF6                     ((volatile unsigned int)0x00100000)
#define DMA_HIFCR_CTEIF6                     ((volatile unsigned int)0x00080000)
#define DMA_HIFCR_CDMEIF6                    ((volatile unsigned int)0x00040000)
#define DMA_HIFCR_CFEIF6                     ((volatile unsigned int)0x00010000)
#define DMA_HIFCR_CTCIF5                     ((volatile unsigned int)0x00000800)
#define DMA_HIFCR_CHTIF5                     ((volatile unsigned int)0x00000400)
#define DMA_HIFCR_CTEIF5                     ((volatile unsigned int)0x00000200)
#define DMA_HIFCR_CDMEIF5                    ((volatile unsigned int)0x00000100)
#define DMA_HIFCR_CFEIF5                     ((volatile unsigned int)0x00000040)
#define DMA_HIFCR_CTCIF4                     ((volatile unsigned int)0x00000020)
#define DMA_HIFCR_CHTIF4                     ((volatile unsigned int)0x00000010)
#define DMA_HIFCR_CTEIF4                     ((volatile unsigned int)0x00000008)
#define DMA_HIFCR_CDMEIF4                    ((volatile unsigned int)0x00000004)
#define DMA_HIFCR_CFEIF4                     ((volatile unsigned int)0x00000001)


/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((volatile unsigned int)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((volatile unsigned int)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((volatile unsigned int)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((volatile unsigned int)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((volatile unsigned int)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((volatile unsigned int)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((volatile unsigned int)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((volatile unsigned int)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((volatile unsigned int)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((volatile unsigned int)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((volatile unsigned int)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((volatile unsigned int)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((volatile unsigned int)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((volatile unsigned int)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((volatile unsigned int)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((volatile unsigned int)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((volatile unsigned int)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((volatile unsigned int)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((volatile unsigned int)0x00040000)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       ((volatile unsigned int)0x00080000)        /*!< Interrupt Mask on line 19 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((volatile unsigned int)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((volatile unsigned int)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((volatile unsigned int)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((volatile unsigned int)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((volatile unsigned int)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((volatile unsigned int)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((volatile unsigned int)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((volatile unsigned int)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((volatile unsigned int)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((volatile unsigned int)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((volatile unsigned int)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((volatile unsigned int)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((volatile unsigned int)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((volatile unsigned int)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((volatile unsigned int)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((volatile unsigned int)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((volatile unsigned int)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((volatile unsigned int)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((volatile unsigned int)0x00040000)        /*!< Event Mask on line 18 */
#define  EXTI_EMR_MR19                       ((volatile unsigned int)0x00080000)        /*!< Event Mask on line 19 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((volatile unsigned int)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((volatile unsigned int)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((volatile unsigned int)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((volatile unsigned int)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((volatile unsigned int)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((volatile unsigned int)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((volatile unsigned int)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((volatile unsigned int)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((volatile unsigned int)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((volatile unsigned int)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((volatile unsigned int)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((volatile unsigned int)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((volatile unsigned int)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((volatile unsigned int)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((volatile unsigned int)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((volatile unsigned int)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((volatile unsigned int)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((volatile unsigned int)0x00020000)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((volatile unsigned int)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      ((volatile unsigned int)0x00080000)        /*!< Rising trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((volatile unsigned int)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((volatile unsigned int)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((volatile unsigned int)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((volatile unsigned int)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((volatile unsigned int)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((volatile unsigned int)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((volatile unsigned int)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((volatile unsigned int)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((volatile unsigned int)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((volatile unsigned int)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((volatile unsigned int)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((volatile unsigned int)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((volatile unsigned int)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((volatile unsigned int)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((volatile unsigned int)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((volatile unsigned int)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((volatile unsigned int)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((volatile unsigned int)0x00020000)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((volatile unsigned int)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      ((volatile unsigned int)0x00080000)        /*!< Falling trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((volatile unsigned int)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((volatile unsigned int)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((volatile unsigned int)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((volatile unsigned int)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((volatile unsigned int)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((volatile unsigned int)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((volatile unsigned int)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((volatile unsigned int)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((volatile unsigned int)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((volatile unsigned int)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((volatile unsigned int)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((volatile unsigned int)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((volatile unsigned int)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((volatile unsigned int)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((volatile unsigned int)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((volatile unsigned int)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((volatile unsigned int)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((volatile unsigned int)0x00020000)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((volatile unsigned int)0x00040000)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  ((volatile unsigned int)0x00080000)        /*!< Software Interrupt on line 19 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((volatile unsigned int)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                         ((volatile unsigned int)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                         ((volatile unsigned int)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                         ((volatile unsigned int)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                         ((volatile unsigned int)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                         ((volatile unsigned int)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                         ((volatile unsigned int)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                         ((volatile unsigned int)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                         ((volatile unsigned int)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                         ((volatile unsigned int)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10                        ((volatile unsigned int)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11                        ((volatile unsigned int)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12                        ((volatile unsigned int)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13                        ((volatile unsigned int)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14                        ((volatile unsigned int)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15                        ((volatile unsigned int)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16                        ((volatile unsigned int)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17                        ((volatile unsigned int)0x00020000)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18                        ((volatile unsigned int)0x00040000)        /*!< Pending bit for line 18 */
#define  EXTI_PR_PR19                        ((volatile unsigned int)0x00080000)        /*!< Pending bit for line 19 */

/******************************************************************************/
/*                                                                            */
/*                                    FLASH                                   */
/*                                                                            */
/******************************************************************************/
/*******************  Bits definition for FLASH_ACR register  *****************/
#define FLASH_ACR_LATENCY                    ((volatile unsigned int)0x0000000F)
#define FLASH_ACR_LATENCY_0WS                ((volatile unsigned int)0x00000000)
#define FLASH_ACR_LATENCY_1WS                ((volatile unsigned int)0x00000001)
#define FLASH_ACR_LATENCY_2WS                ((volatile unsigned int)0x00000002)
#define FLASH_ACR_LATENCY_3WS                ((volatile unsigned int)0x00000003)
#define FLASH_ACR_LATENCY_4WS                ((volatile unsigned int)0x00000004)
#define FLASH_ACR_LATENCY_5WS                ((volatile unsigned int)0x00000005)
#define FLASH_ACR_LATENCY_6WS                ((volatile unsigned int)0x00000006)
#define FLASH_ACR_LATENCY_7WS                ((volatile unsigned int)0x00000007)

#define FLASH_ACR_PRFTEN                     ((volatile unsigned int)0x00000100)
#define FLASH_ACR_ICEN                       ((volatile unsigned int)0x00000200)
#define FLASH_ACR_DCEN                       ((volatile unsigned int)0x00000400)
#define FLASH_ACR_ICRST                      ((volatile unsigned int)0x00000800)
#define FLASH_ACR_DCRST                      ((volatile unsigned int)0x00001000)
#define FLASH_ACR_BYTE0_ADDRESS              ((volatile unsigned int)0x40023C00)
#define FLASH_ACR_BYTE2_ADDRESS              ((volatile unsigned int)0x40023C03)

/*******************  Bits definition for FLASH_SR register  ******************/
#define FLASH_SR_EOP                         ((volatile unsigned int)0x00000001)
#define FLASH_SR_SOP                         ((volatile unsigned int)0x00000002)
#define FLASH_SR_WRPERR                      ((volatile unsigned int)0x00000010)
#define FLASH_SR_PGAERR                      ((volatile unsigned int)0x00000020)
#define FLASH_SR_PGPERR                      ((volatile unsigned int)0x00000040)
#define FLASH_SR_PGSERR                      ((volatile unsigned int)0x00000080)
#define FLASH_SR_BSY                         ((volatile unsigned int)0x00010000)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((volatile unsigned int)0x00000001)
#define FLASH_CR_SER                         ((volatile unsigned int)0x00000002)
#define FLASH_CR_MER                         ((volatile unsigned int)0x00000004)
#define FLASH_CR_SNB                         ((volatile unsigned int)0x000000F8)
#define FLASH_CR_SNB_0                       ((volatile unsigned int)0x00000008)
#define FLASH_CR_SNB_1                       ((volatile unsigned int)0x00000010)
#define FLASH_CR_SNB_2                       ((volatile unsigned int)0x00000020)
#define FLASH_CR_SNB_3                       ((volatile unsigned int)0x00000040)
#define FLASH_CR_SNB_4                       ((volatile unsigned int)0x00000080)
#define FLASH_CR_PSIZE                       ((volatile unsigned int)0x00000300)
#define FLASH_CR_PSIZE_0                     ((volatile unsigned int)0x00000100)
#define FLASH_CR_PSIZE_1                     ((volatile unsigned int)0x00000200)
#define FLASH_CR_STRT                        ((volatile unsigned int)0x00010000)
#define FLASH_CR_EOPIE                       ((volatile unsigned int)0x01000000)
#define FLASH_CR_LOCK                        ((volatile unsigned int)0x80000000)

/*******************  Bits definition for FLASH_OPTCR register  ***************/
#define FLASH_OPTCR_OPTLOCK                 ((volatile unsigned int)0x00000001)
#define FLASH_OPTCR_OPTSTRT                 ((volatile unsigned int)0x00000002)
#define FLASH_OPTCR_BOR_LEV_0               ((volatile unsigned int)0x00000004)
#define FLASH_OPTCR_BOR_LEV_1               ((volatile unsigned int)0x00000008)
#define FLASH_OPTCR_BOR_LEV                 ((volatile unsigned int)0x0000000C)

#define FLASH_OPTCR_WDG_SW                  ((volatile unsigned int)0x00000020)
#define FLASH_OPTCR_nRST_STOP               ((volatile unsigned int)0x00000040)
#define FLASH_OPTCR_nRST_STDBY              ((volatile unsigned int)0x00000080)
#define FLASH_OPTCR_RDP                     ((volatile unsigned int)0x0000FF00)
#define FLASH_OPTCR_RDP_0                   ((volatile unsigned int)0x00000100)
#define FLASH_OPTCR_RDP_1                   ((volatile unsigned int)0x00000200)
#define FLASH_OPTCR_RDP_2                   ((volatile unsigned int)0x00000400)
#define FLASH_OPTCR_RDP_3                   ((volatile unsigned int)0x00000800)
#define FLASH_OPTCR_RDP_4                   ((volatile unsigned int)0x00001000)
#define FLASH_OPTCR_RDP_5                   ((volatile unsigned int)0x00002000)
#define FLASH_OPTCR_RDP_6                   ((volatile unsigned int)0x00004000)
#define FLASH_OPTCR_RDP_7                   ((volatile unsigned int)0x00008000)
#define FLASH_OPTCR_nWRP                    ((volatile unsigned int)0x0FFF0000)
#define FLASH_OPTCR_nWRP_0                  ((volatile unsigned int)0x00010000)
#define FLASH_OPTCR_nWRP_1                  ((volatile unsigned int)0x00020000)
#define FLASH_OPTCR_nWRP_2                  ((volatile unsigned int)0x00040000)
#define FLASH_OPTCR_nWRP_3                  ((volatile unsigned int)0x00080000)
#define FLASH_OPTCR_nWRP_4                  ((volatile unsigned int)0x00100000)
#define FLASH_OPTCR_nWRP_5                  ((volatile unsigned int)0x00200000)
#define FLASH_OPTCR_nWRP_6                  ((volatile unsigned int)0x00400000)
#define FLASH_OPTCR_nWRP_7                  ((volatile unsigned int)0x00800000)
#define FLASH_OPTCR_nWRP_8                  ((volatile unsigned int)0x01000000)
#define FLASH_OPTCR_nWRP_9                  ((volatile unsigned int)0x02000000)
#define FLASH_OPTCR_nWRP_10                 ((volatile unsigned int)0x04000000)
#define FLASH_OPTCR_nWRP_11                 ((volatile unsigned int)0x08000000)

/******************  Bits definition for FLASH_OPTCR1 register  ***************/
#define FLASH_OPTCR1_nWRP                    ((volatile unsigned int)0x0FFF0000)
#define FLASH_OPTCR1_nWRP_0                  ((volatile unsigned int)0x00010000)
#define FLASH_OPTCR1_nWRP_1                  ((volatile unsigned int)0x00020000)
#define FLASH_OPTCR1_nWRP_2                  ((volatile unsigned int)0x00040000)
#define FLASH_OPTCR1_nWRP_3                  ((volatile unsigned int)0x00080000)
#define FLASH_OPTCR1_nWRP_4                  ((volatile unsigned int)0x00100000)
#define FLASH_OPTCR1_nWRP_5                  ((volatile unsigned int)0x00200000)
#define FLASH_OPTCR1_nWRP_6                  ((volatile unsigned int)0x00400000)
#define FLASH_OPTCR1_nWRP_7                  ((volatile unsigned int)0x00800000)
#define FLASH_OPTCR1_nWRP_8                  ((volatile unsigned int)0x01000000)
#define FLASH_OPTCR1_nWRP_9                  ((volatile unsigned int)0x02000000)
#define FLASH_OPTCR1_nWRP_10                 ((volatile unsigned int)0x04000000)
#define FLASH_OPTCR1_nWRP_11                 ((volatile unsigned int)0x08000000)

/******************************************************************************/
/*                                                                            */
/*                       Flexible Static Memory Controller                    */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for FSMC_BCR1 register  *******************/
#define  FSMC_BCR1_MBKEN                     ((volatile unsigned int)0x00000001)        /*!<Memory bank enable bit                 */
#define  FSMC_BCR1_MUXEN                     ((volatile unsigned int)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR1_MTYP                      ((volatile unsigned int)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR1_MTYP_0                    ((volatile unsigned int)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR1_MTYP_1                    ((volatile unsigned int)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR1_MWID                      ((volatile unsigned int)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR1_MWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR1_MWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR1_FACCEN                    ((volatile unsigned int)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR1_BURSTEN                   ((volatile unsigned int)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR1_WAITPOL                   ((volatile unsigned int)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR1_WRAPMOD                   ((volatile unsigned int)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR1_WAITCFG                   ((volatile unsigned int)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR1_WREN                      ((volatile unsigned int)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR1_WAITEN                    ((volatile unsigned int)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR1_EXTMOD                    ((volatile unsigned int)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR1_ASYNCWAIT                 ((volatile unsigned int)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR1_CBURSTRW                  ((volatile unsigned int)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR2 register  *******************/
#define  FSMC_BCR2_MBKEN                     ((volatile unsigned int)0x00000001)        /*!<Memory bank enable bit                */
#define  FSMC_BCR2_MUXEN                     ((volatile unsigned int)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR2_MTYP                      ((volatile unsigned int)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR2_MTYP_0                    ((volatile unsigned int)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR2_MTYP_1                    ((volatile unsigned int)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR2_MWID                      ((volatile unsigned int)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR2_MWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR2_MWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR2_FACCEN                    ((volatile unsigned int)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR2_BURSTEN                   ((volatile unsigned int)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR2_WAITPOL                   ((volatile unsigned int)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR2_WRAPMOD                   ((volatile unsigned int)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR2_WAITCFG                   ((volatile unsigned int)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR2_WREN                      ((volatile unsigned int)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR2_WAITEN                    ((volatile unsigned int)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR2_EXTMOD                    ((volatile unsigned int)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR2_ASYNCWAIT                 ((volatile unsigned int)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR2_CBURSTRW                  ((volatile unsigned int)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR3 register  *******************/
#define  FSMC_BCR3_MBKEN                     ((volatile unsigned int)0x00000001)        /*!<Memory bank enable bit                 */
#define  FSMC_BCR3_MUXEN                     ((volatile unsigned int)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR3_MTYP                      ((volatile unsigned int)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR3_MTYP_0                    ((volatile unsigned int)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR3_MTYP_1                    ((volatile unsigned int)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR3_MWID                      ((volatile unsigned int)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR3_MWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR3_MWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR3_FACCEN                    ((volatile unsigned int)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR3_BURSTEN                   ((volatile unsigned int)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR3_WAITPOL                   ((volatile unsigned int)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR3_WRAPMOD                   ((volatile unsigned int)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR3_WAITCFG                   ((volatile unsigned int)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR3_WREN                      ((volatile unsigned int)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR3_WAITEN                    ((volatile unsigned int)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR3_EXTMOD                    ((volatile unsigned int)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR3_ASYNCWAIT                 ((volatile unsigned int)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR3_CBURSTRW                  ((volatile unsigned int)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BCR4 register  *******************/
#define  FSMC_BCR4_MBKEN                     ((volatile unsigned int)0x00000001)        /*!<Memory bank enable bit */
#define  FSMC_BCR4_MUXEN                     ((volatile unsigned int)0x00000002)        /*!<Address/data multiplexing enable bit   */

#define  FSMC_BCR4_MTYP                      ((volatile unsigned int)0x0000000C)        /*!<MTYP[1:0] bits (Memory type)           */
#define  FSMC_BCR4_MTYP_0                    ((volatile unsigned int)0x00000004)        /*!<Bit 0 */
#define  FSMC_BCR4_MTYP_1                    ((volatile unsigned int)0x00000008)        /*!<Bit 1 */

#define  FSMC_BCR4_MWID                      ((volatile unsigned int)0x00000030)        /*!<MWID[1:0] bits (Memory data bus width) */
#define  FSMC_BCR4_MWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BCR4_MWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_BCR4_FACCEN                    ((volatile unsigned int)0x00000040)        /*!<Flash access enable                    */
#define  FSMC_BCR4_BURSTEN                   ((volatile unsigned int)0x00000100)        /*!<Burst enable bit                       */
#define  FSMC_BCR4_WAITPOL                   ((volatile unsigned int)0x00000200)        /*!<Wait signal polarity bit               */
#define  FSMC_BCR4_WRAPMOD                   ((volatile unsigned int)0x00000400)        /*!<Wrapped burst mode support             */
#define  FSMC_BCR4_WAITCFG                   ((volatile unsigned int)0x00000800)        /*!<Wait timing configuration              */
#define  FSMC_BCR4_WREN                      ((volatile unsigned int)0x00001000)        /*!<Write enable bit                       */
#define  FSMC_BCR4_WAITEN                    ((volatile unsigned int)0x00002000)        /*!<Wait enable bit                        */
#define  FSMC_BCR4_EXTMOD                    ((volatile unsigned int)0x00004000)        /*!<Extended mode enable                   */
#define  FSMC_BCR4_ASYNCWAIT                 ((volatile unsigned int)0x00008000)        /*!<Asynchronous wait                      */
#define  FSMC_BCR4_CBURSTRW                  ((volatile unsigned int)0x00080000)        /*!<Write burst enable                     */

/******************  Bit definition for FSMC_BTR1 register  ******************/
#define  FSMC_BTR1_ADDSET                    ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR1_ADDSET_0                  ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR1_ADDSET_1                  ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR1_ADDSET_2                  ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR1_ADDSET_3                  ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR1_ADDHLD                    ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR1_ADDHLD_0                  ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR1_ADDHLD_1                  ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR1_ADDHLD_2                  ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR1_ADDHLD_3                  ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR1_DATAST                    ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR1_DATAST_0                  ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR1_DATAST_1                  ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR1_DATAST_2                  ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR1_DATAST_3                  ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BTR1_BUSTURN                   ((volatile unsigned int)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR1_BUSTURN_0                 ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR1_BUSTURN_1                 ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR1_BUSTURN_2                 ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR1_BUSTURN_3                 ((volatile unsigned int)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR1_CLKDIV                    ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR1_CLKDIV_0                  ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR1_CLKDIV_1                  ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR1_CLKDIV_2                  ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR1_CLKDIV_3                  ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR1_DATLAT                    ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR1_DATLAT_0                  ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR1_DATLAT_1                  ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR1_DATLAT_2                  ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR1_DATLAT_3                  ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR1_ACCMOD                    ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR1_ACCMOD_0                  ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR1_ACCMOD_1                  ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BTR2 register  *******************/
#define  FSMC_BTR2_ADDSET                    ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR2_ADDSET_0                  ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR2_ADDSET_1                  ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR2_ADDSET_2                  ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR2_ADDSET_3                  ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR2_ADDHLD                    ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR2_ADDHLD_0                  ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR2_ADDHLD_1                  ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR2_ADDHLD_2                  ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR2_ADDHLD_3                  ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR2_DATAST                    ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR2_DATAST_0                  ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR2_DATAST_1                  ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR2_DATAST_2                  ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR2_DATAST_3                  ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BTR2_BUSTURN                   ((volatile unsigned int)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR2_BUSTURN_0                 ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR2_BUSTURN_1                 ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR2_BUSTURN_2                 ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR2_BUSTURN_3                 ((volatile unsigned int)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR2_CLKDIV                    ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR2_CLKDIV_0                  ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR2_CLKDIV_1                  ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR2_CLKDIV_2                  ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR2_CLKDIV_3                  ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR2_DATLAT                    ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR2_DATLAT_0                  ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR2_DATLAT_1                  ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR2_DATLAT_2                  ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR2_DATLAT_3                  ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR2_ACCMOD                    ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR2_ACCMOD_0                  ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR2_ACCMOD_1                  ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/*******************  Bit definition for FSMC_BTR3 register  *******************/
#define  FSMC_BTR3_ADDSET                    ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR3_ADDSET_0                  ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR3_ADDSET_1                  ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR3_ADDSET_2                  ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR3_ADDSET_3                  ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR3_ADDHLD                    ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR3_ADDHLD_0                  ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR3_ADDHLD_1                  ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR3_ADDHLD_2                  ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR3_ADDHLD_3                  ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR3_DATAST                    ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR3_DATAST_0                  ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR3_DATAST_1                  ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR3_DATAST_2                  ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR3_DATAST_3                  ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BTR3_BUSTURN                   ((volatile unsigned int)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR3_BUSTURN_0                 ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR3_BUSTURN_1                 ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR3_BUSTURN_2                 ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR3_BUSTURN_3                 ((volatile unsigned int)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR3_CLKDIV                    ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR3_CLKDIV_0                  ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR3_CLKDIV_1                  ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR3_CLKDIV_2                  ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR3_CLKDIV_3                  ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR3_DATLAT                    ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR3_DATLAT_0                  ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR3_DATLAT_1                  ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR3_DATLAT_2                  ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR3_DATLAT_3                  ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR3_ACCMOD                    ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR3_ACCMOD_0                  ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR3_ACCMOD_1                  ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BTR4 register  *******************/
#define  FSMC_BTR4_ADDSET                    ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BTR4_ADDSET_0                  ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BTR4_ADDSET_1                  ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BTR4_ADDSET_2                  ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BTR4_ADDSET_3                  ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BTR4_ADDHLD                    ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BTR4_ADDHLD_0                  ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BTR4_ADDHLD_1                  ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BTR4_ADDHLD_2                  ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BTR4_ADDHLD_3                  ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BTR4_DATAST                    ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BTR4_DATAST_0                  ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BTR4_DATAST_1                  ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BTR4_DATAST_2                  ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BTR4_DATAST_3                  ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BTR4_BUSTURN                   ((volatile unsigned int)0x000F0000)        /*!<BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  FSMC_BTR4_BUSTURN_0                 ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_BTR4_BUSTURN_1                 ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_BTR4_BUSTURN_2                 ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_BTR4_BUSTURN_3                 ((volatile unsigned int)0x00080000)        /*!<Bit 3 */

#define  FSMC_BTR4_CLKDIV                    ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BTR4_CLKDIV_0                  ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BTR4_CLKDIV_1                  ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BTR4_CLKDIV_2                  ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BTR4_CLKDIV_3                  ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BTR4_DATLAT                    ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BTR4_DATLAT_0                  ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BTR4_DATLAT_1                  ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BTR4_DATLAT_2                  ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BTR4_DATLAT_3                  ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BTR4_ACCMOD                    ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BTR4_ACCMOD_0                  ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BTR4_ACCMOD_1                  ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR1 register  ******************/
#define  FSMC_BWTR1_ADDSET                   ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR1_ADDSET_0                 ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR1_ADDSET_1                 ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR1_ADDSET_2                 ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR1_ADDSET_3                 ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR1_ADDHLD                   ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR1_ADDHLD_0                 ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR1_ADDHLD_1                 ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR1_ADDHLD_2                 ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR1_ADDHLD_3                 ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR1_DATAST                   ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR1_DATAST_0                 ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR1_DATAST_1                 ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR1_DATAST_2                 ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR1_DATAST_3                 ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BWTR1_CLKDIV                   ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR1_CLKDIV_0                 ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR1_CLKDIV_1                 ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BWTR1_CLKDIV_2                 ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR1_CLKDIV_3                 ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR1_DATLAT                   ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR1_DATLAT_0                 ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR1_DATLAT_1                 ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR1_DATLAT_2                 ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR1_DATLAT_3                 ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR1_ACCMOD                   ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR1_ACCMOD_0                 ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR1_ACCMOD_1                 ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR2 register  ******************/
#define  FSMC_BWTR2_ADDSET                   ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR2_ADDSET_0                 ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR2_ADDSET_1                 ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR2_ADDSET_2                 ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR2_ADDSET_3                 ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR2_ADDHLD                   ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR2_ADDHLD_0                 ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR2_ADDHLD_1                 ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR2_ADDHLD_2                 ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR2_ADDHLD_3                 ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR2_DATAST                   ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR2_DATAST_0                 ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR2_DATAST_1                 ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR2_DATAST_2                 ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR2_DATAST_3                 ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BWTR2_CLKDIV                   ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR2_CLKDIV_0                 ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR2_CLKDIV_1                 ((volatile unsigned int)0x00200000)        /*!<Bit 1*/
#define  FSMC_BWTR2_CLKDIV_2                 ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR2_CLKDIV_3                 ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR2_DATLAT                   ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR2_DATLAT_0                 ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR2_DATLAT_1                 ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR2_DATLAT_2                 ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR2_DATLAT_3                 ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR2_ACCMOD                   ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR2_ACCMOD_0                 ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR2_ACCMOD_1                 ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR3 register  ******************/
#define  FSMC_BWTR3_ADDSET                   ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR3_ADDSET_0                 ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR3_ADDSET_1                 ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR3_ADDSET_2                 ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR3_ADDSET_3                 ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR3_ADDHLD                   ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR3_ADDHLD_0                 ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR3_ADDHLD_1                 ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR3_ADDHLD_2                 ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR3_ADDHLD_3                 ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR3_DATAST                   ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR3_DATAST_0                 ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR3_DATAST_1                 ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR3_DATAST_2                 ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR3_DATAST_3                 ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BWTR3_CLKDIV                   ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR3_CLKDIV_0                 ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR3_CLKDIV_1                 ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BWTR3_CLKDIV_2                 ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR3_CLKDIV_3                 ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR3_DATLAT                   ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR3_DATLAT_0                 ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR3_DATLAT_1                 ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR3_DATLAT_2                 ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR3_DATLAT_3                 ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR3_ACCMOD                   ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR3_ACCMOD_0                 ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR3_ACCMOD_1                 ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_BWTR4 register  ******************/
#define  FSMC_BWTR4_ADDSET                   ((volatile unsigned int)0x0000000F)        /*!<ADDSET[3:0] bits (Address setup phase duration) */
#define  FSMC_BWTR4_ADDSET_0                 ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_BWTR4_ADDSET_1                 ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_BWTR4_ADDSET_2                 ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_BWTR4_ADDSET_3                 ((volatile unsigned int)0x00000008)        /*!<Bit 3 */

#define  FSMC_BWTR4_ADDHLD                   ((volatile unsigned int)0x000000F0)        /*!<ADDHLD[3:0] bits (Address-hold phase duration) */
#define  FSMC_BWTR4_ADDHLD_0                 ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_BWTR4_ADDHLD_1                 ((volatile unsigned int)0x00000020)        /*!<Bit 1 */
#define  FSMC_BWTR4_ADDHLD_2                 ((volatile unsigned int)0x00000040)        /*!<Bit 2 */
#define  FSMC_BWTR4_ADDHLD_3                 ((volatile unsigned int)0x00000080)        /*!<Bit 3 */

#define  FSMC_BWTR4_DATAST                   ((volatile unsigned int)0x0000FF00)        /*!<DATAST [3:0] bits (Data-phase duration) */
#define  FSMC_BWTR4_DATAST_0                 ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_BWTR4_DATAST_1                 ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_BWTR4_DATAST_2                 ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_BWTR4_DATAST_3                 ((volatile unsigned int)0x00000800)        /*!<Bit 3 */

#define  FSMC_BWTR4_CLKDIV                   ((volatile unsigned int)0x00F00000)        /*!<CLKDIV[3:0] bits (Clock divide ratio) */
#define  FSMC_BWTR4_CLKDIV_0                 ((volatile unsigned int)0x00100000)        /*!<Bit 0 */
#define  FSMC_BWTR4_CLKDIV_1                 ((volatile unsigned int)0x00200000)        /*!<Bit 1 */
#define  FSMC_BWTR4_CLKDIV_2                 ((volatile unsigned int)0x00400000)        /*!<Bit 2 */
#define  FSMC_BWTR4_CLKDIV_3                 ((volatile unsigned int)0x00800000)        /*!<Bit 3 */

#define  FSMC_BWTR4_DATLAT                   ((volatile unsigned int)0x0F000000)        /*!<DATLA[3:0] bits (Data latency) */
#define  FSMC_BWTR4_DATLAT_0                 ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_BWTR4_DATLAT_1                 ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_BWTR4_DATLAT_2                 ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_BWTR4_DATLAT_3                 ((volatile unsigned int)0x08000000)        /*!<Bit 3 */

#define  FSMC_BWTR4_ACCMOD                   ((volatile unsigned int)0x30000000)        /*!<ACCMOD[1:0] bits (Access mode) */
#define  FSMC_BWTR4_ACCMOD_0                 ((volatile unsigned int)0x10000000)        /*!<Bit 0 */
#define  FSMC_BWTR4_ACCMOD_1                 ((volatile unsigned int)0x20000000)        /*!<Bit 1 */

/******************  Bit definition for FSMC_PCR2 register  *******************/
#define  FSMC_PCR2_PWAITEN                   ((volatile unsigned int)0x00000002)        /*!<Wait feature enable bit */
#define  FSMC_PCR2_PBKEN                     ((volatile unsigned int)0x00000004)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR2_PTYP                      ((volatile unsigned int)0x00000008)        /*!<Memory type */

#define  FSMC_PCR2_PWID                      ((volatile unsigned int)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR2_PWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_PCR2_PWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_PCR2_ECCEN                     ((volatile unsigned int)0x00000040)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR2_TCLR                      ((volatile unsigned int)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR2_TCLR_0                    ((volatile unsigned int)0x00000200)        /*!<Bit 0 */
#define  FSMC_PCR2_TCLR_1                    ((volatile unsigned int)0x00000400)        /*!<Bit 1 */
#define  FSMC_PCR2_TCLR_2                    ((volatile unsigned int)0x00000800)        /*!<Bit 2 */
#define  FSMC_PCR2_TCLR_3                    ((volatile unsigned int)0x00001000)        /*!<Bit 3 */

#define  FSMC_PCR2_TAR                       ((volatile unsigned int)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR2_TAR_0                     ((volatile unsigned int)0x00002000)        /*!<Bit 0 */
#define  FSMC_PCR2_TAR_1                     ((volatile unsigned int)0x00004000)        /*!<Bit 1 */
#define  FSMC_PCR2_TAR_2                     ((volatile unsigned int)0x00008000)        /*!<Bit 2 */
#define  FSMC_PCR2_TAR_3                     ((volatile unsigned int)0x00010000)        /*!<Bit 3 */

#define  FSMC_PCR2_ECCPS                     ((volatile unsigned int)0x000E0000)        /*!<ECCPS[1:0] bits (ECC page size) */
#define  FSMC_PCR2_ECCPS_0                   ((volatile unsigned int)0x00020000)        /*!<Bit 0 */
#define  FSMC_PCR2_ECCPS_1                   ((volatile unsigned int)0x00040000)        /*!<Bit 1 */
#define  FSMC_PCR2_ECCPS_2                   ((volatile unsigned int)0x00080000)        /*!<Bit 2 */

/******************  Bit definition for FSMC_PCR3 register  *******************/
#define  FSMC_PCR3_PWAITEN                   ((volatile unsigned int)0x00000002)        /*!<Wait feature enable bit */
#define  FSMC_PCR3_PBKEN                     ((volatile unsigned int)0x00000004)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR3_PTYP                      ((volatile unsigned int)0x00000008)        /*!<Memory type */

#define  FSMC_PCR3_PWID                      ((volatile unsigned int)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR3_PWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_PCR3_PWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_PCR3_ECCEN                     ((volatile unsigned int)0x00000040)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR3_TCLR                      ((volatile unsigned int)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR3_TCLR_0                    ((volatile unsigned int)0x00000200)        /*!<Bit 0 */
#define  FSMC_PCR3_TCLR_1                    ((volatile unsigned int)0x00000400)        /*!<Bit 1 */
#define  FSMC_PCR3_TCLR_2                    ((volatile unsigned int)0x00000800)        /*!<Bit 2 */
#define  FSMC_PCR3_TCLR_3                    ((volatile unsigned int)0x00001000)        /*!<Bit 3 */

#define  FSMC_PCR3_TAR                       ((volatile unsigned int)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR3_TAR_0                     ((volatile unsigned int)0x00002000)        /*!<Bit 0 */
#define  FSMC_PCR3_TAR_1                     ((volatile unsigned int)0x00004000)        /*!<Bit 1 */
#define  FSMC_PCR3_TAR_2                     ((volatile unsigned int)0x00008000)        /*!<Bit 2 */
#define  FSMC_PCR3_TAR_3                     ((volatile unsigned int)0x00010000)        /*!<Bit 3 */

#define  FSMC_PCR3_ECCPS                     ((volatile unsigned int)0x000E0000)        /*!<ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR3_ECCPS_0                   ((volatile unsigned int)0x00020000)        /*!<Bit 0 */
#define  FSMC_PCR3_ECCPS_1                   ((volatile unsigned int)0x00040000)        /*!<Bit 1 */
#define  FSMC_PCR3_ECCPS_2                   ((volatile unsigned int)0x00080000)        /*!<Bit 2 */

/******************  Bit definition for FSMC_PCR4 register  *******************/
#define  FSMC_PCR4_PWAITEN                   ((volatile unsigned int)0x00000002)        /*!<Wait feature enable bit */
#define  FSMC_PCR4_PBKEN                     ((volatile unsigned int)0x00000004)        /*!<PC Card/NAND Flash memory bank enable bit */
#define  FSMC_PCR4_PTYP                      ((volatile unsigned int)0x00000008)        /*!<Memory type */

#define  FSMC_PCR4_PWID                      ((volatile unsigned int)0x00000030)        /*!<PWID[1:0] bits (NAND Flash databus width) */
#define  FSMC_PCR4_PWID_0                    ((volatile unsigned int)0x00000010)        /*!<Bit 0 */
#define  FSMC_PCR4_PWID_1                    ((volatile unsigned int)0x00000020)        /*!<Bit 1 */

#define  FSMC_PCR4_ECCEN                     ((volatile unsigned int)0x00000040)        /*!<ECC computation logic enable bit */

#define  FSMC_PCR4_TCLR                      ((volatile unsigned int)0x00001E00)        /*!<TCLR[3:0] bits (CLE to RE delay) */
#define  FSMC_PCR4_TCLR_0                    ((volatile unsigned int)0x00000200)        /*!<Bit 0 */
#define  FSMC_PCR4_TCLR_1                    ((volatile unsigned int)0x00000400)        /*!<Bit 1 */
#define  FSMC_PCR4_TCLR_2                    ((volatile unsigned int)0x00000800)        /*!<Bit 2 */
#define  FSMC_PCR4_TCLR_3                    ((volatile unsigned int)0x00001000)        /*!<Bit 3 */

#define  FSMC_PCR4_TAR                       ((volatile unsigned int)0x0001E000)        /*!<TAR[3:0] bits (ALE to RE delay) */
#define  FSMC_PCR4_TAR_0                     ((volatile unsigned int)0x00002000)        /*!<Bit 0 */
#define  FSMC_PCR4_TAR_1                     ((volatile unsigned int)0x00004000)        /*!<Bit 1 */
#define  FSMC_PCR4_TAR_2                     ((volatile unsigned int)0x00008000)        /*!<Bit 2 */
#define  FSMC_PCR4_TAR_3                     ((volatile unsigned int)0x00010000)        /*!<Bit 3 */

#define  FSMC_PCR4_ECCPS                     ((volatile unsigned int)0x000E0000)        /*!<ECCPS[2:0] bits (ECC page size) */
#define  FSMC_PCR4_ECCPS_0                   ((volatile unsigned int)0x00020000)        /*!<Bit 0 */
#define  FSMC_PCR4_ECCPS_1                   ((volatile unsigned int)0x00040000)        /*!<Bit 1 */
#define  FSMC_PCR4_ECCPS_2                   ((volatile unsigned int)0x00080000)        /*!<Bit 2 */

/*******************  Bit definition for FSMC_SR2 register  *******************/
#define  FSMC_SR2_IRS                        ((volatile unsigned int)0x01)               /*!<Interrupt Rising Edge status                */
#define  FSMC_SR2_ILS                        ((volatile unsigned int)0x02)               /*!<Interrupt Level status                      */
#define  FSMC_SR2_IFS                        ((volatile unsigned int)0x04)               /*!<Interrupt Falling Edge status               */
#define  FSMC_SR2_IREN                       ((volatile unsigned int)0x08)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FSMC_SR2_ILEN                       ((volatile unsigned int)0x10)               /*!<Interrupt Level detection Enable bit        */
#define  FSMC_SR2_IFEN                       ((volatile unsigned int)0x20)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR2_FEMPT                      ((volatile unsigned int)0x40)               /*!<FIFO empty */

/*******************  Bit definition for FSMC_SR3 register  *******************/
#define  FSMC_SR3_IRS                        ((volatile unsigned int)0x01)               /*!<Interrupt Rising Edge status                */
#define  FSMC_SR3_ILS                        ((volatile unsigned int)0x02)               /*!<Interrupt Level status                      */
#define  FSMC_SR3_IFS                        ((volatile unsigned int)0x04)               /*!<Interrupt Falling Edge status               */
#define  FSMC_SR3_IREN                       ((volatile unsigned int)0x08)               /*!<Interrupt Rising Edge detection Enable bit  */
#define  FSMC_SR3_ILEN                       ((volatile unsigned int)0x10)               /*!<Interrupt Level detection Enable bit        */
#define  FSMC_SR3_IFEN                       ((volatile unsigned int)0x20)               /*!<Interrupt Falling Edge detection Enable bit */
#define  FSMC_SR3_FEMPT                      ((volatile unsigned int)0x40)               /*!<FIFO empty */

/*******************  Bit definition for FSMC_SR4 register  *******************/
#define  FSMC_SR4_IRS                        ((volatile unsigned int)0x01)               /*!<Interrupt Rising Edge status                 */
#define  FSMC_SR4_ILS                        ((volatile unsigned int)0x02)               /*!<Interrupt Level status                       */
#define  FSMC_SR4_IFS                        ((volatile unsigned int)0x04)               /*!<Interrupt Falling Edge status                */
#define  FSMC_SR4_IREN                       ((volatile unsigned int)0x08)               /*!<Interrupt Rising Edge detection Enable bit   */
#define  FSMC_SR4_ILEN                       ((volatile unsigned int)0x10)               /*!<Interrupt Level detection Enable bit         */
#define  FSMC_SR4_IFEN                       ((volatile unsigned int)0x20)               /*!<Interrupt Falling Edge detection Enable bit  */
#define  FSMC_SR4_FEMPT                      ((volatile unsigned int)0x40)               /*!<FIFO empty */

/******************  Bit definition for FSMC_PMEM2 register  ******************/
#define  FSMC_PMEM2_MEMSET2                  ((volatile unsigned int)0x000000FF)        /*!<MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  FSMC_PMEM2_MEMSET2_0                ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMSET2_1                ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMSET2_2                ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMSET2_3                ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMSET2_4                ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMSET2_5                ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMSET2_6                ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMSET2_7                ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMWAIT2                 ((volatile unsigned int)0x0000FF00)        /*!<MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  FSMC_PMEM2_MEMWAIT2_0               ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMWAIT2_1               ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMWAIT2_2               ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMWAIT2_3               ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMWAIT2_4               ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMWAIT2_5               ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMWAIT2_6               ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMWAIT2_7               ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMHOLD2                 ((volatile unsigned int)0x00FF0000)        /*!<MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  FSMC_PMEM2_MEMHOLD2_0               ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMHOLD2_1               ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMHOLD2_2               ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMHOLD2_3               ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMHOLD2_4               ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMHOLD2_5               ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMHOLD2_6               ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMHOLD2_7               ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PMEM2_MEMHIZ2                  ((volatile unsigned int)0xFF000000)        /*!<MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  FSMC_PMEM2_MEMHIZ2_0                ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PMEM2_MEMHIZ2_1                ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PMEM2_MEMHIZ2_2                ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PMEM2_MEMHIZ2_3                ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PMEM2_MEMHIZ2_4                ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PMEM2_MEMHIZ2_5                ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PMEM2_MEMHIZ2_6                ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PMEM2_MEMHIZ2_7                ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PMEM3 register  ******************/
#define  FSMC_PMEM3_MEMSET3                  ((volatile unsigned int)0x000000FF)        /*!<MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  FSMC_PMEM3_MEMSET3_0                ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMSET3_1                ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMSET3_2                ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMSET3_3                ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMSET3_4                ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMSET3_5                ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMSET3_6                ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMSET3_7                ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMWAIT3                 ((volatile unsigned int)0x0000FF00)        /*!<MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  FSMC_PMEM3_MEMWAIT3_0               ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMWAIT3_1               ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMWAIT3_2               ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMWAIT3_3               ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMWAIT3_4               ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMWAIT3_5               ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMWAIT3_6               ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMWAIT3_7               ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMHOLD3                 ((volatile unsigned int)0x00FF0000)        /*!<MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  FSMC_PMEM3_MEMHOLD3_0               ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMHOLD3_1               ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMHOLD3_2               ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMHOLD3_3               ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMHOLD3_4               ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMHOLD3_5               ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMHOLD3_6               ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMHOLD3_7               ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PMEM3_MEMHIZ3                  ((volatile unsigned int)0xFF000000)        /*!<MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  FSMC_PMEM3_MEMHIZ3_0                ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PMEM3_MEMHIZ3_1                ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PMEM3_MEMHIZ3_2                ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PMEM3_MEMHIZ3_3                ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PMEM3_MEMHIZ3_4                ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PMEM3_MEMHIZ3_5                ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PMEM3_MEMHIZ3_6                ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PMEM3_MEMHIZ3_7                ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PMEM4 register  ******************/
#define  FSMC_PMEM4_MEMSET4                  ((volatile unsigned int)0x000000FF)        /*!<MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  FSMC_PMEM4_MEMSET4_0                ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMSET4_1                ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMSET4_2                ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMSET4_3                ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMSET4_4                ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMSET4_5                ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMSET4_6                ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMSET4_7                ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMWAIT4                 ((volatile unsigned int)0x0000FF00)        /*!<MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  FSMC_PMEM4_MEMWAIT4_0               ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMWAIT4_1               ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMWAIT4_2               ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMWAIT4_3               ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMWAIT4_4               ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMWAIT4_5               ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMWAIT4_6               ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMWAIT4_7               ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMHOLD4                 ((volatile unsigned int)0x00FF0000)        /*!<MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  FSMC_PMEM4_MEMHOLD4_0               ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMHOLD4_1               ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMHOLD4_2               ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMHOLD4_3               ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMHOLD4_4               ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMHOLD4_5               ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMHOLD4_6               ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMHOLD4_7               ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PMEM4_MEMHIZ4                  ((volatile unsigned int)0xFF000000)        /*!<MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  FSMC_PMEM4_MEMHIZ4_0                ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PMEM4_MEMHIZ4_1                ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PMEM4_MEMHIZ4_2                ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PMEM4_MEMHIZ4_3                ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PMEM4_MEMHIZ4_4                ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PMEM4_MEMHIZ4_5                ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PMEM4_MEMHIZ4_6                ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PMEM4_MEMHIZ4_7                ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT2 register  ******************/
#define  FSMC_PATT2_ATTSET2                  ((volatile unsigned int)0x000000FF)        /*!<ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  FSMC_PATT2_ATTSET2_0                ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTSET2_1                ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTSET2_2                ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTSET2_3                ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTSET2_4                ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTSET2_5                ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTSET2_6                ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTSET2_7                ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTWAIT2                 ((volatile unsigned int)0x0000FF00)        /*!<ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  FSMC_PATT2_ATTWAIT2_0               ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTWAIT2_1               ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTWAIT2_2               ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTWAIT2_3               ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTWAIT2_4               ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTWAIT2_5               ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTWAIT2_6               ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTWAIT2_7               ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTHOLD2                 ((volatile unsigned int)0x00FF0000)        /*!<ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  FSMC_PATT2_ATTHOLD2_0               ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTHOLD2_1               ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTHOLD2_2               ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTHOLD2_3               ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTHOLD2_4               ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTHOLD2_5               ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTHOLD2_6               ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTHOLD2_7               ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PATT2_ATTHIZ2                  ((volatile unsigned int)0xFF000000)        /*!<ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  FSMC_PATT2_ATTHIZ2_0                ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PATT2_ATTHIZ2_1                ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PATT2_ATTHIZ2_2                ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PATT2_ATTHIZ2_3                ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PATT2_ATTHIZ2_4                ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PATT2_ATTHIZ2_5                ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PATT2_ATTHIZ2_6                ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PATT2_ATTHIZ2_7                ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT3 register  ******************/
#define  FSMC_PATT3_ATTSET3                  ((volatile unsigned int)0x000000FF)        /*!<ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  FSMC_PATT3_ATTSET3_0                ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTSET3_1                ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTSET3_2                ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTSET3_3                ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTSET3_4                ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTSET3_5                ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTSET3_6                ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTSET3_7                ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTWAIT3                 ((volatile unsigned int)0x0000FF00)        /*!<ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  FSMC_PATT3_ATTWAIT3_0               ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTWAIT3_1               ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTWAIT3_2               ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTWAIT3_3               ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTWAIT3_4               ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTWAIT3_5               ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTWAIT3_6               ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTWAIT3_7               ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTHOLD3                 ((volatile unsigned int)0x00FF0000)        /*!<ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  FSMC_PATT3_ATTHOLD3_0               ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTHOLD3_1               ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTHOLD3_2               ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTHOLD3_3               ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTHOLD3_4               ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTHOLD3_5               ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTHOLD3_6               ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTHOLD3_7               ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PATT3_ATTHIZ3                  ((volatile unsigned int)0xFF000000)        /*!<ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  FSMC_PATT3_ATTHIZ3_0                ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PATT3_ATTHIZ3_1                ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PATT3_ATTHIZ3_2                ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PATT3_ATTHIZ3_3                ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PATT3_ATTHIZ3_4                ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PATT3_ATTHIZ3_5                ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PATT3_ATTHIZ3_6                ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PATT3_ATTHIZ3_7                ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PATT4 register  ******************/
#define  FSMC_PATT4_ATTSET4                  ((volatile unsigned int)0x000000FF)        /*!<ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  FSMC_PATT4_ATTSET4_0                ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTSET4_1                ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTSET4_2                ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTSET4_3                ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTSET4_4                ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTSET4_5                ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTSET4_6                ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTSET4_7                ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTWAIT4                 ((volatile unsigned int)0x0000FF00)        /*!<ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  FSMC_PATT4_ATTWAIT4_0               ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTWAIT4_1               ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTWAIT4_2               ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTWAIT4_3               ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTWAIT4_4               ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTWAIT4_5               ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTWAIT4_6               ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTWAIT4_7               ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTHOLD4                 ((volatile unsigned int)0x00FF0000)        /*!<ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  FSMC_PATT4_ATTHOLD4_0               ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTHOLD4_1               ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTHOLD4_2               ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTHOLD4_3               ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTHOLD4_4               ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTHOLD4_5               ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTHOLD4_6               ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTHOLD4_7               ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PATT4_ATTHIZ4                  ((volatile unsigned int)0xFF000000)        /*!<ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  FSMC_PATT4_ATTHIZ4_0                ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PATT4_ATTHIZ4_1                ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PATT4_ATTHIZ4_2                ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PATT4_ATTHIZ4_3                ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PATT4_ATTHIZ4_4                ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PATT4_ATTHIZ4_5                ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PATT4_ATTHIZ4_6                ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PATT4_ATTHIZ4_7                ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_PIO4 register  *******************/
#define  FSMC_PIO4_IOSET4                    ((volatile unsigned int)0x000000FF)        /*!<IOSET4[7:0] bits (I/O 4 setup time) */
#define  FSMC_PIO4_IOSET4_0                  ((volatile unsigned int)0x00000001)        /*!<Bit 0 */
#define  FSMC_PIO4_IOSET4_1                  ((volatile unsigned int)0x00000002)        /*!<Bit 1 */
#define  FSMC_PIO4_IOSET4_2                  ((volatile unsigned int)0x00000004)        /*!<Bit 2 */
#define  FSMC_PIO4_IOSET4_3                  ((volatile unsigned int)0x00000008)        /*!<Bit 3 */
#define  FSMC_PIO4_IOSET4_4                  ((volatile unsigned int)0x00000010)        /*!<Bit 4 */
#define  FSMC_PIO4_IOSET4_5                  ((volatile unsigned int)0x00000020)        /*!<Bit 5 */
#define  FSMC_PIO4_IOSET4_6                  ((volatile unsigned int)0x00000040)        /*!<Bit 6 */
#define  FSMC_PIO4_IOSET4_7                  ((volatile unsigned int)0x00000080)        /*!<Bit 7 */

#define  FSMC_PIO4_IOWAIT4                   ((volatile unsigned int)0x0000FF00)        /*!<IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  FSMC_PIO4_IOWAIT4_0                 ((volatile unsigned int)0x00000100)        /*!<Bit 0 */
#define  FSMC_PIO4_IOWAIT4_1                 ((volatile unsigned int)0x00000200)        /*!<Bit 1 */
#define  FSMC_PIO4_IOWAIT4_2                 ((volatile unsigned int)0x00000400)        /*!<Bit 2 */
#define  FSMC_PIO4_IOWAIT4_3                 ((volatile unsigned int)0x00000800)        /*!<Bit 3 */
#define  FSMC_PIO4_IOWAIT4_4                 ((volatile unsigned int)0x00001000)        /*!<Bit 4 */
#define  FSMC_PIO4_IOWAIT4_5                 ((volatile unsigned int)0x00002000)        /*!<Bit 5 */
#define  FSMC_PIO4_IOWAIT4_6                 ((volatile unsigned int)0x00004000)        /*!<Bit 6 */
#define  FSMC_PIO4_IOWAIT4_7                 ((volatile unsigned int)0x00008000)        /*!<Bit 7 */

#define  FSMC_PIO4_IOHOLD4                   ((volatile unsigned int)0x00FF0000)        /*!<IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  FSMC_PIO4_IOHOLD4_0                 ((volatile unsigned int)0x00010000)        /*!<Bit 0 */
#define  FSMC_PIO4_IOHOLD4_1                 ((volatile unsigned int)0x00020000)        /*!<Bit 1 */
#define  FSMC_PIO4_IOHOLD4_2                 ((volatile unsigned int)0x00040000)        /*!<Bit 2 */
#define  FSMC_PIO4_IOHOLD4_3                 ((volatile unsigned int)0x00080000)        /*!<Bit 3 */
#define  FSMC_PIO4_IOHOLD4_4                 ((volatile unsigned int)0x00100000)        /*!<Bit 4 */
#define  FSMC_PIO4_IOHOLD4_5                 ((volatile unsigned int)0x00200000)        /*!<Bit 5 */
#define  FSMC_PIO4_IOHOLD4_6                 ((volatile unsigned int)0x00400000)        /*!<Bit 6 */
#define  FSMC_PIO4_IOHOLD4_7                 ((volatile unsigned int)0x00800000)        /*!<Bit 7 */

#define  FSMC_PIO4_IOHIZ4                    ((volatile unsigned int)0xFF000000)        /*!<IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  FSMC_PIO4_IOHIZ4_0                  ((volatile unsigned int)0x01000000)        /*!<Bit 0 */
#define  FSMC_PIO4_IOHIZ4_1                  ((volatile unsigned int)0x02000000)        /*!<Bit 1 */
#define  FSMC_PIO4_IOHIZ4_2                  ((volatile unsigned int)0x04000000)        /*!<Bit 2 */
#define  FSMC_PIO4_IOHIZ4_3                  ((volatile unsigned int)0x08000000)        /*!<Bit 3 */
#define  FSMC_PIO4_IOHIZ4_4                  ((volatile unsigned int)0x10000000)        /*!<Bit 4 */
#define  FSMC_PIO4_IOHIZ4_5                  ((volatile unsigned int)0x20000000)        /*!<Bit 5 */
#define  FSMC_PIO4_IOHIZ4_6                  ((volatile unsigned int)0x40000000)        /*!<Bit 6 */
#define  FSMC_PIO4_IOHIZ4_7                  ((volatile unsigned int)0x80000000)        /*!<Bit 7 */

/******************  Bit definition for FSMC_ECCR2 register  ******************/
#define  FSMC_ECCR2_ECC2                     ((volatile unsigned int)0xFFFFFFFF)        /*!<ECC result */

/******************  Bit definition for FSMC_ECCR3 register  ******************/
#define  FSMC_ECCR3_ECC3                     ((volatile unsigned int)0xFFFFFFFF)        /*!<ECC result */

/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0                    ((volatile unsigned int)0x00000003)
#define GPIO_MODER_MODER0_0                  ((volatile unsigned int)0x00000001)
#define GPIO_MODER_MODER0_1                  ((volatile unsigned int)0x00000002)

#define GPIO_MODER_MODER1                    ((volatile unsigned int)0x0000000C)
#define GPIO_MODER_MODER1_0                  ((volatile unsigned int)0x00000004)
#define GPIO_MODER_MODER1_1                  ((volatile unsigned int)0x00000008)

#define GPIO_MODER_MODER2                    ((volatile unsigned int)0x00000030)
#define GPIO_MODER_MODER2_0                  ((volatile unsigned int)0x00000010)
#define GPIO_MODER_MODER2_1                  ((volatile unsigned int)0x00000020)

#define GPIO_MODER_MODER3                    ((volatile unsigned int)0x000000C0)
#define GPIO_MODER_MODER3_0                  ((volatile unsigned int)0x00000040)
#define GPIO_MODER_MODER3_1                  ((volatile unsigned int)0x00000080)

#define GPIO_MODER_MODER4                    ((volatile unsigned int)0x00000300)
#define GPIO_MODER_MODER4_0                  ((volatile unsigned int)0x00000100)
#define GPIO_MODER_MODER4_1                  ((volatile unsigned int)0x00000200)

#define GPIO_MODER_MODER5                    ((volatile unsigned int)0x00000C00)
#define GPIO_MODER_MODER5_0                  ((volatile unsigned int)0x00000400)
#define GPIO_MODER_MODER5_1                  ((volatile unsigned int)0x00000800)

#define GPIO_MODER_MODER6                    ((volatile unsigned int)0x00003000)
#define GPIO_MODER_MODER6_0                  ((volatile unsigned int)0x00001000)
#define GPIO_MODER_MODER6_1                  ((volatile unsigned int)0x00002000)

#define GPIO_MODER_MODER7                    ((volatile unsigned int)0x0000C000)
#define GPIO_MODER_MODER7_0                  ((volatile unsigned int)0x00004000)
#define GPIO_MODER_MODER7_1                  ((volatile unsigned int)0x00008000)

#define GPIO_MODER_MODER8                    ((volatile unsigned int)0x00030000)
#define GPIO_MODER_MODER8_0                  ((volatile unsigned int)0x00010000)
#define GPIO_MODER_MODER8_1                  ((volatile unsigned int)0x00020000)

#define GPIO_MODER_MODER9                    ((volatile unsigned int)0x000C0000)
#define GPIO_MODER_MODER9_0                  ((volatile unsigned int)0x00040000)
#define GPIO_MODER_MODER9_1                  ((volatile unsigned int)0x00080000)

#define GPIO_MODER_MODER10                   ((volatile unsigned int)0x00300000)
#define GPIO_MODER_MODER10_0                 ((volatile unsigned int)0x00100000)
#define GPIO_MODER_MODER10_1                 ((volatile unsigned int)0x00200000)

#define GPIO_MODER_MODER11                   ((volatile unsigned int)0x00C00000)
#define GPIO_MODER_MODER11_0                 ((volatile unsigned int)0x00400000)
#define GPIO_MODER_MODER11_1                 ((volatile unsigned int)0x00800000)

#define GPIO_MODER_MODER12                   ((volatile unsigned int)0x03000000)
#define GPIO_MODER_MODER12_0                 ((volatile unsigned int)0x01000000)
#define GPIO_MODER_MODER12_1                 ((volatile unsigned int)0x02000000)

#define GPIO_MODER_MODER13                   ((volatile unsigned int)0x0C000000)
#define GPIO_MODER_MODER13_0                 ((volatile unsigned int)0x04000000)
#define GPIO_MODER_MODER13_1                 ((volatile unsigned int)0x08000000)

#define GPIO_MODER_MODER14                   ((volatile unsigned int)0x30000000)
#define GPIO_MODER_MODER14_0                 ((volatile unsigned int)0x10000000)
#define GPIO_MODER_MODER14_1                 ((volatile unsigned int)0x20000000)

#define GPIO_MODER_MODER15                   ((volatile unsigned int)0xC0000000)
#define GPIO_MODER_MODER15_0                 ((volatile unsigned int)0x40000000)
#define GPIO_MODER_MODER15_1                 ((volatile unsigned int)0x80000000)

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT_0                     ((volatile unsigned int)0x00000001)
#define GPIO_OTYPER_OT_1                     ((volatile unsigned int)0x00000002)
#define GPIO_OTYPER_OT_2                     ((volatile unsigned int)0x00000004)
#define GPIO_OTYPER_OT_3                     ((volatile unsigned int)0x00000008)
#define GPIO_OTYPER_OT_4                     ((volatile unsigned int)0x00000010)
#define GPIO_OTYPER_OT_5                     ((volatile unsigned int)0x00000020)
#define GPIO_OTYPER_OT_6                     ((volatile unsigned int)0x00000040)
#define GPIO_OTYPER_OT_7                     ((volatile unsigned int)0x00000080)
#define GPIO_OTYPER_OT_8                     ((volatile unsigned int)0x00000100)
#define GPIO_OTYPER_OT_9                     ((volatile unsigned int)0x00000200)
#define GPIO_OTYPER_OT_10                    ((volatile unsigned int)0x00000400)
#define GPIO_OTYPER_OT_11                    ((volatile unsigned int)0x00000800)
#define GPIO_OTYPER_OT_12                    ((volatile unsigned int)0x00001000)
#define GPIO_OTYPER_OT_13                    ((volatile unsigned int)0x00002000)
#define GPIO_OTYPER_OT_14                    ((volatile unsigned int)0x00004000)
#define GPIO_OTYPER_OT_15                    ((volatile unsigned int)0x00008000)

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDER_OSPEEDR0               ((volatile unsigned int)0x00000003)
#define GPIO_OSPEEDER_OSPEEDR0_0             ((volatile unsigned int)0x00000001)
#define GPIO_OSPEEDER_OSPEEDR0_1             ((volatile unsigned int)0x00000002)

#define GPIO_OSPEEDER_OSPEEDR1               ((volatile unsigned int)0x0000000C)
#define GPIO_OSPEEDER_OSPEEDR1_0             ((volatile unsigned int)0x00000004)
#define GPIO_OSPEEDER_OSPEEDR1_1             ((volatile unsigned int)0x00000008)

#define GPIO_OSPEEDER_OSPEEDR2               ((volatile unsigned int)0x00000030)
#define GPIO_OSPEEDER_OSPEEDR2_0             ((volatile unsigned int)0x00000010)
#define GPIO_OSPEEDER_OSPEEDR2_1             ((volatile unsigned int)0x00000020)

#define GPIO_OSPEEDER_OSPEEDR3               ((volatile unsigned int)0x000000C0)
#define GPIO_OSPEEDER_OSPEEDR3_0             ((volatile unsigned int)0x00000040)
#define GPIO_OSPEEDER_OSPEEDR3_1             ((volatile unsigned int)0x00000080)

#define GPIO_OSPEEDER_OSPEEDR4               ((volatile unsigned int)0x00000300)
#define GPIO_OSPEEDER_OSPEEDR4_0             ((volatile unsigned int)0x00000100)
#define GPIO_OSPEEDER_OSPEEDR4_1             ((volatile unsigned int)0x00000200)

#define GPIO_OSPEEDER_OSPEEDR5               ((volatile unsigned int)0x00000C00)
#define GPIO_OSPEEDER_OSPEEDR5_0             ((volatile unsigned int)0x00000400)
#define GPIO_OSPEEDER_OSPEEDR5_1             ((volatile unsigned int)0x00000800)

#define GPIO_OSPEEDER_OSPEEDR6               ((volatile unsigned int)0x00003000)
#define GPIO_OSPEEDER_OSPEEDR6_0             ((volatile unsigned int)0x00001000)
#define GPIO_OSPEEDER_OSPEEDR6_1             ((volatile unsigned int)0x00002000)

#define GPIO_OSPEEDER_OSPEEDR7               ((volatile unsigned int)0x0000C000)
#define GPIO_OSPEEDER_OSPEEDR7_0             ((volatile unsigned int)0x00004000)
#define GPIO_OSPEEDER_OSPEEDR7_1             ((volatile unsigned int)0x00008000)

#define GPIO_OSPEEDER_OSPEEDR8               ((volatile unsigned int)0x00030000)
#define GPIO_OSPEEDER_OSPEEDR8_0             ((volatile unsigned int)0x00010000)
#define GPIO_OSPEEDER_OSPEEDR8_1             ((volatile unsigned int)0x00020000)

#define GPIO_OSPEEDER_OSPEEDR9               ((volatile unsigned int)0x000C0000)
#define GPIO_OSPEEDER_OSPEEDR9_0             ((volatile unsigned int)0x00040000)
#define GPIO_OSPEEDER_OSPEEDR9_1             ((volatile unsigned int)0x00080000)

#define GPIO_OSPEEDER_OSPEEDR10              ((volatile unsigned int)0x00300000)
#define GPIO_OSPEEDER_OSPEEDR10_0            ((volatile unsigned int)0x00100000)
#define GPIO_OSPEEDER_OSPEEDR10_1            ((volatile unsigned int)0x00200000)

#define GPIO_OSPEEDER_OSPEEDR11              ((volatile unsigned int)0x00C00000)
#define GPIO_OSPEEDER_OSPEEDR11_0            ((volatile unsigned int)0x00400000)
#define GPIO_OSPEEDER_OSPEEDR11_1            ((volatile unsigned int)0x00800000)

#define GPIO_OSPEEDER_OSPEEDR12              ((volatile unsigned int)0x03000000)
#define GPIO_OSPEEDER_OSPEEDR12_0            ((volatile unsigned int)0x01000000)
#define GPIO_OSPEEDER_OSPEEDR12_1            ((volatile unsigned int)0x02000000)

#define GPIO_OSPEEDER_OSPEEDR13              ((volatile unsigned int)0x0C000000)
#define GPIO_OSPEEDER_OSPEEDR13_0            ((volatile unsigned int)0x04000000)
#define GPIO_OSPEEDER_OSPEEDR13_1            ((volatile unsigned int)0x08000000)

#define GPIO_OSPEEDER_OSPEEDR14              ((volatile unsigned int)0x30000000)
#define GPIO_OSPEEDER_OSPEEDR14_0            ((volatile unsigned int)0x10000000)
#define GPIO_OSPEEDER_OSPEEDR14_1            ((volatile unsigned int)0x20000000)

#define GPIO_OSPEEDER_OSPEEDR15              ((volatile unsigned int)0xC0000000)
#define GPIO_OSPEEDER_OSPEEDR15_0            ((volatile unsigned int)0x40000000)
#define GPIO_OSPEEDER_OSPEEDR15_1            ((volatile unsigned int)0x80000000)

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPDR0                    ((volatile unsigned int)0x00000003)
#define GPIO_PUPDR_PUPDR0_0                  ((volatile unsigned int)0x00000001)
#define GPIO_PUPDR_PUPDR0_1                  ((volatile unsigned int)0x00000002)

#define GPIO_PUPDR_PUPDR1                    ((volatile unsigned int)0x0000000C)
#define GPIO_PUPDR_PUPDR1_0                  ((volatile unsigned int)0x00000004)
#define GPIO_PUPDR_PUPDR1_1                  ((volatile unsigned int)0x00000008)

#define GPIO_PUPDR_PUPDR2                    ((volatile unsigned int)0x00000030)
#define GPIO_PUPDR_PUPDR2_0                  ((volatile unsigned int)0x00000010)
#define GPIO_PUPDR_PUPDR2_1                  ((volatile unsigned int)0x00000020)

#define GPIO_PUPDR_PUPDR3                    ((volatile unsigned int)0x000000C0)
#define GPIO_PUPDR_PUPDR3_0                  ((volatile unsigned int)0x00000040)
#define GPIO_PUPDR_PUPDR3_1                  ((volatile unsigned int)0x00000080)

#define GPIO_PUPDR_PUPDR4                    ((volatile unsigned int)0x00000300)
#define GPIO_PUPDR_PUPDR4_0                  ((volatile unsigned int)0x00000100)
#define GPIO_PUPDR_PUPDR4_1                  ((volatile unsigned int)0x00000200)

#define GPIO_PUPDR_PUPDR5                    ((volatile unsigned int)0x00000C00)
#define GPIO_PUPDR_PUPDR5_0                  ((volatile unsigned int)0x00000400)
#define GPIO_PUPDR_PUPDR5_1                  ((volatile unsigned int)0x00000800)

#define GPIO_PUPDR_PUPDR6                    ((volatile unsigned int)0x00003000)
#define GPIO_PUPDR_PUPDR6_0                  ((volatile unsigned int)0x00001000)
#define GPIO_PUPDR_PUPDR6_1                  ((volatile unsigned int)0x00002000)

#define GPIO_PUPDR_PUPDR7                    ((volatile unsigned int)0x0000C000)
#define GPIO_PUPDR_PUPDR7_0                  ((volatile unsigned int)0x00004000)
#define GPIO_PUPDR_PUPDR7_1                  ((volatile unsigned int)0x00008000)

#define GPIO_PUPDR_PUPDR8                    ((volatile unsigned int)0x00030000)
#define GPIO_PUPDR_PUPDR8_0                  ((volatile unsigned int)0x00010000)
#define GPIO_PUPDR_PUPDR8_1                  ((volatile unsigned int)0x00020000)

#define GPIO_PUPDR_PUPDR9                    ((volatile unsigned int)0x000C0000)
#define GPIO_PUPDR_PUPDR9_0                  ((volatile unsigned int)0x00040000)
#define GPIO_PUPDR_PUPDR9_1                  ((volatile unsigned int)0x00080000)

#define GPIO_PUPDR_PUPDR10                   ((volatile unsigned int)0x00300000)
#define GPIO_PUPDR_PUPDR10_0                 ((volatile unsigned int)0x00100000)
#define GPIO_PUPDR_PUPDR10_1                 ((volatile unsigned int)0x00200000)

#define GPIO_PUPDR_PUPDR11                   ((volatile unsigned int)0x00C00000)
#define GPIO_PUPDR_PUPDR11_0                 ((volatile unsigned int)0x00400000)
#define GPIO_PUPDR_PUPDR11_1                 ((volatile unsigned int)0x00800000)

#define GPIO_PUPDR_PUPDR12                   ((volatile unsigned int)0x03000000)
#define GPIO_PUPDR_PUPDR12_0                 ((volatile unsigned int)0x01000000)
#define GPIO_PUPDR_PUPDR12_1                 ((volatile unsigned int)0x02000000)

#define GPIO_PUPDR_PUPDR13                   ((volatile unsigned int)0x0C000000)
#define GPIO_PUPDR_PUPDR13_0                 ((volatile unsigned int)0x04000000)
#define GPIO_PUPDR_PUPDR13_1                 ((volatile unsigned int)0x08000000)

#define GPIO_PUPDR_PUPDR14                   ((volatile unsigned int)0x30000000)
#define GPIO_PUPDR_PUPDR14_0                 ((volatile unsigned int)0x10000000)
#define GPIO_PUPDR_PUPDR14_1                 ((volatile unsigned int)0x20000000)

#define GPIO_PUPDR_PUPDR15                   ((volatile unsigned int)0xC0000000)
#define GPIO_PUPDR_PUPDR15_0                 ((volatile unsigned int)0x40000000)
#define GPIO_PUPDR_PUPDR15_1                 ((volatile unsigned int)0x80000000)

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR_0                       ((volatile unsigned int)0x00000001)
#define GPIO_IDR_IDR_1                       ((volatile unsigned int)0x00000002)
#define GPIO_IDR_IDR_2                       ((volatile unsigned int)0x00000004)
#define GPIO_IDR_IDR_3                       ((volatile unsigned int)0x00000008)
#define GPIO_IDR_IDR_4                       ((volatile unsigned int)0x00000010)
#define GPIO_IDR_IDR_5                       ((volatile unsigned int)0x00000020)
#define GPIO_IDR_IDR_6                       ((volatile unsigned int)0x00000040)
#define GPIO_IDR_IDR_7                       ((volatile unsigned int)0x00000080)
#define GPIO_IDR_IDR_8                       ((volatile unsigned int)0x00000100)
#define GPIO_IDR_IDR_9                       ((volatile unsigned int)0x00000200)
#define GPIO_IDR_IDR_10                      ((volatile unsigned int)0x00000400)
#define GPIO_IDR_IDR_11                      ((volatile unsigned int)0x00000800)
#define GPIO_IDR_IDR_12                      ((volatile unsigned int)0x00001000)
#define GPIO_IDR_IDR_13                      ((volatile unsigned int)0x00002000)
#define GPIO_IDR_IDR_14                      ((volatile unsigned int)0x00004000)
#define GPIO_IDR_IDR_15                      ((volatile unsigned int)0x00008000)
/* Old GPIO_IDR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_IDR_0                    GPIO_IDR_IDR_0
#define GPIO_OTYPER_IDR_1                    GPIO_IDR_IDR_1
#define GPIO_OTYPER_IDR_2                    GPIO_IDR_IDR_2
#define GPIO_OTYPER_IDR_3                    GPIO_IDR_IDR_3
#define GPIO_OTYPER_IDR_4                    GPIO_IDR_IDR_4
#define GPIO_OTYPER_IDR_5                    GPIO_IDR_IDR_5
#define GPIO_OTYPER_IDR_6                    GPIO_IDR_IDR_6
#define GPIO_OTYPER_IDR_7                    GPIO_IDR_IDR_7
#define GPIO_OTYPER_IDR_8                    GPIO_IDR_IDR_8
#define GPIO_OTYPER_IDR_9                    GPIO_IDR_IDR_9
#define GPIO_OTYPER_IDR_10                   GPIO_IDR_IDR_10
#define GPIO_OTYPER_IDR_11                   GPIO_IDR_IDR_11
#define GPIO_OTYPER_IDR_12                   GPIO_IDR_IDR_12
#define GPIO_OTYPER_IDR_13                   GPIO_IDR_IDR_13
#define GPIO_OTYPER_IDR_14                   GPIO_IDR_IDR_14
#define GPIO_OTYPER_IDR_15                   GPIO_IDR_IDR_15

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR_0                       ((volatile unsigned int)0x00000001)
#define GPIO_ODR_ODR_1                       ((volatile unsigned int)0x00000002)
#define GPIO_ODR_ODR_2                       ((volatile unsigned int)0x00000004)
#define GPIO_ODR_ODR_3                       ((volatile unsigned int)0x00000008)
#define GPIO_ODR_ODR_4                       ((volatile unsigned int)0x00000010)
#define GPIO_ODR_ODR_5                       ((volatile unsigned int)0x00000020)
#define GPIO_ODR_ODR_6                       ((volatile unsigned int)0x00000040)
#define GPIO_ODR_ODR_7                       ((volatile unsigned int)0x00000080)
#define GPIO_ODR_ODR_8                       ((volatile unsigned int)0x00000100)
#define GPIO_ODR_ODR_9                       ((volatile unsigned int)0x00000200)
#define GPIO_ODR_ODR_10                      ((volatile unsigned int)0x00000400)
#define GPIO_ODR_ODR_11                      ((volatile unsigned int)0x00000800)
#define GPIO_ODR_ODR_12                      ((volatile unsigned int)0x00001000)
#define GPIO_ODR_ODR_13                      ((volatile unsigned int)0x00002000)
#define GPIO_ODR_ODR_14                      ((volatile unsigned int)0x00004000)
#define GPIO_ODR_ODR_15                      ((volatile unsigned int)0x00008000)
/* Old GPIO_ODR register bits definition, maintained for legacy purpose */
#define GPIO_OTYPER_ODR_0                    GPIO_ODR_ODR_0
#define GPIO_OTYPER_ODR_1                    GPIO_ODR_ODR_1
#define GPIO_OTYPER_ODR_2                    GPIO_ODR_ODR_2
#define GPIO_OTYPER_ODR_3                    GPIO_ODR_ODR_3
#define GPIO_OTYPER_ODR_4                    GPIO_ODR_ODR_4
#define GPIO_OTYPER_ODR_5                    GPIO_ODR_ODR_5
#define GPIO_OTYPER_ODR_6                    GPIO_ODR_ODR_6
#define GPIO_OTYPER_ODR_7                    GPIO_ODR_ODR_7
#define GPIO_OTYPER_ODR_8                    GPIO_ODR_ODR_8
#define GPIO_OTYPER_ODR_9                    GPIO_ODR_ODR_9
#define GPIO_OTYPER_ODR_10                   GPIO_ODR_ODR_10
#define GPIO_OTYPER_ODR_11                   GPIO_ODR_ODR_11
#define GPIO_OTYPER_ODR_12                   GPIO_ODR_ODR_12
#define GPIO_OTYPER_ODR_13                   GPIO_ODR_ODR_13
#define GPIO_OTYPER_ODR_14                   GPIO_ODR_ODR_14
#define GPIO_OTYPER_ODR_15                   GPIO_ODR_ODR_15

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS_0                       ((volatile unsigned int)0x00000001)
#define GPIO_BSRR_BS_1                       ((volatile unsigned int)0x00000002)
#define GPIO_BSRR_BS_2                       ((volatile unsigned int)0x00000004)
#define GPIO_BSRR_BS_3                       ((volatile unsigned int)0x00000008)
#define GPIO_BSRR_BS_4                       ((volatile unsigned int)0x00000010)
#define GPIO_BSRR_BS_5                       ((volatile unsigned int)0x00000020)
#define GPIO_BSRR_BS_6                       ((volatile unsigned int)0x00000040)
#define GPIO_BSRR_BS_7                       ((volatile unsigned int)0x00000080)
#define GPIO_BSRR_BS_8                       ((volatile unsigned int)0x00000100)
#define GPIO_BSRR_BS_9                       ((volatile unsigned int)0x00000200)
#define GPIO_BSRR_BS_10                      ((volatile unsigned int)0x00000400)
#define GPIO_BSRR_BS_11                      ((volatile unsigned int)0x00000800)
#define GPIO_BSRR_BS_12                      ((volatile unsigned int)0x00001000)
#define GPIO_BSRR_BS_13                      ((volatile unsigned int)0x00002000)
#define GPIO_BSRR_BS_14                      ((volatile unsigned int)0x00004000)
#define GPIO_BSRR_BS_15                      ((volatile unsigned int)0x00008000)
#define GPIO_BSRR_BR_0                       ((volatile unsigned int)0x00010000)
#define GPIO_BSRR_BR_1                       ((volatile unsigned int)0x00020000)
#define GPIO_BSRR_BR_2                       ((volatile unsigned int)0x00040000)
#define GPIO_BSRR_BR_3                       ((volatile unsigned int)0x00080000)
#define GPIO_BSRR_BR_4                       ((volatile unsigned int)0x00100000)
#define GPIO_BSRR_BR_5                       ((volatile unsigned int)0x00200000)
#define GPIO_BSRR_BR_6                       ((volatile unsigned int)0x00400000)
#define GPIO_BSRR_BR_7                       ((volatile unsigned int)0x00800000)
#define GPIO_BSRR_BR_8                       ((volatile unsigned int)0x01000000)
#define GPIO_BSRR_BR_9                       ((volatile unsigned int)0x02000000)
#define GPIO_BSRR_BR_10                      ((volatile unsigned int)0x04000000)
#define GPIO_BSRR_BR_11                      ((volatile unsigned int)0x08000000)
#define GPIO_BSRR_BR_12                      ((volatile unsigned int)0x10000000)
#define GPIO_BSRR_BR_13                      ((volatile unsigned int)0x20000000)
#define GPIO_BSRR_BR_14                      ((volatile unsigned int)0x40000000)
#define GPIO_BSRR_BR_15                      ((volatile unsigned int)0x80000000)

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0                       ((volatile unsigned int)0x00000001)
#define GPIO_LCKR_LCK1                       ((volatile unsigned int)0x00000002)
#define GPIO_LCKR_LCK2                       ((volatile unsigned int)0x00000004)
#define GPIO_LCKR_LCK3                       ((volatile unsigned int)0x00000008)
#define GPIO_LCKR_LCK4                       ((volatile unsigned int)0x00000010)
#define GPIO_LCKR_LCK5                       ((volatile unsigned int)0x00000020)
#define GPIO_LCKR_LCK6                       ((volatile unsigned int)0x00000040)
#define GPIO_LCKR_LCK7                       ((volatile unsigned int)0x00000080)
#define GPIO_LCKR_LCK8                       ((volatile unsigned int)0x00000100)
#define GPIO_LCKR_LCK9                       ((volatile unsigned int)0x00000200)
#define GPIO_LCKR_LCK10                      ((volatile unsigned int)0x00000400)
#define GPIO_LCKR_LCK11                      ((volatile unsigned int)0x00000800)
#define GPIO_LCKR_LCK12                      ((volatile unsigned int)0x00001000)
#define GPIO_LCKR_LCK13                      ((volatile unsigned int)0x00002000)
#define GPIO_LCKR_LCK14                      ((volatile unsigned int)0x00004000)
#define GPIO_LCKR_LCK15                      ((volatile unsigned int)0x00008000)
#define GPIO_LCKR_LCKK                       ((volatile unsigned int)0x00010000)

/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define  I2C_CR1_PE                          ((volatile unsigned int)0x00000001)     /*!<Peripheral Enable                             */
#define  I2C_CR1_SMBUS                       ((volatile unsigned int)0x00000002)     /*!<SMBus Mode                                    */
#define  I2C_CR1_SMBTYPE                     ((volatile unsigned int)0x00000008)     /*!<SMBus Type                                    */
#define  I2C_CR1_ENARP                       ((volatile unsigned int)0x00000010)     /*!<ARP Enable                                    */
#define  I2C_CR1_ENPEC                       ((volatile unsigned int)0x00000020)     /*!<PEC Enable                                    */
#define  I2C_CR1_ENGC                        ((volatile unsigned int)0x00000040)     /*!<General Call Enable                           */
#define  I2C_CR1_NOSTRETCH                   ((volatile unsigned int)0x00000080)     /*!<Clock Stretching Disable (Slave mode)  */
#define  I2C_CR1_START                       ((volatile unsigned int)0x00000100)     /*!<Start Generation                              */
#define  I2C_CR1_STOP                        ((volatile unsigned int)0x00000200)     /*!<Stop Generation                               */
#define  I2C_CR1_ACK                         ((volatile unsigned int)0x00000400)     /*!<Acknowledge Enable                            */
#define  I2C_CR1_POS                         ((volatile unsigned int)0x00000800)     /*!<Acknowledge/PEC Position (for data reception) */
#define  I2C_CR1_PEC                         ((volatile unsigned int)0x00001000)     /*!<Packet Error Checking                         */
#define  I2C_CR1_ALERT                       ((volatile unsigned int)0x00002000)     /*!<SMBus Alert                                   */
#define  I2C_CR1_SWRST                       ((volatile unsigned int)0x00008000)     /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define  I2C_CR2_FREQ                        ((volatile unsigned int)0x0000003F)     /*!<FREQ[5:0] bits (Peripheral Clock Frequency)   */
#define  I2C_CR2_FREQ_0                      ((volatile unsigned int)0x00000001)     /*!<Bit 0 */
#define  I2C_CR2_FREQ_1                      ((volatile unsigned int)0x00000002)     /*!<Bit 1 */
#define  I2C_CR2_FREQ_2                      ((volatile unsigned int)0x00000004)     /*!<Bit 2 */
#define  I2C_CR2_FREQ_3                      ((volatile unsigned int)0x00000008)     /*!<Bit 3 */
#define  I2C_CR2_FREQ_4                      ((volatile unsigned int)0x00000010)     /*!<Bit 4 */
#define  I2C_CR2_FREQ_5                      ((volatile unsigned int)0x00000020)     /*!<Bit 5 */

#define  I2C_CR2_ITERREN                     ((volatile unsigned int)0x00000100)     /*!<Error Interrupt Enable  */
#define  I2C_CR2_ITEVTEN                     ((volatile unsigned int)0x00000200)     /*!<Event Interrupt Enable  */
#define  I2C_CR2_ITBUFEN                     ((volatile unsigned int)0x00000400)     /*!<Buffer Interrupt Enable */
#define  I2C_CR2_DMAEN                       ((volatile unsigned int)0x00000800)     /*!<DMA Requests Enable     */
#define  I2C_CR2_LAST                        ((volatile unsigned int)0x00001000)     /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define  I2C_OAR1_ADD1_7                     ((volatile unsigned int)0x000000FE)     /*!<Interface Address */
#define  I2C_OAR1_ADD8_9                     ((volatile unsigned int)0x00000300)     /*!<Interface Address */

#define  I2C_OAR1_ADD0                       ((volatile unsigned int)0x00000001)     /*!<Bit 0 */
#define  I2C_OAR1_ADD1                       ((volatile unsigned int)0x00000002)     /*!<Bit 1 */
#define  I2C_OAR1_ADD2                       ((volatile unsigned int)0x00000004)     /*!<Bit 2 */
#define  I2C_OAR1_ADD3                       ((volatile unsigned int)0x00000008)     /*!<Bit 3 */
#define  I2C_OAR1_ADD4                       ((volatile unsigned int)0x00000010)     /*!<Bit 4 */
#define  I2C_OAR1_ADD5                       ((volatile unsigned int)0x00000020)     /*!<Bit 5 */
#define  I2C_OAR1_ADD6                       ((volatile unsigned int)0x00000040)     /*!<Bit 6 */
#define  I2C_OAR1_ADD7                       ((volatile unsigned int)0x00000080)     /*!<Bit 7 */
#define  I2C_OAR1_ADD8                       ((volatile unsigned int)0x00000100)     /*!<Bit 8 */
#define  I2C_OAR1_ADD9                       ((volatile unsigned int)0x00000200)     /*!<Bit 9 */

#define  I2C_OAR1_ADDMODE                    ((volatile unsigned int)0x00008000)     /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define  I2C_OAR2_ENDUAL                     ((volatile unsigned int)0x00000001)        /*!<Dual addressing mode enable */
#define  I2C_OAR2_ADD2                       ((volatile unsigned int)0x000000FE)        /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define  I2C_DR_DR                           ((volatile unsigned int)0x000000FF)        /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define  I2C_SR1_SB                          ((volatile unsigned int)0x00000001)     /*!<Start Bit (Master mode)                  */
#define  I2C_SR1_ADDR                        ((volatile unsigned int)0x00000002)     /*!<Address sent (master mode)/matched (slave mode) */
#define  I2C_SR1_BTF                         ((volatile unsigned int)0x00000004)     /*!<Byte Transfer Finished                          */
#define  I2C_SR1_ADD10                       ((volatile unsigned int)0x00000008)     /*!<10-bit header sent (Master mode)         */
#define  I2C_SR1_STOPF                       ((volatile unsigned int)0x00000010)     /*!<Stop detection (Slave mode)              */
#define  I2C_SR1_RXNE                        ((volatile unsigned int)0x00000040)     /*!<Data Register not Empty (receivers)      */
#define  I2C_SR1_TXE                         ((volatile unsigned int)0x00000080)     /*!<Data Register Empty (transmitters)       */
#define  I2C_SR1_BERR                        ((volatile unsigned int)0x00000100)     /*!<Bus Error                                       */
#define  I2C_SR1_ARLO                        ((volatile unsigned int)0x00000200)     /*!<Arbitration Lost (master mode)           */
#define  I2C_SR1_AF                          ((volatile unsigned int)0x00000400)     /*!<Acknowledge Failure                             */
#define  I2C_SR1_OVR                         ((volatile unsigned int)0x00000800)     /*!<Overrun/Underrun                                */
#define  I2C_SR1_PECERR                      ((volatile unsigned int)0x00001000)     /*!<PEC Error in reception                          */
#define  I2C_SR1_TIMEOUT                     ((volatile unsigned int)0x00004000)     /*!<Timeout or Tlow Error                           */
#define  I2C_SR1_SMBALERT                    ((volatile unsigned int)0x00008000)     /*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define  I2C_SR2_MSL                         ((volatile unsigned int)0x00000001)     /*!<Master/Slave                              */
#define  I2C_SR2_BUSY                        ((volatile unsigned int)0x00000002)     /*!<Bus Busy                                  */
#define  I2C_SR2_TRA                         ((volatile unsigned int)0x00000004)     /*!<Transmitter/Receiver                      */
#define  I2C_SR2_GENCALL                     ((volatile unsigned int)0x00000010)     /*!<General Call Address (Slave mode)  */
#define  I2C_SR2_SMBDEFAULT                  ((volatile unsigned int)0x00000020)     /*!<SMBus Device Default Address (Slave mode) */
#define  I2C_SR2_SMBHOST                     ((volatile unsigned int)0x00000040)     /*!<SMBus Host Header (Slave mode)     */
#define  I2C_SR2_DUALF                       ((volatile unsigned int)0x00000080)     /*!<Dual Flag (Slave mode)             */
#define  I2C_SR2_PEC                         ((volatile unsigned int)0x0000FF00)     /*!<Packet Error Checking Register            */

/*******************  Bit definition for I2C_CCR register  ********************/
#define  I2C_CCR_CCR                         ((volatile unsigned int)0x00000FFF)     /*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CCR_DUTY                        ((volatile unsigned int)0x00004000)     /*!<Fast Mode Duty Cycle                                       */
#define  I2C_CCR_FS                          ((volatile unsigned int)0x00008000)     /*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define  I2C_TRISE_TRISE                     ((volatile unsigned int)0x0000003F)     /*!<Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************  Bit definition for I2C_FLTR register  *******************/
#define  I2C_FLTR_DNF                        ((volatile unsigned int)0x0000000F)     /*!<Digital Noise Filter */
#define  I2C_FLTR_ANOFF                      ((volatile unsigned int)0x00000010)     /*!<Analog Noise Filter OFF */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for IWDG_KR register  ********************/
#define  IWDG_KR_KEY                         ((volatile unsigned int)0xFFFF)            /*!<Key value (write only, read 0000h)  */

/*******************  Bit definition for IWDG_PR register  ********************/
#define  IWDG_PR_PR                          ((volatile unsigned int)0x07)               /*!<PR[2:0] (Prescaler divider)         */
#define  IWDG_PR_PR_0                        ((volatile unsigned int)0x01)               /*!<Bit 0 */
#define  IWDG_PR_PR_1                        ((volatile unsigned int)0x02)               /*!<Bit 1 */
#define  IWDG_PR_PR_2                        ((volatile unsigned int)0x04)               /*!<Bit 2 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define  IWDG_RLR_RL                         ((volatile unsigned int)0x0FFF)            /*!<Watchdog counter reload value        */

/*******************  Bit definition for IWDG_SR register  ********************/
#define  IWDG_SR_PVU                         ((volatile unsigned int)0x01)               /*!<Watchdog prescaler value update      */
#define  IWDG_SR_RVU                         ((volatile unsigned int)0x02)               /*!<Watchdog counter reload value update */


/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((volatile unsigned int)0x00000001)     /*!< Low-Power Deepsleep                 */
#define  PWR_CR_PDDS                         ((volatile unsigned int)0x00000002)     /*!< Power Down Deepsleep                */
#define  PWR_CR_CWUF                         ((volatile unsigned int)0x00000004)     /*!< Clear Wakeup Flag                   */
#define  PWR_CR_CSBF                         ((volatile unsigned int)0x00000008)     /*!< Clear Standby Flag                  */
#define  PWR_CR_PVDE                         ((volatile unsigned int)0x00000010)     /*!< Power Voltage Detector Enable       */

#define  PWR_CR_PLS                          ((volatile unsigned int)0x000000E0)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((volatile unsigned int)0x00000020)     /*!< Bit 0 */
#define  PWR_CR_PLS_1                        ((volatile unsigned int)0x00000040)     /*!< Bit 1 */
#define  PWR_CR_PLS_2                        ((volatile unsigned int)0x00000080)     /*!< Bit 2 */

/*!< PVD level configuration */
#define  PWR_CR_PLS_LEV0                     ((volatile unsigned int)0x00000000)     /*!< PVD level 0 */
#define  PWR_CR_PLS_LEV1                     ((volatile unsigned int)0x00000020)     /*!< PVD level 1 */
#define  PWR_CR_PLS_LEV2                     ((volatile unsigned int)0x00000040)     /*!< PVD level 2 */
#define  PWR_CR_PLS_LEV3                     ((volatile unsigned int)0x00000060)     /*!< PVD level 3 */
#define  PWR_CR_PLS_LEV4                     ((volatile unsigned int)0x00000080)     /*!< PVD level 4 */
#define  PWR_CR_PLS_LEV5                     ((volatile unsigned int)0x000000A0)     /*!< PVD level 5 */
#define  PWR_CR_PLS_LEV6                     ((volatile unsigned int)0x000000C0)     /*!< PVD level 6 */
#define  PWR_CR_PLS_LEV7                     ((volatile unsigned int)0x000000E0)     /*!< PVD level 7 */

#define  PWR_CR_DBP                          ((volatile unsigned int)0x00000100)     /*!< Disable Backup Domain write protection                     */
#define  PWR_CR_FPDS                         ((volatile unsigned int)0x00000200)     /*!< Flash power down in Stop mode                              */
#define  PWR_CR_VOS                          ((volatile unsigned int)0x0000C000)     /*!< VOS[1:0] bits (Regulator voltage scaling output selection) */
#define  PWR_CR_VOS_0                        ((volatile unsigned int)0x00004000)     /*!< Bit 0 */
#define  PWR_CR_VOS_1                        ((volatile unsigned int)0x00008000)     /*!< Bit 1 */

/* Legacy define */
#define  PWR_CR_PMODE                        PWR_CR_VOS

/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((volatile unsigned int)0x00000001)     /*!< Wakeup Flag                                      */
#define  PWR_CSR_SBF                         ((volatile unsigned int)0x00000002)     /*!< Standby Flag                                     */
#define  PWR_CSR_PVDO                        ((volatile unsigned int)0x00000004)     /*!< PVD Output                                       */
#define  PWR_CSR_BRR                         ((volatile unsigned int)0x00000008)     /*!< Backup regulator ready                           */
#define  PWR_CSR_EWUP                        ((volatile unsigned int)0x00000100)     /*!< Enable WKUP pin                                  */
#define  PWR_CSR_BRE                         ((volatile unsigned int)0x00000200)     /*!< Backup regulator enable                          */
#define  PWR_CSR_VOSRDY                      ((volatile unsigned int)0x00004000)     /*!< Regulator voltage scaling output selection ready */

/* Legacy define */
#define  PWR_CSR_REGRDY                      PWR_CSR_VOSRDY

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((volatile unsigned int)0x00000001)
#define  RCC_CR_HSIRDY                       ((volatile unsigned int)0x00000002)

#define  RCC_CR_HSITRIM                      ((volatile unsigned int)0x000000F8)
#define  RCC_CR_HSITRIM_0                    ((volatile unsigned int)0x00000008)/*!<Bit 0 */
#define  RCC_CR_HSITRIM_1                    ((volatile unsigned int)0x00000010)/*!<Bit 1 */
#define  RCC_CR_HSITRIM_2                    ((volatile unsigned int)0x00000020)/*!<Bit 2 */
#define  RCC_CR_HSITRIM_3                    ((volatile unsigned int)0x00000040)/*!<Bit 3 */
#define  RCC_CR_HSITRIM_4                    ((volatile unsigned int)0x00000080)/*!<Bit 4 */

#define  RCC_CR_HSICAL                       ((volatile unsigned int)0x0000FF00)
#define  RCC_CR_HSICAL_0                     ((volatile unsigned int)0x00000100)/*!<Bit 0 */
#define  RCC_CR_HSICAL_1                     ((volatile unsigned int)0x00000200)/*!<Bit 1 */
#define  RCC_CR_HSICAL_2                     ((volatile unsigned int)0x00000400)/*!<Bit 2 */
#define  RCC_CR_HSICAL_3                     ((volatile unsigned int)0x00000800)/*!<Bit 3 */
#define  RCC_CR_HSICAL_4                     ((volatile unsigned int)0x00001000)/*!<Bit 4 */
#define  RCC_CR_HSICAL_5                     ((volatile unsigned int)0x00002000)/*!<Bit 5 */
#define  RCC_CR_HSICAL_6                     ((volatile unsigned int)0x00004000)/*!<Bit 6 */
#define  RCC_CR_HSICAL_7                     ((volatile unsigned int)0x00008000)/*!<Bit 7 */

#define  RCC_CR_HSEON                        ((volatile unsigned int)0x00010000)
#define  RCC_CR_HSERDY                       ((volatile unsigned int)0x00020000)
#define  RCC_CR_HSEBYP                       ((volatile unsigned int)0x00040000)
#define  RCC_CR_CSSON                        ((volatile unsigned int)0x00080000)
#define  RCC_CR_PLLON                        ((volatile unsigned int)0x01000000)
#define  RCC_CR_PLLRDY                       ((volatile unsigned int)0x02000000)
#define  RCC_CR_PLLI2SON                     ((volatile unsigned int)0x04000000)
#define  RCC_CR_PLLI2SRDY                    ((volatile unsigned int)0x08000000)

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define  RCC_PLLCFGR_PLLM                    ((volatile unsigned int)0x0000003F)
#define  RCC_PLLCFGR_PLLM_0                  ((volatile unsigned int)0x00000001)
#define  RCC_PLLCFGR_PLLM_1                  ((volatile unsigned int)0x00000002)
#define  RCC_PLLCFGR_PLLM_2                  ((volatile unsigned int)0x00000004)
#define  RCC_PLLCFGR_PLLM_3                  ((volatile unsigned int)0x00000008)
#define  RCC_PLLCFGR_PLLM_4                  ((volatile unsigned int)0x00000010)
#define  RCC_PLLCFGR_PLLM_5                  ((volatile unsigned int)0x00000020)

#define  RCC_PLLCFGR_PLLN                     ((volatile unsigned int)0x00007FC0)
#define  RCC_PLLCFGR_PLLN_0                   ((volatile unsigned int)0x00000040)
#define  RCC_PLLCFGR_PLLN_1                   ((volatile unsigned int)0x00000080)
#define  RCC_PLLCFGR_PLLN_2                   ((volatile unsigned int)0x00000100)
#define  RCC_PLLCFGR_PLLN_3                   ((volatile unsigned int)0x00000200)
#define  RCC_PLLCFGR_PLLN_4                   ((volatile unsigned int)0x00000400)
#define  RCC_PLLCFGR_PLLN_5                   ((volatile unsigned int)0x00000800)
#define  RCC_PLLCFGR_PLLN_6                   ((volatile unsigned int)0x00001000)
#define  RCC_PLLCFGR_PLLN_7                   ((volatile unsigned int)0x00002000)
#define  RCC_PLLCFGR_PLLN_8                   ((volatile unsigned int)0x00004000)

#define  RCC_PLLCFGR_PLLP                    ((volatile unsigned int)0x00030000)
#define  RCC_PLLCFGR_PLLP_0                  ((volatile unsigned int)0x00010000)
#define  RCC_PLLCFGR_PLLP_1                  ((volatile unsigned int)0x00020000)

#define  RCC_PLLCFGR_PLLSRC                  ((volatile unsigned int)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSE              ((volatile unsigned int)0x00400000)
#define  RCC_PLLCFGR_PLLSRC_HSI              ((volatile unsigned int)0x00000000)

#define  RCC_PLLCFGR_PLLQ                    ((volatile unsigned int)0x0F000000)
#define  RCC_PLLCFGR_PLLQ_0                  ((volatile unsigned int)0x01000000)
#define  RCC_PLLCFGR_PLLQ_1                  ((volatile unsigned int)0x02000000)
#define  RCC_PLLCFGR_PLLQ_2                  ((volatile unsigned int)0x04000000)
#define  RCC_PLLCFGR_PLLQ_3                  ((volatile unsigned int)0x08000000)

/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((volatile unsigned int)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((volatile unsigned int)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((volatile unsigned int)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((volatile unsigned int)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((volatile unsigned int)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((volatile unsigned int)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((volatile unsigned int)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((volatile unsigned int)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((volatile unsigned int)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((volatile unsigned int)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((volatile unsigned int)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((volatile unsigned int)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((volatile unsigned int)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((volatile unsigned int)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((volatile unsigned int)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((volatile unsigned int)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((volatile unsigned int)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((volatile unsigned int)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((volatile unsigned int)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((volatile unsigned int)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((volatile unsigned int)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((volatile unsigned int)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((volatile unsigned int)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((volatile unsigned int)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((volatile unsigned int)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((volatile unsigned int)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((volatile unsigned int)0x00001C00)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((volatile unsigned int)0x00000400)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((volatile unsigned int)0x00000800)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((volatile unsigned int)0x00001000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((volatile unsigned int)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((volatile unsigned int)0x00001000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((volatile unsigned int)0x00001400)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((volatile unsigned int)0x00001800)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((volatile unsigned int)0x00001C00)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((volatile unsigned int)0x0000E000)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((volatile unsigned int)0x00002000)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((volatile unsigned int)0x00004000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((volatile unsigned int)0x00008000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((volatile unsigned int)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((volatile unsigned int)0x00008000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((volatile unsigned int)0x0000A000)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((volatile unsigned int)0x0000C000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((volatile unsigned int)0x0000E000)        /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define  RCC_CFGR_RTCPRE                     ((volatile unsigned int)0x001F0000)
#define  RCC_CFGR_RTCPRE_0                   ((volatile unsigned int)0x00010000)
#define  RCC_CFGR_RTCPRE_1                   ((volatile unsigned int)0x00020000)
#define  RCC_CFGR_RTCPRE_2                   ((volatile unsigned int)0x00040000)
#define  RCC_CFGR_RTCPRE_3                   ((volatile unsigned int)0x00080000)
#define  RCC_CFGR_RTCPRE_4                   ((volatile unsigned int)0x00100000)

/*!< MCO1 configuration */
#define  RCC_CFGR_MCO1                       ((volatile unsigned int)0x00600000)
#define  RCC_CFGR_MCO1_0                     ((volatile unsigned int)0x00200000)
#define  RCC_CFGR_MCO1_1                     ((volatile unsigned int)0x00400000)

#define  RCC_CFGR_I2SSRC                     ((volatile unsigned int)0x00800000)

#define  RCC_CFGR_MCO1PRE                    ((volatile unsigned int)0x07000000)
#define  RCC_CFGR_MCO1PRE_0                  ((volatile unsigned int)0x01000000)
#define  RCC_CFGR_MCO1PRE_1                  ((volatile unsigned int)0x02000000)
#define  RCC_CFGR_MCO1PRE_2                  ((volatile unsigned int)0x04000000)

#define  RCC_CFGR_MCO2PRE                    ((volatile unsigned int)0x38000000)
#define  RCC_CFGR_MCO2PRE_0                  ((volatile unsigned int)0x08000000)
#define  RCC_CFGR_MCO2PRE_1                  ((volatile unsigned int)0x10000000)
#define  RCC_CFGR_MCO2PRE_2                  ((volatile unsigned int)0x20000000)

#define  RCC_CFGR_MCO2                       ((volatile unsigned int)0xC0000000)
#define  RCC_CFGR_MCO2_0                     ((volatile unsigned int)0x40000000)
#define  RCC_CFGR_MCO2_1                     ((volatile unsigned int)0x80000000)

/********************  Bit definition for RCC_CIR register  *******************/
#define  RCC_CIR_LSIRDYF                     ((volatile unsigned int)0x00000001)
#define  RCC_CIR_LSERDYF                     ((volatile unsigned int)0x00000002)
#define  RCC_CIR_HSIRDYF                     ((volatile unsigned int)0x00000004)
#define  RCC_CIR_HSERDYF                     ((volatile unsigned int)0x00000008)
#define  RCC_CIR_PLLRDYF                     ((volatile unsigned int)0x00000010)
#define  RCC_CIR_PLLI2SRDYF                  ((volatile unsigned int)0x00000020)

#define  RCC_CIR_CSSF                        ((volatile unsigned int)0x00000080)
#define  RCC_CIR_LSIRDYIE                    ((volatile unsigned int)0x00000100)
#define  RCC_CIR_LSERDYIE                    ((volatile unsigned int)0x00000200)
#define  RCC_CIR_HSIRDYIE                    ((volatile unsigned int)0x00000400)
#define  RCC_CIR_HSERDYIE                    ((volatile unsigned int)0x00000800)
#define  RCC_CIR_PLLRDYIE                    ((volatile unsigned int)0x00001000)
#define  RCC_CIR_PLLI2SRDYIE                 ((volatile unsigned int)0x00002000)

#define  RCC_CIR_LSIRDYC                     ((volatile unsigned int)0x00010000)
#define  RCC_CIR_LSERDYC                     ((volatile unsigned int)0x00020000)
#define  RCC_CIR_HSIRDYC                     ((volatile unsigned int)0x00040000)
#define  RCC_CIR_HSERDYC                     ((volatile unsigned int)0x00080000)
#define  RCC_CIR_PLLRDYC                     ((volatile unsigned int)0x00100000)
#define  RCC_CIR_PLLI2SRDYC                  ((volatile unsigned int)0x00200000)

#define  RCC_CIR_CSSC                        ((volatile unsigned int)0x00800000)

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define  RCC_AHB1RSTR_GPIOARST               ((volatile unsigned int)0x00000001)
#define  RCC_AHB1RSTR_GPIOBRST               ((volatile unsigned int)0x00000002)
#define  RCC_AHB1RSTR_GPIOCRST               ((volatile unsigned int)0x00000004)
#define  RCC_AHB1RSTR_GPIODRST               ((volatile unsigned int)0x00000008)
#define  RCC_AHB1RSTR_GPIOERST               ((volatile unsigned int)0x00000010)
#define  RCC_AHB1RSTR_GPIOFRST               ((volatile unsigned int)0x00000020)
#define  RCC_AHB1RSTR_GPIOGRST               ((volatile unsigned int)0x00000040)
#define  RCC_AHB1RSTR_GPIOHRST               ((volatile unsigned int)0x00000080)
#define  RCC_AHB1RSTR_GPIOIRST               ((volatile unsigned int)0x00000100)
#define  RCC_AHB1RSTR_CRCRST                 ((volatile unsigned int)0x00001000)
#define  RCC_AHB1RSTR_DMA1RST                ((volatile unsigned int)0x00200000)
#define  RCC_AHB1RSTR_DMA2RST                ((volatile unsigned int)0x00400000)
#define  RCC_AHB1RSTR_ETHMACRST              ((volatile unsigned int)0x02000000)
#define  RCC_AHB1RSTR_OTGHRST                ((volatile unsigned int)0x10000000)

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define  RCC_AHB2RSTR_DCMIRST                ((volatile unsigned int)0x00000001)
#define  RCC_AHB2RSTR_RNGRST                 ((volatile unsigned int)0x00000040)
#define  RCC_AHB2RSTR_OTGFSRST               ((volatile unsigned int)0x00000080)

/********************  Bit definition for RCC_AHB3RSTR register  **************/

#define  RCC_AHB3RSTR_FSMCRST                ((volatile unsigned int)0x00000001)

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define  RCC_APB1RSTR_TIM2RST                ((volatile unsigned int)0x00000001)
#define  RCC_APB1RSTR_TIM3RST                ((volatile unsigned int)0x00000002)
#define  RCC_APB1RSTR_TIM4RST                ((volatile unsigned int)0x00000004)
#define  RCC_APB1RSTR_TIM5RST                ((volatile unsigned int)0x00000008)
#define  RCC_APB1RSTR_TIM6RST                ((volatile unsigned int)0x00000010)
#define  RCC_APB1RSTR_TIM7RST                ((volatile unsigned int)0x00000020)
#define  RCC_APB1RSTR_TIM12RST               ((volatile unsigned int)0x00000040)
#define  RCC_APB1RSTR_TIM13RST               ((volatile unsigned int)0x00000080)
#define  RCC_APB1RSTR_TIM14RST               ((volatile unsigned int)0x00000100)
#define  RCC_APB1RSTR_WWDGRST                ((volatile unsigned int)0x00000800)
#define  RCC_APB1RSTR_SPI2RST                ((volatile unsigned int)0x00004000)
#define  RCC_APB1RSTR_SPI3RST                ((volatile unsigned int)0x00008000)
#define  RCC_APB1RSTR_USART2RST              ((volatile unsigned int)0x00020000)
#define  RCC_APB1RSTR_USART3RST              ((volatile unsigned int)0x00040000)
#define  RCC_APB1RSTR_UART4RST               ((volatile unsigned int)0x00080000)
#define  RCC_APB1RSTR_UART5RST               ((volatile unsigned int)0x00100000)
#define  RCC_APB1RSTR_I2C1RST                ((volatile unsigned int)0x00200000)
#define  RCC_APB1RSTR_I2C2RST                ((volatile unsigned int)0x00400000)
#define  RCC_APB1RSTR_I2C3RST                ((volatile unsigned int)0x00800000)
#define  RCC_APB1RSTR_CAN1RST                ((volatile unsigned int)0x02000000)
#define  RCC_APB1RSTR_CAN2RST                ((volatile unsigned int)0x04000000)
#define  RCC_APB1RSTR_PWRRST                 ((volatile unsigned int)0x10000000)
#define  RCC_APB1RSTR_DACRST                 ((volatile unsigned int)0x20000000)

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define  RCC_APB2RSTR_TIM1RST                ((volatile unsigned int)0x00000001)
#define  RCC_APB2RSTR_TIM8RST                ((volatile unsigned int)0x00000002)
#define  RCC_APB2RSTR_USART1RST              ((volatile unsigned int)0x00000010)
#define  RCC_APB2RSTR_USART6RST              ((volatile unsigned int)0x00000020)
#define  RCC_APB2RSTR_ADCRST                 ((volatile unsigned int)0x00000100)
#define  RCC_APB2RSTR_SDIORST                ((volatile unsigned int)0x00000800)
#define  RCC_APB2RSTR_SPI1RST                ((volatile unsigned int)0x00001000)
#define  RCC_APB2RSTR_SYSCFGRST              ((volatile unsigned int)0x00004000)
#define  RCC_APB2RSTR_TIM9RST                ((volatile unsigned int)0x00010000)
#define  RCC_APB2RSTR_TIM10RST               ((volatile unsigned int)0x00020000)
#define  RCC_APB2RSTR_TIM11RST               ((volatile unsigned int)0x00040000)

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                   RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define  RCC_AHB1ENR_GPIOAEN                 ((volatile unsigned int)0x00000001)
#define  RCC_AHB1ENR_GPIOBEN                 ((volatile unsigned int)0x00000002)
#define  RCC_AHB1ENR_GPIOCEN                 ((volatile unsigned int)0x00000004)
#define  RCC_AHB1ENR_GPIODEN                 ((volatile unsigned int)0x00000008)
#define  RCC_AHB1ENR_GPIOEEN                 ((volatile unsigned int)0x00000010)
#define  RCC_AHB1ENR_GPIOFEN                 ((volatile unsigned int)0x00000020)
#define  RCC_AHB1ENR_GPIOGEN                 ((volatile unsigned int)0x00000040)
#define  RCC_AHB1ENR_GPIOHEN                 ((volatile unsigned int)0x00000080)
#define  RCC_AHB1ENR_GPIOIEN                 ((volatile unsigned int)0x00000100)
#define  RCC_AHB1ENR_CRCEN                   ((volatile unsigned int)0x00001000)
#define  RCC_AHB1ENR_BKPSRAMEN               ((volatile unsigned int)0x00040000)
#define  RCC_AHB1ENR_CCMDATARAMEN            ((volatile unsigned int)0x00100000)
#define  RCC_AHB1ENR_DMA1EN                  ((volatile unsigned int)0x00200000)
#define  RCC_AHB1ENR_DMA2EN                  ((volatile unsigned int)0x00400000)

#define  RCC_AHB1ENR_ETHMACEN                ((volatile unsigned int)0x02000000)
#define  RCC_AHB1ENR_ETHMACTXEN              ((volatile unsigned int)0x04000000)
#define  RCC_AHB1ENR_ETHMACRXEN              ((volatile unsigned int)0x08000000)
#define  RCC_AHB1ENR_ETHMACPTPEN             ((volatile unsigned int)0x10000000)
#define  RCC_AHB1ENR_OTGHSEN                 ((volatile unsigned int)0x20000000)
#define  RCC_AHB1ENR_OTGHSULPIEN             ((volatile unsigned int)0x40000000)

/********************  Bit definition for RCC_AHB2ENR register  ***************/
#define  RCC_AHB2ENR_DCMIEN                  ((volatile unsigned int)0x00000001)
#define  RCC_AHB2ENR_RNGEN                   ((volatile unsigned int)0x00000040)
#define  RCC_AHB2ENR_OTGFSEN                 ((volatile unsigned int)0x00000080)

/********************  Bit definition for RCC_AHB3ENR register  ***************/

#define  RCC_AHB3ENR_FSMCEN                  ((volatile unsigned int)0x00000001)

/********************  Bit definition for RCC_APB1ENR register  ***************/
#define  RCC_APB1ENR_TIM2EN                  ((volatile unsigned int)0x00000001)
#define  RCC_APB1ENR_TIM3EN                  ((volatile unsigned int)0x00000002)
#define  RCC_APB1ENR_TIM4EN                  ((volatile unsigned int)0x00000004)
#define  RCC_APB1ENR_TIM5EN                  ((volatile unsigned int)0x00000008)
#define  RCC_APB1ENR_TIM6EN                  ((volatile unsigned int)0x00000010)
#define  RCC_APB1ENR_TIM7EN                  ((volatile unsigned int)0x00000020)
#define  RCC_APB1ENR_TIM12EN                 ((volatile unsigned int)0x00000040)
#define  RCC_APB1ENR_TIM13EN                 ((volatile unsigned int)0x00000080)
#define  RCC_APB1ENR_TIM14EN                 ((volatile unsigned int)0x00000100)
#define  RCC_APB1ENR_WWDGEN                  ((volatile unsigned int)0x00000800)
#define  RCC_APB1ENR_SPI2EN                  ((volatile unsigned int)0x00004000)
#define  RCC_APB1ENR_SPI3EN                  ((volatile unsigned int)0x00008000)
#define  RCC_APB1ENR_USART2EN                ((volatile unsigned int)0x00020000)
#define  RCC_APB1ENR_USART3EN                ((volatile unsigned int)0x00040000)
#define  RCC_APB1ENR_UART4EN                 ((volatile unsigned int)0x00080000)
#define  RCC_APB1ENR_UART5EN                 ((volatile unsigned int)0x00100000)
#define  RCC_APB1ENR_I2C1EN                  ((volatile unsigned int)0x00200000)
#define  RCC_APB1ENR_I2C2EN                  ((volatile unsigned int)0x00400000)
#define  RCC_APB1ENR_I2C3EN                  ((volatile unsigned int)0x00800000)
#define  RCC_APB1ENR_CAN1EN                  ((volatile unsigned int)0x02000000)
#define  RCC_APB1ENR_CAN2EN                  ((volatile unsigned int)0x04000000)
#define  RCC_APB1ENR_PWREN                   ((volatile unsigned int)0x10000000)
#define  RCC_APB1ENR_DACEN                   ((volatile unsigned int)0x20000000)

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define  RCC_APB2ENR_TIM1EN                  ((volatile unsigned int)0x00000001)
#define  RCC_APB2ENR_TIM8EN                  ((volatile unsigned int)0x00000002)
#define  RCC_APB2ENR_USART1EN                ((volatile unsigned int)0x00000010)
#define  RCC_APB2ENR_USART6EN                ((volatile unsigned int)0x00000020)
#define  RCC_APB2ENR_ADC1EN                  ((volatile unsigned int)0x00000100)
#define  RCC_APB2ENR_ADC2EN                  ((volatile unsigned int)0x00000200)
#define  RCC_APB2ENR_ADC3EN                  ((volatile unsigned int)0x00000400)
#define  RCC_APB2ENR_SDIOEN                  ((volatile unsigned int)0x00000800)
#define  RCC_APB2ENR_SPI1EN                  ((volatile unsigned int)0x00001000)
#define  RCC_APB2ENR_SYSCFGEN                ((volatile unsigned int)0x00004000)
#define  RCC_APB2ENR_TIM9EN                  ((volatile unsigned int)0x00010000)
#define  RCC_APB2ENR_TIM10EN                 ((volatile unsigned int)0x00020000)
#define  RCC_APB2ENR_TIM11EN                 ((volatile unsigned int)0x00040000)
#define  RCC_APB2ENR_SPI5EN                  ((volatile unsigned int)0x00100000)
#define  RCC_APB2ENR_SPI6EN                  ((volatile unsigned int)0x00200000)

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define  RCC_AHB1LPENR_GPIOALPEN             ((volatile unsigned int)0x00000001)
#define  RCC_AHB1LPENR_GPIOBLPEN             ((volatile unsigned int)0x00000002)
#define  RCC_AHB1LPENR_GPIOCLPEN             ((volatile unsigned int)0x00000004)
#define  RCC_AHB1LPENR_GPIODLPEN             ((volatile unsigned int)0x00000008)
#define  RCC_AHB1LPENR_GPIOELPEN             ((volatile unsigned int)0x00000010)
#define  RCC_AHB1LPENR_GPIOFLPEN             ((volatile unsigned int)0x00000020)
#define  RCC_AHB1LPENR_GPIOGLPEN             ((volatile unsigned int)0x00000040)
#define  RCC_AHB1LPENR_GPIOHLPEN             ((volatile unsigned int)0x00000080)
#define  RCC_AHB1LPENR_GPIOILPEN             ((volatile unsigned int)0x00000100)
#define  RCC_AHB1LPENR_CRCLPEN               ((volatile unsigned int)0x00001000)
#define  RCC_AHB1LPENR_FLITFLPEN             ((volatile unsigned int)0x00008000)
#define  RCC_AHB1LPENR_SRAM1LPEN             ((volatile unsigned int)0x00010000)
#define  RCC_AHB1LPENR_SRAM2LPEN             ((volatile unsigned int)0x00020000)
#define  RCC_AHB1LPENR_BKPSRAMLPEN           ((volatile unsigned int)0x00040000)
#define  RCC_AHB1LPENR_SRAM3LPEN             ((volatile unsigned int)0x00080000)
#define  RCC_AHB1LPENR_DMA1LPEN              ((volatile unsigned int)0x00200000)
#define  RCC_AHB1LPENR_DMA2LPEN              ((volatile unsigned int)0x00400000)
#define  RCC_AHB1LPENR_ETHMACLPEN            ((volatile unsigned int)0x02000000)
#define  RCC_AHB1LPENR_ETHMACTXLPEN          ((volatile unsigned int)0x04000000)
#define  RCC_AHB1LPENR_ETHMACRXLPEN          ((volatile unsigned int)0x08000000)
#define  RCC_AHB1LPENR_ETHMACPTPLPEN         ((volatile unsigned int)0x10000000)
#define  RCC_AHB1LPENR_OTGHSLPEN             ((volatile unsigned int)0x20000000)
#define  RCC_AHB1LPENR_OTGHSULPILPEN         ((volatile unsigned int)0x40000000)

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define  RCC_AHB2LPENR_DCMILPEN              ((volatile unsigned int)0x00000001)
#define  RCC_AHB2LPENR_RNGLPEN               ((volatile unsigned int)0x00000040)
#define  RCC_AHB2LPENR_OTGFSLPEN             ((volatile unsigned int)0x00000080)

/********************  Bit definition for RCC_AHB3LPENR register  *************/

#define  RCC_AHB3LPENR_FSMCLPEN              ((volatile unsigned int)0x00000001)

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define  RCC_APB1LPENR_TIM2LPEN              ((volatile unsigned int)0x00000001)
#define  RCC_APB1LPENR_TIM3LPEN              ((volatile unsigned int)0x00000002)
#define  RCC_APB1LPENR_TIM4LPEN              ((volatile unsigned int)0x00000004)
#define  RCC_APB1LPENR_TIM5LPEN              ((volatile unsigned int)0x00000008)
#define  RCC_APB1LPENR_TIM6LPEN              ((volatile unsigned int)0x00000010)
#define  RCC_APB1LPENR_TIM7LPEN              ((volatile unsigned int)0x00000020)
#define  RCC_APB1LPENR_TIM12LPEN             ((volatile unsigned int)0x00000040)
#define  RCC_APB1LPENR_TIM13LPEN             ((volatile unsigned int)0x00000080)
#define  RCC_APB1LPENR_TIM14LPEN             ((volatile unsigned int)0x00000100)
#define  RCC_APB1LPENR_WWDGLPEN              ((volatile unsigned int)0x00000800)
#define  RCC_APB1LPENR_SPI2LPEN              ((volatile unsigned int)0x00004000)
#define  RCC_APB1LPENR_SPI3LPEN              ((volatile unsigned int)0x00008000)
#define  RCC_APB1LPENR_USART2LPEN            ((volatile unsigned int)0x00020000)
#define  RCC_APB1LPENR_USART3LPEN            ((volatile unsigned int)0x00040000)
#define  RCC_APB1LPENR_UART4LPEN             ((volatile unsigned int)0x00080000)
#define  RCC_APB1LPENR_UART5LPEN             ((volatile unsigned int)0x00100000)
#define  RCC_APB1LPENR_I2C1LPEN              ((volatile unsigned int)0x00200000)
#define  RCC_APB1LPENR_I2C2LPEN              ((volatile unsigned int)0x00400000)
#define  RCC_APB1LPENR_I2C3LPEN              ((volatile unsigned int)0x00800000)
#define  RCC_APB1LPENR_CAN1LPEN              ((volatile unsigned int)0x02000000)
#define  RCC_APB1LPENR_CAN2LPEN              ((volatile unsigned int)0x04000000)
#define  RCC_APB1LPENR_PWRLPEN               ((volatile unsigned int)0x10000000)
#define  RCC_APB1LPENR_DACLPEN               ((volatile unsigned int)0x20000000)

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define  RCC_APB2LPENR_TIM1LPEN              ((volatile unsigned int)0x00000001)
#define  RCC_APB2LPENR_TIM8LPEN              ((volatile unsigned int)0x00000002)
#define  RCC_APB2LPENR_USART1LPEN            ((volatile unsigned int)0x00000010)
#define  RCC_APB2LPENR_USART6LPEN            ((volatile unsigned int)0x00000020)
#define  RCC_APB2LPENR_ADC1LPEN              ((volatile unsigned int)0x00000100)
#define  RCC_APB2LPENR_ADC2LPEN              ((volatile unsigned int)0x00000200)
#define  RCC_APB2LPENR_ADC3LPEN              ((volatile unsigned int)0x00000400)
#define  RCC_APB2LPENR_SDIOLPEN              ((volatile unsigned int)0x00000800)
#define  RCC_APB2LPENR_SPI1LPEN              ((volatile unsigned int)0x00001000)
#define  RCC_APB2LPENR_SYSCFGLPEN            ((volatile unsigned int)0x00004000)
#define  RCC_APB2LPENR_TIM9LPEN              ((volatile unsigned int)0x00010000)
#define  RCC_APB2LPENR_TIM10LPEN             ((volatile unsigned int)0x00020000)
#define  RCC_APB2LPENR_TIM11LPEN             ((volatile unsigned int)0x00040000)

/********************  Bit definition for RCC_BDCR register  ******************/
#define  RCC_BDCR_LSEON                      ((volatile unsigned int)0x00000001)
#define  RCC_BDCR_LSERDY                     ((volatile unsigned int)0x00000002)
#define  RCC_BDCR_LSEBYP                     ((volatile unsigned int)0x00000004)

#define  RCC_BDCR_RTCSEL                    ((volatile unsigned int)0x00000300)
#define  RCC_BDCR_RTCSEL_0                  ((volatile unsigned int)0x00000100)
#define  RCC_BDCR_RTCSEL_1                  ((volatile unsigned int)0x00000200)

#define  RCC_BDCR_RTCEN                      ((volatile unsigned int)0x00008000)
#define  RCC_BDCR_BDRST                      ((volatile unsigned int)0x00010000)

/********************  Bit definition for RCC_CSR register  *******************/
#define  RCC_CSR_LSION                       ((volatile unsigned int)0x00000001)
#define  RCC_CSR_LSIRDY                      ((volatile unsigned int)0x00000002)
#define  RCC_CSR_RMVF                        ((volatile unsigned int)0x01000000)
#define  RCC_CSR_BORRSTF                     ((volatile unsigned int)0x02000000)
#define  RCC_CSR_PADRSTF                     ((volatile unsigned int)0x04000000)
#define  RCC_CSR_PORRSTF                     ((volatile unsigned int)0x08000000)
#define  RCC_CSR_SFTRSTF                     ((volatile unsigned int)0x10000000)
#define  RCC_CSR_WDGRSTF                     ((volatile unsigned int)0x20000000)
#define  RCC_CSR_WWDGRSTF                    ((volatile unsigned int)0x40000000)
#define  RCC_CSR_LPWRRSTF                    ((volatile unsigned int)0x80000000)

/********************  Bit definition for RCC_SSCGR register  *****************/
#define  RCC_SSCGR_MODPER                    ((volatile unsigned int)0x00001FFF)
#define  RCC_SSCGR_INCSTEP                   ((volatile unsigned int)0x0FFFE000)
#define  RCC_SSCGR_SPREADSEL                 ((volatile unsigned int)0x40000000)
#define  RCC_SSCGR_SSCGEN                    ((volatile unsigned int)0x80000000)

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define  RCC_PLLI2SCFGR_PLLI2SN              ((volatile unsigned int)0x00007FC0)
#define  RCC_PLLI2SCFGR_PLLI2SN_0            ((volatile unsigned int)0x00000040)
#define  RCC_PLLI2SCFGR_PLLI2SN_1            ((volatile unsigned int)0x00000080)
#define  RCC_PLLI2SCFGR_PLLI2SN_2            ((volatile unsigned int)0x00000100)
#define  RCC_PLLI2SCFGR_PLLI2SN_3            ((volatile unsigned int)0x00000200)
#define  RCC_PLLI2SCFGR_PLLI2SN_4            ((volatile unsigned int)0x00000400)
#define  RCC_PLLI2SCFGR_PLLI2SN_5            ((volatile unsigned int)0x00000800)
#define  RCC_PLLI2SCFGR_PLLI2SN_6            ((volatile unsigned int)0x00001000)
#define  RCC_PLLI2SCFGR_PLLI2SN_7            ((volatile unsigned int)0x00002000)
#define  RCC_PLLI2SCFGR_PLLI2SN_8            ((volatile unsigned int)0x00004000)

#define  RCC_PLLI2SCFGR_PLLI2SR              ((volatile unsigned int)0x70000000)
#define  RCC_PLLI2SCFGR_PLLI2SR_0            ((volatile unsigned int)0x10000000)
#define  RCC_PLLI2SCFGR_PLLI2SR_1            ((volatile unsigned int)0x20000000)
#define  RCC_PLLI2SCFGR_PLLI2SR_2            ((volatile unsigned int)0x40000000)

/******************************************************************************/
/*                                                                            */
/*                                    RNG                                     */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RNG_CR register  *******************/
#define RNG_CR_RNGEN                         ((volatile unsigned int)0x00000004)
#define RNG_CR_IE                            ((volatile unsigned int)0x00000008)

/********************  Bits definition for RNG_SR register  *******************/
#define RNG_SR_DRDY                          ((volatile unsigned int)0x00000001)
#define RNG_SR_CECS                          ((volatile unsigned int)0x00000002)
#define RNG_SR_SECS                          ((volatile unsigned int)0x00000004)
#define RNG_SR_CEIS                          ((volatile unsigned int)0x00000020)
#define RNG_SR_SEIS                          ((volatile unsigned int)0x00000040)

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for RTC_TR register  *******************/
#define RTC_TR_PM                            ((volatile unsigned int)0x00400000)
#define RTC_TR_HT                            ((volatile unsigned int)0x00300000)
#define RTC_TR_HT_0                          ((volatile unsigned int)0x00100000)
#define RTC_TR_HT_1                          ((volatile unsigned int)0x00200000)
#define RTC_TR_HU                            ((volatile unsigned int)0x000F0000)
#define RTC_TR_HU_0                          ((volatile unsigned int)0x00010000)
#define RTC_TR_HU_1                          ((volatile unsigned int)0x00020000)
#define RTC_TR_HU_2                          ((volatile unsigned int)0x00040000)
#define RTC_TR_HU_3                          ((volatile unsigned int)0x00080000)
#define RTC_TR_MNT                           ((volatile unsigned int)0x00007000)
#define RTC_TR_MNT_0                         ((volatile unsigned int)0x00001000)
#define RTC_TR_MNT_1                         ((volatile unsigned int)0x00002000)
#define RTC_TR_MNT_2                         ((volatile unsigned int)0x00004000)
#define RTC_TR_MNU                           ((volatile unsigned int)0x00000F00)
#define RTC_TR_MNU_0                         ((volatile unsigned int)0x00000100)
#define RTC_TR_MNU_1                         ((volatile unsigned int)0x00000200)
#define RTC_TR_MNU_2                         ((volatile unsigned int)0x00000400)
#define RTC_TR_MNU_3                         ((volatile unsigned int)0x00000800)
#define RTC_TR_ST                            ((volatile unsigned int)0x00000070)
#define RTC_TR_ST_0                          ((volatile unsigned int)0x00000010)
#define RTC_TR_ST_1                          ((volatile unsigned int)0x00000020)
#define RTC_TR_ST_2                          ((volatile unsigned int)0x00000040)
#define RTC_TR_SU                            ((volatile unsigned int)0x0000000F)
#define RTC_TR_SU_0                          ((volatile unsigned int)0x00000001)
#define RTC_TR_SU_1                          ((volatile unsigned int)0x00000002)
#define RTC_TR_SU_2                          ((volatile unsigned int)0x00000004)
#define RTC_TR_SU_3                          ((volatile unsigned int)0x00000008)

/********************  Bits definition for RTC_DR register  *******************/
#define RTC_DR_YT                            ((volatile unsigned int)0x00F00000)
#define RTC_DR_YT_0                          ((volatile unsigned int)0x00100000)
#define RTC_DR_YT_1                          ((volatile unsigned int)0x00200000)
#define RTC_DR_YT_2                          ((volatile unsigned int)0x00400000)
#define RTC_DR_YT_3                          ((volatile unsigned int)0x00800000)
#define RTC_DR_YU                            ((volatile unsigned int)0x000F0000)
#define RTC_DR_YU_0                          ((volatile unsigned int)0x00010000)
#define RTC_DR_YU_1                          ((volatile unsigned int)0x00020000)
#define RTC_DR_YU_2                          ((volatile unsigned int)0x00040000)
#define RTC_DR_YU_3                          ((volatile unsigned int)0x00080000)
#define RTC_DR_WDU                           ((volatile unsigned int)0x0000E000)
#define RTC_DR_WDU_0                         ((volatile unsigned int)0x00002000)
#define RTC_DR_WDU_1                         ((volatile unsigned int)0x00004000)
#define RTC_DR_WDU_2                         ((volatile unsigned int)0x00008000)
#define RTC_DR_MT                            ((volatile unsigned int)0x00001000)
#define RTC_DR_MU                            ((volatile unsigned int)0x00000F00)
#define RTC_DR_MU_0                          ((volatile unsigned int)0x00000100)
#define RTC_DR_MU_1                          ((volatile unsigned int)0x00000200)
#define RTC_DR_MU_2                          ((volatile unsigned int)0x00000400)
#define RTC_DR_MU_3                          ((volatile unsigned int)0x00000800)
#define RTC_DR_DT                            ((volatile unsigned int)0x00000030)
#define RTC_DR_DT_0                          ((volatile unsigned int)0x00000010)
#define RTC_DR_DT_1                          ((volatile unsigned int)0x00000020)
#define RTC_DR_DU                            ((volatile unsigned int)0x0000000F)
#define RTC_DR_DU_0                          ((volatile unsigned int)0x00000001)
#define RTC_DR_DU_1                          ((volatile unsigned int)0x00000002)
#define RTC_DR_DU_2                          ((volatile unsigned int)0x00000004)
#define RTC_DR_DU_3                          ((volatile unsigned int)0x00000008)

/********************  Bits definition for RTC_CR register  *******************/
#define RTC_CR_COE                           ((volatile unsigned int)0x00800000)
#define RTC_CR_OSEL                          ((volatile unsigned int)0x00600000)
#define RTC_CR_OSEL_0                        ((volatile unsigned int)0x00200000)
#define RTC_CR_OSEL_1                        ((volatile unsigned int)0x00400000)
#define RTC_CR_POL                           ((volatile unsigned int)0x00100000)
#define RTC_CR_COSEL                         ((volatile unsigned int)0x00080000)
#define RTC_CR_BCK                           ((volatile unsigned int)0x00040000)
#define RTC_CR_SUB1H                         ((volatile unsigned int)0x00020000)
#define RTC_CR_ADD1H                         ((volatile unsigned int)0x00010000)
#define RTC_CR_TSIE                          ((volatile unsigned int)0x00008000)
#define RTC_CR_WUTIE                         ((volatile unsigned int)0x00004000)
#define RTC_CR_ALRBIE                        ((volatile unsigned int)0x00002000)
#define RTC_CR_ALRAIE                        ((volatile unsigned int)0x00001000)
#define RTC_CR_TSE                           ((volatile unsigned int)0x00000800)
#define RTC_CR_WUTE                          ((volatile unsigned int)0x00000400)
#define RTC_CR_ALRBE                         ((volatile unsigned int)0x00000200)
#define RTC_CR_ALRAE                         ((volatile unsigned int)0x00000100)
#define RTC_CR_DCE                           ((volatile unsigned int)0x00000080)
#define RTC_CR_FMT                           ((volatile unsigned int)0x00000040)
#define RTC_CR_BYPSHAD                       ((volatile unsigned int)0x00000020)
#define RTC_CR_REFCKON                       ((volatile unsigned int)0x00000010)
#define RTC_CR_TSEDGE                        ((volatile unsigned int)0x00000008)
#define RTC_CR_WUCKSEL                       ((volatile unsigned int)0x00000007)
#define RTC_CR_WUCKSEL_0                     ((volatile unsigned int)0x00000001)
#define RTC_CR_WUCKSEL_1                     ((volatile unsigned int)0x00000002)
#define RTC_CR_WUCKSEL_2                     ((volatile unsigned int)0x00000004)

/********************  Bits definition for RTC_ISR register  ******************/
#define RTC_ISR_RECALPF                      ((volatile unsigned int)0x00010000)
#define RTC_ISR_TAMP1F                       ((volatile unsigned int)0x00002000)
#define RTC_ISR_TAMP2F                       ((volatile unsigned int)0x00004000)
#define RTC_ISR_TSOVF                        ((volatile unsigned int)0x00001000)
#define RTC_ISR_TSF                          ((volatile unsigned int)0x00000800)
#define RTC_ISR_WUTF                         ((volatile unsigned int)0x00000400)
#define RTC_ISR_ALRBF                        ((volatile unsigned int)0x00000200)
#define RTC_ISR_ALRAF                        ((volatile unsigned int)0x00000100)
#define RTC_ISR_INIT                         ((volatile unsigned int)0x00000080)
#define RTC_ISR_INITF                        ((volatile unsigned int)0x00000040)
#define RTC_ISR_RSF                          ((volatile unsigned int)0x00000020)
#define RTC_ISR_INITS                        ((volatile unsigned int)0x00000010)
#define RTC_ISR_SHPF                         ((volatile unsigned int)0x00000008)
#define RTC_ISR_WUTWF                        ((volatile unsigned int)0x00000004)
#define RTC_ISR_ALRBWF                       ((volatile unsigned int)0x00000002)
#define RTC_ISR_ALRAWF                       ((volatile unsigned int)0x00000001)

/********************  Bits definition for RTC_PRER register  *****************/
#define RTC_PRER_PREDIV_A                    ((volatile unsigned int)0x007F0000)
#define RTC_PRER_PREDIV_S                    ((volatile unsigned int)0x00001FFF)

/********************  Bits definition for RTC_WUTR register  *****************/
#define RTC_WUTR_WUT                         ((volatile unsigned int)0x0000FFFF)

/********************  Bits definition for RTC_CALIBR register  ***************/
#define RTC_CALIBR_DCS                       ((volatile unsigned int)0x00000080)
#define RTC_CALIBR_DC                        ((volatile unsigned int)0x0000001F)

/********************  Bits definition for RTC_ALRMAR register  ***************/
#define RTC_ALRMAR_MSK4                      ((volatile unsigned int)0x80000000)
#define RTC_ALRMAR_WDSEL                     ((volatile unsigned int)0x40000000)
#define RTC_ALRMAR_DT                        ((volatile unsigned int)0x30000000)
#define RTC_ALRMAR_DT_0                      ((volatile unsigned int)0x10000000)
#define RTC_ALRMAR_DT_1                      ((volatile unsigned int)0x20000000)
#define RTC_ALRMAR_DU                        ((volatile unsigned int)0x0F000000)
#define RTC_ALRMAR_DU_0                      ((volatile unsigned int)0x01000000)
#define RTC_ALRMAR_DU_1                      ((volatile unsigned int)0x02000000)
#define RTC_ALRMAR_DU_2                      ((volatile unsigned int)0x04000000)
#define RTC_ALRMAR_DU_3                      ((volatile unsigned int)0x08000000)
#define RTC_ALRMAR_MSK3                      ((volatile unsigned int)0x00800000)
#define RTC_ALRMAR_PM                        ((volatile unsigned int)0x00400000)
#define RTC_ALRMAR_HT                        ((volatile unsigned int)0x00300000)
#define RTC_ALRMAR_HT_0                      ((volatile unsigned int)0x00100000)
#define RTC_ALRMAR_HT_1                      ((volatile unsigned int)0x00200000)
#define RTC_ALRMAR_HU                        ((volatile unsigned int)0x000F0000)
#define RTC_ALRMAR_HU_0                      ((volatile unsigned int)0x00010000)
#define RTC_ALRMAR_HU_1                      ((volatile unsigned int)0x00020000)
#define RTC_ALRMAR_HU_2                      ((volatile unsigned int)0x00040000)
#define RTC_ALRMAR_HU_3                      ((volatile unsigned int)0x00080000)
#define RTC_ALRMAR_MSK2                      ((volatile unsigned int)0x00008000)
#define RTC_ALRMAR_MNT                       ((volatile unsigned int)0x00007000)
#define RTC_ALRMAR_MNT_0                     ((volatile unsigned int)0x00001000)
#define RTC_ALRMAR_MNT_1                     ((volatile unsigned int)0x00002000)
#define RTC_ALRMAR_MNT_2                     ((volatile unsigned int)0x00004000)
#define RTC_ALRMAR_MNU                       ((volatile unsigned int)0x00000F00)
#define RTC_ALRMAR_MNU_0                     ((volatile unsigned int)0x00000100)
#define RTC_ALRMAR_MNU_1                     ((volatile unsigned int)0x00000200)
#define RTC_ALRMAR_MNU_2                     ((volatile unsigned int)0x00000400)
#define RTC_ALRMAR_MNU_3                     ((volatile unsigned int)0x00000800)
#define RTC_ALRMAR_MSK1                      ((volatile unsigned int)0x00000080)
#define RTC_ALRMAR_ST                        ((volatile unsigned int)0x00000070)
#define RTC_ALRMAR_ST_0                      ((volatile unsigned int)0x00000010)
#define RTC_ALRMAR_ST_1                      ((volatile unsigned int)0x00000020)
#define RTC_ALRMAR_ST_2                      ((volatile unsigned int)0x00000040)
#define RTC_ALRMAR_SU                        ((volatile unsigned int)0x0000000F)
#define RTC_ALRMAR_SU_0                      ((volatile unsigned int)0x00000001)
#define RTC_ALRMAR_SU_1                      ((volatile unsigned int)0x00000002)
#define RTC_ALRMAR_SU_2                      ((volatile unsigned int)0x00000004)
#define RTC_ALRMAR_SU_3                      ((volatile unsigned int)0x00000008)

/********************  Bits definition for RTC_ALRMBR register  ***************/
#define RTC_ALRMBR_MSK4                      ((volatile unsigned int)0x80000000)
#define RTC_ALRMBR_WDSEL                     ((volatile unsigned int)0x40000000)
#define RTC_ALRMBR_DT                        ((volatile unsigned int)0x30000000)
#define RTC_ALRMBR_DT_0                      ((volatile unsigned int)0x10000000)
#define RTC_ALRMBR_DT_1                      ((volatile unsigned int)0x20000000)
#define RTC_ALRMBR_DU                        ((volatile unsigned int)0x0F000000)
#define RTC_ALRMBR_DU_0                      ((volatile unsigned int)0x01000000)
#define RTC_ALRMBR_DU_1                      ((volatile unsigned int)0x02000000)
#define RTC_ALRMBR_DU_2                      ((volatile unsigned int)0x04000000)
#define RTC_ALRMBR_DU_3                      ((volatile unsigned int)0x08000000)
#define RTC_ALRMBR_MSK3                      ((volatile unsigned int)0x00800000)
#define RTC_ALRMBR_PM                        ((volatile unsigned int)0x00400000)
#define RTC_ALRMBR_HT                        ((volatile unsigned int)0x00300000)
#define RTC_ALRMBR_HT_0                      ((volatile unsigned int)0x00100000)
#define RTC_ALRMBR_HT_1                      ((volatile unsigned int)0x00200000)
#define RTC_ALRMBR_HU                        ((volatile unsigned int)0x000F0000)
#define RTC_ALRMBR_HU_0                      ((volatile unsigned int)0x00010000)
#define RTC_ALRMBR_HU_1                      ((volatile unsigned int)0x00020000)
#define RTC_ALRMBR_HU_2                      ((volatile unsigned int)0x00040000)
#define RTC_ALRMBR_HU_3                      ((volatile unsigned int)0x00080000)
#define RTC_ALRMBR_MSK2                      ((volatile unsigned int)0x00008000)
#define RTC_ALRMBR_MNT                       ((volatile unsigned int)0x00007000)
#define RTC_ALRMBR_MNT_0                     ((volatile unsigned int)0x00001000)
#define RTC_ALRMBR_MNT_1                     ((volatile unsigned int)0x00002000)
#define RTC_ALRMBR_MNT_2                     ((volatile unsigned int)0x00004000)
#define RTC_ALRMBR_MNU                       ((volatile unsigned int)0x00000F00)
#define RTC_ALRMBR_MNU_0                     ((volatile unsigned int)0x00000100)
#define RTC_ALRMBR_MNU_1                     ((volatile unsigned int)0x00000200)
#define RTC_ALRMBR_MNU_2                     ((volatile unsigned int)0x00000400)
#define RTC_ALRMBR_MNU_3                     ((volatile unsigned int)0x00000800)
#define RTC_ALRMBR_MSK1                      ((volatile unsigned int)0x00000080)
#define RTC_ALRMBR_ST                        ((volatile unsigned int)0x00000070)
#define RTC_ALRMBR_ST_0                      ((volatile unsigned int)0x00000010)
#define RTC_ALRMBR_ST_1                      ((volatile unsigned int)0x00000020)
#define RTC_ALRMBR_ST_2                      ((volatile unsigned int)0x00000040)
#define RTC_ALRMBR_SU                        ((volatile unsigned int)0x0000000F)
#define RTC_ALRMBR_SU_0                      ((volatile unsigned int)0x00000001)
#define RTC_ALRMBR_SU_1                      ((volatile unsigned int)0x00000002)
#define RTC_ALRMBR_SU_2                      ((volatile unsigned int)0x00000004)
#define RTC_ALRMBR_SU_3                      ((volatile unsigned int)0x00000008)

/********************  Bits definition for RTC_WPR register  ******************/
#define RTC_WPR_KEY                          ((volatile unsigned int)0x000000FF)

/********************  Bits definition for RTC_SSR register  ******************/
#define RTC_SSR_SS                           ((volatile unsigned int)0x0000FFFF)

/********************  Bits definition for RTC_SHIFTR register  ***************/
#define RTC_SHIFTR_SUBFS                     ((volatile unsigned int)0x00007FFF)
#define RTC_SHIFTR_ADD1S                     ((volatile unsigned int)0x80000000)

/********************  Bits definition for RTC_TSTR register  *****************/
#define RTC_TSTR_PM                          ((volatile unsigned int)0x00400000)
#define RTC_TSTR_HT                          ((volatile unsigned int)0x00300000)
#define RTC_TSTR_HT_0                        ((volatile unsigned int)0x00100000)
#define RTC_TSTR_HT_1                        ((volatile unsigned int)0x00200000)
#define RTC_TSTR_HU                          ((volatile unsigned int)0x000F0000)
#define RTC_TSTR_HU_0                        ((volatile unsigned int)0x00010000)
#define RTC_TSTR_HU_1                        ((volatile unsigned int)0x00020000)
#define RTC_TSTR_HU_2                        ((volatile unsigned int)0x00040000)
#define RTC_TSTR_HU_3                        ((volatile unsigned int)0x00080000)
#define RTC_TSTR_MNT                         ((volatile unsigned int)0x00007000)
#define RTC_TSTR_MNT_0                       ((volatile unsigned int)0x00001000)
#define RTC_TSTR_MNT_1                       ((volatile unsigned int)0x00002000)
#define RTC_TSTR_MNT_2                       ((volatile unsigned int)0x00004000)
#define RTC_TSTR_MNU                         ((volatile unsigned int)0x00000F00)
#define RTC_TSTR_MNU_0                       ((volatile unsigned int)0x00000100)
#define RTC_TSTR_MNU_1                       ((volatile unsigned int)0x00000200)
#define RTC_TSTR_MNU_2                       ((volatile unsigned int)0x00000400)
#define RTC_TSTR_MNU_3                       ((volatile unsigned int)0x00000800)
#define RTC_TSTR_ST                          ((volatile unsigned int)0x00000070)
#define RTC_TSTR_ST_0                        ((volatile unsigned int)0x00000010)
#define RTC_TSTR_ST_1                        ((volatile unsigned int)0x00000020)
#define RTC_TSTR_ST_2                        ((volatile unsigned int)0x00000040)
#define RTC_TSTR_SU                          ((volatile unsigned int)0x0000000F)
#define RTC_TSTR_SU_0                        ((volatile unsigned int)0x00000001)
#define RTC_TSTR_SU_1                        ((volatile unsigned int)0x00000002)
#define RTC_TSTR_SU_2                        ((volatile unsigned int)0x00000004)
#define RTC_TSTR_SU_3                        ((volatile unsigned int)0x00000008)

/********************  Bits definition for RTC_TSDR register  *****************/
#define RTC_TSDR_WDU                         ((volatile unsigned int)0x0000E000)
#define RTC_TSDR_WDU_0                       ((volatile unsigned int)0x00002000)
#define RTC_TSDR_WDU_1                       ((volatile unsigned int)0x00004000)
#define RTC_TSDR_WDU_2                       ((volatile unsigned int)0x00008000)
#define RTC_TSDR_MT                          ((volatile unsigned int)0x00001000)
#define RTC_TSDR_MU                          ((volatile unsigned int)0x00000F00)
#define RTC_TSDR_MU_0                        ((volatile unsigned int)0x00000100)
#define RTC_TSDR_MU_1                        ((volatile unsigned int)0x00000200)
#define RTC_TSDR_MU_2                        ((volatile unsigned int)0x00000400)
#define RTC_TSDR_MU_3                        ((volatile unsigned int)0x00000800)
#define RTC_TSDR_DT                          ((volatile unsigned int)0x00000030)
#define RTC_TSDR_DT_0                        ((volatile unsigned int)0x00000010)
#define RTC_TSDR_DT_1                        ((volatile unsigned int)0x00000020)
#define RTC_TSDR_DU                          ((volatile unsigned int)0x0000000F)
#define RTC_TSDR_DU_0                        ((volatile unsigned int)0x00000001)
#define RTC_TSDR_DU_1                        ((volatile unsigned int)0x00000002)
#define RTC_TSDR_DU_2                        ((volatile unsigned int)0x00000004)
#define RTC_TSDR_DU_3                        ((volatile unsigned int)0x00000008)

/********************  Bits definition for RTC_TSSSR register  ****************/
#define RTC_TSSSR_SS                         ((volatile unsigned int)0x0000FFFF)

/********************  Bits definition for RTC_CAL register  *****************/
#define RTC_CALR_CALP                        ((volatile unsigned int)0x00008000)
#define RTC_CALR_CALW8                       ((volatile unsigned int)0x00004000)
#define RTC_CALR_CALW16                      ((volatile unsigned int)0x00002000)
#define RTC_CALR_CALM                        ((volatile unsigned int)0x000001FF)
#define RTC_CALR_CALM_0                      ((volatile unsigned int)0x00000001)
#define RTC_CALR_CALM_1                      ((volatile unsigned int)0x00000002)
#define RTC_CALR_CALM_2                      ((volatile unsigned int)0x00000004)
#define RTC_CALR_CALM_3                      ((volatile unsigned int)0x00000008)
#define RTC_CALR_CALM_4                      ((volatile unsigned int)0x00000010)
#define RTC_CALR_CALM_5                      ((volatile unsigned int)0x00000020)
#define RTC_CALR_CALM_6                      ((volatile unsigned int)0x00000040)
#define RTC_CALR_CALM_7                      ((volatile unsigned int)0x00000080)
#define RTC_CALR_CALM_8                      ((volatile unsigned int)0x00000100)

/********************  Bits definition for RTC_TAFCR register  ****************/
#define RTC_TAFCR_ALARMOUTTYPE               ((volatile unsigned int)0x00040000)
#define RTC_TAFCR_TSINSEL                    ((volatile unsigned int)0x00020000)
#define RTC_TAFCR_TAMPINSEL                  ((volatile unsigned int)0x00010000)
#define RTC_TAFCR_TAMPPUDIS                  ((volatile unsigned int)0x00008000)
#define RTC_TAFCR_TAMPPRCH                   ((volatile unsigned int)0x00006000)
#define RTC_TAFCR_TAMPPRCH_0                 ((volatile unsigned int)0x00002000)
#define RTC_TAFCR_TAMPPRCH_1                 ((volatile unsigned int)0x00004000)
#define RTC_TAFCR_TAMPFLT                    ((volatile unsigned int)0x00001800)
#define RTC_TAFCR_TAMPFLT_0                  ((volatile unsigned int)0x00000800)
#define RTC_TAFCR_TAMPFLT_1                  ((volatile unsigned int)0x00001000)
#define RTC_TAFCR_TAMPFREQ                   ((volatile unsigned int)0x00000700)
#define RTC_TAFCR_TAMPFREQ_0                 ((volatile unsigned int)0x00000100)
#define RTC_TAFCR_TAMPFREQ_1                 ((volatile unsigned int)0x00000200)
#define RTC_TAFCR_TAMPFREQ_2                 ((volatile unsigned int)0x00000400)
#define RTC_TAFCR_TAMPTS                     ((volatile unsigned int)0x00000080)
#define RTC_TAFCR_TAMP2TRG                   ((volatile unsigned int)0x00000010)
#define RTC_TAFCR_TAMP2E                     ((volatile unsigned int)0x00000008)
#define RTC_TAFCR_TAMPIE                     ((volatile unsigned int)0x00000004)
#define RTC_TAFCR_TAMP1TRG                   ((volatile unsigned int)0x00000002)
#define RTC_TAFCR_TAMP1E                     ((volatile unsigned int)0x00000001)

/********************  Bits definition for RTC_ALRMASSR register  *************/
#define RTC_ALRMASSR_MASKSS                  ((volatile unsigned int)0x0F000000)
#define RTC_ALRMASSR_MASKSS_0                ((volatile unsigned int)0x01000000)
#define RTC_ALRMASSR_MASKSS_1                ((volatile unsigned int)0x02000000)
#define RTC_ALRMASSR_MASKSS_2                ((volatile unsigned int)0x04000000)
#define RTC_ALRMASSR_MASKSS_3                ((volatile unsigned int)0x08000000)
#define RTC_ALRMASSR_SS                      ((volatile unsigned int)0x00007FFF)

/********************  Bits definition for RTC_ALRMBSSR register  *************/
#define RTC_ALRMBSSR_MASKSS                  ((volatile unsigned int)0x0F000000)
#define RTC_ALRMBSSR_MASKSS_0                ((volatile unsigned int)0x01000000)
#define RTC_ALRMBSSR_MASKSS_1                ((volatile unsigned int)0x02000000)
#define RTC_ALRMBSSR_MASKSS_2                ((volatile unsigned int)0x04000000)
#define RTC_ALRMBSSR_MASKSS_3                ((volatile unsigned int)0x08000000)
#define RTC_ALRMBSSR_SS                      ((volatile unsigned int)0x00007FFF)

/********************  Bits definition for RTC_BKP0R register  ****************/
#define RTC_BKP0R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP1R register  ****************/
#define RTC_BKP1R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP2R register  ****************/
#define RTC_BKP2R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP3R register  ****************/
#define RTC_BKP3R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP4R register  ****************/
#define RTC_BKP4R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP5R register  ****************/
#define RTC_BKP5R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP6R register  ****************/
#define RTC_BKP6R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP7R register  ****************/
#define RTC_BKP7R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP8R register  ****************/
#define RTC_BKP8R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP9R register  ****************/
#define RTC_BKP9R                            ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP10R register  ***************/
#define RTC_BKP10R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP11R register  ***************/
#define RTC_BKP11R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP12R register  ***************/
#define RTC_BKP12R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP13R register  ***************/
#define RTC_BKP13R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP14R register  ***************/
#define RTC_BKP14R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP15R register  ***************/
#define RTC_BKP15R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP16R register  ***************/
#define RTC_BKP16R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP17R register  ***************/
#define RTC_BKP17R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP18R register  ***************/
#define RTC_BKP18R                           ((volatile unsigned int)0xFFFFFFFF)

/********************  Bits definition for RTC_BKP19R register  ***************/
#define RTC_BKP19R                           ((volatile unsigned int)0xFFFFFFFF)



/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                  ((volatile unsigned int)0x03)               /*!<PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0                ((volatile unsigned int)0x01)               /*!<Bit 0 */
#define  SDIO_POWER_PWRCTRL_1                ((volatile unsigned int)0x02)               /*!<Bit 1 */

/******************  Bit definition for SDIO_CLKCR register  ******************/
#define  SDIO_CLKCR_CLKDIV                   ((volatile unsigned int)0x00FF)            /*!<Clock divide factor             */
#define  SDIO_CLKCR_CLKEN                    ((volatile unsigned int)0x0100)            /*!<Clock enable bit                */
#define  SDIO_CLKCR_PWRSAV                   ((volatile unsigned int)0x0200)            /*!<Power saving configuration bit  */
#define  SDIO_CLKCR_BYPASS                   ((volatile unsigned int)0x0400)            /*!<Clock divider bypass enable bit */

#define  SDIO_CLKCR_WIDBUS                   ((volatile unsigned int)0x1800)            /*!<WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                 ((volatile unsigned int)0x0800)            /*!<Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                 ((volatile unsigned int)0x1000)            /*!<Bit 1 */

#define  SDIO_CLKCR_NEGEDGE                  ((volatile unsigned int)0x2000)            /*!<SDIO_CK dephasing selection bit */
#define  SDIO_CLKCR_HWFC_EN                  ((volatile unsigned int)0x4000)            /*!<HW Flow Control enable          */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_CMDARG                     ((volatile unsigned int)0xFFFFFFFF)            /*!<Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDINDEX                   ((volatile unsigned int)0x003F)            /*!<Command Index                               */

#define  SDIO_CMD_WAITRESP                   ((volatile unsigned int)0x00C0)            /*!<WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                 ((volatile unsigned int)0x0040)            /*!< Bit 0 */
#define  SDIO_CMD_WAITRESP_1                 ((volatile unsigned int)0x0080)            /*!< Bit 1 */

#define  SDIO_CMD_WAITINT                    ((volatile unsigned int)0x0100)            /*!<CPSM Waits for Interrupt Request                               */
#define  SDIO_CMD_WAITPEND                   ((volatile unsigned int)0x0200)            /*!<CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CPSMEN                     ((volatile unsigned int)0x0400)            /*!<Command path state machine (CPSM) Enable bit                   */
#define  SDIO_CMD_SDIOSUSPEND                ((volatile unsigned int)0x0800)            /*!<SD I/O suspend command                                         */
#define  SDIO_CMD_ENCMDCOMPL                 ((volatile unsigned int)0x1000)            /*!<Enable CMD completion                                          */
#define  SDIO_CMD_NIEN                       ((volatile unsigned int)0x2000)            /*!<Not Interrupt Enable */
#define  SDIO_CMD_CEATACMD                   ((volatile unsigned int)0x4000)            /*!<CE-ATA command       */

/*****************  Bit definition for SDIO_RESPCMD register  *****************/
#define  SDIO_RESPCMD_RESPCMD                ((volatile unsigned int)0x3F)               /*!<Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0              ((volatile unsigned int)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP1 register  ******************/
#define  SDIO_RESP1_CARDSTATUS1              ((volatile unsigned int)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP2 register  ******************/
#define  SDIO_RESP2_CARDSTATUS2              ((volatile unsigned int)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP3 register  ******************/
#define  SDIO_RESP3_CARDSTATUS3              ((volatile unsigned int)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_RESP4 register  ******************/
#define  SDIO_RESP4_CARDSTATUS4              ((volatile unsigned int)0xFFFFFFFF)        /*!<Card Status */

/******************  Bit definition for SDIO_DTIMER register  *****************/
#define  SDIO_DTIMER_DATATIME                ((volatile unsigned int)0xFFFFFFFF)        /*!<Data timeout period. */

/******************  Bit definition for SDIO_DLEN register  *******************/
#define  SDIO_DLEN_DATALENGTH                ((volatile unsigned int)0x01FFFFFF)        /*!<Data length value    */

/******************  Bit definition for SDIO_DCTRL register  ******************/
#define  SDIO_DCTRL_DTEN                     ((volatile unsigned int)0x0001)            /*!<Data transfer enabled bit         */
#define  SDIO_DCTRL_DTDIR                    ((volatile unsigned int)0x0002)            /*!<Data transfer direction selection */
#define  SDIO_DCTRL_DTMODE                   ((volatile unsigned int)0x0004)            /*!<Data transfer mode selection      */
#define  SDIO_DCTRL_DMAEN                    ((volatile unsigned int)0x0008)            /*!<DMA enabled bit                   */

#define  SDIO_DCTRL_DBLOCKSIZE               ((volatile unsigned int)0x00F0)            /*!<DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DCTRL_DBLOCKSIZE_0             ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  SDIO_DCTRL_DBLOCKSIZE_1             ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  SDIO_DCTRL_DBLOCKSIZE_2             ((volatile unsigned int)0x0040)            /*!<Bit 2 */
#define  SDIO_DCTRL_DBLOCKSIZE_3             ((volatile unsigned int)0x0080)            /*!<Bit 3 */

#define  SDIO_DCTRL_RWSTART                  ((volatile unsigned int)0x0100)            /*!<Read wait start         */
#define  SDIO_DCTRL_RWSTOP                   ((volatile unsigned int)0x0200)            /*!<Read wait stop          */
#define  SDIO_DCTRL_RWMOD                    ((volatile unsigned int)0x0400)            /*!<Read wait mode          */
#define  SDIO_DCTRL_SDIOEN                   ((volatile unsigned int)0x0800)            /*!<SD I/O enable functions */

/******************  Bit definition for SDIO_DCOUNT register  *****************/
#define  SDIO_DCOUNT_DATACOUNT               ((volatile unsigned int)0x01FFFFFF)        /*!<Data count value */

/******************  Bit definition for SDIO_STA register  ********************/
#define  SDIO_STA_CCRCFAIL                   ((volatile unsigned int)0x00000001)        /*!<Command response received (CRC check failed)  */
#define  SDIO_STA_DCRCFAIL                   ((volatile unsigned int)0x00000002)        /*!<Data block sent/received (CRC check failed)   */
#define  SDIO_STA_CTIMEOUT                   ((volatile unsigned int)0x00000004)        /*!<Command response timeout                      */
#define  SDIO_STA_DTIMEOUT                   ((volatile unsigned int)0x00000008)        /*!<Data timeout                                  */
#define  SDIO_STA_TXUNDERR                   ((volatile unsigned int)0x00000010)        /*!<Transmit FIFO underrun error                  */
#define  SDIO_STA_RXOVERR                    ((volatile unsigned int)0x00000020)        /*!<Received FIFO overrun error                   */
#define  SDIO_STA_CMDREND                    ((volatile unsigned int)0x00000040)        /*!<Command response received (CRC check passed)  */
#define  SDIO_STA_CMDSENT                    ((volatile unsigned int)0x00000080)        /*!<Command sent (no response required)           */
#define  SDIO_STA_DATAEND                    ((volatile unsigned int)0x00000100)        /*!<Data end (data counter, SDIDCOUNT, is zero)   */
#define  SDIO_STA_STBITERR                   ((volatile unsigned int)0x00000200)        /*!<Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STA_DBCKEND                    ((volatile unsigned int)0x00000400)        /*!<Data block sent/received (CRC check passed)   */
#define  SDIO_STA_CMDACT                     ((volatile unsigned int)0x00000800)        /*!<Command transfer in progress                  */
#define  SDIO_STA_TXACT                      ((volatile unsigned int)0x00001000)        /*!<Data transmit in progress                     */
#define  SDIO_STA_RXACT                      ((volatile unsigned int)0x00002000)        /*!<Data receive in progress                      */
#define  SDIO_STA_TXFIFOHE                   ((volatile unsigned int)0x00004000)        /*!<Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STA_RXFIFOHF                   ((volatile unsigned int)0x00008000)        /*!<Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STA_TXFIFOF                    ((volatile unsigned int)0x00010000)        /*!<Transmit FIFO full                            */
#define  SDIO_STA_RXFIFOF                    ((volatile unsigned int)0x00020000)        /*!<Receive FIFO full                             */
#define  SDIO_STA_TXFIFOE                    ((volatile unsigned int)0x00040000)        /*!<Transmit FIFO empty                           */
#define  SDIO_STA_RXFIFOE                    ((volatile unsigned int)0x00080000)        /*!<Receive FIFO empty                            */
#define  SDIO_STA_TXDAVL                     ((volatile unsigned int)0x00100000)        /*!<Data available in transmit FIFO               */
#define  SDIO_STA_RXDAVL                     ((volatile unsigned int)0x00200000)        /*!<Data available in receive FIFO                */
#define  SDIO_STA_SDIOIT                     ((volatile unsigned int)0x00400000)        /*!<SDIO interrupt received                       */
#define  SDIO_STA_CEATAEND                   ((volatile unsigned int)0x00800000)        /*!<CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_ICR register  *******************/
#define  SDIO_ICR_CCRCFAILC                  ((volatile unsigned int)0x00000001)        /*!<CCRCFAIL flag clear bit */
#define  SDIO_ICR_DCRCFAILC                  ((volatile unsigned int)0x00000002)        /*!<DCRCFAIL flag clear bit */
#define  SDIO_ICR_CTIMEOUTC                  ((volatile unsigned int)0x00000004)        /*!<CTIMEOUT flag clear bit */
#define  SDIO_ICR_DTIMEOUTC                  ((volatile unsigned int)0x00000008)        /*!<DTIMEOUT flag clear bit */
#define  SDIO_ICR_TXUNDERRC                  ((volatile unsigned int)0x00000010)        /*!<TXUNDERR flag clear bit */
#define  SDIO_ICR_RXOVERRC                   ((volatile unsigned int)0x00000020)        /*!<RXOVERR flag clear bit  */
#define  SDIO_ICR_CMDRENDC                   ((volatile unsigned int)0x00000040)        /*!<CMDREND flag clear bit  */
#define  SDIO_ICR_CMDSENTC                   ((volatile unsigned int)0x00000080)        /*!<CMDSENT flag clear bit  */
#define  SDIO_ICR_DATAENDC                   ((volatile unsigned int)0x00000100)        /*!<DATAEND flag clear bit  */
#define  SDIO_ICR_STBITERRC                  ((volatile unsigned int)0x00000200)        /*!<STBITERR flag clear bit */
#define  SDIO_ICR_DBCKENDC                   ((volatile unsigned int)0x00000400)        /*!<DBCKEND flag clear bit  */
#define  SDIO_ICR_SDIOITC                    ((volatile unsigned int)0x00400000)        /*!<SDIOIT flag clear bit   */
#define  SDIO_ICR_CEATAENDC                  ((volatile unsigned int)0x00800000)        /*!<CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE                ((volatile unsigned int)0x00000001)        /*!<Command CRC Fail Interrupt Enable          */
#define  SDIO_MASK_DCRCFAILIE                ((volatile unsigned int)0x00000002)        /*!<Data CRC Fail Interrupt Enable             */
#define  SDIO_MASK_CTIMEOUTIE                ((volatile unsigned int)0x00000004)        /*!<Command TimeOut Interrupt Enable           */
#define  SDIO_MASK_DTIMEOUTIE                ((volatile unsigned int)0x00000008)        /*!<Data TimeOut Interrupt Enable              */
#define  SDIO_MASK_TXUNDERRIE                ((volatile unsigned int)0x00000010)        /*!<Tx FIFO UnderRun Error Interrupt Enable    */
#define  SDIO_MASK_RXOVERRIE                 ((volatile unsigned int)0x00000020)        /*!<Rx FIFO OverRun Error Interrupt Enable     */
#define  SDIO_MASK_CMDRENDIE                 ((volatile unsigned int)0x00000040)        /*!<Command Response Received Interrupt Enable */
#define  SDIO_MASK_CMDSENTIE                 ((volatile unsigned int)0x00000080)        /*!<Command Sent Interrupt Enable              */
#define  SDIO_MASK_DATAENDIE                 ((volatile unsigned int)0x00000100)        /*!<Data End Interrupt Enable                  */
#define  SDIO_MASK_STBITERRIE                ((volatile unsigned int)0x00000200)        /*!<Start Bit Error Interrupt Enable           */
#define  SDIO_MASK_DBCKENDIE                 ((volatile unsigned int)0x00000400)        /*!<Data Block End Interrupt Enable            */
#define  SDIO_MASK_CMDACTIE                  ((volatile unsigned int)0x00000800)        /*!<CCommand Acting Interrupt Enable           */
#define  SDIO_MASK_TXACTIE                   ((volatile unsigned int)0x00001000)        /*!<Data Transmit Acting Interrupt Enable      */
#define  SDIO_MASK_RXACTIE                   ((volatile unsigned int)0x00002000)        /*!<Data receive acting interrupt enabled      */
#define  SDIO_MASK_TXFIFOHEIE                ((volatile unsigned int)0x00004000)        /*!<Tx FIFO Half Empty interrupt Enable        */
#define  SDIO_MASK_RXFIFOHFIE                ((volatile unsigned int)0x00008000)        /*!<Rx FIFO Half Full interrupt Enable         */
#define  SDIO_MASK_TXFIFOFIE                 ((volatile unsigned int)0x00010000)        /*!<Tx FIFO Full interrupt Enable              */
#define  SDIO_MASK_RXFIFOFIE                 ((volatile unsigned int)0x00020000)        /*!<Rx FIFO Full interrupt Enable              */
#define  SDIO_MASK_TXFIFOEIE                 ((volatile unsigned int)0x00040000)        /*!<Tx FIFO Empty interrupt Enable             */
#define  SDIO_MASK_RXFIFOEIE                 ((volatile unsigned int)0x00080000)        /*!<Rx FIFO Empty interrupt Enable             */
#define  SDIO_MASK_TXDAVLIE                  ((volatile unsigned int)0x00100000)        /*!<Data available in Tx FIFO interrupt Enable */
#define  SDIO_MASK_RXDAVLIE                  ((volatile unsigned int)0x00200000)        /*!<Data available in Rx FIFO interrupt Enable */
#define  SDIO_MASK_SDIOITIE                  ((volatile unsigned int)0x00400000)        /*!<SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_MASK_CEATAENDIE                ((volatile unsigned int)0x00800000)        /*!<CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_FIFOCNT register  *****************/
#define  SDIO_FIFOCNT_FIFOCOUNT              ((volatile unsigned int)0x00FFFFFF)        /*!<Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_FIFO register  *******************/
#define  SDIO_FIFO_FIFODATA                  ((volatile unsigned int)0xFFFFFFFF)        /*!<Receive and transmit FIFO data */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define  SPI_CR1_CPHA                        ((volatile unsigned int)0x00000001)            /*!<Clock Phase      */
#define  SPI_CR1_CPOL                        ((volatile unsigned int)0x00000002)            /*!<Clock Polarity   */
#define  SPI_CR1_MSTR                        ((volatile unsigned int)0x00000004)            /*!<Master Selection */

#define  SPI_CR1_BR                          ((volatile unsigned int)0x00000038)            /*!<BR[2:0] bits (Baud Rate Control) */
#define  SPI_CR1_BR_0                        ((volatile unsigned int)0x00000008)            /*!<Bit 0 */
#define  SPI_CR1_BR_1                        ((volatile unsigned int)0x00000010)            /*!<Bit 1 */
#define  SPI_CR1_BR_2                        ((volatile unsigned int)0x00000020)            /*!<Bit 2 */

#define  SPI_CR1_SPE                         ((volatile unsigned int)0x00000040)            /*!<SPI Enable                          */
#define  SPI_CR1_LSBFIRST                    ((volatile unsigned int)0x00000080)            /*!<Frame Format                        */
#define  SPI_CR1_SSI                         ((volatile unsigned int)0x00000100)            /*!<Internal slave select               */
#define  SPI_CR1_SSM                         ((volatile unsigned int)0x00000200)            /*!<Software slave management           */
#define  SPI_CR1_RXONLY                      ((volatile unsigned int)0x00000400)            /*!<Receive only                        */
#define  SPI_CR1_DFF                         ((volatile unsigned int)0x00000800)            /*!<Data Frame Format                   */
#define  SPI_CR1_CRCNEXT                     ((volatile unsigned int)0x00001000)            /*!<Transmit CRC next                   */
#define  SPI_CR1_CRCEN                       ((volatile unsigned int)0x00002000)            /*!<Hardware CRC calculation enable     */
#define  SPI_CR1_BIDIOE                      ((volatile unsigned int)0x00004000)            /*!<Output enable in bidirectional mode */
#define  SPI_CR1_BIDIMODE                    ((volatile unsigned int)0x00008000)            /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define  SPI_CR2_RXDMAEN                     ((volatile unsigned int)0x00000001)               /*!<Rx Buffer DMA Enable                 */
#define  SPI_CR2_TXDMAEN                     ((volatile unsigned int)0x00000002)               /*!<Tx Buffer DMA Enable                 */
#define  SPI_CR2_SSOE                        ((volatile unsigned int)0x00000004)               /*!<SS Output Enable                     */
#define  SPI_CR2_FRF                         ((volatile unsigned int)0x00000010)               /*!<Frame Format                         */
#define  SPI_CR2_ERRIE                       ((volatile unsigned int)0x00000020)               /*!<Error Interrupt Enable               */
#define  SPI_CR2_RXNEIE                      ((volatile unsigned int)0x00000040)               /*!<RX buffer Not Empty Interrupt Enable */
#define  SPI_CR2_TXEIE                       ((volatile unsigned int)0x00000080)               /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define  SPI_SR_RXNE                         ((volatile unsigned int)0x00000001)               /*!<Receive buffer Not Empty */
#define  SPI_SR_TXE                          ((volatile unsigned int)0x00000002)               /*!<Transmit buffer Empty    */
#define  SPI_SR_CHSIDE                       ((volatile unsigned int)0x00000004)               /*!<Channel side             */
#define  SPI_SR_UDR                          ((volatile unsigned int)0x00000008)               /*!<Underrun flag            */
#define  SPI_SR_CRCERR                       ((volatile unsigned int)0x00000010)               /*!<CRC Error flag           */
#define  SPI_SR_MODF                         ((volatile unsigned int)0x00000020)               /*!<Mode fault               */
#define  SPI_SR_OVR                          ((volatile unsigned int)0x00000040)               /*!<Overrun flag             */
#define  SPI_SR_BSY                          ((volatile unsigned int)0x00000080)               /*!<Busy flag                */
#define  SPI_SR_FRE                          ((volatile unsigned int)0x00000100)               /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define  SPI_DR_DR                           ((volatile unsigned int)0x0000FFFF)            /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define  SPI_CRCPR_CRCPOLY                   ((volatile unsigned int)0x0000FFFF)            /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define  SPI_RXCRCR_RXCRC                    ((volatile unsigned int)0x0000FFFF)            /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define  SPI_TXCRCR_TXCRC                    ((volatile unsigned int)0x0000FFFF)            /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define  SPI_I2SCFGR_CHLEN                   ((volatile unsigned int)0x00000001)            /*!<Channel length (number of bits per audio channel) */

#define  SPI_I2SCFGR_DATLEN                  ((volatile unsigned int)0x00000006)            /*!<DATLEN[1:0] bits (Data length to be transferred)  */
#define  SPI_I2SCFGR_DATLEN_0                ((volatile unsigned int)0x00000002)            /*!<Bit 0 */
#define  SPI_I2SCFGR_DATLEN_1                ((volatile unsigned int)0x00000004)            /*!<Bit 1 */

#define  SPI_I2SCFGR_CKPOL                   ((volatile unsigned int)0x00000008)            /*!<steady state clock polarity               */

#define  SPI_I2SCFGR_I2SSTD                  ((volatile unsigned int)0x00000030)            /*!<I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCFGR_I2SSTD_0                ((volatile unsigned int)0x00000010)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SSTD_1                ((volatile unsigned int)0x00000020)            /*!<Bit 1 */

#define  SPI_I2SCFGR_PCMSYNC                 ((volatile unsigned int)0x00000080)            /*!<PCM frame synchronization                 */

#define  SPI_I2SCFGR_I2SCFG                  ((volatile unsigned int)0x00000300)            /*!<I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCFGR_I2SCFG_0                ((volatile unsigned int)0x00000100)            /*!<Bit 0 */
#define  SPI_I2SCFGR_I2SCFG_1                ((volatile unsigned int)0x00000200)            /*!<Bit 1 */

#define  SPI_I2SCFGR_I2SE                    ((volatile unsigned int)0x00000400)            /*!<I2S Enable         */
#define  SPI_I2SCFGR_I2SMOD                  ((volatile unsigned int)0x00000800)            /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define  SPI_I2SPR_I2SDIV                    ((volatile unsigned int)0x000000FF)            /*!<I2S Linear prescaler         */
#define  SPI_I2SPR_ODD                       ((volatile unsigned int)0x00000100)            /*!<Odd factor for the prescaler */
#define  SPI_I2SPR_MCKOE                     ((volatile unsigned int)0x00000200)            /*!<Master Clock Output Enable   */

/******************************************************************************/
/*                                                                            */
/*                                 SYSCFG                                     */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for SYSCFG_MEMRMP register  ***************/  
#define SYSCFG_MEMRMP_MEM_MODE          ((volatile unsigned int)0x00000007) /*!< SYSCFG_Memory Remap Config */
#define SYSCFG_MEMRMP_MEM_MODE_0        ((volatile unsigned int)0x00000001)
#define SYSCFG_MEMRMP_MEM_MODE_1        ((volatile unsigned int)0x00000002)
#define SYSCFG_MEMRMP_MEM_MODE_2        ((volatile unsigned int)0x00000004)

/******************  Bit definition for SYSCFG_PMC register  ******************/
#define SYSCFG_PMC_MII_RMII_SEL         ((volatile unsigned int)0x00800000) /*!<Ethernet PHY interface selection */
/* Old MII_RMII_SEL bit definition, maintained for legacy purpose */
#define SYSCFG_PMC_MII_RMII             SYSCFG_PMC_MII_RMII_SEL

/*****************  Bit definition for SYSCFG_EXTICR1 register  ***************/
#define SYSCFG_EXTICR1_EXTI0            ((volatile unsigned int)0x000F) /*!<EXTI 0 configuration */
#define SYSCFG_EXTICR1_EXTI1            ((volatile unsigned int)0x00F0) /*!<EXTI 1 configuration */
#define SYSCFG_EXTICR1_EXTI2            ((volatile unsigned int)0x0F00) /*!<EXTI 2 configuration */
#define SYSCFG_EXTICR1_EXTI3            ((volatile unsigned int)0xF000) /*!<EXTI 3 configuration */
/** 
 * @brief   EXTI0 configuration  
 */ 
#define SYSCFG_EXTICR1_EXTI0_PA         ((volatile unsigned int)0x0000) /*!<PA[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PB         ((volatile unsigned int)0x0001) /*!<PB[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PC         ((volatile unsigned int)0x0002) /*!<PC[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PD         ((volatile unsigned int)0x0003) /*!<PD[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PE         ((volatile unsigned int)0x0004) /*!<PE[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PF         ((volatile unsigned int)0x0005) /*!<PF[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PG         ((volatile unsigned int)0x0006) /*!<PG[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PH         ((volatile unsigned int)0x0007) /*!<PH[0] pin */
#define SYSCFG_EXTICR1_EXTI0_PI         ((volatile unsigned int)0x0008) /*!<PI[0] pin */

/** 
 * @brief   EXTI1 configuration  
 */ 
#define SYSCFG_EXTICR1_EXTI1_PA         ((volatile unsigned int)0x0000) /*!<PA[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PB         ((volatile unsigned int)0x0010) /*!<PB[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PC         ((volatile unsigned int)0x0020) /*!<PC[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PD         ((volatile unsigned int)0x0030) /*!<PD[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PE         ((volatile unsigned int)0x0040) /*!<PE[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PF         ((volatile unsigned int)0x0050) /*!<PF[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PG         ((volatile unsigned int)0x0060) /*!<PG[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PH         ((volatile unsigned int)0x0070) /*!<PH[1] pin */
#define SYSCFG_EXTICR1_EXTI1_PI         ((volatile unsigned int)0x0080) /*!<PI[1] pin */

/** 
 * @brief   EXTI2 configuration  
 */ 
#define SYSCFG_EXTICR1_EXTI2_PA         ((volatile unsigned int)0x0000) /*!<PA[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PB         ((volatile unsigned int)0x0100) /*!<PB[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PC         ((volatile unsigned int)0x0200) /*!<PC[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PD         ((volatile unsigned int)0x0300) /*!<PD[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PE         ((volatile unsigned int)0x0400) /*!<PE[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PF         ((volatile unsigned int)0x0500) /*!<PF[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PG         ((volatile unsigned int)0x0600) /*!<PG[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PH         ((volatile unsigned int)0x0700) /*!<PH[2] pin */
#define SYSCFG_EXTICR1_EXTI2_PI         ((volatile unsigned int)0x0800) /*!<PI[2] pin */

/** 
 * @brief   EXTI3 configuration  
 */ 
#define SYSCFG_EXTICR1_EXTI3_PA         ((volatile unsigned int)0x0000) /*!<PA[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PB         ((volatile unsigned int)0x1000) /*!<PB[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PC         ((volatile unsigned int)0x2000) /*!<PC[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PD         ((volatile unsigned int)0x3000) /*!<PD[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PE         ((volatile unsigned int)0x4000) /*!<PE[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PF         ((volatile unsigned int)0x5000) /*!<PF[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PG         ((volatile unsigned int)0x6000) /*!<PG[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PH         ((volatile unsigned int)0x7000) /*!<PH[3] pin */
#define SYSCFG_EXTICR1_EXTI3_PI         ((volatile unsigned int)0x8000) /*!<PI[3] pin */

/*****************  Bit definition for SYSCFG_EXTICR2 register  ***************/
#define SYSCFG_EXTICR2_EXTI4            ((volatile unsigned int)0x000F) /*!<EXTI 4 configuration */
#define SYSCFG_EXTICR2_EXTI5            ((volatile unsigned int)0x00F0) /*!<EXTI 5 configuration */
#define SYSCFG_EXTICR2_EXTI6            ((volatile unsigned int)0x0F00) /*!<EXTI 6 configuration */
#define SYSCFG_EXTICR2_EXTI7            ((volatile unsigned int)0xF000) /*!<EXTI 7 configuration */
/** 
 * @brief   EXTI4 configuration  
 */ 
#define SYSCFG_EXTICR2_EXTI4_PA         ((volatile unsigned int)0x0000) /*!<PA[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PB         ((volatile unsigned int)0x0001) /*!<PB[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PC         ((volatile unsigned int)0x0002) /*!<PC[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PD         ((volatile unsigned int)0x0003) /*!<PD[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PE         ((volatile unsigned int)0x0004) /*!<PE[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PF         ((volatile unsigned int)0x0005) /*!<PF[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PG         ((volatile unsigned int)0x0006) /*!<PG[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PH         ((volatile unsigned int)0x0007) /*!<PH[4] pin */
#define SYSCFG_EXTICR2_EXTI4_PI         ((volatile unsigned int)0x0008) /*!<PI[4] pin */

/** 
 * @brief   EXTI5 configuration  
 */ 
#define SYSCFG_EXTICR2_EXTI5_PA         ((volatile unsigned int)0x0000) /*!<PA[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PB         ((volatile unsigned int)0x0010) /*!<PB[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PC         ((volatile unsigned int)0x0020) /*!<PC[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PD         ((volatile unsigned int)0x0030) /*!<PD[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PE         ((volatile unsigned int)0x0040) /*!<PE[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PF         ((volatile unsigned int)0x0050) /*!<PF[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PG         ((volatile unsigned int)0x0060) /*!<PG[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PH         ((volatile unsigned int)0x0070) /*!<PH[5] pin */
#define SYSCFG_EXTICR2_EXTI5_PI         ((volatile unsigned int)0x0080) /*!<PI[5] pin */

/** 
 * @brief   EXTI6 configuration  
 */ 
#define SYSCFG_EXTICR2_EXTI6_PA         ((volatile unsigned int)0x0000) /*!<PA[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PB         ((volatile unsigned int)0x0100) /*!<PB[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PC         ((volatile unsigned int)0x0200) /*!<PC[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PD         ((volatile unsigned int)0x0300) /*!<PD[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PE         ((volatile unsigned int)0x0400) /*!<PE[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PF         ((volatile unsigned int)0x0500) /*!<PF[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PG         ((volatile unsigned int)0x0600) /*!<PG[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PH         ((volatile unsigned int)0x0700) /*!<PH[6] pin */
#define SYSCFG_EXTICR2_EXTI6_PI         ((volatile unsigned int)0x0800) /*!<PI[6] pin */

/** 
 * @brief   EXTI7 configuration  
 */ 
#define SYSCFG_EXTICR2_EXTI7_PA         ((volatile unsigned int)0x0000) /*!<PA[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PB         ((volatile unsigned int)0x1000) /*!<PB[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PC         ((volatile unsigned int)0x2000) /*!<PC[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PD         ((volatile unsigned int)0x3000) /*!<PD[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PE         ((volatile unsigned int)0x4000) /*!<PE[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PF         ((volatile unsigned int)0x5000) /*!<PF[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PG         ((volatile unsigned int)0x6000) /*!<PG[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PH         ((volatile unsigned int)0x7000) /*!<PH[7] pin */
#define SYSCFG_EXTICR2_EXTI7_PI         ((volatile unsigned int)0x8000) /*!<PI[7] pin */


/*****************  Bit definition for SYSCFG_EXTICR3 register  ***************/
#define SYSCFG_EXTICR3_EXTI8            ((volatile unsigned int)0x000F) /*!<EXTI 8 configuration */
#define SYSCFG_EXTICR3_EXTI9            ((volatile unsigned int)0x00F0) /*!<EXTI 9 configuration */
#define SYSCFG_EXTICR3_EXTI10           ((volatile unsigned int)0x0F00) /*!<EXTI 10 configuration */
#define SYSCFG_EXTICR3_EXTI11           ((volatile unsigned int)0xF000) /*!<EXTI 11 configuration */

/** 
 * @brief   EXTI8 configuration  
 */ 
#define SYSCFG_EXTICR3_EXTI8_PA         ((volatile unsigned int)0x0000) /*!<PA[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PB         ((volatile unsigned int)0x0001) /*!<PB[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PC         ((volatile unsigned int)0x0002) /*!<PC[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PD         ((volatile unsigned int)0x0003) /*!<PD[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PE         ((volatile unsigned int)0x0004) /*!<PE[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PF         ((volatile unsigned int)0x0005) /*!<PF[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PG         ((volatile unsigned int)0x0006) /*!<PG[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PH         ((volatile unsigned int)0x0007) /*!<PH[8] pin */
#define SYSCFG_EXTICR3_EXTI8_PI         ((volatile unsigned int)0x0008) /*!<PI[8] pin */

/** 
 * @brief   EXTI9 configuration  
 */ 
#define SYSCFG_EXTICR3_EXTI9_PA         ((volatile unsigned int)0x0000) /*!<PA[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PB         ((volatile unsigned int)0x0010) /*!<PB[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PC         ((volatile unsigned int)0x0020) /*!<PC[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PD         ((volatile unsigned int)0x0030) /*!<PD[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PE         ((volatile unsigned int)0x0040) /*!<PE[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PF         ((volatile unsigned int)0x0050) /*!<PF[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PG         ((volatile unsigned int)0x0060) /*!<PG[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PH         ((volatile unsigned int)0x0070) /*!<PH[9] pin */
#define SYSCFG_EXTICR3_EXTI9_PI         ((volatile unsigned int)0x0080) /*!<PI[9] pin */

/** 
 * @brief   EXTI10 configuration  
 */ 
#define SYSCFG_EXTICR3_EXTI10_PA        ((volatile unsigned int)0x0000) /*!<PA[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PB        ((volatile unsigned int)0x0100) /*!<PB[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PC        ((volatile unsigned int)0x0200) /*!<PC[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PD        ((volatile unsigned int)0x0300) /*!<PD[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PE        ((volatile unsigned int)0x0400) /*!<PE[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PF        ((volatile unsigned int)0x0500) /*!<PF[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PG        ((volatile unsigned int)0x0600) /*!<PG[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PH        ((volatile unsigned int)0x0700) /*!<PH[10] pin */
#define SYSCFG_EXTICR3_EXTI10_PI        ((volatile unsigned int)0x0800) /*!<PI[10] pin */

/** 
 * @brief   EXTI11 configuration  
 */ 
#define SYSCFG_EXTICR3_EXTI11_PA        ((volatile unsigned int)0x0000) /*!<PA[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PB        ((volatile unsigned int)0x1000) /*!<PB[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PC        ((volatile unsigned int)0x2000) /*!<PC[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PD        ((volatile unsigned int)0x3000) /*!<PD[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PE        ((volatile unsigned int)0x4000) /*!<PE[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PF        ((volatile unsigned int)0x5000) /*!<PF[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PG        ((volatile unsigned int)0x6000) /*!<PG[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PH        ((volatile unsigned int)0x7000) /*!<PH[11] pin */
#define SYSCFG_EXTICR3_EXTI11_PI        ((volatile unsigned int)0x8000) /*!<PI[11] pin */

/*****************  Bit definition for SYSCFG_EXTICR4 register  ***************/
#define SYSCFG_EXTICR4_EXTI12           ((volatile unsigned int)0x000F) /*!<EXTI 12 configuration */
#define SYSCFG_EXTICR4_EXTI13           ((volatile unsigned int)0x00F0) /*!<EXTI 13 configuration */
#define SYSCFG_EXTICR4_EXTI14           ((volatile unsigned int)0x0F00) /*!<EXTI 14 configuration */
#define SYSCFG_EXTICR4_EXTI15           ((volatile unsigned int)0xF000) /*!<EXTI 15 configuration */
/** 
 * @brief   EXTI12 configuration  
 */ 
#define SYSCFG_EXTICR4_EXTI12_PA        ((volatile unsigned int)0x0000) /*!<PA[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PB        ((volatile unsigned int)0x0001) /*!<PB[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PC        ((volatile unsigned int)0x0002) /*!<PC[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PD        ((volatile unsigned int)0x0003) /*!<PD[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PE        ((volatile unsigned int)0x0004) /*!<PE[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PF        ((volatile unsigned int)0x0005) /*!<PF[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PG        ((volatile unsigned int)0x0006) /*!<PG[12] pin */
#define SYSCFG_EXTICR4_EXTI12_PH        ((volatile unsigned int)0x0007) /*!<PH[12] pin */

/** 
 * @brief   EXTI13 configuration  
 */ 
#define SYSCFG_EXTICR4_EXTI13_PA        ((volatile unsigned int)0x0000) /*!<PA[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PB        ((volatile unsigned int)0x0010) /*!<PB[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PC        ((volatile unsigned int)0x0020) /*!<PC[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PD        ((volatile unsigned int)0x0030) /*!<PD[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PE        ((volatile unsigned int)0x0040) /*!<PE[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PF        ((volatile unsigned int)0x0050) /*!<PF[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PG        ((volatile unsigned int)0x0060) /*!<PG[13] pin */
#define SYSCFG_EXTICR4_EXTI13_PH        ((volatile unsigned int)0x0070) /*!<PH[13] pin */

/** 
 * @brief   EXTI14 configuration  
 */ 
#define SYSCFG_EXTICR4_EXTI14_PA        ((volatile unsigned int)0x0000) /*!<PA[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PB        ((volatile unsigned int)0x0100) /*!<PB[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PC        ((volatile unsigned int)0x0200) /*!<PC[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PD        ((volatile unsigned int)0x0300) /*!<PD[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PE        ((volatile unsigned int)0x0400) /*!<PE[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PF        ((volatile unsigned int)0x0500) /*!<PF[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PG        ((volatile unsigned int)0x0600) /*!<PG[14] pin */
#define SYSCFG_EXTICR4_EXTI14_PH        ((volatile unsigned int)0x0700) /*!<PH[14] pin */

/** 
 * @brief   EXTI15 configuration  
 */ 
#define SYSCFG_EXTICR4_EXTI15_PA        ((volatile unsigned int)0x0000) /*!<PA[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PB        ((volatile unsigned int)0x1000) /*!<PB[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PC        ((volatile unsigned int)0x2000) /*!<PC[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PD        ((volatile unsigned int)0x3000) /*!<PD[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PE        ((volatile unsigned int)0x4000) /*!<PE[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PF        ((volatile unsigned int)0x5000) /*!<PF[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PG        ((volatile unsigned int)0x6000) /*!<PG[15] pin */
#define SYSCFG_EXTICR4_EXTI15_PH        ((volatile unsigned int)0x7000) /*!<PH[15] pin */

/******************  Bit definition for SYSCFG_CMPCR register  ****************/  
#define SYSCFG_CMPCR_CMP_PD             ((volatile unsigned int)0x00000001) /*!<Compensation cell ready flag */
#define SYSCFG_CMPCR_READY              ((volatile unsigned int)0x00000100) /*!<Compensation cell power-down */

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define  TIM_CR1_CEN                         ((volatile unsigned int)0x0001)            /*!<Counter enable        */
#define  TIM_CR1_UDIS                        ((volatile unsigned int)0x0002)            /*!<Update disable        */
#define  TIM_CR1_URS                         ((volatile unsigned int)0x0004)            /*!<Update request source */
#define  TIM_CR1_OPM                         ((volatile unsigned int)0x0008)            /*!<One pulse mode        */
#define  TIM_CR1_DIR                         ((volatile unsigned int)0x0010)            /*!<Direction             */

#define  TIM_CR1_CMS                         ((volatile unsigned int)0x0060)            /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define  TIM_CR1_CMS_0                       ((volatile unsigned int)0x0020)            /*!<Bit 0 */
#define  TIM_CR1_CMS_1                       ((volatile unsigned int)0x0040)            /*!<Bit 1 */

#define  TIM_CR1_ARPE                        ((volatile unsigned int)0x0080)            /*!<Auto-reload preload enable     */

#define  TIM_CR1_CKD                         ((volatile unsigned int)0x0300)            /*!<CKD[1:0] bits (clock division) */
#define  TIM_CR1_CKD_0                       ((volatile unsigned int)0x0100)            /*!<Bit 0 */
#define  TIM_CR1_CKD_1                       ((volatile unsigned int)0x0200)            /*!<Bit 1 */

/*******************  Bit definition for TIM_CR2 register  ********************/
#define  TIM_CR2_CCPC                        ((volatile unsigned int)0x0001)            /*!<Capture/Compare Preloaded Control        */
#define  TIM_CR2_CCUS                        ((volatile unsigned int)0x0004)            /*!<Capture/Compare Control Update Selection */
#define  TIM_CR2_CCDS                        ((volatile unsigned int)0x0008)            /*!<Capture/Compare DMA Selection            */

#define  TIM_CR2_MMS                         ((volatile unsigned int)0x0070)            /*!<MMS[2:0] bits (Master Mode Selection) */
#define  TIM_CR2_MMS_0                       ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  TIM_CR2_MMS_1                       ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  TIM_CR2_MMS_2                       ((volatile unsigned int)0x0040)            /*!<Bit 2 */

#define  TIM_CR2_TI1S                        ((volatile unsigned int)0x0080)            /*!<TI1 Selection */
#define  TIM_CR2_OIS1                        ((volatile unsigned int)0x0100)            /*!<Output Idle state 1 (OC1 output)  */
#define  TIM_CR2_OIS1N                       ((volatile unsigned int)0x0200)            /*!<Output Idle state 1 (OC1N output) */
#define  TIM_CR2_OIS2                        ((volatile unsigned int)0x0400)            /*!<Output Idle state 2 (OC2 output)  */
#define  TIM_CR2_OIS2N                       ((volatile unsigned int)0x0800)            /*!<Output Idle state 2 (OC2N output) */
#define  TIM_CR2_OIS3                        ((volatile unsigned int)0x1000)            /*!<Output Idle state 3 (OC3 output)  */
#define  TIM_CR2_OIS3N                       ((volatile unsigned int)0x2000)            /*!<Output Idle state 3 (OC3N output) */
#define  TIM_CR2_OIS4                        ((volatile unsigned int)0x4000)            /*!<Output Idle state 4 (OC4 output)  */

/*******************  Bit definition for TIM_SMCR register  *******************/
#define  TIM_SMCR_SMS                        ((volatile unsigned int)0x0007)            /*!<SMS[2:0] bits (Slave mode selection)    */
#define  TIM_SMCR_SMS_0                      ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  TIM_SMCR_SMS_1                      ((volatile unsigned int)0x0002)            /*!<Bit 1 */
#define  TIM_SMCR_SMS_2                      ((volatile unsigned int)0x0004)            /*!<Bit 2 */

#define  TIM_SMCR_TS                         ((volatile unsigned int)0x0070)            /*!<TS[2:0] bits (Trigger selection)        */
#define  TIM_SMCR_TS_0                       ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  TIM_SMCR_TS_1                       ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  TIM_SMCR_TS_2                       ((volatile unsigned int)0x0040)            /*!<Bit 2 */

#define  TIM_SMCR_MSM                        ((volatile unsigned int)0x0080)            /*!<Master/slave mode                       */

#define  TIM_SMCR_ETF                        ((volatile unsigned int)0x0F00)            /*!<ETF[3:0] bits (External trigger filter) */
#define  TIM_SMCR_ETF_0                      ((volatile unsigned int)0x0100)            /*!<Bit 0 */
#define  TIM_SMCR_ETF_1                      ((volatile unsigned int)0x0200)            /*!<Bit 1 */
#define  TIM_SMCR_ETF_2                      ((volatile unsigned int)0x0400)            /*!<Bit 2 */
#define  TIM_SMCR_ETF_3                      ((volatile unsigned int)0x0800)            /*!<Bit 3 */

#define  TIM_SMCR_ETPS                       ((volatile unsigned int)0x3000)            /*!<ETPS[1:0] bits (External trigger prescaler) */
#define  TIM_SMCR_ETPS_0                     ((volatile unsigned int)0x1000)            /*!<Bit 0 */
#define  TIM_SMCR_ETPS_1                     ((volatile unsigned int)0x2000)            /*!<Bit 1 */

#define  TIM_SMCR_ECE                        ((volatile unsigned int)0x4000)            /*!<External clock enable     */
#define  TIM_SMCR_ETP                        ((volatile unsigned int)0x8000)            /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  *******************/
#define  TIM_DIER_UIE                        ((volatile unsigned int)0x0001)            /*!<Update interrupt enable */
#define  TIM_DIER_CC1IE                      ((volatile unsigned int)0x0002)            /*!<Capture/Compare 1 interrupt enable   */
#define  TIM_DIER_CC2IE                      ((volatile unsigned int)0x0004)            /*!<Capture/Compare 2 interrupt enable   */
#define  TIM_DIER_CC3IE                      ((volatile unsigned int)0x0008)            /*!<Capture/Compare 3 interrupt enable   */
#define  TIM_DIER_CC4IE                      ((volatile unsigned int)0x0010)            /*!<Capture/Compare 4 interrupt enable   */
#define  TIM_DIER_COMIE                      ((volatile unsigned int)0x0020)            /*!<COM interrupt enable                 */
#define  TIM_DIER_TIE                        ((volatile unsigned int)0x0040)            /*!<Trigger interrupt enable             */
#define  TIM_DIER_BIE                        ((volatile unsigned int)0x0080)            /*!<Break interrupt enable               */
#define  TIM_DIER_UDE                        ((volatile unsigned int)0x0100)            /*!<Update DMA request enable            */
#define  TIM_DIER_CC1DE                      ((volatile unsigned int)0x0200)            /*!<Capture/Compare 1 DMA request enable */
#define  TIM_DIER_CC2DE                      ((volatile unsigned int)0x0400)            /*!<Capture/Compare 2 DMA request enable */
#define  TIM_DIER_CC3DE                      ((volatile unsigned int)0x0800)            /*!<Capture/Compare 3 DMA request enable */
#define  TIM_DIER_CC4DE                      ((volatile unsigned int)0x1000)            /*!<Capture/Compare 4 DMA request enable */
#define  TIM_DIER_COMDE                      ((volatile unsigned int)0x2000)            /*!<COM DMA request enable               */
#define  TIM_DIER_TDE                        ((volatile unsigned int)0x4000)            /*!<Trigger DMA request enable           */

/********************  Bit definition for TIM_SR register  ********************/
#define  TIM_SR_UIF                          ((volatile unsigned int)0x0001)            /*!<Update interrupt Flag              */
#define  TIM_SR_CC1IF                        ((volatile unsigned int)0x0002)            /*!<Capture/Compare 1 interrupt Flag   */
#define  TIM_SR_CC2IF                        ((volatile unsigned int)0x0004)            /*!<Capture/Compare 2 interrupt Flag   */
#define  TIM_SR_CC3IF                        ((volatile unsigned int)0x0008)            /*!<Capture/Compare 3 interrupt Flag   */
#define  TIM_SR_CC4IF                        ((volatile unsigned int)0x0010)            /*!<Capture/Compare 4 interrupt Flag   */
#define  TIM_SR_COMIF                        ((volatile unsigned int)0x0020)            /*!<COM interrupt Flag                 */
#define  TIM_SR_TIF                          ((volatile unsigned int)0x0040)            /*!<Trigger interrupt Flag             */
#define  TIM_SR_BIF                          ((volatile unsigned int)0x0080)            /*!<Break interrupt Flag               */
#define  TIM_SR_CC1OF                        ((volatile unsigned int)0x0200)            /*!<Capture/Compare 1 Overcapture Flag */
#define  TIM_SR_CC2OF                        ((volatile unsigned int)0x0400)            /*!<Capture/Compare 2 Overcapture Flag */
#define  TIM_SR_CC3OF                        ((volatile unsigned int)0x0800)            /*!<Capture/Compare 3 Overcapture Flag */
#define  TIM_SR_CC4OF                        ((volatile unsigned int)0x1000)            /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  ********************/
#define  TIM_EGR_UG                          ((volatile unsigned int)0x01)               /*!<Update Generation                         */
#define  TIM_EGR_CC1G                        ((volatile unsigned int)0x02)               /*!<Capture/Compare 1 Generation              */
#define  TIM_EGR_CC2G                        ((volatile unsigned int)0x04)               /*!<Capture/Compare 2 Generation              */
#define  TIM_EGR_CC3G                        ((volatile unsigned int)0x08)               /*!<Capture/Compare 3 Generation              */
#define  TIM_EGR_CC4G                        ((volatile unsigned int)0x10)               /*!<Capture/Compare 4 Generation              */
#define  TIM_EGR_COMG                        ((volatile unsigned int)0x20)               /*!<Capture/Compare Control Update Generation */
#define  TIM_EGR_TG                          ((volatile unsigned int)0x40)               /*!<Trigger Generation                        */
#define  TIM_EGR_BG                          ((volatile unsigned int)0x80)               /*!<Break Generation                          */

/******************  Bit definition for TIM_CCMR1 register  *******************/
#define  TIM_CCMR1_CC1S                      ((volatile unsigned int)0x0003)            /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TIM_CCMR1_CC1S_0                    ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  TIM_CCMR1_CC1S_1                    ((volatile unsigned int)0x0002)            /*!<Bit 1 */

#define  TIM_CCMR1_OC1FE                     ((volatile unsigned int)0x0004)            /*!<Output Compare 1 Fast enable                 */
#define  TIM_CCMR1_OC1PE                     ((volatile unsigned int)0x0008)            /*!<Output Compare 1 Preload enable              */

#define  TIM_CCMR1_OC1M                      ((volatile unsigned int)0x0070)            /*!<OC1M[2:0] bits (Output Compare 1 Mode)       */
#define  TIM_CCMR1_OC1M_0                    ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR1_OC1M_1                    ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR1_OC1M_2                    ((volatile unsigned int)0x0040)            /*!<Bit 2 */

#define  TIM_CCMR1_OC1CE                     ((volatile unsigned int)0x0080)            /*!<Output Compare 1Clear Enable                 */

#define  TIM_CCMR1_CC2S                      ((volatile unsigned int)0x0300)            /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TIM_CCMR1_CC2S_0                    ((volatile unsigned int)0x0100)            /*!<Bit 0 */
#define  TIM_CCMR1_CC2S_1                    ((volatile unsigned int)0x0200)            /*!<Bit 1 */

#define  TIM_CCMR1_OC2FE                     ((volatile unsigned int)0x0400)            /*!<Output Compare 2 Fast enable                 */
#define  TIM_CCMR1_OC2PE                     ((volatile unsigned int)0x0800)            /*!<Output Compare 2 Preload enable              */

#define  TIM_CCMR1_OC2M                      ((volatile unsigned int)0x7000)            /*!<OC2M[2:0] bits (Output Compare 2 Mode)       */
#define  TIM_CCMR1_OC2M_0                    ((volatile unsigned int)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR1_OC2M_1                    ((volatile unsigned int)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR1_OC2M_2                    ((volatile unsigned int)0x4000)            /*!<Bit 2 */

#define  TIM_CCMR1_OC2CE                     ((volatile unsigned int)0x8000)            /*!<Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR1_IC1PSC                    ((volatile unsigned int)0x000C)            /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TIM_CCMR1_IC1PSC_0                  ((volatile unsigned int)0x0004)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1PSC_1                  ((volatile unsigned int)0x0008)            /*!<Bit 1 */

#define  TIM_CCMR1_IC1F                      ((volatile unsigned int)0x00F0)            /*!<IC1F[3:0] bits (Input Capture 1 Filter)      */
#define  TIM_CCMR1_IC1F_0                    ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR1_IC1F_1                    ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR1_IC1F_2                    ((volatile unsigned int)0x0040)            /*!<Bit 2 */
#define  TIM_CCMR1_IC1F_3                    ((volatile unsigned int)0x0080)            /*!<Bit 3 */

#define  TIM_CCMR1_IC2PSC                    ((volatile unsigned int)0x0C00)            /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler)  */
#define  TIM_CCMR1_IC2PSC_0                  ((volatile unsigned int)0x0400)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2PSC_1                  ((volatile unsigned int)0x0800)            /*!<Bit 1 */

#define  TIM_CCMR1_IC2F                      ((volatile unsigned int)0xF000)            /*!<IC2F[3:0] bits (Input Capture 2 Filter)       */
#define  TIM_CCMR1_IC2F_0                    ((volatile unsigned int)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR1_IC2F_1                    ((volatile unsigned int)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR1_IC2F_2                    ((volatile unsigned int)0x4000)            /*!<Bit 2 */
#define  TIM_CCMR1_IC2F_3                    ((volatile unsigned int)0x8000)            /*!<Bit 3 */

/******************  Bit definition for TIM_CCMR2 register  *******************/
#define  TIM_CCMR2_CC3S                      ((volatile unsigned int)0x0003)            /*!<CC3S[1:0] bits (Capture/Compare 3 Selection)  */
#define  TIM_CCMR2_CC3S_0                    ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  TIM_CCMR2_CC3S_1                    ((volatile unsigned int)0x0002)            /*!<Bit 1 */

#define  TIM_CCMR2_OC3FE                     ((volatile unsigned int)0x0004)            /*!<Output Compare 3 Fast enable           */
#define  TIM_CCMR2_OC3PE                     ((volatile unsigned int)0x0008)            /*!<Output Compare 3 Preload enable        */

#define  TIM_CCMR2_OC3M                      ((volatile unsigned int)0x0070)            /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TIM_CCMR2_OC3M_0                    ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR2_OC3M_1                    ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR2_OC3M_2                    ((volatile unsigned int)0x0040)            /*!<Bit 2 */

#define  TIM_CCMR2_OC3CE                     ((volatile unsigned int)0x0080)            /*!<Output Compare 3 Clear Enable */

#define  TIM_CCMR2_CC4S                      ((volatile unsigned int)0x0300)            /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TIM_CCMR2_CC4S_0                    ((volatile unsigned int)0x0100)            /*!<Bit 0 */
#define  TIM_CCMR2_CC4S_1                    ((volatile unsigned int)0x0200)            /*!<Bit 1 */

#define  TIM_CCMR2_OC4FE                     ((volatile unsigned int)0x0400)            /*!<Output Compare 4 Fast enable    */
#define  TIM_CCMR2_OC4PE                     ((volatile unsigned int)0x0800)            /*!<Output Compare 4 Preload enable */

#define  TIM_CCMR2_OC4M                      ((volatile unsigned int)0x7000)            /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TIM_CCMR2_OC4M_0                    ((volatile unsigned int)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR2_OC4M_1                    ((volatile unsigned int)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR2_OC4M_2                    ((volatile unsigned int)0x4000)            /*!<Bit 2 */

#define  TIM_CCMR2_OC4CE                     ((volatile unsigned int)0x8000)            /*!<Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TIM_CCMR2_IC3PSC                    ((volatile unsigned int)0x000C)            /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TIM_CCMR2_IC3PSC_0                  ((volatile unsigned int)0x0004)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3PSC_1                  ((volatile unsigned int)0x0008)            /*!<Bit 1 */

#define  TIM_CCMR2_IC3F                      ((volatile unsigned int)0x00F0)            /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TIM_CCMR2_IC3F_0                    ((volatile unsigned int)0x0010)            /*!<Bit 0 */
#define  TIM_CCMR2_IC3F_1                    ((volatile unsigned int)0x0020)            /*!<Bit 1 */
#define  TIM_CCMR2_IC3F_2                    ((volatile unsigned int)0x0040)            /*!<Bit 2 */
#define  TIM_CCMR2_IC3F_3                    ((volatile unsigned int)0x0080)            /*!<Bit 3 */

#define  TIM_CCMR2_IC4PSC                    ((volatile unsigned int)0x0C00)            /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TIM_CCMR2_IC4PSC_0                  ((volatile unsigned int)0x0400)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4PSC_1                  ((volatile unsigned int)0x0800)            /*!<Bit 1 */

#define  TIM_CCMR2_IC4F                      ((volatile unsigned int)0xF000)            /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TIM_CCMR2_IC4F_0                    ((volatile unsigned int)0x1000)            /*!<Bit 0 */
#define  TIM_CCMR2_IC4F_1                    ((volatile unsigned int)0x2000)            /*!<Bit 1 */
#define  TIM_CCMR2_IC4F_2                    ((volatile unsigned int)0x4000)            /*!<Bit 2 */
#define  TIM_CCMR2_IC4F_3                    ((volatile unsigned int)0x8000)            /*!<Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define  TIM_CCER_CC1E                       ((volatile unsigned int)0x0001)            /*!<Capture/Compare 1 output enable                 */
#define  TIM_CCER_CC1P                       ((volatile unsigned int)0x0002)            /*!<Capture/Compare 1 output Polarity               */
#define  TIM_CCER_CC1NE                      ((volatile unsigned int)0x0004)            /*!<Capture/Compare 1 Complementary output enable   */
#define  TIM_CCER_CC1NP                      ((volatile unsigned int)0x0008)            /*!<Capture/Compare 1 Complementary output Polarity */
#define  TIM_CCER_CC2E                       ((volatile unsigned int)0x0010)            /*!<Capture/Compare 2 output enable                 */
#define  TIM_CCER_CC2P                       ((volatile unsigned int)0x0020)            /*!<Capture/Compare 2 output Polarity               */
#define  TIM_CCER_CC2NE                      ((volatile unsigned int)0x0040)            /*!<Capture/Compare 2 Complementary output enable   */
#define  TIM_CCER_CC2NP                      ((volatile unsigned int)0x0080)            /*!<Capture/Compare 2 Complementary output Polarity */
#define  TIM_CCER_CC3E                       ((volatile unsigned int)0x0100)            /*!<Capture/Compare 3 output enable                 */
#define  TIM_CCER_CC3P                       ((volatile unsigned int)0x0200)            /*!<Capture/Compare 3 output Polarity               */
#define  TIM_CCER_CC3NE                      ((volatile unsigned int)0x0400)            /*!<Capture/Compare 3 Complementary output enable   */
#define  TIM_CCER_CC3NP                      ((volatile unsigned int)0x0800)            /*!<Capture/Compare 3 Complementary output Polarity */
#define  TIM_CCER_CC4E                       ((volatile unsigned int)0x1000)            /*!<Capture/Compare 4 output enable                 */
#define  TIM_CCER_CC4P                       ((volatile unsigned int)0x2000)            /*!<Capture/Compare 4 output Polarity               */
#define  TIM_CCER_CC4NP                      ((volatile unsigned int)0x8000)            /*!<Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define  TIM_CNT_CNT                         ((volatile unsigned int)0xFFFF)            /*!<Counter Value            */

/*******************  Bit definition for TIM_PSC register  ********************/
#define  TIM_PSC_PSC                         ((volatile unsigned int)0xFFFF)            /*!<Prescaler Value          */

/*******************  Bit definition for TIM_ARR register  ********************/
#define  TIM_ARR_ARR                         ((volatile unsigned int)0xFFFF)            /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  ********************/
#define  TIM_RCR_REP                         ((volatile unsigned int)0xFF)               /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  *******************/
#define  TIM_CCR1_CCR1                       ((volatile unsigned int)0xFFFF)            /*!<Capture/Compare 1 Value  */

/*******************  Bit definition for TIM_CCR2 register  *******************/
#define  TIM_CCR2_CCR2                       ((volatile unsigned int)0xFFFF)            /*!<Capture/Compare 2 Value  */

/*******************  Bit definition for TIM_CCR3 register  *******************/
#define  TIM_CCR3_CCR3                       ((volatile unsigned int)0xFFFF)            /*!<Capture/Compare 3 Value  */

/*******************  Bit definition for TIM_CCR4 register  *******************/
#define  TIM_CCR4_CCR4                       ((volatile unsigned int)0xFFFF)            /*!<Capture/Compare 4 Value  */

/*******************  Bit definition for TIM_BDTR register  *******************/
#define  TIM_BDTR_DTG                        ((volatile unsigned int)0x00FF)            /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TIM_BDTR_DTG_0                      ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  TIM_BDTR_DTG_1                      ((volatile unsigned int)0x0002)            /*!<Bit 1 */
#define  TIM_BDTR_DTG_2                      ((volatile unsigned int)0x0004)            /*!<Bit 2 */
#define  TIM_BDTR_DTG_3                      ((volatile unsigned int)0x0008)            /*!<Bit 3 */
#define  TIM_BDTR_DTG_4                      ((volatile unsigned int)0x0010)            /*!<Bit 4 */
#define  TIM_BDTR_DTG_5                      ((volatile unsigned int)0x0020)            /*!<Bit 5 */
#define  TIM_BDTR_DTG_6                      ((volatile unsigned int)0x0040)            /*!<Bit 6 */
#define  TIM_BDTR_DTG_7                      ((volatile unsigned int)0x0080)            /*!<Bit 7 */

#define  TIM_BDTR_LOCK                       ((volatile unsigned int)0x0300)            /*!<LOCK[1:0] bits (Lock Configuration) */
#define  TIM_BDTR_LOCK_0                     ((volatile unsigned int)0x0100)            /*!<Bit 0 */
#define  TIM_BDTR_LOCK_1                     ((volatile unsigned int)0x0200)            /*!<Bit 1 */

#define  TIM_BDTR_OSSI                       ((volatile unsigned int)0x0400)            /*!<Off-State Selection for Idle mode */
#define  TIM_BDTR_OSSR                       ((volatile unsigned int)0x0800)            /*!<Off-State Selection for Run mode  */
#define  TIM_BDTR_BKE                        ((volatile unsigned int)0x1000)            /*!<Break enable                      */
#define  TIM_BDTR_BKP                        ((volatile unsigned int)0x2000)            /*!<Break Polarity                    */
#define  TIM_BDTR_AOE                        ((volatile unsigned int)0x4000)            /*!<Automatic Output enable           */
#define  TIM_BDTR_MOE                        ((volatile unsigned int)0x8000)            /*!<Main Output enable                */

/*******************  Bit definition for TIM_DCR register  ********************/
#define  TIM_DCR_DBA                         ((volatile unsigned int)0x001F)            /*!<DBA[4:0] bits (DMA Base Address) */
#define  TIM_DCR_DBA_0                       ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  TIM_DCR_DBA_1                       ((volatile unsigned int)0x0002)            /*!<Bit 1 */
#define  TIM_DCR_DBA_2                       ((volatile unsigned int)0x0004)            /*!<Bit 2 */
#define  TIM_DCR_DBA_3                       ((volatile unsigned int)0x0008)            /*!<Bit 3 */
#define  TIM_DCR_DBA_4                       ((volatile unsigned int)0x0010)            /*!<Bit 4 */

#define  TIM_DCR_DBL                         ((volatile unsigned int)0x1F00)            /*!<DBL[4:0] bits (DMA Burst Length) */
#define  TIM_DCR_DBL_0                       ((volatile unsigned int)0x0100)            /*!<Bit 0 */
#define  TIM_DCR_DBL_1                       ((volatile unsigned int)0x0200)            /*!<Bit 1 */
#define  TIM_DCR_DBL_2                       ((volatile unsigned int)0x0400)            /*!<Bit 2 */
#define  TIM_DCR_DBL_3                       ((volatile unsigned int)0x0800)            /*!<Bit 3 */
#define  TIM_DCR_DBL_4                       ((volatile unsigned int)0x1000)            /*!<Bit 4 */

/*******************  Bit definition for TIM_DMAR register  *******************/
#define  TIM_DMAR_DMAB                       ((volatile unsigned int)0xFFFF)            /*!<DMA register for burst accesses                    */

/*******************  Bit definition for TIM_OR register  *********************/
#define TIM_OR_TI4_RMP                       ((volatile unsigned int)0x00C0)            /*!<TI4_RMP[1:0] bits (TIM5 Input 4 remap)             */
#define TIM_OR_TI4_RMP_0                     ((volatile unsigned int)0x0040)            /*!<Bit 0 */
#define TIM_OR_TI4_RMP_1                     ((volatile unsigned int)0x0080)            /*!<Bit 1 */
#define TIM_OR_ITR1_RMP                      ((volatile unsigned int)0x0C00)            /*!<ITR1_RMP[1:0] bits (TIM2 Internal trigger 1 remap) */
#define TIM_OR_ITR1_RMP_0                    ((volatile unsigned int)0x0400)            /*!<Bit 0 */
#define TIM_OR_ITR1_RMP_1                    ((volatile unsigned int)0x0800)            /*!<Bit 1 */


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define  USART_SR_PE                         ((volatile unsigned int)0x0001)            /*!<Parity Error                 */
#define  USART_SR_FE                         ((volatile unsigned int)0x0002)            /*!<Framing Error                */
#define  USART_SR_NE                         ((volatile unsigned int)0x0004)            /*!<Noise Error Flag             */
#define  USART_SR_ORE                        ((volatile unsigned int)0x0008)            /*!<OverRun Error                */
#define  USART_SR_IDLE                       ((volatile unsigned int)0x0010)            /*!<IDLE line detected           */
#define  USART_SR_RXNE                       ((volatile unsigned int)0x0020)            /*!<Read Data Register Not Empty */
#define  USART_SR_TC                         ((volatile unsigned int)0x0040)            /*!<Transmission Complete        */
#define  USART_SR_TXE                        ((volatile unsigned int)0x0080)            /*!<Transmit Data Register Empty */
#define  USART_SR_LBD                        ((volatile unsigned int)0x0100)            /*!<LIN Break Detection Flag     */
#define  USART_SR_CTS                        ((volatile unsigned int)0x0200)            /*!<CTS Flag                     */

/*******************  Bit definition for USART_DR register  *******************/
#define  USART_DR_DR                         ((volatile unsigned int)0x01FF)            /*!<Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define  USART_BRR_DIV_Fraction              ((volatile unsigned int)0x000F)            /*!<Fraction of USARTDIV */
#define  USART_BRR_DIV_Mantissa              ((volatile unsigned int)0xFFF0)            /*!<Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define  USART_CR1_SBK                       ((volatile unsigned int)0x0001)            /*!<Send Break                             */
#define  USART_CR1_RWU                       ((volatile unsigned int)0x0002)            /*!<Receiver wakeup                        */
#define  USART_CR1_RE                        ((volatile unsigned int)0x0004)            /*!<Receiver Enable                        */
#define  USART_CR1_TE                        ((volatile unsigned int)0x0008)            /*!<Transmitter Enable                     */
#define  USART_CR1_IDLEIE                    ((volatile unsigned int)0x0010)            /*!<IDLE Interrupt Enable                  */
#define  USART_CR1_RXNEIE                    ((volatile unsigned int)0x0020)            /*!<RXNE Interrupt Enable                  */
#define  USART_CR1_TCIE                      ((volatile unsigned int)0x0040)            /*!<Transmission Complete Interrupt Enable */
#define  USART_CR1_TXEIE                     ((volatile unsigned int)0x0080)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PEIE                      ((volatile unsigned int)0x0100)            /*!<PE Interrupt Enable                    */
#define  USART_CR1_PS                        ((volatile unsigned int)0x0200)            /*!<Parity Selection                       */
#define  USART_CR1_PCE                       ((volatile unsigned int)0x0400)            /*!<Parity Control Enable                  */
#define  USART_CR1_WAKE                      ((volatile unsigned int)0x0800)            /*!<Wakeup method                          */
#define  USART_CR1_M                         ((volatile unsigned int)0x1000)            /*!<Word length                            */
#define  USART_CR1_UE                        ((volatile unsigned int)0x2000)            /*!<USART Enable                           */
#define  USART_CR1_OVER8                     ((volatile unsigned int)0x8000)            /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define  USART_CR2_ADD                       ((volatile unsigned int)0x000F)            /*!<Address of the USART node            */
#define  USART_CR2_LBDL                      ((volatile unsigned int)0x0020)            /*!<LIN Break Detection Length           */
#define  USART_CR2_LBDIE                     ((volatile unsigned int)0x0040)            /*!<LIN Break Detection Interrupt Enable */
#define  USART_CR2_LBCL                      ((volatile unsigned int)0x0100)            /*!<Last Bit Clock pulse                 */
#define  USART_CR2_CPHA                      ((volatile unsigned int)0x0200)            /*!<Clock Phase                          */
#define  USART_CR2_CPOL                      ((volatile unsigned int)0x0400)            /*!<Clock Polarity                       */
#define  USART_CR2_CLKEN                     ((volatile unsigned int)0x0800)            /*!<Clock Enable                         */

#define  USART_CR2_STOP                      ((volatile unsigned int)0x3000)            /*!<STOP[1:0] bits (STOP bits) */
#define  USART_CR2_STOP_0                    ((volatile unsigned int)0x1000)            /*!<Bit 0 */
#define  USART_CR2_STOP_1                    ((volatile unsigned int)0x2000)            /*!<Bit 1 */

#define  USART_CR2_LINEN                     ((volatile unsigned int)0x4000)            /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define  USART_CR3_EIE                       ((volatile unsigned int)0x0001)            /*!<Error Interrupt Enable      */
#define  USART_CR3_IREN                      ((volatile unsigned int)0x0002)            /*!<IrDA mode Enable            */
#define  USART_CR3_IRLP                      ((volatile unsigned int)0x0004)            /*!<IrDA Low-Power              */
#define  USART_CR3_HDSEL                     ((volatile unsigned int)0x0008)            /*!<Half-Duplex Selection       */
#define  USART_CR3_NACK                      ((volatile unsigned int)0x0010)            /*!<Smartcard NACK enable       */
#define  USART_CR3_SCEN                      ((volatile unsigned int)0x0020)            /*!<Smartcard mode enable       */
#define  USART_CR3_DMAR                      ((volatile unsigned int)0x0040)            /*!<DMA Enable Receiver         */
#define  USART_CR3_DMAT                      ((volatile unsigned int)0x0080)            /*!<DMA Enable Transmitter      */
#define  USART_CR3_RTSE                      ((volatile unsigned int)0x0100)            /*!<RTS Enable                  */
#define  USART_CR3_CTSE                      ((volatile unsigned int)0x0200)            /*!<CTS Enable                  */
#define  USART_CR3_CTSIE                     ((volatile unsigned int)0x0400)            /*!<CTS Interrupt Enable        */
#define  USART_CR3_ONEBIT                    ((volatile unsigned int)0x0800)            /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define  USART_GTPR_PSC                      ((volatile unsigned int)0x00FF)            /*!<PSC[7:0] bits (Prescaler value) */
#define  USART_GTPR_PSC_0                    ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  USART_GTPR_PSC_1                    ((volatile unsigned int)0x0002)            /*!<Bit 1 */
#define  USART_GTPR_PSC_2                    ((volatile unsigned int)0x0004)            /*!<Bit 2 */
#define  USART_GTPR_PSC_3                    ((volatile unsigned int)0x0008)            /*!<Bit 3 */
#define  USART_GTPR_PSC_4                    ((volatile unsigned int)0x0010)            /*!<Bit 4 */
#define  USART_GTPR_PSC_5                    ((volatile unsigned int)0x0020)            /*!<Bit 5 */
#define  USART_GTPR_PSC_6                    ((volatile unsigned int)0x0040)            /*!<Bit 6 */
#define  USART_GTPR_PSC_7                    ((volatile unsigned int)0x0080)            /*!<Bit 7 */

#define  USART_GTPR_GT                       ((volatile unsigned int)0xFF00)            /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for WWDG_CR register  ********************/
#define  WWDG_CR_T                           ((volatile unsigned int)0x7F)               /*!<T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CR_T0                          ((volatile unsigned int)0x01)               /*!<Bit 0 */
#define  WWDG_CR_T1                          ((volatile unsigned int)0x02)               /*!<Bit 1 */
#define  WWDG_CR_T2                          ((volatile unsigned int)0x04)               /*!<Bit 2 */
#define  WWDG_CR_T3                          ((volatile unsigned int)0x08)               /*!<Bit 3 */
#define  WWDG_CR_T4                          ((volatile unsigned int)0x10)               /*!<Bit 4 */
#define  WWDG_CR_T5                          ((volatile unsigned int)0x20)               /*!<Bit 5 */
#define  WWDG_CR_T6                          ((volatile unsigned int)0x40)               /*!<Bit 6 */

#define  WWDG_CR_WDGA                        ((volatile unsigned int)0x80)               /*!<Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define  WWDG_CFR_W                          ((volatile unsigned int)0x007F)            /*!<W[6:0] bits (7-bit window value) */
#define  WWDG_CFR_W0                         ((volatile unsigned int)0x0001)            /*!<Bit 0 */
#define  WWDG_CFR_W1                         ((volatile unsigned int)0x0002)            /*!<Bit 1 */
#define  WWDG_CFR_W2                         ((volatile unsigned int)0x0004)            /*!<Bit 2 */
#define  WWDG_CFR_W3                         ((volatile unsigned int)0x0008)            /*!<Bit 3 */
#define  WWDG_CFR_W4                         ((volatile unsigned int)0x0010)            /*!<Bit 4 */
#define  WWDG_CFR_W5                         ((volatile unsigned int)0x0020)            /*!<Bit 5 */
#define  WWDG_CFR_W6                         ((volatile unsigned int)0x0040)            /*!<Bit 6 */

#define  WWDG_CFR_WDGTB                      ((volatile unsigned int)0x0180)            /*!<WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFR_WDGTB0                     ((volatile unsigned int)0x0080)            /*!<Bit 0 */
#define  WWDG_CFR_WDGTB1                     ((volatile unsigned int)0x0100)            /*!<Bit 1 */

#define  WWDG_CFR_EWI                        ((volatile unsigned int)0x0200)            /*!<Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define  WWDG_SR_EWIF                        ((volatile unsigned int)0x01)               /*!<Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                DBG                                         */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for DBGMCU_IDCODE register  *************/
#define  DBGMCU_IDCODE_DEV_ID                ((volatile unsigned int)0x00000FFF)
#define  DBGMCU_IDCODE_REV_ID                ((volatile unsigned int)0xFFFF0000)

/********************  Bit definition for DBGMCU_CR register  *****************/
#define  DBGMCU_CR_DBG_SLEEP                 ((volatile unsigned int)0x00000001)
#define  DBGMCU_CR_DBG_STOP                  ((volatile unsigned int)0x00000002)
#define  DBGMCU_CR_DBG_STANDBY               ((volatile unsigned int)0x00000004)
#define  DBGMCU_CR_TRACE_IOEN                ((volatile unsigned int)0x00000020)

#define  DBGMCU_CR_TRACE_MODE                ((volatile unsigned int)0x000000C0)
#define  DBGMCU_CR_TRACE_MODE_0              ((volatile unsigned int)0x00000040)/*!<Bit 0 */
#define  DBGMCU_CR_TRACE_MODE_1              ((volatile unsigned int)0x00000080)/*!<Bit 1 */

/********************  Bit definition for DBGMCU_APB1_FZ register  ************/
#define  DBGMCU_APB1_FZ_DBG_TIM2_STOP            ((volatile unsigned int)0x00000001)
#define  DBGMCU_APB1_FZ_DBG_TIM3_STOP            ((volatile unsigned int)0x00000002)
#define  DBGMCU_APB1_FZ_DBG_TIM4_STOP            ((volatile unsigned int)0x00000004)
#define  DBGMCU_APB1_FZ_DBG_TIM5_STOP            ((volatile unsigned int)0x00000008)
#define  DBGMCU_APB1_FZ_DBG_TIM6_STOP            ((volatile unsigned int)0x00000010)
#define  DBGMCU_APB1_FZ_DBG_TIM7_STOP            ((volatile unsigned int)0x00000020)
#define  DBGMCU_APB1_FZ_DBG_TIM12_STOP           ((volatile unsigned int)0x00000040)
#define  DBGMCU_APB1_FZ_DBG_TIM13_STOP           ((volatile unsigned int)0x00000080)
#define  DBGMCU_APB1_FZ_DBG_TIM14_STOP           ((volatile unsigned int)0x00000100)
#define  DBGMCU_APB1_FZ_DBG_RTC_STOP             ((volatile unsigned int)0x00000400)
#define  DBGMCU_APB1_FZ_DBG_WWDG_STOP            ((volatile unsigned int)0x00000800)
#define  DBGMCU_APB1_FZ_DBG_IWDG_STOP            ((volatile unsigned int)0x00001000)
#define  DBGMCU_APB1_FZ_DBG_I2C1_SMBUS_TIMEOUT   ((volatile unsigned int)0x00200000)
#define  DBGMCU_APB1_FZ_DBG_I2C2_SMBUS_TIMEOUT   ((volatile unsigned int)0x00400000)
#define  DBGMCU_APB1_FZ_DBG_I2C3_SMBUS_TIMEOUT   ((volatile unsigned int)0x00800000)
#define  DBGMCU_APB1_FZ_DBG_CAN1_STOP            ((volatile unsigned int)0x02000000)
#define  DBGMCU_APB1_FZ_DBG_CAN2_STOP            ((volatile unsigned int)0x04000000)
/* Old IWDGSTOP bit definition, maintained for legacy purpose */
#define  DBGMCU_APB1_FZ_DBG_IWDEG_STOP           DBGMCU_APB1_FZ_DBG_IWDG_STOP

/********************  Bit definition for DBGMCU_APB2_FZ register  ************/
#define  DBGMCU_APB2_FZ_DBG_TIM1_STOP        ((volatile unsigned int)0x00000001)
#define  DBGMCU_APB2_FZ_DBG_TIM8_STOP        ((volatile unsigned int)0x00000002)
#define  DBGMCU_APB2_FZ_DBG_TIM9_STOP        ((volatile unsigned int)0x00010000)
#define  DBGMCU_APB2_FZ_DBG_TIM10_STOP       ((volatile unsigned int)0x00020000)
#define  DBGMCU_APB2_FZ_DBG_TIM11_STOP       ((volatile unsigned int)0x00040000)

/******************************************************************************/
/*                                                                            */
/*                Ethernet MAC Registers bits definitions                     */
/*                                                                            */
/******************************************************************************/
/* Bit definition for Ethernet MAC Control Register register */
#define ETH_MACCR_WD      ((volatile unsigned int)0x00800000)  /* Watchdog disable */
#define ETH_MACCR_JD      ((volatile unsigned int)0x00400000)  /* Jabber disable */
#define ETH_MACCR_IFG     ((volatile unsigned int)0x000E0000)  /* Inter-frame gap */
#define ETH_MACCR_IFG_96Bit     ((volatile unsigned int)0x00000000)  /* Minimum IFG between frames during transmission is 96Bit */
#define ETH_MACCR_IFG_88Bit     ((volatile unsigned int)0x00020000)  /* Minimum IFG between frames during transmission is 88Bit */
#define ETH_MACCR_IFG_80Bit     ((volatile unsigned int)0x00040000)  /* Minimum IFG between frames during transmission is 80Bit */
#define ETH_MACCR_IFG_72Bit     ((volatile unsigned int)0x00060000)  /* Minimum IFG between frames during transmission is 72Bit */
#define ETH_MACCR_IFG_64Bit     ((volatile unsigned int)0x00080000)  /* Minimum IFG between frames during transmission is 64Bit */        
#define ETH_MACCR_IFG_56Bit     ((volatile unsigned int)0x000A0000)  /* Minimum IFG between frames during transmission is 56Bit */
#define ETH_MACCR_IFG_48Bit     ((volatile unsigned int)0x000C0000)  /* Minimum IFG between frames during transmission is 48Bit */
#define ETH_MACCR_IFG_40Bit     ((volatile unsigned int)0x000E0000)  /* Minimum IFG between frames during transmission is 40Bit */              
#define ETH_MACCR_CSD     ((volatile unsigned int)0x00010000)  /* Carrier sense disable (during transmission) */
#define ETH_MACCR_FES     ((volatile unsigned int)0x00004000)  /* Fast ethernet speed */
#define ETH_MACCR_ROD     ((volatile unsigned int)0x00002000)  /* Receive own disable */
#define ETH_MACCR_LM      ((volatile unsigned int)0x00001000)  /* loopback mode */
#define ETH_MACCR_DM      ((volatile unsigned int)0x00000800)  /* Duplex mode */
#define ETH_MACCR_IPCO    ((volatile unsigned int)0x00000400)  /* IP Checksum offload */
#define ETH_MACCR_RD      ((volatile unsigned int)0x00000200)  /* Retry disable */
#define ETH_MACCR_APCS    ((volatile unsigned int)0x00000080)  /* Automatic Pad/CRC stripping */
#define ETH_MACCR_BL      ((volatile unsigned int)0x00000060)  /* Back-off limit: random integer number (r) of slot time delays before rescheduling
							 a transmission attempt during retries after a collision: 0 =< r <2^k */
#define ETH_MACCR_BL_10    ((volatile unsigned int)0x00000000)  /* k = min (n, 10) */
#define ETH_MACCR_BL_8     ((volatile unsigned int)0x00000020)  /* k = min (n, 8) */
#define ETH_MACCR_BL_4     ((volatile unsigned int)0x00000040)  /* k = min (n, 4) */
#define ETH_MACCR_BL_1     ((volatile unsigned int)0x00000060)  /* k = min (n, 1) */ 
#define ETH_MACCR_DC      ((volatile unsigned int)0x00000010)  /* Defferal check */
#define ETH_MACCR_TE      ((volatile unsigned int)0x00000008)  /* Transmitter enable */
#define ETH_MACCR_RE      ((volatile unsigned int)0x00000004)  /* Receiver enable */

/* Bit definition for Ethernet MAC Frame Filter Register */
#define ETH_MACFFR_RA     ((volatile unsigned int)0x80000000)  /* Receive all */ 
#define ETH_MACFFR_HPF    ((volatile unsigned int)0x00000400)  /* Hash or perfect filter */ 
#define ETH_MACFFR_SAF    ((volatile unsigned int)0x00000200)  /* Source address filter enable */ 
#define ETH_MACFFR_SAIF   ((volatile unsigned int)0x00000100)  /* SA inverse filtering */ 
#define ETH_MACFFR_PCF    ((volatile unsigned int)0x000000C0)  /* Pass control frames: 3 cases */
#define ETH_MACFFR_PCF_BlockAll                ((volatile unsigned int)0x00000040)  /* MAC filters all control frames from reaching the application */
#define ETH_MACFFR_PCF_ForwardAll              ((volatile unsigned int)0x00000080)  /* MAC forwards all control frames to application even if they fail the Address Filter */
#define ETH_MACFFR_PCF_ForwardPassedAddrFilter ((volatile unsigned int)0x000000C0)  /* MAC forwards control frames that pass the Address Filter. */ 
#define ETH_MACFFR_BFD    ((volatile unsigned int)0x00000020)  /* Broadcast frame disable */ 
#define ETH_MACFFR_PAM    ((volatile unsigned int)0x00000010)  /* Pass all mutlicast */ 
#define ETH_MACFFR_DAIF   ((volatile unsigned int)0x00000008)  /* DA Inverse filtering */ 
#define ETH_MACFFR_HM     ((volatile unsigned int)0x00000004)  /* Hash multicast */ 
#define ETH_MACFFR_HU     ((volatile unsigned int)0x00000002)  /* Hash unicast */
#define ETH_MACFFR_PM     ((volatile unsigned int)0x00000001)  /* Promiscuous mode */

/* Bit definition for Ethernet MAC Hash Table High Register */
#define ETH_MACHTHR_HTH   ((volatile unsigned int)0xFFFFFFFF)  /* Hash table high */

/* Bit definition for Ethernet MAC Hash Table Low Register */
#define ETH_MACHTLR_HTL   ((volatile unsigned int)0xFFFFFFFF)  /* Hash table low */

/* Bit definition for Ethernet MAC MII Address Register */
#define ETH_MACMIIAR_PA   ((volatile unsigned int)0x0000F800)  /* Physical layer address */ 
#define ETH_MACMIIAR_MR   ((volatile unsigned int)0x000007C0)  /* MII register in the selected PHY */ 
#define ETH_MACMIIAR_CR   ((volatile unsigned int)0x0000001C)  /* CR clock range: 6 cases */ 
#define ETH_MACMIIAR_CR_Div42   ((volatile unsigned int)0x00000000)  /* HCLK:60-100 MHz; MDC clock= HCLK/42 */
#define ETH_MACMIIAR_CR_Div62   ((volatile unsigned int)0x00000004)  /* HCLK:100-150 MHz; MDC clock= HCLK/62 */
#define ETH_MACMIIAR_CR_Div16   ((volatile unsigned int)0x00000008)  /* HCLK:20-35 MHz; MDC clock= HCLK/16 */
#define ETH_MACMIIAR_CR_Div26   ((volatile unsigned int)0x0000000C)  /* HCLK:35-60 MHz; MDC clock= HCLK/26 */
#define ETH_MACMIIAR_CR_Div102  ((volatile unsigned int)0x00000010)  /* HCLK:150-168 MHz; MDC clock= HCLK/102 */  
#define ETH_MACMIIAR_MW   ((volatile unsigned int)0x00000002)  /* MII write */ 
#define ETH_MACMIIAR_MB   ((volatile unsigned int)0x00000001)  /* MII busy */ 

/* Bit definition for Ethernet MAC MII Data Register */
#define ETH_MACMIIDR_MD   ((volatile unsigned int)0x0000FFFF)  /* MII data: read/write data from/to PHY */

/* Bit definition for Ethernet MAC Flow Control Register */
#define ETH_MACFCR_PT     ((volatile unsigned int)0xFFFF0000)  /* Pause time */
#define ETH_MACFCR_ZQPD   ((volatile unsigned int)0x00000080)  /* Zero-quanta pause disable */
#define ETH_MACFCR_PLT    ((volatile unsigned int)0x00000030)  /* Pause low threshold: 4 cases */
#define ETH_MACFCR_PLT_Minus4   ((volatile unsigned int)0x00000000)  /* Pause time minus 4 slot times */
#define ETH_MACFCR_PLT_Minus28  ((volatile unsigned int)0x00000010)  /* Pause time minus 28 slot times */
#define ETH_MACFCR_PLT_Minus144 ((volatile unsigned int)0x00000020)  /* Pause time minus 144 slot times */
#define ETH_MACFCR_PLT_Minus256 ((volatile unsigned int)0x00000030)  /* Pause time minus 256 slot times */      
#define ETH_MACFCR_UPFD   ((volatile unsigned int)0x00000008)  /* Unicast pause frame detect */
#define ETH_MACFCR_RFCE   ((volatile unsigned int)0x00000004)  /* Receive flow control enable */
#define ETH_MACFCR_TFCE   ((volatile unsigned int)0x00000002)  /* Transmit flow control enable */
#define ETH_MACFCR_FCBBPA ((volatile unsigned int)0x00000001)  /* Flow control busy/backpressure activate */

/* Bit definition for Ethernet MAC VLAN Tag Register */
#define ETH_MACVLANTR_VLANTC ((volatile unsigned int)0x00010000)  /* 12-bit VLAN tag comparison */
#define ETH_MACVLANTR_VLANTI ((volatile unsigned int)0x0000FFFF)  /* VLAN tag identifier (for receive frames) */

/* Bit definition for Ethernet MAC Remote Wake-UpFrame Filter Register */ 
#define ETH_MACRWUFFR_D   ((volatile unsigned int)0xFFFFFFFF)  /* Wake-up frame filter register data */
/* Eight sequential Writes to this address (offset 0x28) will write all Wake-UpFrame Filter Registers.
   Eight sequential Reads from this address (offset 0x28) will read all Wake-UpFrame Filter Registers. */
/* Wake-UpFrame Filter Reg0 : Filter 0 Byte Mask
   Wake-UpFrame Filter Reg1 : Filter 1 Byte Mask
   Wake-UpFrame Filter Reg2 : Filter 2 Byte Mask
   Wake-UpFrame Filter Reg3 : Filter 3 Byte Mask
   Wake-UpFrame Filter Reg4 : RSVD - Filter3 Command - RSVD - Filter2 Command - 
   RSVD - Filter1 Command - RSVD - Filter0 Command
   Wake-UpFrame Filter Re5 : Filter3 Offset - Filter2 Offset - Filter1 Offset - Filter0 Offset
   Wake-UpFrame Filter Re6 : Filter1 CRC16 - Filter0 CRC16
   Wake-UpFrame Filter Re7 : Filter3 CRC16 - Filter2 CRC16 */

/* Bit definition for Ethernet MAC PMT Control and Status Register */ 
#define ETH_MACPMTCSR_WFFRPR ((volatile unsigned int)0x80000000)  /* Wake-Up Frame Filter Register Pointer Reset */
#define ETH_MACPMTCSR_GU     ((volatile unsigned int)0x00000200)  /* Global Unicast */
#define ETH_MACPMTCSR_WFR    ((volatile unsigned int)0x00000040)  /* Wake-Up Frame Received */
#define ETH_MACPMTCSR_MPR    ((volatile unsigned int)0x00000020)  /* Magic Packet Received */
#define ETH_MACPMTCSR_WFE    ((volatile unsigned int)0x00000004)  /* Wake-Up Frame Enable */
#define ETH_MACPMTCSR_MPE    ((volatile unsigned int)0x00000002)  /* Magic Packet Enable */
#define ETH_MACPMTCSR_PD     ((volatile unsigned int)0x00000001)  /* Power Down */

/* Bit definition for Ethernet MAC Status Register */
#define ETH_MACSR_TSTS      ((volatile unsigned int)0x00000200)  /* Time stamp trigger status */
#define ETH_MACSR_MMCTS     ((volatile unsigned int)0x00000040)  /* MMC transmit status */
#define ETH_MACSR_MMMCRS    ((volatile unsigned int)0x00000020)  /* MMC receive status */
#define ETH_MACSR_MMCS      ((volatile unsigned int)0x00000010)  /* MMC status */
#define ETH_MACSR_PMTS      ((volatile unsigned int)0x00000008)  /* PMT status */

/* Bit definition for Ethernet MAC Interrupt Mask Register */
#define ETH_MACIMR_TSTIM     ((volatile unsigned int)0x00000200)  /* Time stamp trigger interrupt mask */
#define ETH_MACIMR_PMTIM     ((volatile unsigned int)0x00000008)  /* PMT interrupt mask */

/* Bit definition for Ethernet MAC Address0 High Register */
#define ETH_MACA0HR_MACA0H   ((volatile unsigned int)0x0000FFFF)  /* MAC address0 high */

/* Bit definition for Ethernet MAC Address0 Low Register */
#define ETH_MACA0LR_MACA0L   ((volatile unsigned int)0xFFFFFFFF)  /* MAC address0 low */

/* Bit definition for Ethernet MAC Address1 High Register */
#define ETH_MACA1HR_AE       ((volatile unsigned int)0x80000000)  /* Address enable */
#define ETH_MACA1HR_SA       ((volatile unsigned int)0x40000000)  /* Source address */
#define ETH_MACA1HR_MBC      ((volatile unsigned int)0x3F000000)  /* Mask byte control: bits to mask for comparison of the MAC Address bytes */
#define ETH_MACA1HR_MBC_HBits15_8    ((volatile unsigned int)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
#define ETH_MACA1HR_MBC_HBits7_0     ((volatile unsigned int)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
#define ETH_MACA1HR_MBC_LBits31_24   ((volatile unsigned int)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
#define ETH_MACA1HR_MBC_LBits23_16   ((volatile unsigned int)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
#define ETH_MACA1HR_MBC_LBits15_8    ((volatile unsigned int)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
#define ETH_MACA1HR_MBC_LBits7_0     ((volatile unsigned int)0x01000000)  /* Mask MAC Address low reg bits [7:0] */ 
#define ETH_MACA1HR_MACA1H   ((volatile unsigned int)0x0000FFFF)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address1 Low Register */
#define ETH_MACA1LR_MACA1L   ((volatile unsigned int)0xFFFFFFFF)  /* MAC address1 low */

/* Bit definition for Ethernet MAC Address2 High Register */
#define ETH_MACA2HR_AE       ((volatile unsigned int)0x80000000)  /* Address enable */
#define ETH_MACA2HR_SA       ((volatile unsigned int)0x40000000)  /* Source address */
#define ETH_MACA2HR_MBC      ((volatile unsigned int)0x3F000000)  /* Mask byte control */
#define ETH_MACA2HR_MBC_HBits15_8    ((volatile unsigned int)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
#define ETH_MACA2HR_MBC_HBits7_0     ((volatile unsigned int)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
#define ETH_MACA2HR_MBC_LBits31_24   ((volatile unsigned int)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
#define ETH_MACA2HR_MBC_LBits23_16   ((volatile unsigned int)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
#define ETH_MACA2HR_MBC_LBits15_8    ((volatile unsigned int)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
#define ETH_MACA2HR_MBC_LBits7_0     ((volatile unsigned int)0x01000000)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA2HR_MACA2H   ((volatile unsigned int)0x0000FFFF)  /* MAC address1 high */

/* Bit definition for Ethernet MAC Address2 Low Register */
#define ETH_MACA2LR_MACA2L   ((volatile unsigned int)0xFFFFFFFF)  /* MAC address2 low */

/* Bit definition for Ethernet MAC Address3 High Register */
#define ETH_MACA3HR_AE       ((volatile unsigned int)0x80000000)  /* Address enable */
#define ETH_MACA3HR_SA       ((volatile unsigned int)0x40000000)  /* Source address */
#define ETH_MACA3HR_MBC      ((volatile unsigned int)0x3F000000)  /* Mask byte control */
#define ETH_MACA3HR_MBC_HBits15_8    ((volatile unsigned int)0x20000000)  /* Mask MAC Address high reg bits [15:8] */
#define ETH_MACA3HR_MBC_HBits7_0     ((volatile unsigned int)0x10000000)  /* Mask MAC Address high reg bits [7:0] */
#define ETH_MACA3HR_MBC_LBits31_24   ((volatile unsigned int)0x08000000)  /* Mask MAC Address low reg bits [31:24] */
#define ETH_MACA3HR_MBC_LBits23_16   ((volatile unsigned int)0x04000000)  /* Mask MAC Address low reg bits [23:16] */
#define ETH_MACA3HR_MBC_LBits15_8    ((volatile unsigned int)0x02000000)  /* Mask MAC Address low reg bits [15:8] */
#define ETH_MACA3HR_MBC_LBits7_0     ((volatile unsigned int)0x01000000)  /* Mask MAC Address low reg bits [70] */
#define ETH_MACA3HR_MACA3H   ((volatile unsigned int)0x0000FFFF)  /* MAC address3 high */

/* Bit definition for Ethernet MAC Address3 Low Register */
#define ETH_MACA3LR_MACA3L   ((volatile unsigned int)0xFFFFFFFF)  /* MAC address3 low */

/******************************************************************************/
/*                Ethernet MMC Registers bits definition                      */
/******************************************************************************/

/* Bit definition for Ethernet MMC Contol Register */
#define ETH_MMCCR_MCFHP      ((volatile unsigned int)0x00000020)  /* MMC counter Full-Half preset */
#define ETH_MMCCR_MCP        ((volatile unsigned int)0x00000010)  /* MMC counter preset */
#define ETH_MMCCR_MCF        ((volatile unsigned int)0x00000008)  /* MMC Counter Freeze */
#define ETH_MMCCR_ROR        ((volatile unsigned int)0x00000004)  /* Reset on Read */
#define ETH_MMCCR_CSR        ((volatile unsigned int)0x00000002)  /* Counter Stop Rollover */
#define ETH_MMCCR_CR         ((volatile unsigned int)0x00000001)  /* Counters Reset */

/* Bit definition for Ethernet MMC Receive Interrupt Register */
#define ETH_MMCRIR_RGUFS     ((volatile unsigned int)0x00020000)  /* Set when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIR_RFAES     ((volatile unsigned int)0x00000040)  /* Set when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIR_RFCES     ((volatile unsigned int)0x00000020)  /* Set when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Register */
#define ETH_MMCTIR_TGFS      ((volatile unsigned int)0x00200000)  /* Set when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIR_TGFMSCS   ((volatile unsigned int)0x00008000)  /* Set when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIR_TGFSCS    ((volatile unsigned int)0x00004000)  /* Set when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Receive Interrupt Mask Register */
#define ETH_MMCRIMR_RGUFM    ((volatile unsigned int)0x00020000)  /* Mask the interrupt when Rx good unicast frames counter reaches half the maximum value */
#define ETH_MMCRIMR_RFAEM    ((volatile unsigned int)0x00000040)  /* Mask the interrupt when when Rx alignment error counter reaches half the maximum value */
#define ETH_MMCRIMR_RFCEM    ((volatile unsigned int)0x00000020)  /* Mask the interrupt when Rx crc error counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmit Interrupt Mask Register */
#define ETH_MMCTIMR_TGFM     ((volatile unsigned int)0x00200000)  /* Mask the interrupt when Tx good frame count counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFMSCM  ((volatile unsigned int)0x00008000)  /* Mask the interrupt when Tx good multi col counter reaches half the maximum value */
#define ETH_MMCTIMR_TGFSCM   ((volatile unsigned int)0x00004000)  /* Mask the interrupt when Tx good single col counter reaches half the maximum value */

/* Bit definition for Ethernet MMC Transmitted Good Frames after Single Collision Counter Register */
#define ETH_MMCTGFSCCR_TGFSCC     ((volatile unsigned int)0xFFFFFFFF)  /* Number of successfully transmitted frames after a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames after More than a Single Collision Counter Register */
#define ETH_MMCTGFMSCCR_TGFMSCC   ((volatile unsigned int)0xFFFFFFFF)  /* Number of successfully transmitted frames after more than a single collision in Half-duplex mode. */

/* Bit definition for Ethernet MMC Transmitted Good Frames Counter Register */
#define ETH_MMCTGFCR_TGFC    ((volatile unsigned int)0xFFFFFFFF)  /* Number of good frames transmitted. */

/* Bit definition for Ethernet MMC Received Frames with CRC Error Counter Register */
#define ETH_MMCRFCECR_RFCEC  ((volatile unsigned int)0xFFFFFFFF)  /* Number of frames received with CRC error. */

/* Bit definition for Ethernet MMC Received Frames with Alignement Error Counter Register */
#define ETH_MMCRFAECR_RFAEC  ((volatile unsigned int)0xFFFFFFFF)  /* Number of frames received with alignment (dribble) error */

/* Bit definition for Ethernet MMC Received Good Unicast Frames Counter Register */
#define ETH_MMCRGUFCR_RGUFC  ((volatile unsigned int)0xFFFFFFFF)  /* Number of good unicast frames received. */

/******************************************************************************/
/*               Ethernet PTP Registers bits definition                       */
/******************************************************************************/

/* Bit definition for Ethernet PTP Time Stamp Contol Register */
#define ETH_PTPTSCR_TSCNT       ((volatile unsigned int)0x00030000)  /* Time stamp clock node type */
#define ETH_PTPTSSR_TSSMRME     ((volatile unsigned int)0x00008000)  /* Time stamp snapshot for message relevant to master enable */
#define ETH_PTPTSSR_TSSEME      ((volatile unsigned int)0x00004000)  /* Time stamp snapshot for event message enable */
#define ETH_PTPTSSR_TSSIPV4FE   ((volatile unsigned int)0x00002000)  /* Time stamp snapshot for IPv4 frames enable */
#define ETH_PTPTSSR_TSSIPV6FE   ((volatile unsigned int)0x00001000)  /* Time stamp snapshot for IPv6 frames enable */
#define ETH_PTPTSSR_TSSPTPOEFE  ((volatile unsigned int)0x00000800)  /* Time stamp snapshot for PTP over ethernet frames enable */
#define ETH_PTPTSSR_TSPTPPSV2E  ((volatile unsigned int)0x00000400)  /* Time stamp PTP packet snooping for version2 format enable */
#define ETH_PTPTSSR_TSSSR       ((volatile unsigned int)0x00000200)  /* Time stamp Sub-seconds rollover */
#define ETH_PTPTSSR_TSSARFE     ((volatile unsigned int)0x00000100)  /* Time stamp snapshot for all received frames enable */

#define ETH_PTPTSCR_TSARU    ((volatile unsigned int)0x00000020)  /* Addend register update */
#define ETH_PTPTSCR_TSITE    ((volatile unsigned int)0x00000010)  /* Time stamp interrupt trigger enable */
#define ETH_PTPTSCR_TSSTU    ((volatile unsigned int)0x00000008)  /* Time stamp update */
#define ETH_PTPTSCR_TSSTI    ((volatile unsigned int)0x00000004)  /* Time stamp initialize */
#define ETH_PTPTSCR_TSFCU    ((volatile unsigned int)0x00000002)  /* Time stamp fine or coarse update */
#define ETH_PTPTSCR_TSE      ((volatile unsigned int)0x00000001)  /* Time stamp enable */

/* Bit definition for Ethernet PTP Sub-Second Increment Register */
#define ETH_PTPSSIR_STSSI    ((volatile unsigned int)0x000000FF)  /* System time Sub-second increment value */

/* Bit definition for Ethernet PTP Time Stamp High Register */
#define ETH_PTPTSHR_STS      ((volatile unsigned int)0xFFFFFFFF)  /* System Time second */

/* Bit definition for Ethernet PTP Time Stamp Low Register */
#define ETH_PTPTSLR_STPNS    ((volatile unsigned int)0x80000000)  /* System Time Positive or negative time */
#define ETH_PTPTSLR_STSS     ((volatile unsigned int)0x7FFFFFFF)  /* System Time sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp High Update Register */
#define ETH_PTPTSHUR_TSUS    ((volatile unsigned int)0xFFFFFFFF)  /* Time stamp update seconds */

/* Bit definition for Ethernet PTP Time Stamp Low Update Register */
#define ETH_PTPTSLUR_TSUPNS  ((volatile unsigned int)0x80000000)  /* Time stamp update Positive or negative time */
#define ETH_PTPTSLUR_TSUSS   ((volatile unsigned int)0x7FFFFFFF)  /* Time stamp update sub-seconds */

/* Bit definition for Ethernet PTP Time Stamp Addend Register */
#define ETH_PTPTSAR_TSA      ((volatile unsigned int)0xFFFFFFFF)  /* Time stamp addend */

/* Bit definition for Ethernet PTP Target Time High Register */
#define ETH_PTPTTHR_TTSH     ((volatile unsigned int)0xFFFFFFFF)  /* Target time stamp high */

/* Bit definition for Ethernet PTP Target Time Low Register */
#define ETH_PTPTTLR_TTSL     ((volatile unsigned int)0xFFFFFFFF)  /* Target time stamp low */

/* Bit definition for Ethernet PTP Time Stamp Status Register */
#define ETH_PTPTSSR_TSTTR    ((volatile unsigned int)0x00000020)  /* Time stamp target time reached */
#define ETH_PTPTSSR_TSSO     ((volatile unsigned int)0x00000010)  /* Time stamp seconds overflow */

/******************************************************************************/
/*                 Ethernet DMA Registers bits definition                     */
/******************************************************************************/

/* Bit definition for Ethernet DMA Bus Mode Register */
#define ETH_DMABMR_AAB       ((volatile unsigned int)0x02000000)  /* Address-Aligned beats */
#define ETH_DMABMR_FPM        ((volatile unsigned int)0x01000000)  /* 4xPBL mode */
#define ETH_DMABMR_USP       ((volatile unsigned int)0x00800000)  /* Use separate PBL */
#define ETH_DMABMR_RDP       ((volatile unsigned int)0x007E0000)  /* RxDMA PBL */
#define ETH_DMABMR_RDP_1Beat    ((volatile unsigned int)0x00020000)  /* maximum number of beats to be transferred in one RxDMA transaction is 1 */
#define ETH_DMABMR_RDP_2Beat    ((volatile unsigned int)0x00040000)  /* maximum number of beats to be transferred in one RxDMA transaction is 2 */
#define ETH_DMABMR_RDP_4Beat    ((volatile unsigned int)0x00080000)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_DMABMR_RDP_8Beat    ((volatile unsigned int)0x00100000)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_DMABMR_RDP_16Beat   ((volatile unsigned int)0x00200000)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_DMABMR_RDP_32Beat   ((volatile unsigned int)0x00400000)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */                
#define ETH_DMABMR_RDP_4xPBL_4Beat   ((volatile unsigned int)0x01020000)  /* maximum number of beats to be transferred in one RxDMA transaction is 4 */
#define ETH_DMABMR_RDP_4xPBL_8Beat   ((volatile unsigned int)0x01040000)  /* maximum number of beats to be transferred in one RxDMA transaction is 8 */
#define ETH_DMABMR_RDP_4xPBL_16Beat  ((volatile unsigned int)0x01080000)  /* maximum number of beats to be transferred in one RxDMA transaction is 16 */
#define ETH_DMABMR_RDP_4xPBL_32Beat  ((volatile unsigned int)0x01100000)  /* maximum number of beats to be transferred in one RxDMA transaction is 32 */
#define ETH_DMABMR_RDP_4xPBL_64Beat  ((volatile unsigned int)0x01200000)  /* maximum number of beats to be transferred in one RxDMA transaction is 64 */
#define ETH_DMABMR_RDP_4xPBL_128Beat ((volatile unsigned int)0x01400000)  /* maximum number of beats to be transferred in one RxDMA transaction is 128 */  
#define ETH_DMABMR_FB        ((volatile unsigned int)0x00010000)  /* Fixed Burst */
#define ETH_DMABMR_RTPR      ((volatile unsigned int)0x0000C000)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_1_1     ((volatile unsigned int)0x00000000)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_2_1     ((volatile unsigned int)0x00004000)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_3_1     ((volatile unsigned int)0x00008000)  /* Rx Tx priority ratio */
#define ETH_DMABMR_RTPR_4_1     ((volatile unsigned int)0x0000C000)  /* Rx Tx priority ratio */  
#define ETH_DMABMR_PBL    ((volatile unsigned int)0x00003F00)  /* Programmable burst length */
#define ETH_DMABMR_PBL_1Beat    ((volatile unsigned int)0x00000100)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 1 */
#define ETH_DMABMR_PBL_2Beat    ((volatile unsigned int)0x00000200)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 2 */
#define ETH_DMABMR_PBL_4Beat    ((volatile unsigned int)0x00000400)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_DMABMR_PBL_8Beat    ((volatile unsigned int)0x00000800)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_DMABMR_PBL_16Beat   ((volatile unsigned int)0x00001000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_DMABMR_PBL_32Beat   ((volatile unsigned int)0x00002000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */                
#define ETH_DMABMR_PBL_4xPBL_4Beat   ((volatile unsigned int)0x01000100)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 4 */
#define ETH_DMABMR_PBL_4xPBL_8Beat   ((volatile unsigned int)0x01000200)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 8 */
#define ETH_DMABMR_PBL_4xPBL_16Beat  ((volatile unsigned int)0x01000400)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 16 */
#define ETH_DMABMR_PBL_4xPBL_32Beat  ((volatile unsigned int)0x01000800)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 32 */
#define ETH_DMABMR_PBL_4xPBL_64Beat  ((volatile unsigned int)0x01001000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 64 */
#define ETH_DMABMR_PBL_4xPBL_128Beat ((volatile unsigned int)0x01002000)  /* maximum number of beats to be transferred in one TxDMA (or both) transaction is 128 */
#define ETH_DMABMR_EDE       ((volatile unsigned int)0x00000080)  /* Enhanced Descriptor Enable */
#define ETH_DMABMR_DSL       ((volatile unsigned int)0x0000007C)  /* Descriptor Skip Length */
#define ETH_DMABMR_DA        ((volatile unsigned int)0x00000002)  /* DMA arbitration scheme */
#define ETH_DMABMR_SR        ((volatile unsigned int)0x00000001)  /* Software reset */

/* Bit definition for Ethernet DMA Transmit Poll Demand Register */
#define ETH_DMATPDR_TPD      ((volatile unsigned int)0xFFFFFFFF)  /* Transmit poll demand */

/* Bit definition for Ethernet DMA Receive Poll Demand Register */
#define ETH_DMARPDR_RPD      ((volatile unsigned int)0xFFFFFFFF)  /* Receive poll demand  */

/* Bit definition for Ethernet DMA Receive Descriptor List Address Register */
#define ETH_DMARDLAR_SRL     ((volatile unsigned int)0xFFFFFFFF)  /* Start of receive list */

/* Bit definition for Ethernet DMA Transmit Descriptor List Address Register */
#define ETH_DMATDLAR_STL     ((volatile unsigned int)0xFFFFFFFF)  /* Start of transmit list */

/* Bit definition for Ethernet DMA Status Register */
#define ETH_DMASR_TSTS       ((volatile unsigned int)0x20000000)  /* Time-stamp trigger status */
#define ETH_DMASR_PMTS       ((volatile unsigned int)0x10000000)  /* PMT status */
#define ETH_DMASR_MMCS       ((volatile unsigned int)0x08000000)  /* MMC status */
#define ETH_DMASR_EBS        ((volatile unsigned int)0x03800000)  /* Error bits status */
/* combination with EBS[2:0] for GetFlagStatus function */
#define ETH_DMASR_EBS_DescAccess      ((volatile unsigned int)0x02000000)  /* Error bits 0-data buffer, 1-desc. access */
#define ETH_DMASR_EBS_ReadTransf      ((volatile unsigned int)0x01000000)  /* Error bits 0-write trnsf, 1-read transfr */
#define ETH_DMASR_EBS_DataTransfTx    ((volatile unsigned int)0x00800000)  /* Error bits 0-Rx DMA, 1-Tx DMA */
#define ETH_DMASR_TPS         ((volatile unsigned int)0x00700000)  /* Transmit process state */
#define ETH_DMASR_TPS_Stopped         ((volatile unsigned int)0x00000000)  /* Stopped - Reset or Stop Tx Command issued  */
#define ETH_DMASR_TPS_Fetching        ((volatile unsigned int)0x00100000)  /* Running - fetching the Tx descriptor */
#define ETH_DMASR_TPS_Waiting         ((volatile unsigned int)0x00200000)  /* Running - waiting for status */
#define ETH_DMASR_TPS_Reading         ((volatile unsigned int)0x00300000)  /* Running - reading the data from host memory */
#define ETH_DMASR_TPS_Suspended       ((volatile unsigned int)0x00600000)  /* Suspended - Tx Descriptor unavailabe */
#define ETH_DMASR_TPS_Closing         ((volatile unsigned int)0x00700000)  /* Running - closing Rx descriptor */
#define ETH_DMASR_RPS         ((volatile unsigned int)0x000E0000)  /* Receive process state */
#define ETH_DMASR_RPS_Stopped         ((volatile unsigned int)0x00000000)  /* Stopped - Reset or Stop Rx Command issued */
#define ETH_DMASR_RPS_Fetching        ((volatile unsigned int)0x00020000)  /* Running - fetching the Rx descriptor */
#define ETH_DMASR_RPS_Waiting         ((volatile unsigned int)0x00060000)  /* Running - waiting for packet */
#define ETH_DMASR_RPS_Suspended       ((volatile unsigned int)0x00080000)  /* Suspended - Rx Descriptor unavailable */
#define ETH_DMASR_RPS_Closing         ((volatile unsigned int)0x000A0000)  /* Running - closing descriptor */
#define ETH_DMASR_RPS_Queuing         ((volatile unsigned int)0x000E0000)  /* Running - queuing the recieve frame into host memory */
#define ETH_DMASR_NIS        ((volatile unsigned int)0x00010000)  /* Normal interrupt summary */
#define ETH_DMASR_AIS        ((volatile unsigned int)0x00008000)  /* Abnormal interrupt summary */
#define ETH_DMASR_ERS        ((volatile unsigned int)0x00004000)  /* Early receive status */
#define ETH_DMASR_FBES       ((volatile unsigned int)0x00002000)  /* Fatal bus error status */
#define ETH_DMASR_ETS        ((volatile unsigned int)0x00000400)  /* Early transmit status */
#define ETH_DMASR_RWTS       ((volatile unsigned int)0x00000200)  /* Receive watchdog timeout status */
#define ETH_DMASR_RPSS       ((volatile unsigned int)0x00000100)  /* Receive process stopped status */
#define ETH_DMASR_RBUS       ((volatile unsigned int)0x00000080)  /* Receive buffer unavailable status */
#define ETH_DMASR_RS         ((volatile unsigned int)0x00000040)  /* Receive status */
#define ETH_DMASR_TUS        ((volatile unsigned int)0x00000020)  /* Transmit underflow status */
#define ETH_DMASR_ROS        ((volatile unsigned int)0x00000010)  /* Receive overflow status */
#define ETH_DMASR_TJTS       ((volatile unsigned int)0x00000008)  /* Transmit jabber timeout status */
#define ETH_DMASR_TBUS       ((volatile unsigned int)0x00000004)  /* Transmit buffer unavailable status */
#define ETH_DMASR_TPSS       ((volatile unsigned int)0x00000002)  /* Transmit process stopped status */
#define ETH_DMASR_TS         ((volatile unsigned int)0x00000001)  /* Transmit status */

/* Bit definition for Ethernet DMA Operation Mode Register */
#define ETH_DMAOMR_DTCEFD    ((volatile unsigned int)0x04000000)  /* Disable Dropping of TCP/IP checksum error frames */
#define ETH_DMAOMR_RSF       ((volatile unsigned int)0x02000000)  /* Receive store and forward */
#define ETH_DMAOMR_DFRF      ((volatile unsigned int)0x01000000)  /* Disable flushing of received frames */
#define ETH_DMAOMR_TSF       ((volatile unsigned int)0x00200000)  /* Transmit store and forward */
#define ETH_DMAOMR_FTF       ((volatile unsigned int)0x00100000)  /* Flush transmit FIFO */
#define ETH_DMAOMR_TTC       ((volatile unsigned int)0x0001C000)  /* Transmit threshold control */
#define ETH_DMAOMR_TTC_64Bytes       ((volatile unsigned int)0x00000000)  /* threshold level of the MTL Transmit FIFO is 64 Bytes */
#define ETH_DMAOMR_TTC_128Bytes      ((volatile unsigned int)0x00004000)  /* threshold level of the MTL Transmit FIFO is 128 Bytes */
#define ETH_DMAOMR_TTC_192Bytes      ((volatile unsigned int)0x00008000)  /* threshold level of the MTL Transmit FIFO is 192 Bytes */
#define ETH_DMAOMR_TTC_256Bytes      ((volatile unsigned int)0x0000C000)  /* threshold level of the MTL Transmit FIFO is 256 Bytes */
#define ETH_DMAOMR_TTC_40Bytes       ((volatile unsigned int)0x00010000)  /* threshold level of the MTL Transmit FIFO is 40 Bytes */
#define ETH_DMAOMR_TTC_32Bytes       ((volatile unsigned int)0x00014000)  /* threshold level of the MTL Transmit FIFO is 32 Bytes */
#define ETH_DMAOMR_TTC_24Bytes       ((volatile unsigned int)0x00018000)  /* threshold level of the MTL Transmit FIFO is 24 Bytes */
#define ETH_DMAOMR_TTC_16Bytes       ((volatile unsigned int)0x0001C000)  /* threshold level of the MTL Transmit FIFO is 16 Bytes */
#define ETH_DMAOMR_ST        ((volatile unsigned int)0x00002000)  /* Start/stop transmission command */
#define ETH_DMAOMR_FEF       ((volatile unsigned int)0x00000080)  /* Forward error frames */
#define ETH_DMAOMR_FUGF      ((volatile unsigned int)0x00000040)  /* Forward undersized good frames */
#define ETH_DMAOMR_RTC       ((volatile unsigned int)0x00000018)  /* receive threshold control */
#define ETH_DMAOMR_RTC_64Bytes       ((volatile unsigned int)0x00000000)  /* threshold level of the MTL Receive FIFO is 64 Bytes */
#define ETH_DMAOMR_RTC_32Bytes       ((volatile unsigned int)0x00000008)  /* threshold level of the MTL Receive FIFO is 32 Bytes */
#define ETH_DMAOMR_RTC_96Bytes       ((volatile unsigned int)0x00000010)  /* threshold level of the MTL Receive FIFO is 96 Bytes */
#define ETH_DMAOMR_RTC_128Bytes      ((volatile unsigned int)0x00000018)  /* threshold level of the MTL Receive FIFO is 128 Bytes */
#define ETH_DMAOMR_OSF       ((volatile unsigned int)0x00000004)  /* operate on second frame */
#define ETH_DMAOMR_SR        ((volatile unsigned int)0x00000002)  /* Start/stop receive */

/* Bit definition for Ethernet DMA Interrupt Enable Register */
#define ETH_DMAIER_NISE      ((volatile unsigned int)0x00010000)  /* Normal interrupt summary enable */
#define ETH_DMAIER_AISE      ((volatile unsigned int)0x00008000)  /* Abnormal interrupt summary enable */
#define ETH_DMAIER_ERIE      ((volatile unsigned int)0x00004000)  /* Early receive interrupt enable */
#define ETH_DMAIER_FBEIE     ((volatile unsigned int)0x00002000)  /* Fatal bus error interrupt enable */
#define ETH_DMAIER_ETIE      ((volatile unsigned int)0x00000400)  /* Early transmit interrupt enable */
#define ETH_DMAIER_RWTIE     ((volatile unsigned int)0x00000200)  /* Receive watchdog timeout interrupt enable */
#define ETH_DMAIER_RPSIE     ((volatile unsigned int)0x00000100)  /* Receive process stopped interrupt enable */
#define ETH_DMAIER_RBUIE     ((volatile unsigned int)0x00000080)  /* Receive buffer unavailable interrupt enable */
#define ETH_DMAIER_RIE       ((volatile unsigned int)0x00000040)  /* Receive interrupt enable */
#define ETH_DMAIER_TUIE      ((volatile unsigned int)0x00000020)  /* Transmit Underflow interrupt enable */
#define ETH_DMAIER_ROIE      ((volatile unsigned int)0x00000010)  /* Receive Overflow interrupt enable */
#define ETH_DMAIER_TJTIE     ((volatile unsigned int)0x00000008)  /* Transmit jabber timeout interrupt enable */
#define ETH_DMAIER_TBUIE     ((volatile unsigned int)0x00000004)  /* Transmit buffer unavailable interrupt enable */
#define ETH_DMAIER_TPSIE     ((volatile unsigned int)0x00000002)  /* Transmit process stopped interrupt enable */
#define ETH_DMAIER_TIE       ((volatile unsigned int)0x00000001)  /* Transmit interrupt enable */

/* Bit definition for Ethernet DMA Missed Frame and Buffer Overflow Counter Register */
#define ETH_DMAMFBOCR_OFOC   ((volatile unsigned int)0x10000000)  /* Overflow bit for FIFO overflow counter */
#define ETH_DMAMFBOCR_MFA    ((volatile unsigned int)0x0FFE0000)  /* Number of frames missed by the application */
#define ETH_DMAMFBOCR_OMFC   ((volatile unsigned int)0x00010000)  /* Overflow bit for missed frame counter */
#define ETH_DMAMFBOCR_MFC    ((volatile unsigned int)0x0000FFFF)  /* Number of frames missed by the controller */

/* Bit definition for Ethernet DMA Current Host Transmit Descriptor Register */
#define ETH_DMACHTDR_HTDAP   ((volatile unsigned int)0xFFFFFFFF)  /* Host transmit descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Descriptor Register */
#define ETH_DMACHRDR_HRDAP   ((volatile unsigned int)0xFFFFFFFF)  /* Host receive descriptor address pointer */

/* Bit definition for Ethernet DMA Current Host Transmit Buffer Address Register */
#define ETH_DMACHTBAR_HTBAP  ((volatile unsigned int)0xFFFFFFFF)  /* Host transmit buffer address pointer */

/* Bit definition for Ethernet DMA Current Host Receive Buffer Address Register */
#define ETH_DMACHRBAR_HRBAP  ((volatile unsigned int)0xFFFFFFFF)  /* Host receive buffer address pointer */

/******************************************************************************/
/*                                                                            */
/*                                       USB_OTG			                        */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition forUSB_OTG_GOTGCTL register  ********************/
#define USB_OTG_GOTGCTL_SRQSCS                  ((volatile unsigned int)0x00000001)            /*!< Session request success */
#define USB_OTG_GOTGCTL_SRQ                     ((volatile unsigned int)0x00000002)            /*!< Session request */
#define USB_OTG_GOTGCTL_HNGSCS                  ((volatile unsigned int)0x00000100)            /*!< Host negotiation success */
#define USB_OTG_GOTGCTL_HNPRQ                   ((volatile unsigned int)0x00000200)            /*!< HNP request */
#define USB_OTG_GOTGCTL_HSHNPEN                 ((volatile unsigned int)0x00000400)            /*!< Host set HNP enable */
#define USB_OTG_GOTGCTL_DHNPEN                  ((volatile unsigned int)0x00000800)            /*!< Device HNP enabled */
#define USB_OTG_GOTGCTL_CIDSTS                  ((volatile unsigned int)0x00010000)            /*!< Connector ID status */
#define USB_OTG_GOTGCTL_DBCT                    ((volatile unsigned int)0x00020000)            /*!< Long/short debounce time */
#define USB_OTG_GOTGCTL_ASVLD                   ((volatile unsigned int)0x00040000)            /*!< A-session valid */
#define USB_OTG_GOTGCTL_BSVLD                   ((volatile unsigned int)0x00080000)            /*!< B-session valid */

/********************  Bit definition forUSB_OTG_HCFG register  ********************/

#define USB_OTG_HCFG_FSLSPCS                 ((volatile unsigned int)0x00000003)            /*!< FS/LS PHY clock select */
#define USB_OTG_HCFG_FSLSPCS_0               ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_HCFG_FSLSPCS_1               ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_HCFG_FSLSS                   ((volatile unsigned int)0x00000004)            /*!< FS- and LS-only support */

/********************  Bit definition forUSB_OTG_DCFG register  ********************/

#define USB_OTG_DCFG_DSPD                    ((volatile unsigned int)0x00000003)            /*!< Device speed */
#define USB_OTG_DCFG_DSPD_0                  ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_DCFG_DSPD_1                  ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_DCFG_NZLSOHSK                ((volatile unsigned int)0x00000004)            /*!< Nonzero-length status OUT handshake */

#define USB_OTG_DCFG_DAD                     ((volatile unsigned int)0x000007F0)            /*!< Device address */
#define USB_OTG_DCFG_DAD_0                   ((volatile unsigned int)0x00000010)            /*!<Bit 0 */
#define USB_OTG_DCFG_DAD_1                   ((volatile unsigned int)0x00000020)            /*!<Bit 1 */
#define USB_OTG_DCFG_DAD_2                   ((volatile unsigned int)0x00000040)            /*!<Bit 2 */
#define USB_OTG_DCFG_DAD_3                   ((volatile unsigned int)0x00000080)            /*!<Bit 3 */
#define USB_OTG_DCFG_DAD_4                   ((volatile unsigned int)0x00000100)            /*!<Bit 4 */
#define USB_OTG_DCFG_DAD_5                   ((volatile unsigned int)0x00000200)            /*!<Bit 5 */
#define USB_OTG_DCFG_DAD_6                   ((volatile unsigned int)0x00000400)            /*!<Bit 6 */

#define USB_OTG_DCFG_PFIVL                   ((volatile unsigned int)0x00001800)            /*!< Periodic (micro)frame interval */
#define USB_OTG_DCFG_PFIVL_0                 ((volatile unsigned int)0x00000800)            /*!<Bit 0 */
#define USB_OTG_DCFG_PFIVL_1                 ((volatile unsigned int)0x00001000)            /*!<Bit 1 */

#define USB_OTG_DCFG_PERSCHIVL               ((volatile unsigned int)0x03000000)            /*!< Periodic scheduling interval */
#define USB_OTG_DCFG_PERSCHIVL_0             ((volatile unsigned int)0x01000000)            /*!<Bit 0 */
#define USB_OTG_DCFG_PERSCHIVL_1             ((volatile unsigned int)0x02000000)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_PCGCR register  ********************/
#define USB_OTG_PCGCR_STPPCLK                 ((volatile unsigned int)0x00000001)            /*!< Stop PHY clock */
#define USB_OTG_PCGCR_GATEHCLK                ((volatile unsigned int)0x00000002)            /*!< Gate HCLK */
#define USB_OTG_PCGCR_PHYSUSP                 ((volatile unsigned int)0x00000010)            /*!< PHY suspended */

/********************  Bit definition forUSB_OTG_GOTGINT register  ********************/
#define USB_OTG_GOTGINT_SEDET                   ((volatile unsigned int)0x00000004)            /*!< Session end detected */
#define USB_OTG_GOTGINT_SRSSCHG                 ((volatile unsigned int)0x00000100)            /*!< Session request success status change */
#define USB_OTG_GOTGINT_HNSSCHG                 ((volatile unsigned int)0x00000200)            /*!< Host negotiation success status change */
#define USB_OTG_GOTGINT_HNGDET                  ((volatile unsigned int)0x00020000)            /*!< Host negotiation detected */
#define USB_OTG_GOTGINT_ADTOCHG                 ((volatile unsigned int)0x00040000)            /*!< A-device timeout change */
#define USB_OTG_GOTGINT_DBCDNE                  ((volatile unsigned int)0x00080000)            /*!< Debounce done */

/********************  Bit definition forUSB_OTG_DCTL register  ********************/
#define USB_OTG_DCTL_RWUSIG                  ((volatile unsigned int)0x00000001)            /*!< Remote wakeup signaling */
#define USB_OTG_DCTL_SDIS                    ((volatile unsigned int)0x00000002)            /*!< Soft disconnect */
#define USB_OTG_DCTL_GINSTS                  ((volatile unsigned int)0x00000004)            /*!< Global IN NAK status */
#define USB_OTG_DCTL_GONSTS                  ((volatile unsigned int)0x00000008)            /*!< Global OUT NAK status */

#define USB_OTG_DCTL_TCTL                    ((volatile unsigned int)0x00000070)            /*!< Test control */
#define USB_OTG_DCTL_TCTL_0                  ((volatile unsigned int)0x00000010)            /*!<Bit 0 */
#define USB_OTG_DCTL_TCTL_1                  ((volatile unsigned int)0x00000020)            /*!<Bit 1 */
#define USB_OTG_DCTL_TCTL_2                  ((volatile unsigned int)0x00000040)            /*!<Bit 2 */
#define USB_OTG_DCTL_SGINAK                  ((volatile unsigned int)0x00000080)            /*!< Set global IN NAK */
#define USB_OTG_DCTL_CGINAK                  ((volatile unsigned int)0x00000100)            /*!< Clear global IN NAK */
#define USB_OTG_DCTL_SGONAK                  ((volatile unsigned int)0x00000200)            /*!< Set global OUT NAK */
#define USB_OTG_DCTL_CGONAK                  ((volatile unsigned int)0x00000400)            /*!< Clear global OUT NAK */
#define USB_OTG_DCTL_POPRGDNE                ((volatile unsigned int)0x00000800)            /*!< Power-on programming done */

/********************  Bit definition forUSB_OTG_HFIR register  ********************/
#define USB_OTG_HFIR_FRIVL                   ((volatile unsigned int)0x0000FFFF)            /*!< Frame interval */

/********************  Bit definition forUSB_OTG_HFNUM register  ********************/
#define USB_OTG_HFNUM_FRNUM                   ((volatile unsigned int)0x0000FFFF)            /*!< Frame number */
#define USB_OTG_HFNUM_FTREM                   ((volatile unsigned int)0xFFFF0000)            /*!< Frame time remaining */

/********************  Bit definition forUSB_OTG_DSTS register  ********************/
#define USB_OTG_DSTS_SUSPSTS                 ((volatile unsigned int)0x00000001)            /*!< Suspend status */

#define USB_OTG_DSTS_ENUMSPD                 ((volatile unsigned int)0x00000006)            /*!< Enumerated speed */
#define USB_OTG_DSTS_ENUMSPD_0               ((volatile unsigned int)0x00000002)            /*!<Bit 0 */
#define USB_OTG_DSTS_ENUMSPD_1               ((volatile unsigned int)0x00000004)            /*!<Bit 1 */
#define USB_OTG_DSTS_EERR                    ((volatile unsigned int)0x00000008)            /*!< Erratic error */
#define USB_OTG_DSTS_FNSOF                   ((volatile unsigned int)0x003FFF00)            /*!< Frame number of the received SOF */

/********************  Bit definition forUSB_OTG_GAHBCFG register  ********************/
#define USB_OTG_GAHBCFG_GINT                    ((volatile unsigned int)0x00000001)            /*!< Global interrupt mask */

#define USB_OTG_GAHBCFG_HBSTLEN                 ((volatile unsigned int)0x0000001E)            /*!< Burst length/type */
#define USB_OTG_GAHBCFG_HBSTLEN_0               ((volatile unsigned int)0x00000002)            /*!<Bit 0 */
#define USB_OTG_GAHBCFG_HBSTLEN_1               ((volatile unsigned int)0x00000004)            /*!<Bit 1 */
#define USB_OTG_GAHBCFG_HBSTLEN_2               ((volatile unsigned int)0x00000008)            /*!<Bit 2 */
#define USB_OTG_GAHBCFG_HBSTLEN_3               ((volatile unsigned int)0x00000010)            /*!<Bit 3 */
#define USB_OTG_GAHBCFG_DMAEN                   ((volatile unsigned int)0x00000020)            /*!< DMA enable */
#define USB_OTG_GAHBCFG_TXFELVL                 ((volatile unsigned int)0x00000080)            /*!< TxFIFO empty level */
#define USB_OTG_GAHBCFG_PTXFELVL                ((volatile unsigned int)0x00000100)            /*!< Periodic TxFIFO empty level */

/********************  Bit definition forUSB_OTG_GUSBCFG register  ********************/

#define USB_OTG_GUSBCFG_TOCAL                   ((volatile unsigned int)0x00000007)            /*!< FS timeout calibration */
#define USB_OTG_GUSBCFG_TOCAL_0                 ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TOCAL_1                 ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TOCAL_2                 ((volatile unsigned int)0x00000004)            /*!<Bit 2 */
#define USB_OTG_GUSBCFG_PHYSEL                  ((volatile unsigned int)0x00000040)            /*!< USB 2.0 high-speed ULPI PHY or USB 1.1 full-speed serial transceiver select */
#define USB_OTG_GUSBCFG_SRPCAP                  ((volatile unsigned int)0x00000100)            /*!< SRP-capable */
#define USB_OTG_GUSBCFG_HNPCAP                  ((volatile unsigned int)0x00000200)            /*!< HNP-capable */

#define USB_OTG_GUSBCFG_TRDT                    ((volatile unsigned int)0x00003C00)            /*!< USB turnaround time */
#define USB_OTG_GUSBCFG_TRDT_0                  ((volatile unsigned int)0x00000400)            /*!<Bit 0 */
#define USB_OTG_GUSBCFG_TRDT_1                  ((volatile unsigned int)0x00000800)            /*!<Bit 1 */
#define USB_OTG_GUSBCFG_TRDT_2                  ((volatile unsigned int)0x00001000)            /*!<Bit 2 */
#define USB_OTG_GUSBCFG_TRDT_3                  ((volatile unsigned int)0x00002000)            /*!<Bit 3 */
#define USB_OTG_GUSBCFG_PHYLPCS                 ((volatile unsigned int)0x00008000)            /*!< PHY Low-power clock select */
#define USB_OTG_GUSBCFG_ULPIFSLS                ((volatile unsigned int)0x00020000)            /*!< ULPI FS/LS select */
#define USB_OTG_GUSBCFG_ULPIAR                  ((volatile unsigned int)0x00040000)            /*!< ULPI Auto-resume */
#define USB_OTG_GUSBCFG_ULPICSM                 ((volatile unsigned int)0x00080000)            /*!< ULPI Clock SuspendM */
#define USB_OTG_GUSBCFG_ULPIEVBUSD              ((volatile unsigned int)0x00100000)            /*!< ULPI External VBUS Drive */
#define USB_OTG_GUSBCFG_ULPIEVBUSI              ((volatile unsigned int)0x00200000)            /*!< ULPI external VBUS indicator */
#define USB_OTG_GUSBCFG_TSDPS                   ((volatile unsigned int)0x00400000)            /*!< TermSel DLine pulsing selection */
#define USB_OTG_GUSBCFG_PCCI                    ((volatile unsigned int)0x00800000)            /*!< Indicator complement */
#define USB_OTG_GUSBCFG_PTCI                    ((volatile unsigned int)0x01000000)            /*!< Indicator pass through */
#define USB_OTG_GUSBCFG_ULPIIPD                 ((volatile unsigned int)0x02000000)            /*!< ULPI interface protect disable */
#define USB_OTG_GUSBCFG_FHMOD                   ((volatile unsigned int)0x20000000)            /*!< Forced host mode */
#define USB_OTG_GUSBCFG_FDMOD                   ((volatile unsigned int)0x40000000)            /*!< Forced peripheral mode */
#define USB_OTG_GUSBCFG_CTXPKT                  ((volatile unsigned int)0x80000000)            /*!< Corrupt Tx packet */

/********************  Bit definition forUSB_OTG_GRSTCTL register  ********************/
#define USB_OTG_GRSTCTL_CSRST                   ((volatile unsigned int)0x00000001)            /*!< Core soft reset */
#define USB_OTG_GRSTCTL_HSRST                   ((volatile unsigned int)0x00000002)            /*!< HCLK soft reset */
#define USB_OTG_GRSTCTL_FCRST                   ((volatile unsigned int)0x00000004)            /*!< Host frame counter reset */
#define USB_OTG_GRSTCTL_RXFFLSH                 ((volatile unsigned int)0x00000010)            /*!< RxFIFO flush */
#define USB_OTG_GRSTCTL_TXFFLSH                 ((volatile unsigned int)0x00000020)            /*!< TxFIFO flush */

#define USB_OTG_GRSTCTL_TXFNUM                  ((volatile unsigned int)0x000007C0)            /*!< TxFIFO number */
#define USB_OTG_GRSTCTL_TXFNUM_0                ((volatile unsigned int)0x00000040)            /*!<Bit 0 */
#define USB_OTG_GRSTCTL_TXFNUM_1                ((volatile unsigned int)0x00000080)            /*!<Bit 1 */
#define USB_OTG_GRSTCTL_TXFNUM_2                ((volatile unsigned int)0x00000100)            /*!<Bit 2 */
#define USB_OTG_GRSTCTL_TXFNUM_3                ((volatile unsigned int)0x00000200)            /*!<Bit 3 */
#define USB_OTG_GRSTCTL_TXFNUM_4                ((volatile unsigned int)0x00000400)            /*!<Bit 4 */
#define USB_OTG_GRSTCTL_DMAREQ                  ((volatile unsigned int)0x40000000)            /*!< DMA request signal */
#define USB_OTG_GRSTCTL_AHBIDL                  ((volatile unsigned int)0x80000000)            /*!< AHB master idle */

/********************  Bit definition forUSB_OTG_DIEPMSK register  ********************/
#define USB_OTG_DIEPMSK_XFRCM                   ((volatile unsigned int)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPMSK_EPDM                    ((volatile unsigned int)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPMSK_TOM                     ((volatile unsigned int)0x00000008)            /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPMSK_ITTXFEMSK               ((volatile unsigned int)0x00000010)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPMSK_INEPNMM                 ((volatile unsigned int)0x00000020)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPMSK_INEPNEM                 ((volatile unsigned int)0x00000040)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPMSK_TXFURM                  ((volatile unsigned int)0x00000100)            /*!< FIFO underrun mask */
#define USB_OTG_DIEPMSK_BIM                     ((volatile unsigned int)0x00000200)            /*!< BNA interrupt mask */

/********************  Bit definition forUSB_OTG_HPTXSTS register  ********************/
#define USB_OTG_HPTXSTS_PTXFSAVL                ((volatile unsigned int)0x0000FFFF)            /*!< Periodic transmit data FIFO space available */

#define USB_OTG_HPTXSTS_PTXQSAV                 ((volatile unsigned int)0x00FF0000)            /*!< Periodic transmit request queue space available */
#define USB_OTG_HPTXSTS_PTXQSAV_0               ((volatile unsigned int)0x00010000)            /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQSAV_1               ((volatile unsigned int)0x00020000)            /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQSAV_2               ((volatile unsigned int)0x00040000)            /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQSAV_3               ((volatile unsigned int)0x00080000)            /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQSAV_4               ((volatile unsigned int)0x00100000)            /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQSAV_5               ((volatile unsigned int)0x00200000)            /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQSAV_6               ((volatile unsigned int)0x00400000)            /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQSAV_7               ((volatile unsigned int)0x00800000)            /*!<Bit 7 */

#define USB_OTG_HPTXSTS_PTXQTOP                 ((volatile unsigned int)0xFF000000)            /*!< Top of the periodic transmit request queue */
#define USB_OTG_HPTXSTS_PTXQTOP_0               ((volatile unsigned int)0x01000000)            /*!<Bit 0 */
#define USB_OTG_HPTXSTS_PTXQTOP_1               ((volatile unsigned int)0x02000000)            /*!<Bit 1 */
#define USB_OTG_HPTXSTS_PTXQTOP_2               ((volatile unsigned int)0x04000000)            /*!<Bit 2 */
#define USB_OTG_HPTXSTS_PTXQTOP_3               ((volatile unsigned int)0x08000000)            /*!<Bit 3 */
#define USB_OTG_HPTXSTS_PTXQTOP_4               ((volatile unsigned int)0x10000000)            /*!<Bit 4 */
#define USB_OTG_HPTXSTS_PTXQTOP_5               ((volatile unsigned int)0x20000000)            /*!<Bit 5 */
#define USB_OTG_HPTXSTS_PTXQTOP_6               ((volatile unsigned int)0x40000000)            /*!<Bit 6 */
#define USB_OTG_HPTXSTS_PTXQTOP_7               ((volatile unsigned int)0x80000000)            /*!<Bit 7 */

/********************  Bit definition forUSB_OTG_HAINT register  ********************/
#define USB_OTG_HAINT_HAINT                   ((volatile unsigned int)0x0000FFFF)            /*!< Channel interrupts */

/********************  Bit definition forUSB_OTG_DOEPMSK register  ********************/
#define USB_OTG_DOEPMSK_XFRCM                   ((volatile unsigned int)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPMSK_EPDM                    ((volatile unsigned int)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPMSK_STUPM                   ((volatile unsigned int)0x00000008)            /*!< SETUP phase done mask */
#define USB_OTG_DOEPMSK_OTEPDM                  ((volatile unsigned int)0x00000010)            /*!< OUT token received when endpoint disabled mask */
#define USB_OTG_DOEPMSK_B2BSTUP                 ((volatile unsigned int)0x00000040)            /*!< Back-to-back SETUP packets received mask */
#define USB_OTG_DOEPMSK_OPEM                    ((volatile unsigned int)0x00000100)            /*!< OUT packet error mask */
#define USB_OTG_DOEPMSK_BOIM                    ((volatile unsigned int)0x00000200)            /*!< BNA interrupt mask */

/********************  Bit definition forUSB_OTG_GINTSTS register  ********************/
#define USB_OTG_GINTSTS_CMOD                    ((volatile unsigned int)0x00000001)            /*!< Current mode of operation */
#define USB_OTG_GINTSTS_MMIS                    ((volatile unsigned int)0x00000002)            /*!< Mode mismatch interrupt */
#define USB_OTG_GINTSTS_OTGINT                  ((volatile unsigned int)0x00000004)            /*!< OTG interrupt */
#define USB_OTG_GINTSTS_SOF                     ((volatile unsigned int)0x00000008)            /*!< Start of frame */
#define USB_OTG_GINTSTS_RXFLVL                  ((volatile unsigned int)0x00000010)            /*!< RxFIFO nonempty */
#define USB_OTG_GINTSTS_NPTXFE                  ((volatile unsigned int)0x00000020)            /*!< Nonperiodic TxFIFO empty */
#define USB_OTG_GINTSTS_GINAKEFF                ((volatile unsigned int)0x00000040)            /*!< Global IN nonperiodic NAK effective */
#define USB_OTG_GINTSTS_BOUTNAKEFF              ((volatile unsigned int)0x00000080)            /*!< Global OUT NAK effective */
#define USB_OTG_GINTSTS_ESUSP                   ((volatile unsigned int)0x00000400)            /*!< Early suspend */
#define USB_OTG_GINTSTS_USBSUSP                 ((volatile unsigned int)0x00000800)            /*!< USB suspend */
#define USB_OTG_GINTSTS_USBRST                  ((volatile unsigned int)0x00001000)            /*!< USB reset */
#define USB_OTG_GINTSTS_ENUMDNE                 ((volatile unsigned int)0x00002000)            /*!< Enumeration done */
#define USB_OTG_GINTSTS_ISOODRP                 ((volatile unsigned int)0x00004000)            /*!< Isochronous OUT packet dropped interrupt */
#define USB_OTG_GINTSTS_EOPF                    ((volatile unsigned int)0x00008000)            /*!< End of periodic frame interrupt */
#define USB_OTG_GINTSTS_IEPINT                  ((volatile unsigned int)0x00040000)            /*!< IN endpoint interrupt */
#define USB_OTG_GINTSTS_OEPINT                  ((volatile unsigned int)0x00080000)            /*!< OUT endpoint interrupt */
#define USB_OTG_GINTSTS_IISOIXFR                ((volatile unsigned int)0x00100000)            /*!< Incomplete isochronous IN transfer */
#define USB_OTG_GINTSTS_PXFR_INCOMPISOOUT       ((volatile unsigned int)0x00200000)            /*!< Incomplete periodic transfer */
#define USB_OTG_GINTSTS_DATAFSUSP               ((volatile unsigned int)0x00400000)            /*!< Data fetch suspended */
#define USB_OTG_GINTSTS_HPRTINT                 ((volatile unsigned int)0x01000000)            /*!< Host port interrupt */
#define USB_OTG_GINTSTS_HCINT                   ((volatile unsigned int)0x02000000)            /*!< Host channels interrupt */
#define USB_OTG_GINTSTS_PTXFE                   ((volatile unsigned int)0x04000000)            /*!< Periodic TxFIFO empty */
#define USB_OTG_GINTSTS_CIDSCHG                 ((volatile unsigned int)0x10000000)            /*!< Connector ID status change */
#define USB_OTG_GINTSTS_DISCINT                 ((volatile unsigned int)0x20000000)            /*!< Disconnect detected interrupt */
#define USB_OTG_GINTSTS_SRQINT                  ((volatile unsigned int)0x40000000)            /*!< Session request/new session detected interrupt */
#define USB_OTG_GINTSTS_WKUINT                  ((volatile unsigned int)0x80000000)            /*!< Resume/remote wakeup detected interrupt */

/********************  Bit definition forUSB_OTG_GINTMSK register  ********************/
#define USB_OTG_GINTMSK_MMISM                   ((volatile unsigned int)0x00000002)            /*!< Mode mismatch interrupt mask */
#define USB_OTG_GINTMSK_OTGINT                  ((volatile unsigned int)0x00000004)            /*!< OTG interrupt mask */
#define USB_OTG_GINTMSK_SOFM                    ((volatile unsigned int)0x00000008)            /*!< Start of frame mask */
#define USB_OTG_GINTMSK_RXFLVLM                 ((volatile unsigned int)0x00000010)            /*!< Receive FIFO nonempty mask */
#define USB_OTG_GINTMSK_NPTXFEM                 ((volatile unsigned int)0x00000020)            /*!< Nonperiodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_GINAKEFFM               ((volatile unsigned int)0x00000040)            /*!< Global nonperiodic IN NAK effective mask */
#define USB_OTG_GINTMSK_GONAKEFFM               ((volatile unsigned int)0x00000080)            /*!< Global OUT NAK effective mask */
#define USB_OTG_GINTMSK_ESUSPM                  ((volatile unsigned int)0x00000400)            /*!< Early suspend mask */
#define USB_OTG_GINTMSK_USBSUSPM                ((volatile unsigned int)0x00000800)            /*!< USB suspend mask */
#define USB_OTG_GINTMSK_USBRST                  ((volatile unsigned int)0x00001000)            /*!< USB reset mask */
#define USB_OTG_GINTMSK_ENUMDNEM                ((volatile unsigned int)0x00002000)            /*!< Enumeration done mask */
#define USB_OTG_GINTMSK_ISOODRPM                ((volatile unsigned int)0x00004000)            /*!< Isochronous OUT packet dropped interrupt mask */
#define USB_OTG_GINTMSK_EOPFM                   ((volatile unsigned int)0x00008000)            /*!< End of periodic frame interrupt mask */
#define USB_OTG_GINTMSK_EPMISM                  ((volatile unsigned int)0x00020000)            /*!< Endpoint mismatch interrupt mask */
#define USB_OTG_GINTMSK_IEPINT                  ((volatile unsigned int)0x00040000)            /*!< IN endpoints interrupt mask */
#define USB_OTG_GINTMSK_OEPINT                  ((volatile unsigned int)0x00080000)            /*!< OUT endpoints interrupt mask */
#define USB_OTG_GINTMSK_IISOIXFRM               ((volatile unsigned int)0x00100000)            /*!< Incomplete isochronous IN transfer mask */
#define USB_OTG_GINTMSK_PXFRM_IISOOXFRM         ((volatile unsigned int)0x00200000)            /*!< Incomplete periodic transfer mask */
#define USB_OTG_GINTMSK_FSUSPM                  ((volatile unsigned int)0x00400000)            /*!< Data fetch suspended mask */
#define USB_OTG_GINTMSK_PRTIM                   ((volatile unsigned int)0x01000000)            /*!< Host port interrupt mask */
#define USB_OTG_GINTMSK_HCIM                    ((volatile unsigned int)0x02000000)            /*!< Host channels interrupt mask */
#define USB_OTG_GINTMSK_PTXFEM                  ((volatile unsigned int)0x04000000)            /*!< Periodic TxFIFO empty mask */
#define USB_OTG_GINTMSK_CIDSCHGM                ((volatile unsigned int)0x10000000)            /*!< Connector ID status change mask */
#define USB_OTG_GINTMSK_DISCINT                 ((volatile unsigned int)0x20000000)            /*!< Disconnect detected interrupt mask */
#define USB_OTG_GINTMSK_SRQIM                   ((volatile unsigned int)0x40000000)            /*!< Session request/new session detected interrupt mask */
#define USB_OTG_GINTMSK_WUIM                    ((volatile unsigned int)0x80000000)            /*!< Resume/remote wakeup detected interrupt mask */

/********************  Bit definition forUSB_OTG_DAINT register  ********************/
#define USB_OTG_DAINT_IEPINT                  ((volatile unsigned int)0x0000FFFF)            /*!< IN endpoint interrupt bits */
#define USB_OTG_DAINT_OEPINT                  ((volatile unsigned int)0xFFFF0000)            /*!< OUT endpoint interrupt bits */

/********************  Bit definition forUSB_OTG_HAINTMSK register  ********************/
#define USB_OTG_HAINTMSK_HAINTM                  ((volatile unsigned int)0x0000FFFF)            /*!< Channel interrupt mask */

/********************  Bit definition for USB_OTG_GRXSTSP register  ********************/
#define USB_OTG_GRXSTSP_EPNUM                    ((volatile unsigned int)0x0000000F)            /*!< IN EP interrupt mask bits */
#define USB_OTG_GRXSTSP_BCNT                     ((volatile unsigned int)0x00007FF0)            /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_DPID                     ((volatile unsigned int)0x00018000)            /*!< OUT EP interrupt mask bits */
#define USB_OTG_GRXSTSP_PKTSTS                   ((volatile unsigned int)0x001E0000)            /*!< OUT EP interrupt mask bits */

/********************  Bit definition forUSB_OTG_DAINTMSK register  ********************/
#define USB_OTG_DAINTMSK_IEPM                    ((volatile unsigned int)0x0000FFFF)            /*!< IN EP interrupt mask bits */
#define USB_OTG_DAINTMSK_OEPM                    ((volatile unsigned int)0xFFFF0000)            /*!< OUT EP interrupt mask bits */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   ((volatile unsigned int)0x0000000F)            /*!< Channel number */
#define USB_OTG_CHNUM_0                 ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 ((volatile unsigned int)0x00000004)            /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 ((volatile unsigned int)0x00000008)            /*!<Bit 3 */
#define USB_OTG_BCNT                    ((volatile unsigned int)0x00007FF0)            /*!< Byte count */

#define USB_OTG_DPID                    ((volatile unsigned int)0x00018000)            /*!< Data PID */
#define USB_OTG_DPID_0                  ((volatile unsigned int)0x00008000)            /*!<Bit 0 */
#define USB_OTG_DPID_1                  ((volatile unsigned int)0x00010000)            /*!<Bit 1 */

#define USB_OTG_PKTSTS                  ((volatile unsigned int)0x001E0000)            /*!< Packet status */
#define USB_OTG_PKTSTS_0                ((volatile unsigned int)0x00020000)            /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                ((volatile unsigned int)0x00040000)            /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                ((volatile unsigned int)0x00080000)            /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                ((volatile unsigned int)0x00100000)            /*!<Bit 3 */

#define USB_OTG_EPNUM                   ((volatile unsigned int)0x0000000F)            /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 ((volatile unsigned int)0x00000004)            /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 ((volatile unsigned int)0x00000008)            /*!<Bit 3 */

#define USB_OTG_FRMNUM                  ((volatile unsigned int)0x01E00000)            /*!< Frame number */
#define USB_OTG_FRMNUM_0                ((volatile unsigned int)0x00200000)            /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                ((volatile unsigned int)0x00400000)            /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                ((volatile unsigned int)0x00800000)            /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                ((volatile unsigned int)0x01000000)            /*!<Bit 3 */

/********************  Bit definition for OTG register  ********************/

#define USB_OTG_CHNUM                   ((volatile unsigned int)0x0000000F)            /*!< Channel number */
#define USB_OTG_CHNUM_0                 ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_CHNUM_1                 ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_CHNUM_2                 ((volatile unsigned int)0x00000004)            /*!<Bit 2 */
#define USB_OTG_CHNUM_3                 ((volatile unsigned int)0x00000008)            /*!<Bit 3 */
#define USB_OTG_BCNT                    ((volatile unsigned int)0x00007FF0)            /*!< Byte count */

#define USB_OTG_DPID                    ((volatile unsigned int)0x00018000)            /*!< Data PID */
#define USB_OTG_DPID_0                  ((volatile unsigned int)0x00008000)            /*!<Bit 0 */
#define USB_OTG_DPID_1                  ((volatile unsigned int)0x00010000)            /*!<Bit 1 */

#define USB_OTG_PKTSTS                  ((volatile unsigned int)0x001E0000)            /*!< Packet status */
#define USB_OTG_PKTSTS_0                ((volatile unsigned int)0x00020000)            /*!<Bit 0 */
#define USB_OTG_PKTSTS_1                ((volatile unsigned int)0x00040000)            /*!<Bit 1 */
#define USB_OTG_PKTSTS_2                ((volatile unsigned int)0x00080000)            /*!<Bit 2 */
#define USB_OTG_PKTSTS_3                ((volatile unsigned int)0x00100000)            /*!<Bit 3 */

#define USB_OTG_EPNUM                   ((volatile unsigned int)0x0000000F)            /*!< Endpoint number */
#define USB_OTG_EPNUM_0                 ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_EPNUM_1                 ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_EPNUM_2                 ((volatile unsigned int)0x00000004)            /*!<Bit 2 */
#define USB_OTG_EPNUM_3                 ((volatile unsigned int)0x00000008)            /*!<Bit 3 */

#define USB_OTG_FRMNUM                  ((volatile unsigned int)0x01E00000)            /*!< Frame number */
#define USB_OTG_FRMNUM_0                ((volatile unsigned int)0x00200000)            /*!<Bit 0 */
#define USB_OTG_FRMNUM_1                ((volatile unsigned int)0x00400000)            /*!<Bit 1 */
#define USB_OTG_FRMNUM_2                ((volatile unsigned int)0x00800000)            /*!<Bit 2 */
#define USB_OTG_FRMNUM_3                ((volatile unsigned int)0x01000000)            /*!<Bit 3 */

/********************  Bit definition forUSB_OTG_GRXFSIZ register  ********************/
#define USB_OTG_GRXFSIZ_RXFD                    ((volatile unsigned int)0x0000FFFF)            /*!< RxFIFO depth */

/********************  Bit definition forUSB_OTG_DVBUSDIS register  ********************/
#define USB_OTG_DVBUSDIS_VBUSDT                  ((volatile unsigned int)0x0000FFFF)            /*!< Device VBUS discharge time */

/********************  Bit definition for OTG register  ********************/
#define USB_OTG_NPTXFSA                 ((volatile unsigned int)0x0000FFFF)            /*!< Nonperiodic transmit RAM start address */
#define USB_OTG_NPTXFD                  ((volatile unsigned int)0xFFFF0000)            /*!< Nonperiodic TxFIFO depth */
#define USB_OTG_TX0FSA                  ((volatile unsigned int)0x0000FFFF)            /*!< Endpoint 0 transmit RAM start address */
#define USB_OTG_TX0FD                   ((volatile unsigned int)0xFFFF0000)            /*!< Endpoint 0 TxFIFO depth */

/********************  Bit definition forUSB_OTG_DVBUSPULSE register  ********************/
#define USB_OTG_DVBUSPULSE_DVBUSP                  ((volatile unsigned int)0x00000FFF)            /*!< Device VBUS pulsing time */

/********************  Bit definition forUSB_OTG_GNPTXSTS register  ********************/
#define USB_OTG_GNPTXSTS_NPTXFSAV                ((volatile unsigned int)0x0000FFFF)            /*!< Nonperiodic TxFIFO space available */

#define USB_OTG_GNPTXSTS_NPTQXSAV                ((volatile unsigned int)0x00FF0000)            /*!< Nonperiodic transmit request queue space available */
#define USB_OTG_GNPTXSTS_NPTQXSAV_0              ((volatile unsigned int)0x00010000)            /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_1              ((volatile unsigned int)0x00020000)            /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_2              ((volatile unsigned int)0x00040000)            /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_3              ((volatile unsigned int)0x00080000)            /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_4              ((volatile unsigned int)0x00100000)            /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_5              ((volatile unsigned int)0x00200000)            /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_6              ((volatile unsigned int)0x00400000)            /*!<Bit 6 */
#define USB_OTG_GNPTXSTS_NPTQXSAV_7              ((volatile unsigned int)0x00800000)            /*!<Bit 7 */

#define USB_OTG_GNPTXSTS_NPTXQTOP                ((volatile unsigned int)0x7F000000)            /*!< Top of the nonperiodic transmit request queue */
#define USB_OTG_GNPTXSTS_NPTXQTOP_0              ((volatile unsigned int)0x01000000)            /*!<Bit 0 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_1              ((volatile unsigned int)0x02000000)            /*!<Bit 1 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_2              ((volatile unsigned int)0x04000000)            /*!<Bit 2 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_3              ((volatile unsigned int)0x08000000)            /*!<Bit 3 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_4              ((volatile unsigned int)0x10000000)            /*!<Bit 4 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_5              ((volatile unsigned int)0x20000000)            /*!<Bit 5 */
#define USB_OTG_GNPTXSTS_NPTXQTOP_6              ((volatile unsigned int)0x40000000)            /*!<Bit 6 */

/********************  Bit definition forUSB_OTG_DTHRCTL register  ********************/
#define USB_OTG_DTHRCTL_NONISOTHREN             ((volatile unsigned int)0x00000001)            /*!< Nonisochronous IN endpoints threshold enable */
#define USB_OTG_DTHRCTL_ISOTHREN                ((volatile unsigned int)0x00000002)            /*!< ISO IN endpoint threshold enable */

#define USB_OTG_DTHRCTL_TXTHRLEN                ((volatile unsigned int)0x000007FC)            /*!< Transmit threshold length */
#define USB_OTG_DTHRCTL_TXTHRLEN_0              ((volatile unsigned int)0x00000004)            /*!<Bit 0 */
#define USB_OTG_DTHRCTL_TXTHRLEN_1              ((volatile unsigned int)0x00000008)            /*!<Bit 1 */
#define USB_OTG_DTHRCTL_TXTHRLEN_2              ((volatile unsigned int)0x00000010)            /*!<Bit 2 */
#define USB_OTG_DTHRCTL_TXTHRLEN_3              ((volatile unsigned int)0x00000020)            /*!<Bit 3 */
#define USB_OTG_DTHRCTL_TXTHRLEN_4              ((volatile unsigned int)0x00000040)            /*!<Bit 4 */
#define USB_OTG_DTHRCTL_TXTHRLEN_5              ((volatile unsigned int)0x00000080)            /*!<Bit 5 */
#define USB_OTG_DTHRCTL_TXTHRLEN_6              ((volatile unsigned int)0x00000100)            /*!<Bit 6 */
#define USB_OTG_DTHRCTL_TXTHRLEN_7              ((volatile unsigned int)0x00000200)            /*!<Bit 7 */
#define USB_OTG_DTHRCTL_TXTHRLEN_8              ((volatile unsigned int)0x00000400)            /*!<Bit 8 */
#define USB_OTG_DTHRCTL_RXTHREN                 ((volatile unsigned int)0x00010000)            /*!< Receive threshold enable */

#define USB_OTG_DTHRCTL_RXTHRLEN                ((volatile unsigned int)0x03FE0000)            /*!< Receive threshold length */
#define USB_OTG_DTHRCTL_RXTHRLEN_0              ((volatile unsigned int)0x00020000)            /*!<Bit 0 */
#define USB_OTG_DTHRCTL_RXTHRLEN_1              ((volatile unsigned int)0x00040000)            /*!<Bit 1 */
#define USB_OTG_DTHRCTL_RXTHRLEN_2              ((volatile unsigned int)0x00080000)            /*!<Bit 2 */
#define USB_OTG_DTHRCTL_RXTHRLEN_3              ((volatile unsigned int)0x00100000)            /*!<Bit 3 */
#define USB_OTG_DTHRCTL_RXTHRLEN_4              ((volatile unsigned int)0x00200000)            /*!<Bit 4 */
#define USB_OTG_DTHRCTL_RXTHRLEN_5              ((volatile unsigned int)0x00400000)            /*!<Bit 5 */
#define USB_OTG_DTHRCTL_RXTHRLEN_6              ((volatile unsigned int)0x00800000)            /*!<Bit 6 */
#define USB_OTG_DTHRCTL_RXTHRLEN_7              ((volatile unsigned int)0x01000000)            /*!<Bit 7 */
#define USB_OTG_DTHRCTL_RXTHRLEN_8              ((volatile unsigned int)0x02000000)            /*!<Bit 8 */
#define USB_OTG_DTHRCTL_ARPEN                   ((volatile unsigned int)0x08000000)            /*!< Arbiter parking enable */

/********************  Bit definition forUSB_OTG_DIEPEMPMSK register  ********************/
#define USB_OTG_DIEPEMPMSK_INEPTXFEM               ((volatile unsigned int)0x0000FFFF)            /*!< IN EP Tx FIFO empty interrupt mask bits */

/********************  Bit definition forUSB_OTG_DEACHINT register  ********************/
#define USB_OTG_DEACHINT_IEP1INT                 ((volatile unsigned int)0x00000002)            /*!< IN endpoint 1interrupt bit */
#define USB_OTG_DEACHINT_OEP1INT                 ((volatile unsigned int)0x00020000)            /*!< OUT endpoint 1 interrupt bit */

/********************  Bit definition forUSB_OTG_GCCFG register  ********************/
#define USB_OTG_GCCFG_PWRDWN                  ((volatile unsigned int)0x00010000)            /*!< Power down */
#define USB_OTG_GCCFG_I2CPADEN                ((volatile unsigned int)0x00020000)            /*!< Enable I2C bus connection for the external I2C PHY interface */
#define USB_OTG_GCCFG_VBUSASEN                ((volatile unsigned int)0x00040000)            /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_VBUSBSEN                ((volatile unsigned int)0x00080000)            /*!< Enable the VBUS sensing device */
#define USB_OTG_GCCFG_SOFOUTEN                ((volatile unsigned int)0x00100000)            /*!< SOF output enable */
#define USB_OTG_GCCFG_NOVBUSSENS              ((volatile unsigned int)0x00200000)            /*!< VBUS sensing disable option */

/********************  Bit definition forUSB_OTG_DEACHINTMSK register  ********************/
#define USB_OTG_DEACHINTMSK_IEP1INTM                ((volatile unsigned int)0x00000002)            /*!< IN Endpoint 1 interrupt mask bit */
#define USB_OTG_DEACHINTMSK_OEP1INTM                ((volatile unsigned int)0x00020000)            /*!< OUT Endpoint 1 interrupt mask bit */

/********************  Bit definition forUSB_OTG_CID register  ********************/
#define USB_OTG_CID_PRODUCT_ID              ((volatile unsigned int)0xFFFFFFFF)            /*!< Product ID field */

/********************  Bit definition forUSB_OTG_DIEPEACHMSK1 register  ********************/
#define USB_OTG_DIEPEACHMSK1_XFRCM                   ((volatile unsigned int)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DIEPEACHMSK1_EPDM                    ((volatile unsigned int)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DIEPEACHMSK1_TOM                     ((volatile unsigned int)0x00000008)            /*!< Timeout condition mask (nonisochronous endpoints) */
#define USB_OTG_DIEPEACHMSK1_ITTXFEMSK               ((volatile unsigned int)0x00000010)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DIEPEACHMSK1_INEPNMM                 ((volatile unsigned int)0x00000020)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DIEPEACHMSK1_INEPNEM                 ((volatile unsigned int)0x00000040)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DIEPEACHMSK1_TXFURM                  ((volatile unsigned int)0x00000100)            /*!< FIFO underrun mask */
#define USB_OTG_DIEPEACHMSK1_BIM                     ((volatile unsigned int)0x00000200)            /*!< BNA interrupt mask */
#define USB_OTG_DIEPEACHMSK1_NAKM                    ((volatile unsigned int)0x00002000)            /*!< NAK interrupt mask */

/********************  Bit definition forUSB_OTG_HPRT register  ********************/
#define USB_OTG_HPRT_PCSTS                   ((volatile unsigned int)0x00000001)            /*!< Port connect status */
#define USB_OTG_HPRT_PCDET                   ((volatile unsigned int)0x00000002)            /*!< Port connect detected */
#define USB_OTG_HPRT_PENA                    ((volatile unsigned int)0x00000004)            /*!< Port enable */
#define USB_OTG_HPRT_PENCHNG                 ((volatile unsigned int)0x00000008)            /*!< Port enable/disable change */
#define USB_OTG_HPRT_POCA                    ((volatile unsigned int)0x00000010)            /*!< Port overcurrent active */
#define USB_OTG_HPRT_POCCHNG                 ((volatile unsigned int)0x00000020)            /*!< Port overcurrent change */
#define USB_OTG_HPRT_PRES                    ((volatile unsigned int)0x00000040)            /*!< Port resume */
#define USB_OTG_HPRT_PSUSP                   ((volatile unsigned int)0x00000080)            /*!< Port suspend */
#define USB_OTG_HPRT_PRST                    ((volatile unsigned int)0x00000100)            /*!< Port reset */

#define USB_OTG_HPRT_PLSTS                   ((volatile unsigned int)0x00000C00)            /*!< Port line status */
#define USB_OTG_HPRT_PLSTS_0                 ((volatile unsigned int)0x00000400)            /*!<Bit 0 */
#define USB_OTG_HPRT_PLSTS_1                 ((volatile unsigned int)0x00000800)            /*!<Bit 1 */
#define USB_OTG_HPRT_PPWR                    ((volatile unsigned int)0x00001000)            /*!< Port power */

#define USB_OTG_HPRT_PTCTL                   ((volatile unsigned int)0x0001E000)            /*!< Port test control */
#define USB_OTG_HPRT_PTCTL_0                 ((volatile unsigned int)0x00002000)            /*!<Bit 0 */
#define USB_OTG_HPRT_PTCTL_1                 ((volatile unsigned int)0x00004000)            /*!<Bit 1 */
#define USB_OTG_HPRT_PTCTL_2                 ((volatile unsigned int)0x00008000)            /*!<Bit 2 */
#define USB_OTG_HPRT_PTCTL_3                 ((volatile unsigned int)0x00010000)            /*!<Bit 3 */

#define USB_OTG_HPRT_PSPD                    ((volatile unsigned int)0x00060000)            /*!< Port speed */
#define USB_OTG_HPRT_PSPD_0                  ((volatile unsigned int)0x00020000)            /*!<Bit 0 */
#define USB_OTG_HPRT_PSPD_1                  ((volatile unsigned int)0x00040000)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_DOEPEACHMSK1 register  ********************/
#define USB_OTG_DOEPEACHMSK1_XFRCM                   ((volatile unsigned int)0x00000001)            /*!< Transfer completed interrupt mask */
#define USB_OTG_DOEPEACHMSK1_EPDM                    ((volatile unsigned int)0x00000002)            /*!< Endpoint disabled interrupt mask */
#define USB_OTG_DOEPEACHMSK1_TOM                     ((volatile unsigned int)0x00000008)            /*!< Timeout condition mask */
#define USB_OTG_DOEPEACHMSK1_ITTXFEMSK               ((volatile unsigned int)0x00000010)            /*!< IN token received when TxFIFO empty mask */
#define USB_OTG_DOEPEACHMSK1_INEPNMM                 ((volatile unsigned int)0x00000020)            /*!< IN token received with EP mismatch mask */
#define USB_OTG_DOEPEACHMSK1_INEPNEM                 ((volatile unsigned int)0x00000040)            /*!< IN endpoint NAK effective mask */
#define USB_OTG_DOEPEACHMSK1_TXFURM                  ((volatile unsigned int)0x00000100)            /*!< OUT packet error mask */
#define USB_OTG_DOEPEACHMSK1_BIM                     ((volatile unsigned int)0x00000200)            /*!< BNA interrupt mask */
#define USB_OTG_DOEPEACHMSK1_BERRM                   ((volatile unsigned int)0x00001000)            /*!< Bubble error interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NAKM                    ((volatile unsigned int)0x00002000)            /*!< NAK interrupt mask */
#define USB_OTG_DOEPEACHMSK1_NYETM                   ((volatile unsigned int)0x00004000)            /*!< NYET interrupt mask */

/********************  Bit definition forUSB_OTG_HPTXFSIZ register  ********************/
#define USB_OTG_HPTXFSIZ_PTXSA                   ((volatile unsigned int)0x0000FFFF)            /*!< Host periodic TxFIFO start address */
#define USB_OTG_HPTXFSIZ_PTXFD                   ((volatile unsigned int)0xFFFF0000)            /*!< Host periodic TxFIFO depth */

/********************  Bit definition forUSB_OTG_DIEPCTL register  ********************/
#define USB_OTG_DIEPCTL_MPSIZ                   ((volatile unsigned int)0x000007FF)            /*!< Maximum packet size */
#define USB_OTG_DIEPCTL_USBAEP                  ((volatile unsigned int)0x00008000)            /*!< USB active endpoint */
#define USB_OTG_DIEPCTL_EONUM_DPID              ((volatile unsigned int)0x00010000)            /*!< Even/odd frame */
#define USB_OTG_DIEPCTL_NAKSTS                  ((volatile unsigned int)0x00020000)            /*!< NAK status */

#define USB_OTG_DIEPCTL_EPTYP                   ((volatile unsigned int)0x000C0000)            /*!< Endpoint type */
#define USB_OTG_DIEPCTL_EPTYP_0                 ((volatile unsigned int)0x00040000)            /*!<Bit 0 */
#define USB_OTG_DIEPCTL_EPTYP_1                 ((volatile unsigned int)0x00080000)            /*!<Bit 1 */
#define USB_OTG_DIEPCTL_STALL                   ((volatile unsigned int)0x00200000)            /*!< STALL handshake */

#define USB_OTG_DIEPCTL_TXFNUM                  ((volatile unsigned int)0x03C00000)            /*!< TxFIFO number */
#define USB_OTG_DIEPCTL_TXFNUM_0                ((volatile unsigned int)0x00400000)            /*!<Bit 0 */
#define USB_OTG_DIEPCTL_TXFNUM_1                ((volatile unsigned int)0x00800000)            /*!<Bit 1 */
#define USB_OTG_DIEPCTL_TXFNUM_2                ((volatile unsigned int)0x01000000)            /*!<Bit 2 */
#define USB_OTG_DIEPCTL_TXFNUM_3                ((volatile unsigned int)0x02000000)            /*!<Bit 3 */
#define USB_OTG_DIEPCTL_CNAK                    ((volatile unsigned int)0x04000000)            /*!< Clear NAK */
#define USB_OTG_DIEPCTL_SNAK                    ((volatile unsigned int)0x08000000)            /*!< Set NAK */
#define USB_OTG_DIEPCTL_SD0PID_SEVNFRM          ((volatile unsigned int)0x10000000)            /*!< Set DATA0 PID */
#define USB_OTG_DIEPCTL_SODDFRM                 ((volatile unsigned int)0x20000000)            /*!< Set odd frame */
#define USB_OTG_DIEPCTL_EPDIS                   ((volatile unsigned int)0x40000000)            /*!< Endpoint disable */
#define USB_OTG_DIEPCTL_EPENA                   ((volatile unsigned int)0x80000000)            /*!< Endpoint enable */

/********************  Bit definition forUSB_OTG_HCCHAR register  ********************/
#define USB_OTG_HCCHAR_MPSIZ                   ((volatile unsigned int)0x000007FF)            /*!< Maximum packet size */

#define USB_OTG_HCCHAR_EPNUM                   ((volatile unsigned int)0x00007800)            /*!< Endpoint number */
#define USB_OTG_HCCHAR_EPNUM_0                 ((volatile unsigned int)0x00000800)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPNUM_1                 ((volatile unsigned int)0x00001000)            /*!<Bit 1 */
#define USB_OTG_HCCHAR_EPNUM_2                 ((volatile unsigned int)0x00002000)            /*!<Bit 2 */
#define USB_OTG_HCCHAR_EPNUM_3                 ((volatile unsigned int)0x00004000)            /*!<Bit 3 */
#define USB_OTG_HCCHAR_EPDIR                   ((volatile unsigned int)0x00008000)            /*!< Endpoint direction */
#define USB_OTG_HCCHAR_LSDEV                   ((volatile unsigned int)0x00020000)            /*!< Low-speed device */

#define USB_OTG_HCCHAR_EPTYP                   ((volatile unsigned int)0x000C0000)            /*!< Endpoint type */
#define USB_OTG_HCCHAR_EPTYP_0                 ((volatile unsigned int)0x00040000)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_EPTYP_1                 ((volatile unsigned int)0x00080000)            /*!<Bit 1 */

#define USB_OTG_HCCHAR_MC                      ((volatile unsigned int)0x00300000)            /*!< Multi Count (MC) / Error Count (EC) */
#define USB_OTG_HCCHAR_MC_0                    ((volatile unsigned int)0x00100000)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_MC_1                    ((volatile unsigned int)0x00200000)            /*!<Bit 1 */

#define USB_OTG_HCCHAR_DAD                     ((volatile unsigned int)0x1FC00000)            /*!< Device address */
#define USB_OTG_HCCHAR_DAD_0                   ((volatile unsigned int)0x00400000)            /*!<Bit 0 */
#define USB_OTG_HCCHAR_DAD_1                   ((volatile unsigned int)0x00800000)            /*!<Bit 1 */
#define USB_OTG_HCCHAR_DAD_2                   ((volatile unsigned int)0x01000000)            /*!<Bit 2 */
#define USB_OTG_HCCHAR_DAD_3                   ((volatile unsigned int)0x02000000)            /*!<Bit 3 */
#define USB_OTG_HCCHAR_DAD_4                   ((volatile unsigned int)0x04000000)            /*!<Bit 4 */
#define USB_OTG_HCCHAR_DAD_5                   ((volatile unsigned int)0x08000000)            /*!<Bit 5 */
#define USB_OTG_HCCHAR_DAD_6                   ((volatile unsigned int)0x10000000)            /*!<Bit 6 */
#define USB_OTG_HCCHAR_ODDFRM                  ((volatile unsigned int)0x20000000)            /*!< Odd frame */
#define USB_OTG_HCCHAR_CHDIS                   ((volatile unsigned int)0x40000000)            /*!< Channel disable */
#define USB_OTG_HCCHAR_CHENA                   ((volatile unsigned int)0x80000000)            /*!< Channel enable */

/********************  Bit definition forUSB_OTG_HCSPLT register  ********************/

#define USB_OTG_HCSPLT_PRTADDR                 ((volatile unsigned int)0x0000007F)            /*!< Port address */
#define USB_OTG_HCSPLT_PRTADDR_0               ((volatile unsigned int)0x00000001)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_PRTADDR_1               ((volatile unsigned int)0x00000002)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_PRTADDR_2               ((volatile unsigned int)0x00000004)            /*!<Bit 2 */
#define USB_OTG_HCSPLT_PRTADDR_3               ((volatile unsigned int)0x00000008)            /*!<Bit 3 */
#define USB_OTG_HCSPLT_PRTADDR_4               ((volatile unsigned int)0x00000010)            /*!<Bit 4 */
#define USB_OTG_HCSPLT_PRTADDR_5               ((volatile unsigned int)0x00000020)            /*!<Bit 5 */
#define USB_OTG_HCSPLT_PRTADDR_6               ((volatile unsigned int)0x00000040)            /*!<Bit 6 */

#define USB_OTG_HCSPLT_HUBADDR                 ((volatile unsigned int)0x00003F80)            /*!< Hub address */
#define USB_OTG_HCSPLT_HUBADDR_0               ((volatile unsigned int)0x00000080)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_HUBADDR_1               ((volatile unsigned int)0x00000100)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_HUBADDR_2               ((volatile unsigned int)0x00000200)            /*!<Bit 2 */
#define USB_OTG_HCSPLT_HUBADDR_3               ((volatile unsigned int)0x00000400)            /*!<Bit 3 */
#define USB_OTG_HCSPLT_HUBADDR_4               ((volatile unsigned int)0x00000800)            /*!<Bit 4 */
#define USB_OTG_HCSPLT_HUBADDR_5               ((volatile unsigned int)0x00001000)            /*!<Bit 5 */
#define USB_OTG_HCSPLT_HUBADDR_6               ((volatile unsigned int)0x00002000)            /*!<Bit 6 */

#define USB_OTG_HCSPLT_XACTPOS                 ((volatile unsigned int)0x0000C000)            /*!< XACTPOS */
#define USB_OTG_HCSPLT_XACTPOS_0               ((volatile unsigned int)0x00004000)            /*!<Bit 0 */
#define USB_OTG_HCSPLT_XACTPOS_1               ((volatile unsigned int)0x00008000)            /*!<Bit 1 */
#define USB_OTG_HCSPLT_COMPLSPLT               ((volatile unsigned int)0x00010000)            /*!< Do complete split */
#define USB_OTG_HCSPLT_SPLITEN                 ((volatile unsigned int)0x80000000)            /*!< Split enable */

/********************  Bit definition forUSB_OTG_HCINT register  ********************/
#define USB_OTG_HCINT_XFRC                    ((volatile unsigned int)0x00000001)            /*!< Transfer completed */
#define USB_OTG_HCINT_CHH                     ((volatile unsigned int)0x00000002)            /*!< Channel halted */
#define USB_OTG_HCINT_AHBERR                  ((volatile unsigned int)0x00000004)            /*!< AHB error */
#define USB_OTG_HCINT_STALL                   ((volatile unsigned int)0x00000008)            /*!< STALL response received interrupt */
#define USB_OTG_HCINT_NAK                     ((volatile unsigned int)0x00000010)            /*!< NAK response received interrupt */
#define USB_OTG_HCINT_ACK                     ((volatile unsigned int)0x00000020)            /*!< ACK response received/transmitted interrupt */
#define USB_OTG_HCINT_NYET                    ((volatile unsigned int)0x00000040)            /*!< Response received interrupt */
#define USB_OTG_HCINT_TXERR                   ((volatile unsigned int)0x00000080)            /*!< Transaction error */
#define USB_OTG_HCINT_BBERR                   ((volatile unsigned int)0x00000100)            /*!< Babble error */
#define USB_OTG_HCINT_FRMOR                   ((volatile unsigned int)0x00000200)            /*!< Frame overrun */
#define USB_OTG_HCINT_DTERR                   ((volatile unsigned int)0x00000400)            /*!< Data toggle error */

/********************  Bit definition forUSB_OTG_DIEPINT register  ********************/
#define USB_OTG_DIEPINT_XFRC                    ((volatile unsigned int)0x00000001)            /*!< Transfer completed interrupt */
#define USB_OTG_DIEPINT_EPDISD                  ((volatile unsigned int)0x00000002)            /*!< Endpoint disabled interrupt */
#define USB_OTG_DIEPINT_TOC                     ((volatile unsigned int)0x00000008)            /*!< Timeout condition */
#define USB_OTG_DIEPINT_ITTXFE                  ((volatile unsigned int)0x00000010)            /*!< IN token received when TxFIFO is empty */
#define USB_OTG_DIEPINT_INEPNE                  ((volatile unsigned int)0x00000040)            /*!< IN endpoint NAK effective */
#define USB_OTG_DIEPINT_TXFE                    ((volatile unsigned int)0x00000080)            /*!< Transmit FIFO empty */
#define USB_OTG_DIEPINT_TXFIFOUDRN              ((volatile unsigned int)0x00000100)            /*!< Transmit Fifo Underrun */
#define USB_OTG_DIEPINT_BNA                     ((volatile unsigned int)0x00000200)            /*!< Buffer not available interrupt */
#define USB_OTG_DIEPINT_PKTDRPSTS               ((volatile unsigned int)0x00000800)            /*!< Packet dropped status */
#define USB_OTG_DIEPINT_BERR                    ((volatile unsigned int)0x00001000)            /*!< Babble error interrupt */
#define USB_OTG_DIEPINT_NAK                     ((volatile unsigned int)0x00002000)            /*!< NAK interrupt */

/********************  Bit definition forUSB_OTG_HCINTMSK register  ********************/
#define USB_OTG_HCINTMSK_XFRCM                   ((volatile unsigned int)0x00000001)            /*!< Transfer completed mask */
#define USB_OTG_HCINTMSK_CHHM                    ((volatile unsigned int)0x00000002)            /*!< Channel halted mask */
#define USB_OTG_HCINTMSK_AHBERR                  ((volatile unsigned int)0x00000004)            /*!< AHB error */
#define USB_OTG_HCINTMSK_STALLM                  ((volatile unsigned int)0x00000008)            /*!< STALL response received interrupt mask */
#define USB_OTG_HCINTMSK_NAKM                    ((volatile unsigned int)0x00000010)            /*!< NAK response received interrupt mask */
#define USB_OTG_HCINTMSK_ACKM                    ((volatile unsigned int)0x00000020)            /*!< ACK response received/transmitted interrupt mask */
#define USB_OTG_HCINTMSK_NYET                    ((volatile unsigned int)0x00000040)            /*!< response received interrupt mask */
#define USB_OTG_HCINTMSK_TXERRM                  ((volatile unsigned int)0x00000080)            /*!< Transaction error mask */
#define USB_OTG_HCINTMSK_BBERRM                  ((volatile unsigned int)0x00000100)            /*!< Babble error mask */
#define USB_OTG_HCINTMSK_FRMORM                  ((volatile unsigned int)0x00000200)            /*!< Frame overrun mask */
#define USB_OTG_HCINTMSK_DTERRM                  ((volatile unsigned int)0x00000400)            /*!< Data toggle error mask */

/********************  Bit definition for USB_OTG_DIEPTSIZ register  ********************/

#define USB_OTG_DIEPTSIZ_XFRSIZ                  ((volatile unsigned int)0x0007FFFF)            /*!< Transfer size */
#define USB_OTG_DIEPTSIZ_PKTCNT                  ((volatile unsigned int)0x1FF80000)            /*!< Packet count */
#define USB_OTG_DIEPTSIZ_MULCNT                  ((volatile unsigned int)0x60000000)            /*!< Packet count */
/********************  Bit definition forUSB_OTG_HCTSIZ register  ********************/
#define USB_OTG_HCTSIZ_XFRSIZ                    ((volatile unsigned int)0x0007FFFF)            /*!< Transfer size */
#define USB_OTG_HCTSIZ_PKTCNT                    ((volatile unsigned int)0x1FF80000)            /*!< Packet count */
#define USB_OTG_HCTSIZ_DOPING                    ((volatile unsigned int)0x80000000)            /*!< Do PING */
#define USB_OTG_HCTSIZ_DPID                      ((volatile unsigned int)0x60000000)            /*!< Data PID */
#define USB_OTG_HCTSIZ_DPID_0                    ((volatile unsigned int)0x20000000)            /*!<Bit 0 */
#define USB_OTG_HCTSIZ_DPID_1                    ((volatile unsigned int)0x40000000)            /*!<Bit 1 */

/********************  Bit definition forUSB_OTG_DIEPDMA register  ********************/
#define USB_OTG_DIEPDMA_DMAADDR                  ((volatile unsigned int)0xFFFFFFFF)            /*!< DMA address */

/********************  Bit definition forUSB_OTG_HCDMA register  ********************/
#define USB_OTG_HCDMA_DMAADDR                    ((volatile unsigned int)0xFFFFFFFF)            /*!< DMA address */

/********************  Bit definition forUSB_OTG_DTXFSTS register  ********************/
#define USB_OTG_DTXFSTS_INEPTFSAV                ((volatile unsigned int)0x0000FFFF)            /*!< IN endpoint TxFIFO space avail */

/********************  Bit definition forUSB_OTG_DIEPTXF register  ********************/
#define USB_OTG_DIEPTXF_INEPTXSA                 ((volatile unsigned int)0x0000FFFF)            /*!< IN endpoint FIFOx transmit RAM start address */
#define USB_OTG_DIEPTXF_INEPTXFD                 ((volatile unsigned int)0xFFFF0000)            /*!< IN endpoint TxFIFO depth */

/********************  Bit definition forUSB_OTG_DOEPCTL register  ********************/

#define USB_OTG_DOEPCTL_MPSIZ                     ((volatile unsigned int)0x000007FF)            /*!< Maximum packet size */          /*!<Bit 1 */
#define USB_OTG_DOEPCTL_USBAEP                    ((volatile unsigned int)0x00008000)            /*!< USB active endpoint */
#define USB_OTG_DOEPCTL_NAKSTS                    ((volatile unsigned int)0x00020000)            /*!< NAK status */
#define USB_OTG_DOEPCTL_SD0PID_SEVNFRM            ((volatile unsigned int)0x10000000)            /*!< Set DATA0 PID */
#define USB_OTG_DOEPCTL_SODDFRM                   ((volatile unsigned int)0x20000000)            /*!< Set odd frame */
#define USB_OTG_DOEPCTL_EPTYP                     ((volatile unsigned int)0x000C0000)            /*!< Endpoint type */
#define USB_OTG_DOEPCTL_EPTYP_0                   ((volatile unsigned int)0x00040000)            /*!<Bit 0 */
#define USB_OTG_DOEPCTL_EPTYP_1                   ((volatile unsigned int)0x00080000)            /*!<Bit 1 */
#define USB_OTG_DOEPCTL_SNPM                      ((volatile unsigned int)0x00100000)            /*!< Snoop mode */
#define USB_OTG_DOEPCTL_STALL                     ((volatile unsigned int)0x00200000)            /*!< STALL handshake */
#define USB_OTG_DOEPCTL_CNAK                      ((volatile unsigned int)0x04000000)            /*!< Clear NAK */
#define USB_OTG_DOEPCTL_SNAK                      ((volatile unsigned int)0x08000000)            /*!< Set NAK */
#define USB_OTG_DOEPCTL_EPDIS                     ((volatile unsigned int)0x40000000)            /*!< Endpoint disable */
#define USB_OTG_DOEPCTL_EPENA                     ((volatile unsigned int)0x80000000)            /*!< Endpoint enable */

/********************  Bit definition forUSB_OTG_DOEPINT register  ********************/
#define USB_OTG_DOEPINT_XFRC                    ((volatile unsigned int)0x00000001)            /*!< Transfer completed interrupt */
#define USB_OTG_DOEPINT_EPDISD                  ((volatile unsigned int)0x00000002)            /*!< Endpoint disabled interrupt */
#define USB_OTG_DOEPINT_STUP                    ((volatile unsigned int)0x00000008)            /*!< SETUP phase done */
#define USB_OTG_DOEPINT_OTEPDIS                 ((volatile unsigned int)0x00000010)            /*!< OUT token received when endpoint disabled */
#define USB_OTG_DOEPINT_B2BSTUP                 ((volatile unsigned int)0x00000040)            /*!< Back-to-back SETUP packets received */
#define USB_OTG_DOEPINT_NYET                    ((volatile unsigned int)0x00004000)            /*!< NYET interrupt */

/********************  Bit definition forUSB_OTG_DOEPTSIZ register  ********************/

#define USB_OTG_DOEPTSIZ_XFRSIZ                  ((volatile unsigned int)0x0007FFFF)            /*!< Transfer size */
#define USB_OTG_DOEPTSIZ_PKTCNT                  ((volatile unsigned int)0x1FF80000)            /*!< Packet count */

#define USB_OTG_DOEPTSIZ_STUPCNT                 ((volatile unsigned int)0x60000000)            /*!< SETUP packet count */
#define USB_OTG_DOEPTSIZ_STUPCNT_0               ((volatile unsigned int)0x20000000)            /*!<Bit 0 */
#define USB_OTG_DOEPTSIZ_STUPCNT_1               ((volatile unsigned int)0x40000000)            /*!<Bit 1 */

/********************  Bit definition for PCGCCTL register  ********************/
#define USB_OTG_PCGCCTL_STOPCLK                 ((volatile unsigned int)0x00000001)            /*!< SETUP packet count */
#define USB_OTG_PCGCCTL_GATECLK                 ((volatile unsigned int)0x00000002)            /*!<Bit 0 */
#define USB_OTG_PCGCCTL_PHYSUSP                 ((volatile unsigned int)0x00000010)            /*!<Bit 1 */

/**
 * @}
 */ 

/**
 * @}
 */

/** @addtogroup Exported_macros
 * @{
 */

/******************************* ADC Instances ********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || \
		((INSTANCE) == ADC2) || \
		((INSTANCE) == ADC3))

/******************************* CAN Instances ********************************/
#define IS_CAN_ALL_INSTANCE(INSTANCE) (((INSTANCE) == CAN1) || \
		((INSTANCE) == CAN2))

/******************************* CRC Instances ********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/******************************* DAC Instances ********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC)

/******************************* DCMI Instances *******************************/
#define IS_DCMI_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DCMI)

/******************************** DMA Instances *******************************/
#define IS_DMA_STREAM_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Stream0) || \
		((INSTANCE) == DMA1_Stream1) || \
		((INSTANCE) == DMA1_Stream2) || \
		((INSTANCE) == DMA1_Stream3) || \
		((INSTANCE) == DMA1_Stream4) || \
		((INSTANCE) == DMA1_Stream5) || \
		((INSTANCE) == DMA1_Stream6) || \
		((INSTANCE) == DMA1_Stream7) || \
		((INSTANCE) == DMA2_Stream0) || \
		((INSTANCE) == DMA2_Stream1) || \
		((INSTANCE) == DMA2_Stream2) || \
		((INSTANCE) == DMA2_Stream3) || \
		((INSTANCE) == DMA2_Stream4) || \
		((INSTANCE) == DMA2_Stream5) || \
		((INSTANCE) == DMA2_Stream6) || \
		((INSTANCE) == DMA2_Stream7))

/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
		((INSTANCE) == GPIOB) || \
		((INSTANCE) == GPIOC) || \
		((INSTANCE) == GPIOD) || \
		((INSTANCE) == GPIOE) || \
		((INSTANCE) == GPIOF) || \
		((INSTANCE) == GPIOG) || \
		((INSTANCE) == GPIOH) || \
		((INSTANCE) == GPIOI))

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
		((INSTANCE) == I2C2) || \
		((INSTANCE) == I2C3))

/******************************** I2S Instances *******************************/
#define IS_I2S_INSTANCE(INSTANCE)  (((INSTANCE) == SPI2) || \
		((INSTANCE) == SPI3))

/*************************** I2S Extended Instances ***************************/
#define IS_I2S_INSTANCE_EXT(PERIPH)  (((INSTANCE) == SPI2)    || \
		((INSTANCE) == SPI3)    || \
		((INSTANCE) == I2S2ext) || \
		((INSTANCE) == I2S3ext))

/******************************* RNG Instances ********************************/
#define IS_RNG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RNG)

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
		((INSTANCE) == SPI2) || \
		((INSTANCE) == SPI3))

/*************************** SPI Extended Instances ***************************/
#define IS_SPI_ALL_INSTANCE_EXT(INSTANCE) (((INSTANCE) == SPI1)    || \
		((INSTANCE) == SPI2)    || \
		((INSTANCE) == SPI3)    || \
		((INSTANCE) == I2S2ext) || \
		((INSTANCE) == I2S3ext))

/****************** TIM Instances : All supported instances *******************/
#define IS_TIM_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)   || \
		((INSTANCE) == TIM2)   || \
		((INSTANCE) == TIM3)   || \
		((INSTANCE) == TIM4)   || \
		((INSTANCE) == TIM5)   || \
		((INSTANCE) == TIM6)   || \
		((INSTANCE) == TIM7)   || \
		((INSTANCE) == TIM8)   || \
		((INSTANCE) == TIM9)   || \
		((INSTANCE) == TIM10)  || \
		((INSTANCE) == TIM11)  || \
		((INSTANCE) == TIM12)  || \
		((INSTANCE) == TIM13)  || \
		((INSTANCE) == TIM14))

/************* TIM Instances : at least 1 capture/compare channel *************/
#define IS_TIM_CC1_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1)  || \
		((INSTANCE) == TIM2)  || \
		((INSTANCE) == TIM3)  || \
		((INSTANCE) == TIM4)  || \
		((INSTANCE) == TIM5)  || \
		((INSTANCE) == TIM8)  || \
		((INSTANCE) == TIM9)  || \
		((INSTANCE) == TIM10) || \
		((INSTANCE) == TIM11) || \
		((INSTANCE) == TIM12) || \
		((INSTANCE) == TIM13) || \
		((INSTANCE) == TIM14))

/************ TIM Instances : at least 2 capture/compare channels *************/
#define IS_TIM_CC2_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8) || \
		((INSTANCE) == TIM9) || \
		((INSTANCE) == TIM12))

/************ TIM Instances : at least 3 capture/compare channels *************/
#define IS_TIM_CC3_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8))

/************ TIM Instances : at least 4 capture/compare channels *************/
#define IS_TIM_CC4_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8))

/******************** TIM Instances : Advanced-control timers *****************/
#define IS_TIM_ADVANCED_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM8))

/******************* TIM Instances : Timer input XOR function *****************/
#define IS_TIM_XOR_INSTANCE(INSTANCE)   (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8))

/****************** TIM Instances : DMA requests generation (UDE) *************/
#define IS_TIM_DMA_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM6) || \
		((INSTANCE) == TIM7) || \
		((INSTANCE) == TIM8))

/************ TIM Instances : DMA requests generation (CCxDE) *****************/
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8))

/************ TIM Instances : DMA requests generation (COMDE) *****************/
#define IS_TIM_CCDMA_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8)) 

/******************** TIM Instances : DMA burst feature ***********************/
#define IS_TIM_DMABURST_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8))

/****** TIM Instances : master mode available (TIMx_CR2.MMS available )********/
#define IS_TIM_MASTER_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM6) || \
		((INSTANCE) == TIM7) || \
		((INSTANCE) == TIM8) || \
		((INSTANCE) == TIM9) || \
		((INSTANCE) == TIM12))

/*********** TIM Instances : Slave mode available (TIMx_SMCR available )*******/
#define IS_TIM_SLAVE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8) || \
		((INSTANCE) == TIM9) || \
		((INSTANCE) == TIM12))

/********************** TIM Instances : 32 bit Counter ************************/
#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)(((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM5))

/***************** TIM Instances : external trigger input availabe ************/
#define IS_TIM_ETR_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1) || \
		((INSTANCE) == TIM2) || \
		((INSTANCE) == TIM3) || \
		((INSTANCE) == TIM4) || \
		((INSTANCE) == TIM5) || \
		((INSTANCE) == TIM8))

/****************** TIM Instances : remapping capability **********************/
#define IS_TIM_REMAP_INSTANCE(INSTANCE) (((INSTANCE) == TIM2)  || \
		((INSTANCE) == TIM5)  || \
		((INSTANCE) == TIM11))

/******************* TIM Instances : output(s) available **********************/
#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
	((((INSTANCE) == TIM1) &&                  \
	  (((CHANNEL) == TIM_CHANNEL_1) ||          \
	   ((CHANNEL) == TIM_CHANNEL_2) ||          \
	   ((CHANNEL) == TIM_CHANNEL_3) ||          \
	   ((CHANNEL) == TIM_CHANNEL_4)))           \
	 ||                                         \
	 (((INSTANCE) == TIM2) &&                   \
	  (((CHANNEL) == TIM_CHANNEL_1) ||          \
	   ((CHANNEL) == TIM_CHANNEL_2) ||          \
	   ((CHANNEL) == TIM_CHANNEL_3) ||          \
	   ((CHANNEL) == TIM_CHANNEL_4)))           \
	 ||                                         \
	 (((INSTANCE) == TIM3) &&                   \
	  (((CHANNEL) == TIM_CHANNEL_1) ||          \
	   ((CHANNEL) == TIM_CHANNEL_2) ||          \
	   ((CHANNEL) == TIM_CHANNEL_3) ||          \
	   ((CHANNEL) == TIM_CHANNEL_4)))           \
	 ||                                         \
	 (((INSTANCE) == TIM4) &&                   \
	  (((CHANNEL) == TIM_CHANNEL_1) ||          \
	   ((CHANNEL) == TIM_CHANNEL_2) ||          \
	   ((CHANNEL) == TIM_CHANNEL_3) ||          \
	   ((CHANNEL) == TIM_CHANNEL_4)))           \
||                                         \
(((INSTANCE) == TIM5) &&                   \
 (((CHANNEL) == TIM_CHANNEL_1) ||          \
  ((CHANNEL) == TIM_CHANNEL_2) ||          \
  ((CHANNEL) == TIM_CHANNEL_3) ||          \
  ((CHANNEL) == TIM_CHANNEL_4)))           \
||                                         \
(((INSTANCE) == TIM8) &&                   \
 (((CHANNEL) == TIM_CHANNEL_1) ||          \
  ((CHANNEL) == TIM_CHANNEL_2) ||          \
  ((CHANNEL) == TIM_CHANNEL_3) ||          \
  ((CHANNEL) == TIM_CHANNEL_4)))           \
||                                         \
(((INSTANCE) == TIM9) &&                   \
 (((CHANNEL) == TIM_CHANNEL_1) ||          \
  ((CHANNEL) == TIM_CHANNEL_2)))           \
||                                         \
(((INSTANCE) == TIM10) &&                  \
 (((CHANNEL) == TIM_CHANNEL_1)))           \
||                                         \
(((INSTANCE) == TIM11) &&                  \
 (((CHANNEL) == TIM_CHANNEL_1)))           \
||                                         \
(((INSTANCE) == TIM12) &&                  \
 (((CHANNEL) == TIM_CHANNEL_1) ||          \
  ((CHANNEL) == TIM_CHANNEL_2)))           \
||                                         \
(((INSTANCE) == TIM13) &&                  \
 (((CHANNEL) == TIM_CHANNEL_1)))           \
||                                         \
(((INSTANCE) == TIM14) &&                  \
 (((CHANNEL) == TIM_CHANNEL_1))))

/************ TIM Instances : complementary output(s) available ***************/
#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
	((((INSTANCE) == TIM1) &&                    \
	  (((CHANNEL) == TIM_CHANNEL_1) ||           \
	   ((CHANNEL) == TIM_CHANNEL_2) ||           \
	   ((CHANNEL) == TIM_CHANNEL_3)))            \
	 ||                                          \
	 (((INSTANCE) == TIM8) &&                    \
	  (((CHANNEL) == TIM_CHANNEL_1) ||           \
	   ((CHANNEL) == TIM_CHANNEL_2) ||           \
	   ((CHANNEL) == TIM_CHANNEL_3))))

/******************** USART Instances : Synchronous mode **********************/
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
		((INSTANCE) == USART2) || \
		((INSTANCE) == USART3) || \
		((INSTANCE) == USART6))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
		((INSTANCE) == USART2) || \
		((INSTANCE) == USART3) || \
		((INSTANCE) == UART4)  || \
		((INSTANCE) == UART5)  || \
		((INSTANCE) == USART6))

/****************** UART Instances : Hardware Flow control ********************/
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
		((INSTANCE) == USART2) || \
		((INSTANCE) == USART3) || \
		((INSTANCE) == USART6))

/********************* UART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
		((INSTANCE) == USART2) || \
		((INSTANCE) == USART3) || \
		((INSTANCE) == USART6))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
		((INSTANCE) == USART2) || \
		((INSTANCE) == USART3) || \
		((INSTANCE) == UART4)  || \
		((INSTANCE) == UART5)  || \
		((INSTANCE) == USART6))     

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** WWDG Instances ********************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/******************************************************************************/
/*  For a painless codes migration between the STM32F4xx device product       */
/*  lines, the aliases defined below are put in place to overcome the         */
/*  differences in the interrupt handlers and IRQn definitions.               */
/*  No need to update developed interrupt code when moving across             */
/*  product lines within the same STM32F4 Family                              */
/******************************************************************************/

/* Aliases for RQn */
#define FMC_IRQn              FSMC_IRQn

/* Aliases for RQHandler */
#define FMC_IRQHandler        FSMC_IRQHandler

/**
 * @}
 */ 

/**
 * @}
 */

/**
 * @}
 */
#endif /* STM32F401_H */
