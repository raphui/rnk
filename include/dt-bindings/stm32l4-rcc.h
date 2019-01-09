#ifndef _DT_BINDINGS_MFD_STM32L4_RCC_H
#define _DT_BINDINGS_MFD_STM32L4_RCC_H

/* AHB1 */
#define STM32L4_RCC_AHB1_DMA1	0
#define STM32L4_RCC_AHB1_DMA2	1
#define STM32L4_RCC_AHB1_FLASH	8
#define STM32L4_RCC_AHB1_CRC	12
#define STM32L4_RCC_AHB1_TSC	16

#define STM32L4_AHB1_RESET(bit) (STM32L4_RCC_AHB1_##bit + (0x10 * 8))
#define STM32L4_AHB1_CLOCK(bit) (STM32L4_RCC_AHB1_##bit)


/* AHB2 */
#define STM32L4_RCC_AHB2_GPIOA	0
#define STM32L4_RCC_AHB2_GPIOB	1
#define STM32L4_RCC_AHB2_GPIOC	2
#define STM32L4_RCC_AHB2_GPIOD	3
#define STM32L4_RCC_AHB2_GPIOE	4
#define STM32L4_RCC_AHB2_GPIOH	7
#define STM32L4_RCC_AHB2_ADC	13
#define STM32L4_RCC_AHB2_AES	16
#define STM32L4_RCC_AHB2_RNG	18

#define STM32L4_AHB2_RESET(bit)	(STM32L4_RCC_AHB2_##bit + (0x14 * 8))
#define STM32L4_AHB2_CLOCK(bit)	(STM32L4_RCC_AHB2_##bit + 0x20)

/* AHB3 */
#define STM32L4_RCC_AHB3_QSPI	8

#define STM32L4_AHB3_RESET(bit)	(STM32L4_RCC_AHB3_##bit + (0x18 * 8))
#define STM32L4_AHB3_CLOCK(bit)	(STM32L4_RCC_AHB3_##bit + 0x40)

/* APB1 */
#define STM32L4_RCC_APB1_TIM2	0
#define STM32L4_RCC_APB1_TIM3	1
#define STM32L4_RCC_APB1_TIM6	4
#define STM32L4_RCC_APB1_TIM7	5
#define STM32L4_RCC_APB1_LCD	9
#define STM32L4_RCC_APB1_RTC	10
#define STM32L4_RCC_APB1_WWDG	11
#define STM32L4_RCC_APB1_SPI2	14
#define STM32L4_RCC_APB1_SPI3	15
#define STM32L4_RCC_APB1_UART2	17
#define STM32L4_RCC_APB1_UART3	18
#define STM32L4_RCC_APB1_UART4	19
#define STM32L4_RCC_APB1_I2C1	21
#define STM32L4_RCC_APB1_I2C2	22
#define STM32L4_RCC_APB1_I2C3	23
#define STM32L4_RCC_APB1_CRS	24
#define STM32L4_RCC_APB1_CAN1	25
#define STM32L4_RCC_APB1_USBFS	26
#define STM32L4_RCC_APB1_PWR	28
#define STM32L4_RCC_APB1_DAC1	29
#define STM32L4_RCC_APB1_OPAMP	30
#define STM32L4_RCC_APB1_LPTIM1	31

#define STM32L4_APB1_RESET(bit)	(STM32L4_RCC_APB1_##bit + (0x20 * 8))
#define STM32L4_APB1_CLOCK(bit)	(STM32L4_RCC_APB1_##bit + 0x80)

/* APB1-2 */
#define STM32L4_RCC_APB1_2_LPUSART1	0
#define STM32L4_RCC_APB1_2_I2C4		1
#define STM32L4_RCC_APB1_2_SWPMI1	2
#define STM32L4_RCC_APB1_2_LPTIM2	5

#define STM32L4_APB1_2_RESET(bit)	(STM32L4_RCC_APB1_##bit + (0x24 * 8))
#define STM32L4_APB1_2_CLOCK(bit)	(STM32L4_RCC_APB1_##bit + 0xA0)

/* APB2 */
#define STM32L4_RCC_APB2_SYSCFG	0
#define STM32L4_RCC_APB2_FW	7
#define STM32L4_RCC_APB2_SDMMC1	10
#define STM32L4_RCC_APB2_TIM1	11
#define STM32L4_RCC_APB2_SPI1	12
#define STM32L4_RCC_APB2_USART1	14
#define STM32L4_RCC_APB2_TIM15	16
#define STM32L4_RCC_APB2_TIM16	17
#define STM32L4_RCC_APB2_SAI1	21
#define STM32L4_RCC_APB2_DFSDM1	24

#define STM32L4_APB2_RESET(bit)	(STM32L4_RCC_APB2_##bit + (0x28 * 8))
#define STM32L4_APB2_CLOCK(bit)	(STM32L4_RCC_APB2_##bit + 0xC0)

#endif /* _DT_BINDINGS_MFD_STM32L4_RCC_H */
