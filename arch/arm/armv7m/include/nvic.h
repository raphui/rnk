#ifndef NVIC_H
#define NVIC_H

enum
{
	/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
	nonmaskableInt_irq        = -14,    /*!< 2 Non Maskable Interrupt                                          */
	memorymanagement_irq      = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
	busfault_irq              = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
	usagefault_irq		  = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
	svcall_irq	          = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
	debugmonitor_irq          = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
	pendsv_irq                = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
	systick_irq               = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
};

void nvic_enable_interrupt(unsigned int num);
void nvic_disable_interrupt(unsigned int num);
void nvic_set_interrupt(unsigned int num);
void nvic_clear_interrupt(unsigned int num);
void nvic_set_priority_interrupt(int num, unsigned char priority);

#endif /* NVIC_H */
