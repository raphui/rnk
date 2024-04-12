#ifndef PWR_STM32_H
#define PWR_STM32_H

enum {
	STOP0_MODE,
	STOP1_MODE,
	STOP2_MODE,
};

void stm32_pwr_enter_lpsleep(int mode);

#endif /* PWR_STM32_H */
