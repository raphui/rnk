#ifndef PWR_STM32_H
#define PWR_STM32_H

enum {
	STOP0_MODE,
	STOP1_MODE,
	STOP2_MODE,
};

void stm32_pwr_set_regulator(int freq);
void stm32_pwr_enter_lpsleep(int mode);
void stm32_pwr_exit_lpsleep(int mode);
void stm32_pwr_enable_vusb(void);

#endif /* PWR_STM32_H */
