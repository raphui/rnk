#ifndef FLASH_STM32_H
#define FLASH_STM32_H

#include <drv/mtd.h>

void stm32_flash_lock(void);
void stm32_flash_unlock(void);
int stm32_flash_wait_operation(void);
int stm32_flash_erase(struct mtd *mtd, unsigned int sector);
int stm32_flash_write(struct mtd *mtd, unsigned char *buff, unsigned int size, struct mtd_page *page);

#endif /* FLASH_STM32_H */
