#include <board.h>
#include <errno.h>
#include <kernel/printk.h>
#include <mm/mm.h>
#include <string.h>
#include <init.h>
#include <drv/device.h>
#include <fdtparse.h>
#include <drv/mtd.h>
#include <sizes.h>
#include <mach/flash-stm32.h>

#define FLASH_PSIZE_BYTE	(0 << 8)
#define FLASH_PSIZE_WORD	(2 << 8)
#define FLASH_PSIZE_MASK	(3 << 8)
#define CR_PSIZE_MASK		0xFFFFFCFF
#define SR_ERR_MASK		0xF3

#define FLASH_KEY1	0x45670123
#define FLASH_KEY2      0xCDEF89AB

#define SECTOR_MASK	0xFFFFFF07

static int stm32_flash_erase_needed(unsigned char *addr, unsigned char *buff, unsigned int size)
{
	int ret = 0;

	while (size--) {
		if (((*addr ^ *buff) & *buff) != 0) {
			ret = 1;
			break;
		}

		addr++;
		buff++;
	}

	return ret;
}

int stm32_flash_erase(struct mtd *mtd, unsigned int sector)
{
	int ret = 0;

	stm32_flash_unlock();

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("last operation did not success\n");
		return ret;
	}

	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_BYTE;
	FLASH->CR &= SECTOR_MASK;
	FLASH->CR |= FLASH_CR_SER | (sector << 3);
	FLASH->CR |= FLASH_CR_STRT;

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("failed to erase sector\n");
		return ret;
	}

	FLASH->CR &= (~FLASH_CR_SER);
	FLASH->CR &= SECTOR_MASK;

	stm32_flash_lock();

	return ret;
}

int stm32_flash_write_byte(unsigned int address, void *data, int byte_access)
{
	int ret = 0;

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("last operation did not success\n");
		return ret;
	}

	FLASH->CR &= CR_PSIZE_MASK;
	if (!byte_access)
		FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;

	if (byte_access)
		*(volatile unsigned char *)address = *(unsigned char *)data;
	else
		*(volatile unsigned int *)address = *(unsigned int *)data;

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("failed to write 0x%x at 0x%x\n", data, address);
		return ret;
	}

	FLASH->CR &= (~FLASH_CR_PG);

	return ret;
}

int stm32_flash_write(struct mtd *mtd, unsigned char *buff, unsigned int size, struct mtd_page *page)
{
	int ret = 0;
	int i;
	int step;
	int byte_access = size % sizeof(unsigned int);
	unsigned int addr = mtd->base_addr + mtd->curr_off;

	FLASH->ACR &= ~FLASH_ACR_DCEN;

	ret = stm32_flash_erase_needed((unsigned char *)addr, buff, size);
	if (ret) {
		ret = stm32_flash_erase(mtd, page->index);
		if (ret < 0)
			goto err;
	}

	stm32_flash_unlock();

	if (byte_access)
		step = 1;
	else
		step = 4;

	for (i = 0; i < size; i += step) {
		ret = stm32_flash_write_byte(addr, &buff[i], byte_access);
		if (ret < 0)
			break;

		if (byte_access)
			addr += step;
	}

	stm32_flash_lock();

	FLASH->ACR |= FLASH_ACR_DCRST;
	FLASH->ACR |= FLASH_ACR_DCEN;

err:
	return ret;
}
