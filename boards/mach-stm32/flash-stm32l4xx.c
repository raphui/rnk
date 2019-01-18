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

#define CR_PSIZE_MASK		0xFFFFFCFF
#define SR_ERR_MASK		0xF3

#define FLASH_KEY1	0x45670123
#define FLASH_KEY2      0xCDEF89AB

static int stm32_flash_erase_needed(unsigned char *addr, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	uint64_t *flash = (uint64_t *)addr;
	uint64_t *bword = (uint64_t *)buff;
	int n = size / sizeof(uint64_t);

	while (n--) {
		if (*flash != *bword) {
			if (*flash != 0xFFFFFFFFFFFFFFFF) {
				if (*bword != 0) {
					ret = 1;
					break;
				}
			}
		}

		flash++;
		bword++;
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

	FLASH->CR &= ~FLASH_CR_PNB_Msk;
	FLASH->CR |= (sector << FLASH_CR_PNB_Pos);
	FLASH->CR |= FLASH_CR_PER;
	FLASH->CR |= FLASH_CR_STRT;

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("failed to erase sector\n");
		return ret;
	}

	FLASH->CR &= ~FLASH_CR_PER;
	FLASH->CR &= ~FLASH_CR_PNB_Msk;

	stm32_flash_lock();

	return ret;
}

int stm32_flash_write_dword(unsigned int address, void *data)
{
	int ret = 0;
	uint64_t *flash = (uint64_t *)address;
	uint64_t *dword = (uint64_t *)data;


	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("last operation did not success\n");
		return ret;
	}

	FLASH->CR |= FLASH_CR_PG;

	if (*flash != *dword)
		*flash = *dword;

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
	int n = 0;
	unsigned int addr = mtd->base_addr + mtd->curr_off;

	FLASH->ACR &= ~FLASH_ACR_DCEN;

	if (size % sizeof(uint64_t))
		return -EINVAL;

	ret = stm32_flash_erase_needed((unsigned char *)addr, buff, size);
	if (ret) {
		ret = stm32_flash_erase(mtd, page->index);
		if (ret < 0)
			goto err;
	}

	stm32_flash_unlock();

	for (i = 0; i < size; i += 8) {
		ret = stm32_flash_write_dword(addr, &buff[i]);
		if (ret < 0)
			break;

		addr += 8;
		n += 8;
	}

	stm32_flash_lock();

	FLASH->ACR |= FLASH_ACR_DCRST;
	FLASH->ACR |= FLASH_ACR_DCEN;

	if (!ret)
		ret = n;

err:
	return ret;
}
