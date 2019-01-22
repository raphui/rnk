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
#include <utils.h>
#include <mach/flash-stm32.h>
#include <arch/spinlock.h>

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
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

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

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);

	return ret;
}

static int stm32_flash_write_dword(unsigned int address, void *data)
{
	int ret = 0;
	uint64_t *flash = (uint64_t *)address;
	uint64_t *dword = (uint64_t *)data;
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);


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

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);

	return ret;
}

static int stm32_flash_program(unsigned int addr, unsigned char *data, int size)
{
	int i;
	int n = 0;
	int ret = 0;

	stm32_flash_unlock();

	for (i = 0; i < size; i += 8) {
		ret = stm32_flash_write_dword(addr, &data[i]);
		if (ret < 0)
			break;

		addr += 8;
		n += 8;
	}

	stm32_flash_lock();

	return n;
}

int stm32_flash_write(struct mtd *mtd, unsigned char *buff, unsigned int size, struct mtd_page *page)
{
	int ret = 0;
	int n = 0;
	int erase = 0;
	int aligned_size;
	int pad_size;
	unsigned int addr = mtd->base_addr + mtd->curr_off;
	int page_size = page->end - page->start;
	int page_offset = mtd->curr_off & (page_size - 1);
	unsigned char *page_cache = kmalloc(page_size);

	memcpy(page_cache, (void *)(mtd->base_addr + page->start), page_size);

	if (!memcmp(page_cache + page_offset, buff, size)) {
		ret = size;
		goto err;
	}

	memcpy(page_cache + page_offset, buff, size);

	aligned_size = ALIGN(size, 8);
	pad_size = aligned_size - size;
	addr = ROUND_DOWN(addr, sizeof(uint64_t));
	page_offset = ROUND_DOWN(page_offset, sizeof(uint64_t));

	FLASH->SR = SR_ERR_MASK;

	FLASH->ACR &= ~FLASH_ACR_DCEN;

	ret = stm32_flash_erase_needed((unsigned char *)addr, buff, size);
	if (ret) {
		ret = stm32_flash_erase(mtd, page->index);
		if (ret < 0)
			goto err_cache;

		erase = 1;
	}

	if (erase) {
		ret = stm32_flash_program(mtd->base_addr + page->start, page_cache, page_size);
		if (ret < 0)
			goto err_cache;

		ret = aligned_size;
	}
	else
		ret = stm32_flash_program(addr, &page_cache[page_offset], aligned_size);


	ret -= pad_size;

err_cache:
	FLASH->ACR |= FLASH_ACR_DCRST;
	FLASH->ACR |= FLASH_ACR_DCEN;
err:
	kfree(page_cache);
	return ret;
}
