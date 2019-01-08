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

#define FLASH_PSIZE_BYTE	(0 << 8)
#define FLASH_PSIZE_WORD	(2 << 8)
#define FLASH_PSIZE_MASK	(3 << 8)
#define CR_PSIZE_MASK		0xFFFFFCFF
#define SR_ERR_MASK		0xF3

#define FLASH_KEY1	0x45670123
#define FLASH_KEY2      0xCDEF89AB

#define SECTOR_MASK	0xFFFFFF07

static void stm32_flash_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

static void stm32_flash_unlock(void)
{
	if (FLASH->CR & FLASH_CR_LOCK) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}
}

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

static int stm32_flash_wait_operation(void)
{
	int ret = 0;

	if (FLASH->SR & FLASH_SR_BSY)
		while (FLASH->SR & FLASH_SR_BSY)
			;
	else if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR)) {
		FLASH->SR = SR_ERR_MASK;
		ret = -EIO;
	}

	return ret;
}

static int stm32_flash_erase(struct mtd *mtd, unsigned int sector)
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

static int stm32_flash_write_byte(unsigned int address, void *data, int byte_access)
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

static int stm32_flash_write(struct mtd *mtd, unsigned char *buff, unsigned int size, struct mtd_page *page)
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
			addr += sizeof(unsigned char);
		else
			addr += sizeof(unsigned int);
	}

	stm32_flash_lock();

	FLASH->ACR |= FLASH_ACR_DCRST;
	FLASH->ACR |= FLASH_ACR_DCEN;

err:
	return ret;
}

static int stm32_flash_read(struct mtd *mtd, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	int i;
	unsigned int addr = mtd->base_addr + mtd->curr_off;

	stm32_flash_unlock();

	for (i = 0; i < size; i++) {
		buff[i] = *(unsigned char *)(addr + i);
	}

	stm32_flash_lock();

	return ret;
}

struct mtd_operations mtd_ops = {
	.erase = stm32_flash_erase,
	.write = stm32_flash_write,
	.read = stm32_flash_read,
};

static int stm32_flash_of_init(struct mtd *mtd)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, mtd->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, mtd->dev.of_compat);
	if (ret < 0)
		goto out;

	mtd->base_addr = (unsigned int)fdtparse_get_addr32(offset, "base");
	if (!mtd->base_addr) {
		error_printk("failed to retrieve mtd controller base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

out:
	return ret;
}

struct mtd_layout stm32_flash_layout[] = {
	{.pages_count = 4, .pages_size = SZ_16K},
	{.pages_count = 1, .pages_size = SZ_64K},
	{.pages_count = 1, .pages_size = SZ_128K},
};

static int stm32_flash_init(struct device *dev)
{
	int ret = 0;
	struct mtd *mtd = NULL;

	mtd = mtd_new_controller();
	if (!mtd) {
		error_printk("failed to request new mtd controller\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&mtd->dev, dev, sizeof(struct device));
	mtd->mtd_ops = &mtd_ops;

	ret = stm32_flash_of_init(mtd);
	if (ret < 0) {
		error_printk("failed to init mtd controller with fdt data\n");
		goto err;
	}

	mtd->mtd_map = stm32_flash_layout;
	mtd->layout_size = sizeof(stm32_flash_layout) / sizeof(struct mtd_layout);

	ret = mtd_register_controller(mtd);
	if (ret < 0) {
		error_printk("failed to register stm32 mtd controller\n");
		goto err;
	}

	FLASH->SR = SR_ERR_MASK;

	return 0;

err:
	return ret;
}

struct device stm32_flash_driver = {
	.of_compat = "st,stm32f4xx-flash",
	.probe = stm32_flash_init,
};

static int stm32_flash_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_flash_driver);
	if (ret < 0)
		error_printk("failed to register stm32_flash device\n");
	return ret;
}
postarch_initcall(stm32_flash_register);
