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
#define FLASH_PSIZE_DWORD	(3 << 8)
#define FLASH_PSIZE_MASK	(3 << 8)
#define CR_PSIZE_MASK		0xFFFFFCFF
#define SR_ERR_MASK		0xF3

#define FLASH_KEY1	0x45670123
#define FLASH_KEY2      0xCDEF89AB

#define SECTOR_MASK	0xFFFFFF07

void stm32_flash_lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

void stm32_flash_unlock(void)
{
	if (FLASH->CR & FLASH_CR_LOCK) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}
}

int stm32_flash_wait_operation(void)
{
	int ret = 0;

	if (FLASH->SR & FLASH_SR_BSY)
		while (FLASH->SR & FLASH_SR_BSY)
			;

#ifdef CONFIG_STM32F4XX
	if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR)) {
#else
	if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PGAERR)) {
#endif
		FLASH->SR = SR_ERR_MASK;
		ret = -EIO;
	}

	return ret;
}


static int stm32_flash_read(struct mtd *mtd, unsigned char *buff, unsigned int size)
{
	int i;
	unsigned int addr = mtd->base_addr + mtd->curr_off;

	stm32_flash_unlock();

	memcpy(buff, (void *)addr, size);

	stm32_flash_lock();

	return size;
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

#ifdef CONFIG_STM32F4XX
struct mtd_layout stm32_flash_layout[] = {
	{.pages_count = 4, .pages_size = SZ_16K},
	{.pages_count = 1, .pages_size = SZ_64K},
	{.pages_count = 1, .pages_size = SZ_128K},
};
#else
struct mtd_layout stm32_flash_layout[] = {
	{.pages_count = 127, .pages_size = SZ_2K},
};
#endif

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
	.of_compat = "st,stm32-flash",
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
