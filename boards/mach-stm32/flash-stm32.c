/*
 * Copyright (C) 2016  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <board.h>
#include <errno.h>
#include <stdio.h>
#include <mm.h>
#include <string.h>
#include <init.h>
#include <device.h>
#include <fdtparse.h>
#include <mtd.h>

#define FLASH_PSIZE_BYTE	0x200
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

static int stm32_flash_write_byte(unsigned int address, unsigned char data)
{
	int ret = 0;

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("last operation did not success\n");
		return ret;
	}

	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_BYTE;
	FLASH->CR |= FLASH_CR_PG;

	*(unsigned int *)address = data;

	ret = stm32_flash_wait_operation();
	if (ret < 0) {
		error_printk("failed to write 0x%x at 0x%x\n", data, address);
		return ret;
	}

	FLASH->CR &= (~FLASH_CR_PG);

	return ret;
}

static int stm32_flash_write(struct mtd *mtd, unsigned char *dst, unsigned char *src, unsigned int size)
{
	int ret = 0;
	int i;

	stm32_flash_unlock();

	for (i = 0; i < size; i++) {
		ret = stm32_flash_write_byte(dst++, src[i]);
		if (ret < 0)
			break;
	}

	stm32_flash_lock();

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
	int i, len, offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	fdt32_t *cell;

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

	ret = fdtparse_get_int(offset, "sector_size", &mtd->sector_size);
	if (ret < 0) {
		error_printk("failed to retrieve sector size\n");
		ret = -ENOENT;
		goto out;
	}

	prop = fdt_get_property(fdt_blob, offset, "sector_table", &len);
	if (!prop) {
		error_printk("cannot find sector table in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	cell = (fdt32_t *)prop->data;

	for (i = 0; i < len; i++)
		mtd->sector_table[i] = fdt32_to_cpu(cell[i]);

	mtd->num_sectors = len;

out:
	return ret;
}

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
