/*
 * Copyright (C) 2018  Raphaël Poggi <poggi.raph@gmail.com>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <armv7m/mpu.h>
#include <armv7m/system.h>
#include <printk.h>
#include <sizes.h>
#include <errno.h>
#include <utils.h>

#define MPU_MAX_REGION	8
#define MPU_RBAR_BASE_MASK	0xFFFFFFC0

#define MPU_RASR_SIZE_MASK	0x3E

extern unsigned int _sstack[];
extern unsigned int _estack[];

extern unsigned int _slibbss[];
extern unsigned int _elibbss[];

extern unsigned int _slibdata[];
extern unsigned int _elibdata[];

static unsigned int mpu_read_reg(unsigned int reg)
{
	return readl(reg);
}
static void mpu_write_reg(unsigned reg, unsigned val)
{
	writel(reg, val);
}

static int mpu_map(void *base, int size, int prio, int attr)
{
	int tmp, mpu_en;
	int ret = 0;

	if (size < 32)
		return -ENOTSUP;

	__disable_it();

	tmp = mpu_read_reg(MPU_CTRL);

	mpu_en = tmp & MPU_CTRL_ENABLE;
	tmp &= ~MPU_CTRL_ENABLE;

	mpu_write_reg(MPU_CTRL, tmp);

	mpu_write_reg(MPU_RNR, prio);
	mpu_write_reg(MPU_RBAR, ((unsigned int)base) & ~0x1F);

	size = 32 - __builtin_clz(size) - 2;
	mpu_write_reg(MPU_RASR, MPU_RASR_SIZE(size) | attr | MPU_RASR_ENABLE);

	if (mpu_en)
		tmp |= MPU_CTRL_ENABLE;

	mpu_write_reg(MPU_CTRL, tmp);

	__enable_it();
	__isb();
	__dsb();

	return ret;
}

int mpu_init(void)
{
	int ret = 0;
	int supp;

	supp = mpu_read_reg(MPU_TYPER);

	if (!(supp & 0xFF00))
		return -ENOTSUP;

	mpu_map_from_low((void *)CORTEX_M_PERIPH_BASE, SZ_512M, MPU_RASR_DEVICE_SHARE | MPU_RASR_AP_PRIV_RW_UN_NO);
	mpu_map_from_low(NULL, 32, MPU_RASR_AP_PRIV_NO_UN_NO | MPU_RASR_XN);
	mpu_map_from_low((void *)CONFIG_USER_HEAP_START, CONFIG_USER_HEAP_END - CONFIG_USER_HEAP_START, MPU_RASR_AP_PRIV_RW_UN_RW);
	mpu_map_from_high((void *)_slibbss, (unsigned int)_elibbss - (unsigned int)_slibbss, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW);
	mpu_map_from_high((void *)_slibdata, (unsigned int)_elibdata - (unsigned int)_slibdata, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW);


	mpu_write_reg(MPU_CTRL, MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA);

	__isb();
	__dsb();

	return ret;
}

int mpu_map_from_low(void *base, int size, int attr)
{
	int i, tmp;
	int ret = 0;
	int prio = -1;

	for (i = 0; i < MPU_MAX_REGION; i++) {
		mpu_write_reg(MPU_RNR, i);
		tmp = mpu_read_reg(MPU_RBAR) & MPU_RBAR_BASE_MASK;

		if (!tmp) {
			prio = i;
			break;
		}
	}

	if (prio >= 0) {
		ret = mpu_map(base, size, prio, attr);
		if (!ret)
			ret = prio;
	}
	else
		ret = -ENOMEM;

	return ret;
}

int mpu_map_from_high(void *base, int size, int attr)
{
	int i, tmp;
	int ret = 0;
	int prio = -1;

	for (i = MPU_MAX_REGION - 1; i >= 0 ; i--) {
		mpu_write_reg(MPU_RNR, i);
		tmp = mpu_read_reg(MPU_RBAR) & MPU_RBAR_BASE_MASK;

		if (!tmp) {
			prio = i;
			break;
		}
	}

	if (prio >= 0) {
		ret = mpu_map(base, size, prio, attr);
		if (!ret)
			ret = prio;
	}
	else
		ret = -ENOMEM;

	return ret;
}

int mpu_unmap(void *base, int size)
{
	int i, tmp;
	int ret = 0;
	int prio = -1;

	__disable_it();

	size = 32 - __builtin_clz(size) - 2;

	for (i = 0; i < MPU_MAX_REGION; i++) {
		mpu_write_reg(MPU_RNR, i);
		tmp = mpu_read_reg(MPU_RBAR) & MPU_RBAR_BASE_MASK;

		if (tmp == (int)base) {
			tmp = mpu_read_reg(MPU_RASR);

			if ((tmp & MPU_RASR_SIZE_MASK) == size) {
				prio = i;
				break;
			}
		}
	}

	if (prio >= 0) {
		mpu_write_reg(MPU_RBAR, 0x0);
		mpu_write_reg(MPU_RASR, 0x0);
	}
	else
		ret = -ENOENT;

	__enable_it();
	__isb();
	__dsb();

	return ret;
}

int mpu_unmap_prio(int prio)
{
	int ret = 0;

	__disable_it();

	if (prio >= 0) {
		mpu_write_reg(MPU_RNR, prio);
		mpu_write_reg(MPU_RBAR, 0x0);
		mpu_write_reg(MPU_RASR, 0x0);
	}
	else
		ret = -ENOENT;

	__enable_it();
	__isb();
	__dsb();

	return ret;
}
