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

#include <armv7m/system.h>
#include <stdio.h>
#include <sizes.h>
#include <errno.h>
#include <utils.h>

#define MPU_MAX_REGION	8
#define MPU_RBAR_BASE_MASK	0xFFFFFFC0

static unsigned int mpu_read_reg(unsigned int reg)
{
	return readl(reg);
}
static void mpu_write_reg(unsigned reg, unsigned val)
{
	writel(reg, val);
}

int mpu_init(void)
{
	int ret = 0;
	int supp;

	supp = mpu_read_reg(MPU_TYPER);

	if (!(supp & 0xFF00))
		return -ENOTSUP;

	mpu_map(CORTEX_M_PERIPH_BASE, SZ_512M, MPU_RASR_DEVICE_SHARE | MPU_RASR_AP_PRIV_RW_UN_NO);
	mpu_map(NULL, 32, MPU_RASR_AP_PRIV_NO_UN_NO | MPU_RASR_XN);

	mpu_write_reg(MPU_CTRL, MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA);

	__isb();
	__dsb();

	return ret;
}

int mpu_map(void *base, int size, int attr)
{
	int i, tmp, mpu_en;
	int prio = -1;
	int ret = 0;

	__disable_it();

	if (size < 32)
		return -ENOTSUP;

	for (i = 0; i < MPU_MAX_REGION; i++) {
		mpu_write_reg(MPU_RNR, i);
		tmp = mpu_read_reg(MPU_RBAR) & MPU_RBAR_BASE_MASK;

		if (!tmp) {
			prio = i;
			break;
		}
	}

	if (prio >= 0) {
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

	} else {
		ret = -ENOMEM;
	}
	
	__enable_it();
	__isb();
	__dsb();

	return ret;
}
