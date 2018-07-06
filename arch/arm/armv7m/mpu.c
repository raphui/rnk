#include <armv7m/mpu.h>
#include <armv7m/system.h>
#include <kernel/printk.h>
#include <sizes.h>
#include <errno.h>
#include <utils.h>
#include <kernel/spinlock.h>

#define MPU_MAX_REGION	8
#define MPU_RBAR_BASE_MASK	0xFFFFFFC0

#define MPU_RASR_SIZE_MASK	0x3E

extern unsigned int _sstack[];
extern unsigned int _estack[];

extern unsigned int _slibtext[];
extern unsigned int _elibtext[];

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

static inline int mpu_get_size_field(int size)
{
	size = 1 << (32 - __builtin_clz(size - 1));
	return (32 - __builtin_clz(size) - 2) << 1;
}

static int mpu_map(void *base, int size, int prio, int attr)
{
	int tmp, mpu_en;
	int ret = 0;
	unsigned long irqstate;

	if (size < 32)
		return -ENOTSUP;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	tmp = mpu_read_reg(MPU_CTRL);

	mpu_en = tmp & MPU_CTRL_ENABLE;
	tmp &= ~MPU_CTRL_ENABLE;

	mpu_write_reg(MPU_CTRL, tmp);

	mpu_write_reg(MPU_RNR, prio);

	size = mpu_get_size_field(size);

	base = (void *)ALIGN((unsigned int)base, size);

	mpu_write_reg(MPU_RASR, MPU_RASR_SIZE(size) | attr | MPU_RASR_ENABLE);
	mpu_write_reg(MPU_RBAR, (unsigned int)base);

	if (mpu_en)
		tmp |= MPU_CTRL_ENABLE;

	mpu_write_reg(MPU_CTRL, tmp);

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
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

	mpu_map_from_low((void *)CONFIG_USER_HEAP_START, CONFIG_USER_HEAP_END - CONFIG_USER_HEAP_START, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW);
	mpu_map_from_low((void *)_slibtext, (unsigned int)_elibtext - (unsigned int)_slibtext, MPU_RASR_NORMAL_CACHE | MPU_RASR_AP_PRIV_RW_UN_RO);
	mpu_map_from_low((void *)_slibbss, (unsigned int)_elibbss - (unsigned int)_slibbss, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW | MPU_RASR_XN);
	mpu_map_from_low((void *)_slibdata, (unsigned int)_elibdata - (unsigned int)_slibdata, MPU_RASR_SHARE_CACHE | MPU_RASR_AP_PRIV_RW_UN_RW | MPU_RASR_XN);

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
		tmp = mpu_read_reg(MPU_RASR) & MPU_RASR_ENABLE;

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
		tmp = mpu_read_reg(MPU_RASR) & MPU_RASR_ENABLE;

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
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	size = mpu_get_size_field(size);

	for (i = 0; i < MPU_MAX_REGION; i++) {
		mpu_write_reg(MPU_RNR, i);
		tmp = mpu_read_reg(MPU_RASR) & MPU_RASR_ENABLE;

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

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	__isb();
	__dsb();

	return ret;
}

int mpu_unmap_prio(int prio)
{
	int ret = 0;
	unsigned long irqstate;

	arch_interrupt_save(&irqstate, SPIN_LOCK_FLAG_IRQ);

	if (prio >= 0) {
		mpu_write_reg(MPU_RNR, prio);
		mpu_write_reg(MPU_RBAR, 0x0);
		mpu_write_reg(MPU_RASR, 0x0);
	}
	else
		ret = -ENOENT;

	arch_interrupt_restore(irqstate, SPIN_LOCK_FLAG_IRQ);
	__isb();
	__dsb();

	return ret;
}
