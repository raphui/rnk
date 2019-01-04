#include <armv7m/nvic.h>
#include <armv7m/system.h>
#include <kernel/printk.h>
#include <utils.h>
#include <board.h>

void nvic_enable_interrupt(unsigned int num)
{
	unsigned int *nvic_reg = (unsigned int *)NVIC_ISER0;

	writel((unsigned int)&nvic_reg[num >> 0x5], (1 << (num & 0x1F)));
}

void nvic_disable_interrupt(unsigned int num)
{
	unsigned int *nvic_reg = (unsigned int *)NVIC_ICER0;

	writel((unsigned int)&nvic_reg[num >> 0x5], (1 << (num & 0x1F)));
}

void nvic_set_interrupt(unsigned int num)
{
	unsigned int *nvic_reg = (unsigned int *)NVIC_ISPR0;

	writel((unsigned int)&nvic_reg[num >> 0x5], (1 << (num & 0x1F)));
}

void nvic_clear_interrupt(unsigned int num)
{

	unsigned int *nvic_reg = (unsigned int *)NVIC_ICPR0;

	writel((unsigned int)&nvic_reg[num >> 0x5], (1 << (num & 0x1F)));
}

void nvic_set_priority_interrupt(int num, unsigned char priority)
{
	unsigned int reg;
	unsigned int shift = 0;

	if (num < 0) {
		switch (num) {
		case svcall_irq:
			reg = SCB_SHPR2;
			shift = 24;
			break;
		case pendsv_irq:
			reg = SCB_SHPR3;
			shift = 16;
			break;
		case systick_irq:
			reg = SCB_SHPR3;
			shift = 24;
			break;
		default:
			return;
		}

		writel(reg, (priority << (8 - __NVIC_PRIO_BITS)) << shift);
	}
	else
		writel(NVIC_IPR((num >> 2)), ((priority & 0xF) << (8 - __NVIC_PRIO_BITS)));
}
