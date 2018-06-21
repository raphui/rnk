#include <armv7m/nvic.h>
#include <armv7m/system.h>
#include <printk.h>
#include <utils.h>

void nvic_enable_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ISER0;
			break;
		case 1:
			nvic_reg = NVIC_ISER1;
			break;
		case 2:
			nvic_reg = NVIC_ISER2;
			break;
		case 3:
			nvic_reg = NVIC_ISER3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_disable_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ICER0;
			break;
		case 1:
			nvic_reg = NVIC_ICER1;
			break;
		case 2:
			nvic_reg = NVIC_ICER2;
			break;
		case 3:
			nvic_reg = NVIC_ICER3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_set_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ISPR0;
			break;
		case 1:
			nvic_reg = NVIC_ISPR1;
			break;
		case 2:
			nvic_reg = NVIC_ISPR2;
			break;
		case 3:
			nvic_reg = NVIC_ISPR3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_clear_interrupt(unsigned int num)
{
	unsigned int base = num / 32;
	unsigned int off = num % 32;
	unsigned int nvic_reg;

	switch (base) {
		case 0:
			nvic_reg = NVIC_ICPR0;
			break;
		case 1:
			nvic_reg = NVIC_ICPR1;
			break;
		case 2:
			nvic_reg = NVIC_ICPR2;
			break;
		case 3:
			nvic_reg = NVIC_ICPR3;
			break;
	}

	writel(nvic_reg, (1 << off));
}

void nvic_set_priority_interrupt(int num, unsigned char priority)
{
	unsigned int reg;
	unsigned int shift = 0;

	if (num < 0) {
		switch (num) {
		case svcall_irq:
			reg = SCB_SHPR(2);
			shift = 24;
			break;
		case pendsv_irq:
			reg = SCB_SHPR(3);
			shift = 16;
			break;
		case systick_irq:
			reg = SCB_SHPR(3);
			shift = 24;
			break;
		default:
			return;
		}

		writel(reg, priority << shift);
	}
	else
		writel(NVIC_IPR(num), priority);
}
