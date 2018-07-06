#include <kernel/printk.h>
#include <errno.h>
#include <string.h>
#include <mm/mm.h>
#include <drv/irq.h>
#include <armv7m/vector.h>
#include <init.h>

int irq_action(void)
{
	int ret = 0;
	unsigned char irq;
	struct isr_entry *entry = NULL;

	irq = vector_current_irq();

	entry = vector_get_isr_entry(irq);
	if (!entry) {
		error_printk("failed to retrieve isr entry in vector table\n");
		return -ENXIO;
	}

	if (entry->isr)
		entry->isr(entry->arg);

	return ret;	
}

int irq_request(unsigned int irq, void (*handler)(void *), void *arg)
{
	int ret = 0;
	struct isr_entry entry;

	entry.isr = handler;
	entry.arg = arg;

	ret = vector_set_isr_entry(&entry, irq);
	if (ret < 0)
		error_printk("failed to set isr entry, for irq: %d\n", irq);

	return ret;
}

int irq_init(void)
{
	int ret = 0;

	return ret;
}
coredevice_initcall(irq_init);
