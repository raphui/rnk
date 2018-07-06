#include <drv/console.h>
#include <mm/mm.h>
#include <errno.h>
#include <init.h>
#include <fdtparse.h>
#include <kernel/printk.h>

static struct console *cons;

int console_write(unsigned char *buff, unsigned int len)
{
	int ret = 0;

#ifndef CONFIG_USART_DEBUG
	if (!cons || !cons->io_ops)
		return -ENOTTY;

	ret = cons->io_ops->write(cons->pdata, buff, len);
#else
	if (!cons)
		return -ENOTTY;

	ret = usart_write(&cons->usart->dev, buff, len);
#endif

	return ret;
}

static int console_init(void)
{
	int offset;
	int ret = 0;
	char *path = NULL;
	struct device *dev = NULL;

#ifdef CONFIG_USART_DEBUG
	struct usart_master *master;

	offset = fdtparse_alias_offset("console");
	if (offset < 0) {
		error_printk("failed to get offset for alias: console\n");
		return -ENOENT;
	}

	path = fdtparse_get_path(offset);
	if (!path) {
		error_printk("failed to retrieve console alias path\n");
		return -ENOENT;
	}

	dev = device_from_of_path(path);
	if (!dev) {
		error_printk("failed to retrieve console device struct\n");
		return -ENOENT;
	}

	master = container_of(dev, struct usart_master, dev);
#endif

	cons = (struct console *)kmalloc(sizeof(struct console));
	if (!cons) {
		error_printk("failed to allocate console struct\n");
		return -ENOMEM;
	}

#ifdef CONFIG_USART_DEBUG
	cons->usart = usart_new_device();
	if (!cons->usart) {
		error_printk("failed to allocate usart device struct\n");
		ret = -ENOMEM;
		goto err_usart;
	}

	cons->usart->master = master;

	ret = usart_register_device(cons->usart);
	if (ret < 0) {
		error_printk("failed to register console as usart device\n");
		goto err_register;
	}
#endif
	cons->pdata = dev;

#ifndef CONFIG_USART_DEBUG
	cons->io_ops = &io_op;
#endif

	return ret;

#ifdef CONFIG_USART_DEBUG
err_register:
	kfree(cons->usart);
err_usart:
	kfree(cons);

	return ret;
#endif
}
#if defined(CONFIG_SWO_DEBUG) || defined(CONFIG_SEMIHOSTING_DEBUG)
core_initcall(console_init);
#else
late_initcall(console_init);
#endif
