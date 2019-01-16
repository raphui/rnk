#include <kernel/printk.h>
#include <drv/usart.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>

int usartdev_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct usart_device *usart = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}

	usart = usart_new_device_with_master(offset);
	if (!usart) {
		error_printk("failed to retrive new usart device\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&usart->dev, dev, sizeof(struct device));

	ret = usart_register_device(usart);
	if (ret < 0) {
		error_printk("failed to register usart device\n");
		goto free_usart;
	}

	return 0;

free_usart:
	kfree(usart);
err:
	return ret;
}

struct device usartdev_driver = {
	.of_compat = "usart,usartdev",
	.probe = usartdev_init,
};

static int usartdev_register(void)
{
	int ret = 0;

	ret = device_of_register(&usartdev_driver);
	if (ret < 0)
		error_printk("failed to register usartdev device\n");
	return ret;
}
coredevice_initcall(usartdev_register);
