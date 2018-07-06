#include <kernel/printk.h>
#include <drv/i2c.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>
#include <drv/i2cdev.h>
#include <drv/pio.h>

static int i2cdev_of_init(struct i2cdev_device *i2c)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, i2c->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = pio_of_configure(offset);

out:
	return ret;
}

int i2cdev_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct i2cdev_device *i2cdev = NULL;
	struct i2c_device *i2c = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	i2cdev = (struct i2cdev_device *)kmalloc(sizeof(struct i2cdev_device));
	if (!i2cdev) {
		error_printk("cannot allocate i2cdev device\n");
		return -ENOMEM;
	}

	memcpy(&i2cdev->dev, dev, sizeof(struct device));

	offset = fdt_path_offset(fdt_blob, i2cdev->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto free_i2cdev;
	}

	i2c = i2c_new_device_with_master(offset);
	if (!i2c) {
		error_printk("failed to retrive new i2c device\n");
		ret = -EIO;
		goto free_i2cdev;
	}

	ret = i2c_register_device(i2c);
	if (ret < 0) {
		error_printk("failed to register i2c device\n");
		goto free_i2c;
	}

	i2cdev->i2c = i2c;

	return 0;

free_i2c:
	kfree(i2c);
free_i2cdev:
	kfree(i2cdev);
	return ret;
}

struct device i2cdev_driver = {
	.of_compat = "i2c,i2cdev",
	.probe = i2cdev_init,
};

static int i2cdev_register(void)
{
	int ret = 0;

	ret = device_of_register(&i2cdev_driver);
	if (ret < 0)
		error_printk("failed to register i2cdev device\n");
	return ret;
}
coredevice_initcall(i2cdev_register);
