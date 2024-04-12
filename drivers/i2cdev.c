#include <kernel/printk.h>
#include <drv/i2c.h>
#include <mm/mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>
#include <drv/pio.h>
#include <ioctl.h>

struct i2cdev_priv {
	struct i2c_msg curr_msg;
};

static int i2cdev_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct i2cdev_priv *priv = (struct i2cdev_priv *)i2c->priv;

	priv->curr_msg.buff = buff;
	priv->curr_msg.size = size;

	ret = i2c_transfer(i2c, &priv->curr_msg, I2C_TRANSFER_READ);

	return ret;
}


static int i2cdev_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct i2cdev_priv *priv = (struct i2cdev_priv *)i2c->priv;

	priv->curr_msg.buff = buff;
	priv->curr_msg.size = size;

	ret = i2c_transfer(i2c, &priv->curr_msg, I2C_TRANSFER_WRITE);

	return ret;
}

static int i2cdev_ioctl(struct device *dev, int request, char *arg)
{
	int ret = 0;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);
	struct i2cdev_priv *priv = (struct i2cdev_priv *)i2c->priv;

	switch (request) {
	case IOCTL_SET_ADDRESS:
		ret = i2c->master->i2c_ops->ioctl(i2c, request, arg);
		break;
	case IOCTL_I2C_REG:
		priv->curr_msg.reg = (unsigned short)arg;
		break;
	}

	return ret;
}

static int i2cdev_of_init(int offset)
{
	int ret = 0;


	return ret;
}

static struct device_operations dev_ops = {
	.read = i2cdev_read,
	.write = i2cdev_write,
	.ioctl = i2cdev_ioctl,
};

int i2cdev_init(struct device *dev)
{
	int offset;
	int ret = 0;
	struct i2cdev_priv *priv = NULL;
	struct i2c_device *i2c = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, dev->of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto err;
	}
	
	priv = (struct i2cdev_priv *)kmalloc(sizeof(struct i2cdev_priv));
	if (!priv) {
		error_printk("failed to allocate i2cdev private struct\n");
		ret = -ENOMEM;
		goto err;
	}

	i2c = i2c_new_device_with_master(offset);
	if (!i2c) {
		error_printk("failed to retrive new i2c device\n");
		ret = -EIO;
		goto free_i2c;
	}

	memcpy(&i2c->dev, dev, sizeof(struct device));

	i2c->priv = priv;

	ret = i2cdev_of_init(offset);
	if (ret < 0) {
		error_printk("failed to init fdt data\n");
		goto free_i2c;
	}

	ret = i2c_register_device(i2c, &dev_ops);
	if (ret < 0) {
		error_printk("failed to register i2c device\n");
		goto free_i2c;
	}

	return 0;

free_i2c:
	kfree(i2c);
err:
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
