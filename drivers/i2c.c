#include <printk.h>
#include <i2c.h>
#include <mm.h>
#include <errno.h>
#include <string.h>
#include <utils.h>
#include <init.h>
#include <fdtparse.h>

static int dev_count = 0;
static int master_count = 0;
static struct list_node i2c_device_list;
static struct list_node i2c_master_list;

static int i2c_write(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);

	verbose_printk("writing from i2c !\n");

	kmutex_lock(&i2c->master->i2c_mutex);

	ret = i2c->master->i2c_ops->write(i2c, buff, size);

	kmutex_unlock(&i2c->master->i2c_mutex);

	return ret;
}

static int i2c_read(struct device *dev, unsigned char *buff, unsigned int size)
{
	int ret;
	struct i2c_device *i2c = container_of(dev, struct i2c_device, dev);

	verbose_printk("reading from i2c !\n");

	kmutex_lock(&i2c->master->i2c_mutex);

	ret = i2c->master->i2c_ops->read(i2c, buff, size);

	kmutex_unlock(&i2c->master->i2c_mutex);

	return ret;
}

int i2c_transfer(struct i2c_device *i2c, unsigned char *buff, unsigned int size, int direction)
{
	int ret = 0;

	verbose_printk("i2c %s transfer\n", (direction == I2C_TRANSFER_READ) ? "read" : "write");

	kmutex_lock(&i2c->master->i2c_mutex);

	if (direction == I2C_TRANSFER_READ)
		ret = i2c_read(&i2c->dev, buff, size);
	else if (direction == I2C_TRANSFER_WRITE)
		ret = i2c_write(&i2c->dev, buff, size);
	else {
		error_printk("invalid i2c transfer direction\n");
		ret = -EINVAL;
	}

	kmutex_unlock(&i2c->master->i2c_mutex);

	return ret;
}

struct i2c_device *i2c_new_device(void)
{
	struct i2c_device *i2cdev = NULL;

	i2cdev = (struct i2c_device *)kmalloc(sizeof(struct i2c_device));
	if (!i2cdev) {
		error_printk("cannot allocate i2c device\n");
		return NULL;
	}

	memset(i2cdev, 0, sizeof(struct i2c_device));

	dev_count++;

	return i2cdev;
}

struct i2c_device *i2c_new_device_with_master(int fdt_offset)
{
	int ret;
	int parent_offset;
	char fdt_path[64];
	struct device *dev;
	struct i2c_master *i2c;
	struct i2c_device *i2cdev = NULL;
	const void *fdt_blob = fdtparse_get_blob();

	i2cdev = (struct i2c_device *)kmalloc(sizeof(struct i2c_device));
	if (!i2cdev) {
		error_printk("cannot allocate i2c device\n");
		goto err;
	}

	memset(i2cdev, 0, sizeof(struct i2c_device));

	parent_offset = fdt_parent_offset(fdt_blob, fdt_offset);
	if (parent_offset < 0) {
		error_printk("failed to retrive i2c master for this i2c device\n");
		goto err;
	}

	memset(fdt_path, 0, 64 * sizeof(char));

	ret = fdt_get_path(fdt_blob, parent_offset, fdt_path, 64);
	if (ret < 0) {
		error_printk("failed to fdt path of i2c device parent node\n");
		goto err;
	}

	dev = device_from_of_path((const char *)fdt_path);
	if (!dev) {
		error_printk("failed to find device with fdt path: %s\n", fdt_path);
		goto err;
	}

	i2c = container_of(dev, struct i2c_master, dev);

	i2cdev->master = i2c;

	dev_count++;

	return i2cdev;

err:
	return NULL;
}

int i2c_remove_device(struct i2c_device *i2c)
{
	int ret = 0;
	struct i2c_device *i2cdev = NULL;

	ret = device_unregister(&i2c->dev);
	if (ret < 0) {
		error_printk("failed to unregister i2c device");
		return ret;
	}

	list_for_every_entry(&i2c_device_list, i2cdev, struct i2c_device, node)
		if (i2cdev == i2c)
			break;

	if (i2cdev) {
		list_delete(&i2cdev->node);
		kfree(i2cdev);
	}
	else
		ret = -ENOENT;


	dev_count--;

	return ret;
}

int i2c_register_device(struct i2c_device *i2c)
{
	int ret = 0;

	snprintf(i2c->dev.name, sizeof(i2c->dev.name), "/dev/i2c%d", dev_count);

	i2c->dev.read = i2c_read;
	i2c->dev.write = i2c_write;

	list_add_tail(&i2c_device_list, &i2c->node);

	ret = device_register(&i2c->dev);
	if (ret < 0)
		error_printk("failed to register i2c device\n");

	return ret;
}

struct i2c_master *i2c_new_master(void)
{
	struct i2c_master *i2c = NULL;

	i2c = (struct i2c_master *)kmalloc(sizeof(struct i2c_master));
	if (!i2c) {
		error_printk("cannot allocate i2c master\n");
		return NULL;
	}

	memset(i2c, 0, sizeof(struct i2c_master));

	master_count++;

	return i2c;
}

int i2c_remove_master(struct i2c_master *i2c)
{
	int ret = 0;
	struct i2c_master *i2cdev = NULL;

	ret = device_unregister(&i2c->dev);
	if (ret < 0) {
		error_printk("failed to unregister i2c master");
		return ret;
	}

	list_for_every_entry(&i2c_master_list, i2cdev, struct i2c_master, node)
		if (i2cdev == i2c)
			break;

	if (i2cdev) {
		list_delete(&i2cdev->node);
		kfree(i2cdev);
	}
	else
		ret = -ENOENT;


	master_count--;

	return ret;
}

int i2c_register_master(struct i2c_master *i2c)
{
	int ret = 0;
	char tmp[10] = {0};

	memcpy(tmp, dev_prefix, sizeof(dev_prefix));

	/* XXX: ascii 0 start at 0x30 */
	tmp[8] = 0x30 + master_count;

	memcpy(i2c->dev.name, tmp, sizeof(tmp));

	kmutex_init(&i2c->i2c_mutex);

	list_add_tail(&i2c_master_list, &i2c->node);

	ret = device_register(&i2c->dev);
	if (ret < 0)
		error_printk("failed to register i2c device\n");

	return ret;
}

int i2c_init(void)
{
	int ret = 0;

	list_initialize(&i2c_device_list);
	list_initialize(&i2c_master_list);

	return ret;
}
postcore_initcall(i2c_init);
