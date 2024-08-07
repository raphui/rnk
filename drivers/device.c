#include <drv/device.h>
#include <list.h>
#include <string.h>
#include <kernel/printk.h>
#include <fdtparse.h>
#include <init.h>

static int device_count = 0;
static int device_of_count = 0;

static struct list_node device_list;
static struct list_node device_of_list;

static void insert_of_device(struct device *dev)
{
	list_add_tail(&device_of_list, &dev->next);

	device_of_count++;
}

static void insert_device(struct device *dev)
{
	list_add_head(&device_list, &dev->next);

	device_count++;
}

int device_register(struct device *dev)
{
	int ret = 0;

	if (!device_count)
		list_initialize(&device_list);

	insert_device(dev);

	dev->id = device_count;

	return ret;
}

int device_unregister(struct device *dev)
{
	int ret = 0;

	list_delete(&dev->next);

	return ret;
}

struct device *device_from_name(const char *name)
{
	int found = 0;
	struct device *dev = NULL;

	list_for_every_entry(&device_list, dev, struct device, next) {
		if (!strcmp(name, dev->name)) {
			found = 1;
			break;
		}
	}

	if (!found)
		dev = NULL;

	return dev;
}

struct device *device_from_of_path(const char *path)
{
	int found = 0;
	struct device *dev = NULL;

	list_for_every_entry(&device_list, dev, struct device, next) {
		if (!strcmp(path, dev->of_path)) {
			found = 1;
			break;
		}
	}

	if (!found)
		dev = NULL;

	return dev;
}

int device_of_register(struct device *dev)
{
	int ret = 0;

	if (!device_of_count)
		list_initialize(&device_of_list);

	insert_of_device(dev);

	return ret;
}

int device_of_unregister(struct device *dev)
{
	int ret = 0;

	list_delete(&dev->next);

	return ret;
}

int device_of_probe(void)
{
	int ret = 0;
	int offset = 0;
	const void *blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	const char *compat;
	char *path;
	struct device *dev = NULL;
	struct device *tmp = NULL;

	do {
		offset = fdt_next_node(blob, offset, NULL);

		prop = fdt_get_property(blob, offset, "compatible", NULL);
		if (!prop)
			continue;

		compat = (const char *)prop->data;

		list_for_every_entry_safe(&device_of_list, dev, tmp, struct device, next) {
			if (!strcmp(compat, dev->of_compat)) {
				path = fdtparse_get_path(offset);
				if (!path) {
					error_printk("cannot find fdt path for of compat: %s\n", dev->of_compat);
					continue;
				}

				memset(dev->of_path, 0, sizeof(dev->of_path));
				memcpy(dev->of_path, path, strlen(path));

				dev->probe(dev);

				continue;
			}
		}

	} while (offset >= 0);

	return ret;
}
device_initcall(device_of_probe);
