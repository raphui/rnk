#ifndef DEVICE_H
#define DEVICE_H

#include <list.h>
#include <stddef.h>

struct device {
	unsigned int id;
	char name[32];
	const char *of_compat;
	char of_path[256];
	struct list_node next;
	int (*probe)(struct device *device);
	int (*remove)(struct device *device);
	int (*open)(struct device *device);
	int (*read)(struct device *device, unsigned char *buff, unsigned int size);
	int (*write)(struct device *device, unsigned char *buff, unsigned int size);
	int (*lseek)(struct device *device, int offset, int whence);
};

struct device_operations {
	int (*open)(struct device *device);
	int (*read)(struct device *device, unsigned char *buff, unsigned int size);
	int (*write)(struct device *device, unsigned char *buff, unsigned int size);
};

int device_register(struct device *dev);
int device_unregister(struct device *dev);
int device_of_register(struct device *dev);
int device_of_unregister(struct device *dev);
struct device *device_from_name(const char *name);
struct device *device_from_of_path(const char *path);
int device_of_probe(void);

#endif /* DEVICE_H */
