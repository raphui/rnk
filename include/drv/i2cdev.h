#ifndef I2CDEV_H
#define I2CDEV_H

#include <drv/device.h>
#include <kernel/i2c.h>

struct i2cdev_device {
	struct i2c_device *i2c;
	struct device dev;
};

#endif /* I2CDEV_H */
