#ifndef I2CDEV_H
#define I2CDEV_H

#include <device.h>
#include <i2c.h>

struct i2cdev_device {
	struct i2c_device *i2c;
	struct device dev;
};

#endif /* I2CDEV_H */
