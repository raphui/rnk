#ifndef I2C_H
#define I2C_H

#include <drv/device.h>
#include <drv/dma.h>
#include <list.h>
#include <kernel/kmutex.h>
#include <kernel/ksem.h>

#define I2C_TRANSFER_READ	0
#define I2C_TRANSFER_WRITE	1

#define I2C_MASTER	0
#define I2C_SLAVE	1

struct i2c_device {
	struct i2c_master *master;
	unsigned int address;
	struct device dev;
	struct list_node node;
	void *priv;
};

struct i2c_master {
	unsigned char num;
	unsigned int base_reg;
	unsigned char mode;
	unsigned int rate;	/* current rate */
	unsigned int speed;	/* wanted speed */
	unsigned int irq;
	unsigned int timing;
	unsigned short addressing_mode;
	unsigned short master;
	unsigned char use_dma;
	struct clk clock;
	struct dma_transfer dma_trans_w;
	struct dma_transfer dma_trans_r;
	struct mutex i2c_mutex;
	struct semaphore sem;
	struct list_node node;
	struct device dev;
	struct dma_stream dma_stream[2];
	struct i2c_operations *i2c_ops;
};

struct i2c_msg {
	unsigned short reg;
	unsigned int size;
	unsigned char *buff;
};

struct i2c_operations
{
	int (*write)(struct i2c_device *i2c, unsigned char *buff, unsigned int size);
	int (*read)(struct i2c_device *i2c, unsigned char *buff, unsigned int size);
	int (*ioctl)(struct i2c_device *i2c, int request, char *arg);
	int (*transfer)(struct i2c_device *i2c, struct i2c_msg *msg, int direction);
};

int i2c_transfer(struct i2c_device *i2c, struct i2c_msg *msg, int direction);
struct i2c_device *i2c_new_device_with_master(int fdt_offset);
struct i2c_device *i2c_new_device(void);
int i2c_remove_device(struct i2c_device *i2c);
int i2c_register_device(struct i2c_device *i2c, struct device_operations *dev_ops);
struct i2c_master *i2c_new_master(void);
int i2c_remove_master(struct i2c_master *i2c);
int i2c_register_master(struct i2c_master *i2c);
int i2c_init(void);

#endif /* I2C_H */
