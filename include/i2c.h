/*
 * Copyright (C) 2015  Raphaël Poggi <poggi.raph@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Frrestore * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef I2C_H
#define I2C_H

#include <device.h>
#include <dma.h>
#include <list.h>
#include <kmutex.h>

#define I2C_TRANSFER_READ	0
#define I2C_TRANSFER_WRITE	1

#define I2C_MASTER	0
#define I2C_SLAVE	1

struct i2c_device {
	struct i2c_master *master;
	unsigned int address;
	struct device dev;
	struct list_node node;
};

struct i2c_operations
{
	int (*write)(struct i2c_device *i2c, unsigned char *buff, unsigned int size);
	int (*read)(struct i2c_device *i2c, unsigned char *buff, unsigned int size);
};


struct i2c_master {
	unsigned char num;
	unsigned int base_reg;
	unsigned char mode;
	unsigned long clk_rate;
	unsigned int irq;
	unsigned char use_dma;
	struct dma_transfer dma_trans;
	struct mutex i2c_mutex;
	struct list_node node;
	struct device dev;
	struct dma_stream dma_stream[2];
	struct i2c_operations *i2c_ops;
};

int i2c_transfer(struct i2c_device *i2c, unsigned char *buff, unsigned int size, int direction);
struct i2c_device *i2c_new_device_with_master(int fdt_offset);
struct i2c_device *i2c_new_device(void);
int i2c_remove_device(struct i2c_device *i2c);
int i2c_register_device(struct i2c_device *i2c);
struct i2c_master *i2c_new_master(void);
int i2c_remove_master(struct i2c_master *i2c);
int i2c_register_master(struct i2c_master *i2c);
int i2c_init(void);

#endif /* I2C_H */
