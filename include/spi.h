#ifndef SPI_H
#define SPI_H

#include <device.h>
#include <dma.h>
#include <list.h>
#include <kmutex.h>
#include <ksem.h>

#define SPI_TRANSFER_READ	0
#define SPI_TRANSFER_WRITE	1

struct spi_device {
	struct spi_master *master;
	unsigned int cs_port;
	unsigned int cs_pin;
	unsigned int speed;
	struct device dev;
	struct list_node node;
	void *priv;
};

struct spi_operations
{
	int (*write)(struct spi_device *spi, unsigned char *buff, unsigned int size);
	int (*read)(struct spi_device *spi, unsigned char *buff, unsigned int size);
	int (*exchange)(struct spi_device *spi, unsigned char *in, unsigned char *out, unsigned int size);
};

struct spi_master {
	unsigned int num;
	unsigned int base_reg;
	unsigned int source_clk;
	unsigned int rate;	/* current rate */
	unsigned int speed;	/* wanted speed */
	unsigned int irq;
	unsigned short mode;
	struct dma_transfer dma_trans_w;
	struct dma_transfer dma_trans_r;
	unsigned char only_tx;
	unsigned char only_rx;
	unsigned char use_dma;
	struct mutex spi_mutex;
	struct semaphore sem;
	struct list_node node;
	struct device dev;
	struct dma_stream dma_stream[2];
	struct spi_operations *spi_ops;
};

int spi_transfer(struct spi_device *spi, unsigned char *buff, unsigned int size);
struct spi_device *spi_new_device_with_master(int fdt_offset);
struct spi_device *spi_new_device(void);
int spi_remove_device(struct spi_device *spi);
int spi_register_device(struct spi_device *spi, struct device_operations *dev_ops);
struct spi_master *spi_new_master(void);
int spi_remove_master(struct spi_master *spi);
int spi_register_master(struct spi_master *spi);
int spi_init(void);

#endif /* SPI_H */
