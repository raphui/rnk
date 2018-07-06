#ifndef DMA_H
#define DMA_H

#include <drv/device.h>
#include <list.h>

#define INCR0	0x0
#define INCR4	0x1
#define INCR8	0x2
#define INCR16	0x3

#define DATA_SIZE_BYTE		0x0
#define DATA_SIZE_HALF_WORD	0x1
#define DATA_SIZE_WORD		0x2

#define DMA_P_M		0x0
#define DMA_M_P		0x1
#define DMA_M_M		0x2

struct dma_stream {
	unsigned int stream_base;
	unsigned char stream_num;
	unsigned char channel;
	unsigned char dir;
	unsigned char mdata_size;
	unsigned char pdata_size;
	unsigned char mburst;
	unsigned char pburst;
	unsigned int minc;
	unsigned int pinc;
	unsigned int pincos;
	unsigned char use_fifo;
	unsigned int priority;
	unsigned int irq;
	unsigned char enable_interrupt;
	void (*handler)(struct device *dev);
	void *arg;
	struct dma_controller *dma;
};

struct dma_controller {
	unsigned char num;
	unsigned int base_reg;
	unsigned int mem2mem;
	struct device dev;
	struct list_node node;
	struct dma_operations *dma_ops;
};

struct dma_transfer {
	unsigned int src_addr;
	unsigned int dest_addr;
	unsigned short size;
};

struct dma_operations
{
	int (*stream_init)(struct dma_stream *dma_stream);
	int (*transfer)(struct dma_stream *dma_stream, struct dma_transfer *dma_trans);
	int (*enable)(struct dma_stream *dma_stream);
	int (*disable)(struct dma_stream *dma_stream);
};

int dma_transfer(struct dma_stream *dma_stream, struct dma_transfer *dma_trans);
int dma_enable(struct dma_stream *dma_stream);
int dma_disable(struct dma_stream *dma_stream);
struct dma_controller *dma_new_controller(void);
int dma_remove_controller(struct dma_controller *dma);
int dma_register_controller(struct dma_controller *dma);
int dma_init(void);

#endif /* DMA_H */
