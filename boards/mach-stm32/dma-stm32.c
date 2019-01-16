#include <board.h>
#include <mach/rcc-stm32.h>
#include <mach/dma-stm32.h>
#include <drv/dma.h>
#include <utils.h>
#include <kernel/printk.h>
#include <armv7m/nvic.h>
#include <errno.h>
#include <drv/device.h>
#include <init.h>
#include <string.h>
#include <fdtparse.h>
#include <mm/mm.h>
#include <drv/irq.h>
#include <armv7m/vector.h>

#define MAX_DMA_SIZE 0xFFFF

#define DMA_OF_PINC_MASK	0x200
#define DMA_OF_MINC_MASK	0x400
#define DMA_OF_PINCOS_MASK	0x8000
#define DMA_OF_PRIO_MASK	0x30000

#define DMA_OF_FIFO_THRESH_MASK 0x3

struct dma_of {
	unsigned int controller;
	unsigned int stream;
	unsigned int channel;
	unsigned int dma_conf;
	unsigned int dma_fifo_conf;
};

static inline unsigned int stm32_dma_get_base(struct dma_stream *dma_stream)
{
	return dma_stream->dma->base_reg;
}

static int stm32_dma_get_nvic_number(struct dma_stream *dma_stream)
{

#ifdef CONFIG_STM32F4XX
	return dma_stream->dma->interrupts[dma_stream->stream_num];
#else
	return dma_stream->dma->interrupts[dma_stream->stream_num - 1];
#endif
}

static int stm32_dma_get_stream_base(struct dma_stream *stream)
{
	unsigned int base = stm32_dma_get_base(stream);

#ifdef CONFIG_STM32F4XX
	return (base + 0x10 + 0x18 * stream->stream_num);
#else
	return (base + 0x08 + 0x14 * (stream->stream_num - 1));
#endif
}

struct dma_operations dma_ops = {
	.stream_init = stm32_dma_stream_init,
	.transfer = stm32_dma_transfer,
	.enable = stm32_dma_enable,
	.disable = stm32_dma_disable,
};

int stm32_dma_stream_of_configure(int fdt_offset, void (*handler)(void *arg), void *arg, struct dma_stream *dma_stream, int size)
{
	int i;
	int parent_phandle, parent_offset;
	int ret = 0;
	char *path = NULL;
	struct dma_controller *dma_ctrl = NULL;
	struct dma_of *dmas;
	struct device *dev = NULL;
	const void *fdt = fdtparse_get_blob();

	if (size > 2) {
		ret = -EINVAL;
		goto err_malloc;
	}

	dmas = (struct dma_of *)kmalloc(size * sizeof(struct dma_of));
	if (!dmas) {
		error_printk("failed to allocate temp dma_of\n");
		ret = -ENOMEM;
		goto err_malloc;
	}

	memset(dmas, 0, size * sizeof(struct dma_of));

	ret = fdtparse_get_u32_array(fdt_offset, "dmas", (unsigned int *)dmas, size * sizeof(struct dma_of) / sizeof(unsigned int));
	if (ret < 0)
		goto err;

	for (i = 0; i < size; i++) {
		parent_phandle = dmas[i].controller;
		parent_offset = fdt_node_offset_by_phandle(fdt, parent_phandle);

		path = fdtparse_get_path(parent_offset);
		if (!path) {
			error_printk("failed to retrieve parent dma path\n");
			ret = -ENOENT;
			goto err;
		}

		dev = device_from_of_path(path);
		if (!dev) {
			error_printk("failed to retrieve parent device struct\n");
			ret = -ENOENT;
			goto err;
		}

		dma_ctrl = container_of(dev, struct dma_controller, dev);

		dma_stream[i].dma = dma_ctrl;
		dma_stream[i].stream_num = dmas[i].stream;
		dma_stream[i].stream_base = stm32_dma_get_stream_base(&dma_stream[i]);
		dma_stream[i].channel = dmas[i].channel;
		dma_stream[i].minc = (dmas[i].dma_conf & DMA_OF_MINC_MASK);
		dma_stream[i].pinc = (dmas[i].dma_conf & DMA_OF_PINC_MASK);
		dma_stream[i].pincos = (dmas[i].dma_conf & DMA_OF_PINCOS_MASK);
		dma_stream[i].priority = (dmas[i].dma_conf & DMA_OF_PRIO_MASK);
		dma_stream[i].handler = handler;
		dma_stream[i].arg = arg;
		dma_stream[i].irq = stm32_dma_get_nvic_number(&dma_stream[i]);

#ifdef CONFIG_STM32L4XX
		/* FIXME: ugly fix */
		dma_stream[i].minc >>= 10;
		dma_stream[i].pinc >>= 9;
		dma_stream[i].pincos >>= 15;
		dma_stream[i].priority >>= 16;
#endif
	}

err:
	kfree(dmas);
err_malloc:
	return ret;
}

int stm32_dma_of_init(struct dma_controller *dma)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	int len, num, i;
	fdt32_t *cell;

	offset = fdt_path_offset(fdt_blob, dma->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, dma->dev.of_compat);
	if (ret < 0)
		goto out;

	dma->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!dma->base_reg) {
		error_printk("failed to retrieve dma controller base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

	ret = stm32_rcc_of_enable_clk(offset, &dma->clock);
	if (ret < 0) {
		error_printk("failed to retrieve dma clock\n");
		ret = -EIO;
		goto out;
	}

	prop = fdt_get_property(fdt_blob, offset, "interrupts", &len);
	if (len < 0) {
		return len;
	}

	num = len;

	cell = (fdt32_t *)prop->data;

	for(i = 0; i < num; i++, cell++)
		dma->interrupts[i] = fdt32_to_cpu(cell[0]);

	ret = stm32_rcc_of_enable_clk(offset, &dma->clock);
	if (ret < 0) {
		error_printk("failed to retrieve dma clock\n");
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

int stm32_dma_init(struct device *dev)
{
	int ret = 0;
	struct dma_controller *dma = NULL;

	dma = dma_new_controller();
	if (!dma) {
		error_printk("failed to request new dma controller\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&dma->dev, dev, sizeof(struct device));
	dma->dma_ops = &dma_ops;

	ret = stm32_dma_of_init(dma);
	if (ret < 0) {
		error_printk("failed to init dma controller with fdt data\n");
		goto err;
	}

	ret = dma_register_controller(dma);
	if (ret < 0) {
		error_printk("failed to register stm32 dma controller\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	stm32_rcc_disable_clk(dma->clock.gated, dma->clock.id);
err:
	return ret;
}

struct device stm32_dma_driver = {
	.of_compat = "st,stm32-dma",
	.probe = stm32_dma_init,
};

static int stm32_dma_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_dma_driver);
	if (ret < 0)
		error_printk("failed to register stm32_dma device\n");
	return ret;
}
postarch_initcall(stm32_dma_register);
