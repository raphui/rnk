#include <dma.h>
#include <errno.h>
#include <string.h>
#include <printk.h>
#include <init.h>
#include <mm.h>

static int dev_count = 0;
static struct list_node dma_controller_list;

int dma_transfer(struct dma_stream *dma_stream, struct dma_transfer *dma_trans)
{
	return dma_stream->dma->dma_ops->transfer(dma_stream, dma_trans);
}

int dma_enable(struct dma_stream *dma_stream)
{
	return dma_stream->dma->dma_ops->enable(dma_stream);
}

int dma_disable(struct dma_stream *dma_stream)
{
	return dma_stream->dma->dma_ops->disable(dma_stream);
}

struct dma_controller *dma_new_controller(void)
{
	struct dma_controller *dma = NULL;

	dma = (struct dma_controller *)kmalloc(sizeof(struct dma_controller));
	if (!dma) {
		error_printk("cannot allocate dma controller");
		return NULL;
	}

	memset(dma, 0, sizeof(struct dma_controller));

	dev_count++;

	return dma;
}

int dma_remove_controller(struct dma_controller *dma)
{
	int ret = 0;
	struct dma_controller *dmadev = NULL;

	ret = device_unregister(&dma->dev);
	if (ret < 0) {
		error_printk("failed to unregister dma controller");
		return ret;
	}

	list_for_every_entry(&dma_controller_list, dmadev, struct dma_controller, node)
		if (dmadev == dma)
			break;

	if (dmadev) {
		list_delete(&dmadev->node);
		kfree(dmadev);
	}
	else
		ret = -ENOENT;



	dev_count--;

	return ret;
}

int dma_register_controller(struct dma_controller *dma)
{
	int ret = 0;

	list_add_tail(&dma_controller_list, &dma->node);

	ret = device_register(&dma->dev);
	if (ret < 0)
		error_printk("failed to register dma controller\n");

	return ret;
}

int dma_init(void)
{
	int ret = 0;

	list_initialize(&dma_controller_list);

	return ret;
}
postcore_initcall(dma_init);
