#include <drv/clk.h>
#include <kernel/printk.h>
#include <errno.h>

/* framework only support 1 clock device */
static struct clk_device clock_dev;
static int dev_count;

int clk_get_sysfreq(void)
{
	return clock_dev.clk_ops->clk_get_sysfreq();
}

struct clk_device *clk_new_device(void)
{
	if (dev_count)
		return NULL;
	else
		return &clock_dev;
}

int clk_remove_device(struct clk_device *clk_dev)
{
	int ret;

	ret = device_unregister(&clk_dev->dev);
	if (ret < 0) {
		error_printk("failed to unregister dma controller");
		return ret;
	}

	return ret;
}

int clk_register_device(struct clk_device *clk_dev)
{
	int ret;

	ret = device_register(&clk_dev->dev);
	if (ret < 0)
		error_printk("failed to register dma controller\n");

	return ret;
}
