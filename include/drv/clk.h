#ifndef CLK_H
#define CLK_H

#include <drv/device.h>

struct clk
{
	unsigned int id;
	unsigned int gated;
	unsigned int source_clk;
};

struct clk_device
{
	struct device dev;
	struct list_node node;
	struct clk_operations *clk_ops;
};

struct clk_operations
{
	int (*clk_get_sysfreq)(void);
	int (*clk_enable)(struct clk *clock);
	int (*clk_disable)(struct clk *clock);
};

int clk_get_sysfreq(void);
struct clk_device *clk_new_device(void);
int clk_register_device(struct clk_device *clk_dev);
int clk_remove_device(struct clk_device *clk_dev);

#endif /* CLK_H */
