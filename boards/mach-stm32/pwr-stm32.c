#include <board.h>
#include <utils.h>
#include <mm/mm.h>
#include <kernel/printk.h>
#include <drv/device.h>
#include <mach/rcc-stm32.h>
#include <armv7m/nvic.h>
#include <fdtparse.h>
#include <init.h>
#include <errno.h>
#include <ioctl.h>
#include <drv/clk.h>

struct pwr {
	struct device dev;
};

void stm32_pwr_enter_lpsleep(int mode)
{
	PWR->CR1 |= mode;
}

static int stm32_pwr_of_init(struct pwr *pwr)
{
	int offset;
	int ret = 0;
	struct clk clock;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, pwr->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, pwr->dev.of_compat);
	if (ret < 0)
		goto out;

	ret = stm32_rcc_of_enable_clk(offset, &clock);
	if (ret < 0) {
		error_printk("failed to retrieve pwr clock\n");
		ret = -EIO;
	}

out:
	return ret;
}

static int stm32_pwr_init(struct device *dev)
{
	int ret = 0;
	struct pwr *pwr = NULL;

	pwr = kmalloc(sizeof(*pwr));
	if (!pwr) {
		error_printk("failed to allocate pwr structure\n");
		ret = ENOMEM;
		goto err;
	}

	memcpy(&pwr->dev, dev, sizeof(struct device));

	ret = stm32_pwr_of_init(pwr);
	if (ret < 0) {
		error_printk("failed to init pwr with fdt data\n");
		goto err_alloc;
	}

	return 0;

err_alloc:
	kfree(pwr);
err:
	return ret;
}

struct device stm32_pwr_driver = {
	.of_compat = "st,stm32-pwr",
	.probe = stm32_pwr_init,
};

static int stm32_pwr_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_pwr_driver);
	if (ret < 0)
		error_printk("failed to register stm32_pwr device\n");
	return ret;
}
postarch_initcall(stm32_pwr_register);
