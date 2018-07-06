#include <board.h>
#include <drv/lcd.h>
#include <utils.h>
#include <kernel/printk.h>
#include <mach/pio-stm32.h>
#include <mach/rcc-stm32.h>
#include <errno.h>
#include <drv/device.h>
#include <init.h>
#include <string.h>
#include <fdtparse.h>

#define GCR_MASK		((unsigned int)0x0FFE888F)
#define RCC_PLLSAIDivR_Div4	((unsigned int)0x00010000)

#define GPIO_AF_LTDC	((unsigned char)0x0E)
#define GPIO_AF_LCD	((unsigned char)0x09)

struct lcd_operations lcd_ops;

static void stm32_ltdc_pll_sai_config(unsigned int n, unsigned int q, unsigned int r)
{
	RCC->PLLSAICFGR = (n << 6) | (q << 24) | (r << 28);
}

static void stm32_ltdc_clk_divconfig(unsigned int div_r)
{
	RCC->DCKCFGR &= ~RCC_DCKCFGR_PLLSAIDIVR;
	RCC->DCKCFGR |= div_r;
}

static int stm32_ltdc_enable_fb(struct lcd *ltdc)
{
	int ret = 0;

	LTDC_Layer1->WHPCR = ((ltdc->hsync + ltdc->hbp) << 0) | ((ltdc->hsync + ltdc->hbp + ltdc->width - 1) << 16);
	LTDC_Layer1->WVPCR = ((ltdc->vsync + ltdc->vbp) << 0) | ((ltdc->vsync + ltdc->vbp + ltdc->height - 1) << 16);

	switch (ltdc->bpp) {
		case 2:
			LTDC_Layer1->PFCR = 2;
			break;
		default:
			debug_printk("Unknow bits per pixel\r\n");
			return -EINVAL;
	}

//	LTDC_Layer1->BFCR = 0x00000400 | 0x00000005;

	LTDC_Layer1->CFBAR = ltdc->fb_addr;
	LTDC_Layer1->CFBLR = ((ltdc->width * ltdc->bpp) << 16) | ((ltdc->width * ltdc->bpp + 3) << 0);
	LTDC_Layer1->CFBLNR = (ltdc->height << 0);

//	LTDC_Layer1->CACR &= ~LTDC_LxCACR_CONSTA;

	LTDC_Layer1->CR |= LTDC_LxCR_LEN;

	return ret;
}

static int stm32_ltdc_configure(struct lcd *ltdc)
{
	int ret = 0;
	unsigned int h_cycles;
	unsigned int v_cycles;

	LTDC->GCR &= ~LTDC_GCR_LTDCEN;
	LTDC->GCR |= LTDC_GCR_DTEN;

	h_cycles = ltdc->hsync - 1;
	v_cycles = ltdc->vsync - 1;

	LTDC->SSCR = (h_cycles << 16) | (v_cycles << 0);

	h_cycles += ltdc->hbp;
	v_cycles += ltdc->vbp;

	LTDC->BPCR = (h_cycles << 16) | (v_cycles << 0);

	h_cycles += ltdc->width;
	v_cycles += ltdc->height;

	LTDC->AWCR = (h_cycles << 16) | (v_cycles << 0);

	h_cycles += ltdc->hfp;
	v_cycles += ltdc->vfp;

	LTDC->TWCR = (h_cycles << 16) | (v_cycles << 0);

	/* Clean general config */
	LTDC->GCR &= ~(GCR_MASK);

	/* Background color to blue */
	LTDC->BCCR = (0xFF << 0);

	ret = stm32_ltdc_enable_fb(ltdc);
	if (ret < 0) {
		error_printk("failed to enable framebuffer\r\n");
		goto out;
	}

	LTDC->SRCR = LTDC_SRCR_IMR;

	/* Enable LCD Controller */
	LTDC->GCR |= LTDC_GCR_LTDCEN;

out:
	return ret;
}

static int stm32_ltdc_of_init(struct lcd *ltdc)
{
	int offset;
	int ret = 0;
	const void *fdt_blob = fdtparse_get_blob();

	offset = fdt_path_offset(fdt_blob, ltdc->dev.of_path);
	if (offset < 0) {
		ret = -ENOENT;
		goto out;
	}

	ret = fdt_node_check_compatible(fdt_blob, offset, ltdc->dev.of_compat);
	if (ret < 0)
		goto out;

	ltdc->base_reg = (unsigned int)fdtparse_get_addr32(offset, "reg");
	if (!ltdc->base_reg) {
		error_printk("failed to retrieve ltdc base reg from fdt\n");
		ret = -ENOENT;
		goto out;
	}

//	ret = fdtparse_get_u16_array(offset, "sync", (unsigned short *)&ltdc->hsync, 2 * sizeof(unsigned short));
//	if (ret < 0)
//		goto out;
//
//	ret = fdtparse_get_u16_array(offset, "porch", (unsigned short *)&ltdc->hbp, 4 * sizeof(unsigned short));
//	if (ret < 0)
//		goto out;
//
//	ret = fdtparse_get_u32_array(offset, "resolution", (unsigned int *)&ltdc->width, 2 * sizeof(unsigned int));
//	if (ret < 0)
//		goto out;
//
//	ret = fdtparse_get_int(offset, "bpp", &ltdc->bpp);
//	if (ret < 0)
//		goto out;

out:
	return ret;
}

static int stm32_ltdc_init(struct device *dev)
{
	int ret = 0;
	struct lcd *ltdc;

	ltdc = lcd_new_device();
	if (!ltdc) {
		error_printk("failed to request new lcd device\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&ltdc->dev, dev, sizeof(struct device));
	ltdc->lcd_ops = &lcd_ops;

	ret = stm32_ltdc_of_init(ltdc);
	if (ret < 0) {
		error_printk("failed to init ltdc with fdt data\n");
		goto err;
	}

	ret = stm32_rcc_enable_clk(ltdc->base_reg);
	if (ret < 0) {
		error_printk("cannot enable LTDC periph clock\r\n");
		goto err;
	}

	/* XXX: retrieve all of these from device tree */
	stm32_ltdc_pll_sai_config(127, 7, 5);
	stm32_ltdc_clk_divconfig(RCC_PLLSAIDivR_Div4);

	RCC->CR |= RCC_CR_PLLSAION;

	while (!(RCC->CR & RCC_CR_PLLSAIRDY))
		;

	ret = lcd_register_device(ltdc);
	if (ret < 0) {
		error_printk("failed to register ltdc device\n");
		goto disable_clk;
	}

	return 0;

disable_clk:
	stm32_rcc_disable_clk(ltdc->base_reg);
err:
	return ret;
}

struct lcd_operations lcd_ops = {
	.configure = stm32_ltdc_configure,
};

struct device stm32_ltdc_driver = {
	.of_compat = "st,stm32f4xx-ltdc",
	.probe = stm32_ltdc_init,
};

static int stm32_ltdc_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_ltdc_driver);
	if (ret < 0)
		error_printk("failed to register stm32_ltdc device\n");
	return ret;
}
postarch_initcall(stm32_ltdc_register);
