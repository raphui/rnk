#include <board.h>
#include <mach/rcc-stm32.h>
#include <mach/pwr-stm32.h>
#include <errno.h>
#include <fdtparse.h>
#include <kernel/printk.h>
#include <drv/clk.h>
#include <init.h>

#define LSI_FREQ	32000

struct clk_div_table {
        unsigned int val;
        unsigned int div;
};

static const struct clk_div_table ahb_div_table[] = {
        { 0x0,   1 }, { 0x1,   1 }, { 0x2,   1 }, { 0x3,   1 },
        { 0x4,   1 }, { 0x5,   1 }, { 0x6,   1 }, { 0x7,   1 },
        { 0x8,   2 }, { 0x9,   4 }, { 0xa,   8 }, { 0xb,  16 },
        { 0xc,  64 }, { 0xd, 128 }, { 0xe, 256 }, { 0xf, 512 },
        { 0 },
};

static const struct clk_div_table apb_div_table[] = {
        { 0,  1 }, { 0,  1 }, { 0,  1 }, { 0,  1 },
        { 4,  2 }, { 5,  4 }, { 6,  8 }, { 7, 16 },
        { 0 },
};

#ifdef CONFIG_STM32L4XX
static const int clk_msi_range[] = {
	100000,
	200000,
	400000,
	800000,
	1000000,
	2000000,
	4000000,
	8000000,
	16000000,
	24000000,
	32000000,
	48000000,
};
#endif

static int sysclk;
static int sysclk_freq;
static int ahb_freq;
static int apb1_freq;
static int apb2_freq;
static int ahb_pres;
static int apb1_pres;
static int apb2_pres;

#ifdef CONFIG_STM32L4XX
static int stm32_rcc_find_msi_range(int freq)
{
	int i;
	int ret = -EINVAL;
	int size = sizeof(clk_msi_range) / sizeof(int);

	for (i = 0; i < size; i++) {
		if (freq == clk_msi_range[i]) {
			ret = i;
			break;
		}
	}

	return ret;
}
#endif

static int stm32_rcc_find_ahb_div(int pres)
{
	int i;
	int ret = -EINVAL;
	int size = sizeof(ahb_div_table) / sizeof(struct clk_div_table);

	for (i = 0; i < size; i++) {
		if (pres == ahb_div_table[i].div) {
			ret = ahb_div_table[i].val;
			break;
		}
	}

	return ret;
}

static int stm32_rcc_find_apb_div(int pres)
{
	int i;
	int ret = -EINVAL;
	int size = sizeof(apb_div_table) / sizeof(struct clk_div_table);

	for (i = 0; i < size; i++) {
		if (pres == apb_div_table[i].div) {
			ret = apb_div_table[i].val;
			break;
		}
	}

	return ret;
}

static int stm32_rcc_enable_internal_clk(unsigned int clk)
{
	int ret = 0;

	switch (clk) {
	case CLK_LSI:
		RCC->CSR |= RCC_CSR_LSION;

		while (!(RCC->CSR & RCC_CSR_LSIRDY))
			;
		break;
	case CLK_HSI:
		RCC->CR |= RCC_CR_HSION;

		while (!(RCC->CR & RCC_CR_HSIRDY))
			;
		break;
	case CLK_RTC:
#ifdef CONFIG_STM32L4XX
		RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
		PWR->CR1 |= PWR_CR1_DBP;
#endif
		RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_1;
		break;
#ifdef CONFIG_STM32L4XX
	case CLK_LPTIMER:
		/* FIXME: Hard set to LSI clock, it would be nice to make it more clever */
		RCC->CCIPR |= (1 << RCC_CCIPR_LPTIM1SEL_Pos);
		RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
		break;
#ifndef CONFIG_STM32L476
	case CLK_USB:
		RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL;
		RCC->APB1ENR1 |= RCC_APB1ENR1_USBFSEN;
		break;
	case CLK_HSI48:
		RCC->CRRCR |= RCC_CRRCR_HSI48ON;

		while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY))
			;
		break;
#endif
#endif
	default:
		ret = -EINVAL;
		break;
	}

	return ret;

}

static int stm32_rcc_disable_internal_clk(unsigned int clk)
{
	int ret = 0;

	switch (clk) {
	case CLK_LSI:
		RCC->CSR &= ~RCC_CSR_LSION;
		break;
	case CLK_HSI:
		RCC->CR &= ~RCC_CR_HSION;
		break;
	case CLK_RTC:
		RCC->BDCR &= ~RCC_BDCR_RTCEN;
		break;
#ifdef CONFIG_STM32L4XX
	case CLK_LPTIMER:
		RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;
		break;
	case CLK_USB:
		RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL;
		break;
#ifndef CONFIG_STM32L476
	case CLK_HSI48:
		RCC->CRRCR &= ~RCC_CRRCR_HSI48ON;
		break;
#endif
#endif
	default:
		ret = -EINVAL;
		break;
	}

	return ret;

}

static int stm32_rcc_enable_gated_clk(int id)
{
	int ret = 0;
	int bus_id = id / 32;
	volatile unsigned int *p;

	p = (volatile unsigned int *)(&RCC->AHB1ENR + bus_id);

	id %= 32;

	*p |= (1 << id);

#ifdef CONFIG_STM32F4XX
	if (bus_id < 2)
		ret = ahb_freq;
	else if (bus_id < 3)
		ret = apb1_freq;
	else
		ret = apb2_freq;
#else
	if (bus_id < 3)
		ret = ahb_freq;
	else if (bus_id < 5)
		ret = apb1_freq;
	else
		ret = apb2_freq;
#endif

	return ret;
}

static int stm32_rcc_disable_gated_clk(int id)
{
	int ret = 0;
	int bus_id = id / 32;
	volatile unsigned int *p;

	p = (volatile unsigned int *)(&RCC->AHB1ENR + bus_id);

	id %= 32;

	*p &= ~(1 << id);

	return ret;
}

static void stm32_rcc_set_sysclk(int source)
{
	RCC->CFGR &= (unsigned int)((unsigned int)~(RCC_CFGR_SW));

	switch (source) {
	case CLK_HSI:
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		while ((RCC->CFGR & (unsigned int)RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI)
			;
		break;
#ifdef CONFIG_STM32L4XX
	case CLK_MSI:
		RCC->CFGR |= RCC_CFGR_SW_MSI;
		while ((RCC->CFGR & (unsigned int)RCC_CFGR_SWS ) != RCC_CFGR_SWS_MSI)
			;
#endif
	case CLK_LSI:
	case CLK_LSE:
		break;
	case CLK_PLL:
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		while ((RCC->CFGR & (unsigned int)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
			;
		break;
	}

	sysclk = source;
}

static void stm32_rcc_adjust_flash_ws(int freq)
{
	if (freq <= 16000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_0WS;
	else if (freq <= 32000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
	else if (freq <= 48000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
	else if (freq <= 64000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
	else if (freq <= 80000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
#ifdef CONFIG_STM32F4XX
	else if (freq <= 84000000)
		FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
#endif
}

static int stm32_rcc_find_parent_clock(int offset, int *source_freq)
{
	int ret = -ENOENT;
	int len;
	int parent_phandle;
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	fdt32_t *cell;

	ret = fdtparse_get_int(offset, "clock-frequency", source_freq);
	if (ret)
		*source_freq = 0; // means field is not present, set to 0 and try in parent node

	prop = fdt_get_property(fdt_blob, offset, "assigned-clock", &len);
	if (prop) {
		cell = (fdt32_t *)prop->data;

		parent_phandle = fdt32_to_cpu(cell[0]);
		offset = fdt_node_offset_by_phandle(fdt_blob, parent_phandle);
	}

	prop = fdt_get_property(fdt_blob, offset, "clocks", &len);
	if (!prop) {
		error_printk("cannot find source clock in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	if (len <= 3) {
		error_printk("not enough parameters for clock\n");
		ret = -EINVAL;
		goto out;
	}

	cell = (fdt32_t *)prop->data;

	ret = fdt32_to_cpu(cell[2]);

	if (!*source_freq)
		fdtparse_get_int(offset, "clock-frequency", source_freq);

out:
	return ret;
}

static int stm32_rcc_get_sysfreq(void)
{
	return sysclk_freq;
}

int stm32_rcc_get_freq_clk(unsigned int clk)
{
	int ret = 0;

	switch (clk) {
	case CLK_SYSCLK:
		ret = sysclk_freq;
		break;
	case CLK_AHB:
		ret = ahb_freq;
		break;
	case CLK_APB1:
		ret = apb1_freq;
		break;
	case CLK_APB2:
		ret = apb2_freq;
		break;
	case CLK_LSI:
		ret = LSI_FREQ;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int stm32_rcc_get_pres_clk(unsigned int clk)
{
	int ret = 0;

	switch (clk) {
	case CLK_AHB:
		ret = ahb_pres;
		break;
	case CLK_APB1:
		ret = apb1_pres;
		break;
	case CLK_APB2:
		ret = apb2_pres;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int stm32_rcc_enable_clk(int secondary, int id)
{
	int ret = 0;

	if (secondary)
		ret = stm32_rcc_enable_internal_clk(id);
	else
		ret = stm32_rcc_enable_gated_clk(id);

	return ret;
}

int stm32_rcc_disable_clk(int secondary, int id)
{

	int ret = 0;

	if (secondary)
		ret = stm32_rcc_disable_internal_clk(id);
	else
		ret = stm32_rcc_disable_gated_clk(id);

	return ret;
}

int stm32_rcc_of_enable_clk(int offset, struct clk *clk)
{
	int ret = 0;
	int len;
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	int parent_phandle, parent_offset;
	fdt32_t *cell;

	prop = fdt_get_property(fdt_blob, offset, "clocks", &len);
	if (!prop) {
		error_printk("cannot find clock in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	if (len < 0) {
		error_printk("not enough parameters for clock\n");
		ret = -EINVAL;
		goto out;
	}

	cell = (fdt32_t *)prop->data;

	ret = stm32_rcc_enable_clk(fdt32_to_cpu(cell[1]), fdt32_to_cpu(cell[2]));

	if (clk) {
		clk->gated = !fdt32_to_cpu(cell[1]);
		clk->id = fdt32_to_cpu(cell[2]);

		if (!clk->gated) {
			parent_phandle = fdt32_to_cpu(cell[0]);
			parent_offset = fdt_node_offset_by_phandle(fdt_blob, parent_phandle);

			fdtparse_get_int(parent_offset, "clock-frequency", (int *)&clk->source_clk);

			/* maybe parent clock is not init... */
			prop = fdt_get_property(fdt_blob, parent_offset, "clocks", &len);
			cell = (fdt32_t *)prop->data;
			ret = stm32_rcc_enable_clk(fdt32_to_cpu(cell[1]), fdt32_to_cpu(cell[2]));

		} else {
			clk->source_clk = ret;
		}
	}

out:
	return ret;
}

int stm32_rcc_set_sysclk_freq(int freq)
{
	int ret = 0;
	int reg;
	int msi_range;

	switch (sysclk) {
	case CLK_HSI:
		break;
#ifdef CONFIG_STM32L4XX
	case CLK_MSI:
		msi_range = stm32_rcc_find_msi_range(freq);

		sysclk_freq = freq;
		ahb_freq = sysclk_freq / ahb_pres;
		apb1_freq = (sysclk_freq / ahb_pres) / apb1_pres;
		apb2_freq = (sysclk_freq / ahb_pres) / apb2_pres;

		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		stm32_rcc_adjust_flash_ws(sysclk_freq);

		reg = RCC->CR;
		reg &= ~RCC_CR_MSIRANGE_Msk;
		reg |= (msi_range << RCC_CR_MSIRANGE_Pos);

		RCC->CR = reg;

		while (!(RCC->CR & RCC_CR_MSIRDY))
			;

		/* Select regulator voltage output Scale 1 mode */
		stm32_pwr_set_regulator(freq);
#endif
	case CLK_LSI:
	case CLK_LSE:
	case CLK_PLL:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int stm32_rcc_enable_sys_clk(void)
{
	int ret = 0;
	int offset;
	int len;
#ifdef CONFIG_STM32F4XX
	int sysclk_source, pll_m, pll_q, pll_n, pll_p;
#else
	int msi_range;
	int sysclk_source, pll_m, pll_q, pll_n, pll_r;
#endif

	int div_ahb, div_apb1, div_apb2;
	int source, source_freq;
	const void *fdt_blob = fdtparse_get_blob();
	const struct fdt_property *prop;
	int parent_offset, parent_phandle;
	fdt32_t *cell;

	offset = fdt_path_offset(fdt_blob, "/clocks/sysclk");
	if (offset < 0) {
		error_printk("cannot find clock definition in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	prop = fdt_get_property(fdt_blob, offset, "clocks", &len);
	if (!prop) {
		error_printk("cannot find source clock in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	if (len <= 0) {
		error_printk("not enough parameters for source\n");
		ret = -EINVAL;
		goto out;
	}

	cell = (fdt32_t *)prop->data;

	parent_phandle = fdt32_to_cpu(cell[0]);
	parent_offset = fdt_node_offset_by_phandle(fdt_blob, parent_phandle);
	sysclk_source = fdt32_to_cpu(cell[2]);

	prop = fdt_get_property(fdt_blob, offset, "prescaler", &len);
	if (!prop) {
		error_printk("cannot find prescaler clocks in fdt\n");
		ret = -ENOENT;
		goto out;
	}

	if (len < 5) {
		error_printk("not enough parameters for prescaler (has to be 3)\n");
		ret = -EINVAL;
		goto out;
	}

	cell = (fdt32_t *)prop->data;

	ahb_pres = fdt32_to_cpu(cell[0]);
	div_ahb = stm32_rcc_find_ahb_div(ahb_pres);
	if (div_ahb < 0) {
		error_printk("cannot find ahb pres value\n");
		ret = div_ahb;
		goto out;
	}

	apb1_pres = fdt32_to_cpu(cell[1]);
	div_apb1 = stm32_rcc_find_apb_div(apb1_pres);
	if (div_apb1 < 0) {
		error_printk("cannot find apb1 pres value\n");
		ret = div_apb1;
		goto out;
	}

	apb2_pres = fdt32_to_cpu(cell[2]);
	div_apb2 = stm32_rcc_find_apb_div(apb2_pres);
	if (div_apb2 < 0) {
		error_printk("cannot find ap2 pres value\n");
		ret = div_apb2;
		goto out;
	}

#ifdef CONFIG_STM32F4XX
	RCC->CFGR |= (div_ahb << 4);
	RCC->CFGR |= (div_apb1 << 10);
	RCC->CFGR |= (div_apb2 << 13);
#else
	RCC->CFGR |= (div_ahb << RCC_CFGR_HPRE_Pos);
	RCC->CFGR |= (div_apb1 << RCC_CFGR_PPRE1_Pos);
	RCC->CFGR |= (div_apb2 << RCC_CFGR_PPRE2_Pos);
#endif

	switch (sysclk_source) {
	case CLK_HSI:
	case CLK_LSI:
	case CLK_LSE:
		ret = fdtparse_get_int(offset, "clock-frequency", &source_freq);
		if (ret < 0) {
			error_printk("failed to retrieve sysclk source freq\n");
			ret = -EIO;
			goto out;
		}

		sysclk_freq = source_freq;
		break;
#ifdef CONFIG_STM32L4XX
	case CLK_MSI:
		source = stm32_rcc_find_parent_clock(parent_offset, &source_freq);

		msi_range = stm32_rcc_find_msi_range(source_freq);

		RCC->CR |= RCC_CR_MSIRGSEL;
		RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk) | (msi_range << RCC_CR_MSIRANGE_Pos);

		while (!(RCC->CR & RCC_CR_MSIRDY))
			;

		sysclk_freq = source_freq;
		break;
#endif
	case CLK_PLL:
		source = stm32_rcc_find_parent_clock(parent_offset, &source_freq);

		prop = fdt_get_property(fdt_blob, parent_offset, "settings", &len);
		if (!prop) {
			error_printk("cannot find source clock in fdt\n");
			ret = -ENOENT;
			goto out;
		}

		if (len <= 4) {
			error_printk("not enough parameters for source\n");
			ret = -EINVAL;
			goto out;
		}

		cell = (fdt32_t *)prop->data;

		pll_m = fdt32_to_cpu(cell[0]);
		pll_n = fdt32_to_cpu(cell[1]);
		pll_q = fdt32_to_cpu(cell[3]);
#ifdef CONFIG_STM32F4XX
		pll_p = fdt32_to_cpu(cell[2]);

		RCC->PLLCFGR = pll_m | (pll_n << 6) | (((pll_p >> 1) -1) << 16) | (pll_q << 24);
#else
		pll_r = fdt32_to_cpu(cell[2]);
		RCC->PLLCFGR = (pll_m << 4) | (pll_n << 8) | (((pll_r >> 1) -1) << 25) | (pll_q << 21);
#endif


		if (source == CLK_HSI) {
			RCC->CR |= RCC_CR_HSION;

			while (!(RCC->CR & RCC_CR_HSIRDY))
				;
		} else {

			RCC->CR |= RCC_CR_HSEON;

			// Wait till HSE is ready
			while(!(RCC->CR & RCC_CR_HSERDY))
				;

			RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
		}
			
		RCC->CR |= RCC_CR_PLLON;

		while(!(RCC->CR & RCC_CR_PLLRDY))
			; // Wait till the main PLL is ready

#ifdef CONFIG_STM32F4XX
		sysclk_freq = ((source_freq / pll_m) * pll_n) / pll_p;
#else
		sysclk_freq = ((source_freq / pll_m) * pll_n) / pll_r;
#endif

		break;
	}

	ahb_freq = sysclk_freq / ahb_pres;
	apb1_freq = (sysclk_freq / ahb_pres) / apb1_pres;
	apb2_freq = (sysclk_freq / ahb_pres) / apb2_pres;

	/* Select regulator voltage output Scale 1 mode */
#ifdef CONFIG_STM32F4XX
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_DBP;
#else
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	stm32_pwr_set_regulator(sysclk_freq);
#endif

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	stm32_rcc_adjust_flash_ws(sysclk_freq);

	FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

	stm32_rcc_set_sysclk(sysclk_source);

out:
	return ret;
}


struct clk_operations clk_ops = {
	.clk_get_sysfreq = stm32_rcc_get_sysfreq,
};

int stm32_rcc_init(struct device *dev)
{
	int ret = 0;
	struct clk_device *clk_dev = NULL;

	clk_dev = clk_new_device();
	if (!clk_dev) {
		error_printk("failed to request new clk device\n");
		ret = -EIO;
		goto err;
	}

	memcpy(&clk_dev->dev, dev, sizeof(struct device));
	clk_dev->clk_ops = &clk_ops;

err:
	return ret;
}

struct device stm32_rcc_driver = {
	.of_compat = "st,stm32-rcc",
	.probe = stm32_rcc_init,
};

static int stm32_rcc_register(void)
{
	int ret = 0;

	ret = device_of_register(&stm32_rcc_driver);
	if (ret < 0) {
		error_printk("failed to register stm32_rcc device\n");
		goto err;
	}

	ret = stm32_rcc_enable_sys_clk();

err:
	return ret;
}
arch_initcall(stm32_rcc_register);
