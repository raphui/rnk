#include <board.h>
#include <fsmc.h>


static unsigned int fsmc_rcc_bit[] = {
	RCC_APB1ENR_I2C1EN,
	RCC_APB1ENR_I2C2EN,
	RCC_APB1ENR_I2C3EN,
};

static unsigned int fsmc_bus_base[] = {
	FSMC_Bank1_R_BASE,
	FSMC_Bank1E_R_BASE,
	FSMC_Bank2_3_R_BASE,
	FSMC_Bank4_R_BASE,
};

void stm32_fsmc_init(struct fsmc *fsmc)
{

}
