#include <board.h>
#include <utils.h>
#include <init.h>
#include <sizes.h>
#include <armv7m/system.h>
#include <armv7m/mpu.h>

void low_level_init(void)
{
}

int device_init(void)
{
	int ret = 0;

	return ret;
}
coredevice_initcall(device_init);
