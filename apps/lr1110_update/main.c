#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <gpiolib.h>
#include <errno.h>
#include <time.h>
#include "lr1110_update.h"
#include "lr1110_modem_1.1.7.h"

#define DEMO_VERSION "v2.3.0"

/* PA15 */
#define BUSY_PIN	16
/* PB4 */
#define RADIO_EVENT_PIN	21

#define SPI_DEVICE	"/dev/spi1"

int main(void)
{
	lr11xx_fw_update_status_t status;
	struct lr1110_update *lr1110_update;

	printf( "LR11XX updater tool %s\n", DEMO_VERSION );

	lr1110_update = malloc(sizeof(*lr1110_update));
	if (!lr1110_update) {
		printf("cannot allocate lr1110_update\n");
		return -ENOMEM;
	}

	lr1110_update->spi_fd = open(SPI_DEVICE, O_RDWR);
	if (lr1110_update->spi_fd < 0) {
		printf("failed to open spi device\n");
		free(lr1110_update);
		return -EIO;
	}

	lr1110_update->busy = gpiolib_export(BUSY_PIN);
	lr1110_update->radio_event = gpiolib_export(RADIO_EVENT_PIN);

	gpiolib_set_input(lr1110_update->busy);
	gpiolib_set_output(lr1110_update->radio_event, 1);

	time_usleep(2000 * 1000);

	switch (LR11XX_FIRMWARE_UPDATE_TO) {
	case LR1110_FIRMWARE_UPDATE_TO_TRX:
		printf( "Update LR1110 to transceiver firmware 0x%04x\n", LR11XX_FIRMWARE_VERSION );
		break;
	case LR1120_FIRMWARE_UPDATE_TO_TRX:
		printf( "Update LR1120 to transceiver firmware 0x%04x\n", LR11XX_FIRMWARE_VERSION );
		break;
	case LR1121_FIRMWARE_UPDATE_TO_TRX:
		printf( "Update LR1121 to transceiver firmware 0x%04x\n", LR11XX_FIRMWARE_VERSION );
		break;
	case LR1110_FIRMWARE_UPDATE_TO_MODEM:
		printf( "Update LR1110 to modem firmware 0x%06x\n", LR11XX_FIRMWARE_VERSION );
		break;
	}

	while (1) {
		if (lr1110_update->is_updated == false) {
			status = lr11xx_update_firmware((void *)lr1110_update, LR11XX_FIRMWARE_UPDATE_TO, LR11XX_FIRMWARE_VERSION,
						lr11xx_firmware_image, ( uint32_t ) LR11XX_FIRMWARE_IMAGE_SIZE);

			switch (status) {
			case LR11XX_FW_UPDATE_OK:
				printf( "Expected firmware running!\n" );
				printf( "Please flash another application (like EVK Demo App).\n" );
				break;
			case LR11XX_FW_UPDATE_WRONG_CHIP_TYPE:
				printf( "Wrong chip type!\n" );
				break;
			case LR11XX_FW_UPDATE_ERROR:
				printf( "Error! Wrong firmware version - please retry.\n" );
				break;
			}

			lr1110_update->is_updated = true;
		}
	}
}
