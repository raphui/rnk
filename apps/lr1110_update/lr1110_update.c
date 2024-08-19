#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <gpiolib.h>
#include <time.h>

#include "lr1110_update.h"

bool lr11xx_is_chip_in_production_mode(uint8_t type);

bool lr11xx_is_fw_compatible_with_chip(lr11xx_fw_update_t update, uint16_t bootloader_version);

int lr11xx_update_init(void *context)
{
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;
	lr11xx_bootloader_version_t version_bootloader = { 0 };
	lr11xx_system_version_t version_trx = { 0x00 };
	lr11xx_system_uid_t     uid         = { 0x00 };
	lr1110_modem_version_t version_modem = { 0 };
	lr11xx_bootloader_stat1_t stat1;
	lr11xx_bootloader_stat2_t stat2;
	lr11xx_bootloader_irq_mask_t irq_status;

	printf("Reset the chip...\n");

#if 0
	gpiolib_output_set_value(lr1110_update->radio_event, 1);
//	gpiolib_set_pull_up(lr1110_update->busy, GPIO_PULL_DOWN);
	gpiolib_set_output(lr1110_update->busy, 0);

	ioctl(lr1110_update->spi_fd, IOCTL_RESET, 0);

	time_usleep(500 * 1000);
	gpiolib_set_input(lr1110_update->busy);
	time_usleep(100 * 1000);
	gpiolib_output_set_value(lr1110_update->radio_event, 0);
#else
	lr1110_modem_system_reboot(context, true);
	time_usleep(2000 * 1000);
	//lr11xx_bootloader_get_status(context, &stat1, &stat2, &irq_status);
#endif

	printf("> Reset done!\n");

	lr11xx_bootloader_get_version(context, &version_bootloader);
	printf("Chip in bootloader mode:\n");
	printf(" - Chip type               = 0x%02X (0xDF for production)\n", version_bootloader.type);
	printf(" - Chip hardware version   = 0x%02X (0x22 for V2C)\n", version_bootloader.hw);
	printf(" - Chip bootloader version = 0x%04X \n", version_bootloader.fw);

	if (lr11xx_is_chip_in_production_mode(version_bootloader.type) == false) {
		return -EINVAL;
		//return LR11XX_FW_UPDATE_WRONG_CHIP_TYPE;
	}

	if (lr11xx_is_fw_compatible_with_chip(LR1110_FIRMWARE_UPDATE_TO_MODEM, version_bootloader.fw) == false) {
		return -EINVAL;
		//return LR11XX_FW_UPDATE_WRONG_CHIP_TYPE;
	}

	lr11xx_bootloader_pin_t      pin      = { 0x00 };
	lr11xx_bootloader_chip_eui_t chip_eui = { 0x00 };
	lr11xx_bootloader_join_eui_t join_eui = { 0x00 };

	lr11xx_bootloader_read_pin(context, pin);
	lr11xx_bootloader_read_chip_eui(context, chip_eui);
	lr11xx_bootloader_read_join_eui(context, join_eui);

	printf("PIN is     0x%02X%02X%02X%02X\n", pin[0], pin[1], pin[2], pin[3]);
	printf("ChipEUI is 0x%02X%02X%02X%02X%02X%02X%02X%02X\n", chip_eui[0], chip_eui[1], chip_eui[2], chip_eui[3],
			chip_eui[4], chip_eui[5], chip_eui[6], chip_eui[7]);
	printf("JoinEUI is 0x%02X%02X%02X%02X%02X%02X%02X%02X\n", join_eui[0], join_eui[1], join_eui[2], join_eui[3],
			join_eui[4], join_eui[5], join_eui[6], join_eui[7]);

	printf("Start flash erase...\n");
	lr11xx_bootloader_erase_flash(context);
	printf("> Flash erase done!\n");

	return 0;
}

int lr11xx_update_finalize(void *context, uint32_t fw_expected)
{
	lr1110_modem_version_t version_modem = { 0 };

	printf("Rebooting...\n");
	lr11xx_bootloader_reboot(context, false);
	printf("> Reboot done!\n");

	time_usleep(2000 * 1000);

	lr1110_modem_get_version(context, &version_modem);
	printf("Chip in LoRa Basics Modem-E mode:\n");
	printf(" - Chip bootloader version = 0x%08x\n", version_modem.bootloader);
	printf(" - Chip firmware version   = 0x%08x\n", version_modem.firmware);
	printf(" - Chip LoRaWAN version    = 0x%04x\n", version_modem.lorawan);

	uint32_t fw_version = ((uint32_t) (version_modem.functionality) << 24) + version_modem.firmware;

	if (fw_version == fw_expected) {
		return 0;
	} else {
		return -EINVAL;
	}

	return 0;
}

lr11xx_fw_update_status_t lr11xx_update_firmware(void* context, lr11xx_fw_update_t fw_update_direction,
		uint32_t fw_expected, const uint32_t* buffer, uint32_t length)
{
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;
	lr11xx_bootloader_version_t version_bootloader = { 0 };
	lr11xx_system_version_t version_trx = { 0x00 };
	lr11xx_system_uid_t     uid         = { 0x00 };
	lr1110_modem_version_t version_modem = { 0 };
	lr11xx_bootloader_stat1_t stat1;
	lr11xx_bootloader_stat2_t stat2;
	lr11xx_bootloader_irq_mask_t irq_status;

	printf("Reset the chip...\n");

#if 0
	gpiolib_output_set_value(lr1110_update->radio_event, 1);
//	gpiolib_set_pull_up(lr1110_update->busy, GPIO_PULL_DOWN);
	gpiolib_set_output(lr1110_update->busy, 0);

	ioctl(lr1110_update->spi_fd, IOCTL_RESET, 0);

	time_usleep(500 * 1000);
	gpiolib_set_input(lr1110_update->busy);
	time_usleep(100 * 1000);
	gpiolib_output_set_value(lr1110_update->radio_event, 0);
#else
	lr1110_modem_system_reboot(context, true);
	time_usleep(2000 * 1000);
	//lr11xx_bootloader_get_status(context, &stat1, &stat2, &irq_status);
#endif

	printf("> Reset done!\n");

	lr11xx_bootloader_get_version(context, &version_bootloader);
	printf("Chip in bootloader mode:\n");
	printf(" - Chip type               = 0x%02X (0xDF for production)\n", version_bootloader.type);
	printf(" - Chip hardware version   = 0x%02X (0x22 for V2C)\n", version_bootloader.hw);
	printf(" - Chip bootloader version = 0x%04X \n", version_bootloader.fw);

	if (lr11xx_is_chip_in_production_mode(version_bootloader.type) == false) {
		return LR11XX_FW_UPDATE_WRONG_CHIP_TYPE;
	}

	if (lr11xx_is_fw_compatible_with_chip(fw_update_direction, version_bootloader.fw) == false) {
		return LR11XX_FW_UPDATE_WRONG_CHIP_TYPE;
	}

	lr11xx_bootloader_pin_t      pin      = { 0x00 };
	lr11xx_bootloader_chip_eui_t chip_eui = { 0x00 };
	lr11xx_bootloader_join_eui_t join_eui = { 0x00 };

	lr11xx_bootloader_read_pin(context, pin);
	lr11xx_bootloader_read_chip_eui(context, chip_eui);
	lr11xx_bootloader_read_join_eui(context, join_eui);

	printf("PIN is     0x%02X%02X%02X%02X\n", pin[0], pin[1], pin[2], pin[3]);
	printf("ChipEUI is 0x%02X%02X%02X%02X%02X%02X%02X%02X\n", chip_eui[0], chip_eui[1], chip_eui[2], chip_eui[3],
			chip_eui[4], chip_eui[5], chip_eui[6], chip_eui[7]);
	printf("JoinEUI is 0x%02X%02X%02X%02X%02X%02X%02X%02X\n", join_eui[0], join_eui[1], join_eui[2], join_eui[3],
			join_eui[4], join_eui[5], join_eui[6], join_eui[7]);

	printf("Start flash erase...\n");
	lr11xx_bootloader_erase_flash(context);
	printf("> Flash erase done!\n");

	printf("Start flashing firmware...\n");
	lr11xx_bootloader_write_flash_encrypted_full(context, 0, buffer, length);
	printf("> Flashing done!\n");

	printf("Rebooting...\n");
	lr11xx_bootloader_reboot(context, false);
	printf("> Reboot done!\n");

	switch (fw_update_direction) {
	case LR1110_FIRMWARE_UPDATE_TO_TRX:
	case LR1120_FIRMWARE_UPDATE_TO_TRX:
	case LR1121_FIRMWARE_UPDATE_TO_TRX:
		lr11xx_system_get_version(context, &version_trx);
		printf("Chip in transceiver mode:\n");
		printf(" - Chip type             = 0x%02X\n", version_trx.type);
		printf(" - Chip hardware version = 0x%02X\n", version_trx.hw);
		printf(" - Chip firmware version = 0x%04X\n", version_trx.fw);

		lr11xx_system_read_uid(context, uid);

		if (version_trx.fw == fw_expected) {
			return LR11XX_FW_UPDATE_OK;
		} else {
			return LR11XX_FW_UPDATE_ERROR;
		}
		break;
	case LR1110_FIRMWARE_UPDATE_TO_MODEM:
		time_usleep(2000 * 1000);

		lr1110_modem_get_version(context, &version_modem);
		printf("Chip in LoRa Basics Modem-E mode:\n");
		printf(" - Chip bootloader version = 0x%08x\n", version_modem.bootloader);
		printf(" - Chip firmware version   = 0x%08x\n", version_modem.firmware);
		printf(" - Chip LoRaWAN version    = 0x%04x\n", version_modem.lorawan);

		uint32_t fw_version = ((uint32_t) (version_modem.functionality) << 24) + version_modem.firmware;

		if (fw_version == fw_expected) {
			return LR11XX_FW_UPDATE_OK;
		} else {
			return LR11XX_FW_UPDATE_ERROR;
		}
		break;
	}

	return LR11XX_FW_UPDATE_ERROR;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

bool lr11xx_is_chip_in_production_mode(uint8_t type) {
	return (type == LR11XX_TYPE_PRODUCTION_MODE) ? true : false;
}

bool lr11xx_is_fw_compatible_with_chip(lr11xx_fw_update_t update, uint16_t bootloader_version) {
	if (((update == LR1110_FIRMWARE_UPDATE_TO_TRX) || (update == LR1110_FIRMWARE_UPDATE_TO_MODEM)) &&
			(bootloader_version != 0x6500)) {
		return false;
	} else if ((update == LR1120_FIRMWARE_UPDATE_TO_TRX) && (bootloader_version != 0x2000)) {
		return false;
	} else if ((update == LR1121_FIRMWARE_UPDATE_TO_TRX) && (bootloader_version != 0x2100)) {
		return false;
	}

	return true;
}

/* --- EOF ------------------------------------------------------------------ */
