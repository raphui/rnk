#include <stdio.h>
#include <gpiolib.h>
#include <string.h>

#include "lr1110_update.h"

#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 (64)
#define LR11XX_FLASH_DATA_MAX_LENGTH_UINT8 (LR11XX_FLASH_DATA_MAX_LENGTH_UINT32 * 4)

#define LR11XX_BL_CMD_NO_PARAM_LENGTH (2)
#define LR11XX_BL_GET_STATUS_CMD_LENGTH (2 + 4)
#define LR11XX_BL_VERSION_CMD_LENGTH LR11XX_BL_CMD_NO_PARAM_LENGTH
#define LR11XX_BL_ERASE_FLASH_CMD_LENGTH LR11XX_BL_CMD_NO_PARAM_LENGTH
#define LR11XX_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH (LR11XX_BL_CMD_NO_PARAM_LENGTH + 4)
#define LR11XX_BL_REBOOT_CMD_LENGTH (LR11XX_BL_CMD_NO_PARAM_LENGTH + 1)
#define LR11XX_BL_GET_PIN_CMD_LENGTH (LR11XX_BL_CMD_NO_PARAM_LENGTH)
#define LR11XX_BL_READ_CHIP_EUI_CMD_LENGTH (LR11XX_BL_CMD_NO_PARAM_LENGTH)
#define LR11XX_BL_READ_JOIN_EUI_CMD_LENGTH (LR11XX_BL_CMD_NO_PARAM_LENGTH)

enum {
	LR11XX_BL_GET_STATUS_OC            = 0x0100,
	LR11XX_BL_GET_VERSION_OC           = 0x0101,
	LR11XX_BL_ERASE_FLASH_OC           = 0x8000,
	LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC = 0x8003,
	LR11XX_BL_REBOOT_OC                = 0x8005,
	LR11XX_BL_GET_PIN_OC               = 0x800B,
	LR11XX_BL_READ_CHIP_EUI_OC         = 0x800C,
	LR11XX_BL_READ_JOIN_EUI_OC         = 0x800D,
};

struct lr1110_transfer {
	const uint8_t *command;
	uint16_t command_length;
	const uint8_t *data;
	uint16_t data_length;
};

/*!
 * @brief Returns the minimum of the operand given as parameter and the maximum allowed block size
 *
 * @param [in] operand Size to compare
 *
 * @returns Minimum between operand and @ref LR11XX_FLASH_DATA_MAX_LENGTH_UINT32
 */
static uint8_t lr11xx_bootloader_get_min_from_operand_and_max_block_size(uint32_t operand);

static lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* cbuffer, const uint16_t cbuffer_length,
		                                     uint8_t* rbuffer, const uint16_t rbuffer_length )
{
	uint8_t dummy = 0x00;
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;

	while (gpiolib_get_value(lr1110_update->busy) == 1)
		;

	/* 1st SPI transaction */
	ioctl(lr1110_update->spi_fd, IOCTL_SPI_SET_NSS, 0);

	for (int i = 0; i < cbuffer_length; i++) {
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&cbuffer[i]);
	}

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_CLEAR_NSS, 0);

	gpiolib_output_set_value(lr1110_update->radio_event, 0);
	while (gpiolib_get_value(lr1110_update->busy) == 1)
		;
	gpiolib_output_set_value(lr1110_update->radio_event, 1);

	/* 2nd SPI transaction */
	ioctl(lr1110_update->spi_fd, IOCTL_SPI_SET_NSS, 0);

	ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&dummy);

	memset(rbuffer, 0x0, rbuffer_length);

	for (int i = 0; i < rbuffer_length; i++) {
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&rbuffer[i]);
	}

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_CLEAR_NSS, 0);

	return LR11XX_HAL_STATUS_OK;
}

static lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* cbuffer, const uint16_t cbuffer_length,
		                                      const uint8_t* cdata, const uint16_t cdata_length )
{
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;

#if 0
	printf("%s: cbuffer_length: %d - cdata_length: %d\n", __func__, cbuffer_length, cdata_length);

	printf("%s: write cbuffer: ", __func__);
	for (int i = 0; i < cbuffer_length; i++) {
		printf("[0x%x] - 0x%x\n", &cbuffer[i], cbuffer[i]);
	}
	printf("\n");

	printf("%s: write cdata: ", __func__);
	for (int i = 0; i < cdata_length; i++) {
		printf("[0x%x] - 0x%x\n", &cdata[i], cdata[i]);
	}
	printf("\n");

#else
	while (gpiolib_get_value(lr1110_update->busy) != 0)
		;

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_SET_NSS, 0);

	for (int i = 0; i < cbuffer_length; i++) {
		printf("/");
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&cbuffer[i]);
		printf("\\");
	}

	for (int i = 0; i < cdata_length; i++) {
		printf("/");
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&cdata[i]);
		printf("\\");
	}

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_CLEAR_NSS, 0);
#endif

	return LR11XX_HAL_STATUS_OK;
}

lr1110_modem_hal_status_t lr1110_modem_hal_write_without_rc( const void* context, const uint8_t* cbuffer,
                                                            const uint16_t cbuffer_length, const uint8_t* cdata,
                                                            const uint16_t cdata_length )
{
	struct lr1110_transfer transfer;
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;
	lr1110_modem_hal_status_t status = LR1110_MODEM_HAL_STATUS_OK;

	transfer.command = cbuffer;
	transfer.command_length = cbuffer_length;
	transfer.data = cdata;
	transfer.data_length = cdata_length;

	write(lr1110_update->spi_fd, &transfer, sizeof(struct lr1110_transfer));

	return status;
}

static lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;

	while (gpiolib_get_value(lr1110_update->busy) != 0)
		;

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_SET_NSS, 0);

	for (int i = 0; i < data_length; i++) {
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&data[i]);
	}

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_CLEAR_NSS, 0);

	return LR11XX_HAL_STATUS_OK;
}

lr1110_modem_hal_status_t lr1110_modem_hal_read( const void* context, const uint8_t* command,
	                                         const uint16_t command_length, uint8_t* data,
	                                         const uint16_t data_length )
{
	uint8_t crc = 0;
	struct lr1110_transfer transfer;
	lr1110_modem_hal_status_t rc;
	struct lr1110_update *lr1110_update = (struct lr1110_update *)context;

	transfer.command = command;
	transfer.command_length = command_length;
	transfer.data = data;
	transfer.data_length = data_length;

	if (gpiolib_get_value(lr1110_update->busy) == 1) {
		ioctl(lr1110_update->spi_fd, IOCTL_SPI_SET_NSS, 0);
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&crc);
		ioctl(lr1110_update->spi_fd, IOCTL_SPI_CLEAR_NSS, 0);
        }

	while (gpiolib_get_value(lr1110_update->busy) != 0)
		;

	read(lr1110_update->spi_fd, &transfer, sizeof(struct lr1110_transfer));
	
	crc = 0;

	ioctl(lr1110_update->spi_fd, IOCTL_SPI_SET_NSS, 0);
	ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&rc);

	for (int i = 0; i < data_length; i++) {
		ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&data[i]);
	}

	ioctl(lr1110_update->spi_fd, IOCTL_WRITE_CHAR, (char *)&crc);
	ioctl(lr1110_update->spi_fd, IOCTL_SPI_CLEAR_NSS, 0);

    return LR1110_MODEM_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_status_t lr11xx_bootloader_get_status(const void* context, lr11xx_bootloader_stat1_t* stat1,
		lr11xx_bootloader_stat2_t*    stat2,
		lr11xx_bootloader_irq_mask_t* irq_status)
{
	uint8_t data[LR11XX_BL_GET_STATUS_CMD_LENGTH] = { 0 };

	const lr11xx_status_t status =
		(lr11xx_status_t) lr11xx_hal_direct_read(context, data, LR11XX_BL_GET_STATUS_CMD_LENGTH);

	if (status == LR11XX_STATUS_OK)
	{
		stat1->is_interrupt_active = ((data[0] & 0x01) != 0) ? true : false;
		stat1->command_status      = (lr11xx_bootloader_command_status_t) (data[0] >> 1);

		stat2->is_running_from_flash = ((data[1] & 0x01) != 0) ? true : false;
		stat2->chip_mode             = (lr11xx_bootloader_chip_modes_t) ((data[1] & 0x0F) >> 1);
		stat2->reset_status          = (lr11xx_bootloader_reset_status_t) ((data[1] & 0xF0) >> 4);

		*irq_status =
			((lr11xx_bootloader_irq_mask_t) data[2] << 24) + ((lr11xx_bootloader_irq_mask_t) data[3] << 16) +
			((lr11xx_bootloader_irq_mask_t) data[4] << 8) + ((lr11xx_bootloader_irq_mask_t) data[5] << 0);
	}

	return status;
}

lr11xx_status_t lr11xx_bootloader_clear_reset_status_info(const void* context)
{
	const uint8_t cbuffer[LR11XX_BL_CMD_NO_PARAM_LENGTH] = {
		(uint8_t) (LR11XX_BL_GET_STATUS_OC >> 8),
		(uint8_t) (LR11XX_BL_GET_STATUS_OC >> 0),
	};

	return (lr11xx_status_t) lr11xx_hal_write(context, cbuffer, LR11XX_BL_CMD_NO_PARAM_LENGTH, 0, 0);
}

lr11xx_status_t lr11xx_bootloader_get_version(const void* context, lr11xx_bootloader_version_t* version)
{
	const uint8_t cbuffer[LR11XX_BL_VERSION_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_GET_VERSION_OC >> 8),
		(uint8_t) (LR11XX_BL_GET_VERSION_OC >> 0),
	};
	uint8_t rbuffer[LR11XX_BL_VERSION_LENGTH] = { 0x00 };

	const lr11xx_status_t status = (lr11xx_status_t) lr11xx_hal_read(context, cbuffer, LR11XX_BL_VERSION_CMD_LENGTH,
			rbuffer, LR11XX_BL_VERSION_LENGTH);

	if (status == LR11XX_STATUS_OK)
	{
		version->hw   = rbuffer[0];
		version->type = rbuffer[1];
		version->fw   = ((uint16_t) rbuffer[2] << 8) + (uint16_t) rbuffer[3];
	}

	return status;
}

lr11xx_status_t lr11xx_bootloader_erase_flash(const void* context)
{
	const uint8_t cbuffer[LR11XX_BL_ERASE_FLASH_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_ERASE_FLASH_OC >> 8),
		(uint8_t) (LR11XX_BL_ERASE_FLASH_OC >> 0),
	};

	return (lr11xx_status_t) lr11xx_hal_write(context, cbuffer, LR11XX_BL_ERASE_FLASH_CMD_LENGTH, 0, 0);
}

lr11xx_status_t lr11xx_bootloader_write_flash_encrypted(const void* context, const uint32_t offset_in_byte,
		const uint32_t* data, uint8_t length_in_word)
{
	const uint8_t cbuffer[LR11XX_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC >> 8),
		(uint8_t) (LR11XX_BL_WRITE_FLASH_ENCRYPTED_OC >> 0),
		(uint8_t) (offset_in_byte >> 24),
		(uint8_t) (offset_in_byte >> 16),
		(uint8_t) (offset_in_byte >> 8),
		(uint8_t) (offset_in_byte >> 0),
	};

	uint8_t cdata[LR11XX_FLASH_DATA_MAX_LENGTH_UINT8] = { 0 };
	for(uint8_t index = 0; index < length_in_word; index++)
	{
		uint8_t* cdata_local = &cdata[index * sizeof(uint32_t)];

		cdata_local[0] = (uint8_t) (data[index] >> 24);
		cdata_local[1] = (uint8_t) (data[index] >> 16);
		cdata_local[2] = (uint8_t) (data[index] >> 8);
		cdata_local[3] = (uint8_t) (data[index] >> 0);
	}

	return (lr11xx_status_t) lr11xx_hal_write(context, cbuffer, LR11XX_BL_WRITE_FLASH_ENCRYPTED_CMD_LENGTH, cdata,
			length_in_word * sizeof(uint32_t));
}

lr11xx_status_t lr11xx_bootloader_write_flash_encrypted_full(const void* context, const uint32_t offset_in_byte,
		const uint32_t* buffer, const uint32_t length_in_word)
{
	uint32_t remaining_length = length_in_word;
	uint32_t local_offset     = offset_in_byte;
	uint32_t loop             = 0;

	while(remaining_length != 0)
	{
		const lr11xx_status_t status = lr11xx_bootloader_write_flash_encrypted(
				context, local_offset, buffer + loop * LR11XX_FLASH_DATA_MAX_LENGTH_UINT32,
				lr11xx_bootloader_get_min_from_operand_and_max_block_size(remaining_length));

		if (status != LR11XX_STATUS_OK)
		{
			return status;
		}

		local_offset += LR11XX_FLASH_DATA_MAX_LENGTH_UINT8;
		remaining_length = (remaining_length < LR11XX_FLASH_DATA_MAX_LENGTH_UINT32)
			? 0
			: (remaining_length - LR11XX_FLASH_DATA_MAX_LENGTH_UINT32);

		loop++;
	}

	return LR11XX_STATUS_OK;
}

lr11xx_status_t lr11xx_bootloader_reboot(const void* context, const bool stay_in_bootloader)
{
	const uint8_t cbuffer[LR11XX_BL_REBOOT_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_REBOOT_OC >> 8),
		(uint8_t) (LR11XX_BL_REBOOT_OC >> 0),
		(stay_in_bootloader == true) ? 0x03 : 0x00,
	};

	return (lr11xx_status_t) lr11xx_hal_write(context, cbuffer, LR11XX_BL_REBOOT_CMD_LENGTH, 0, 0);
}

lr11xx_status_t lr11xx_bootloader_read_pin(const void* context, lr11xx_bootloader_pin_t pin)
{
	const uint8_t cbuffer[LR11XX_BL_GET_PIN_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_GET_PIN_OC >> 8),
		(uint8_t) (LR11XX_BL_GET_PIN_OC >> 0),
	};

	return (lr11xx_status_t) lr11xx_hal_read(context, cbuffer, LR11XX_BL_GET_PIN_CMD_LENGTH, pin,
			LR11XX_BL_PIN_LENGTH);
}

lr11xx_status_t lr11xx_bootloader_read_chip_eui(const void* context, lr11xx_bootloader_chip_eui_t chip_eui)
{
	const uint8_t cbuffer[LR11XX_BL_READ_CHIP_EUI_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_READ_CHIP_EUI_OC >> 8),
		(uint8_t) (LR11XX_BL_READ_CHIP_EUI_OC >> 0),
	};

	return (lr11xx_status_t) lr11xx_hal_read(context, cbuffer, LR11XX_BL_READ_CHIP_EUI_CMD_LENGTH, chip_eui,
			LR11XX_BL_CHIP_EUI_LENGTH);
}

lr11xx_status_t lr11xx_bootloader_read_join_eui(const void* context, lr11xx_bootloader_join_eui_t join_eui)
{
	const uint8_t cbuffer[LR11XX_BL_READ_JOIN_EUI_CMD_LENGTH] = {
		(uint8_t) (LR11XX_BL_READ_JOIN_EUI_OC >> 8),
		(uint8_t) (LR11XX_BL_READ_JOIN_EUI_OC >> 0),
	};

	return (lr11xx_status_t) lr11xx_hal_read(context, cbuffer, LR11XX_BL_READ_JOIN_EUI_CMD_LENGTH, join_eui,
			LR11XX_BL_JOIN_EUI_LENGTH);
}

lr11xx_status_t lr11xx_system_get_version(const void* context, lr11xx_system_version_t* version)
{
	const uint8_t cbuffer[LR11XX_SYSTEM_GET_VERSION_CMD_LENGTH] = {
		(uint8_t) (LR11XX_SYSTEM_GET_VERSION_OC >> 8),
		(uint8_t) (LR11XX_SYSTEM_GET_VERSION_OC >> 0),
	};
	uint8_t rbuffer[LR11XX_SYSTEM_VERSION_LENGTH] = { 0x00 };

	const lr11xx_status_t status = (lr11xx_status_t) lr11xx_hal_read(
			context, cbuffer, LR11XX_SYSTEM_GET_VERSION_CMD_LENGTH, rbuffer, LR11XX_SYSTEM_VERSION_LENGTH);

	if (status == LR11XX_STATUS_OK)
	{
		version->hw   = rbuffer[0];
		version->type = (lr11xx_system_version_type_t) rbuffer[1];
		version->fw   = ((uint16_t) rbuffer[2] << 8) + (uint16_t) rbuffer[3];
	}

	return status;
}

lr11xx_status_t lr11xx_system_read_uid(const void* context, lr11xx_system_uid_t unique_identifier)
{
	const uint8_t cbuffer[LR11XX_SYSTEM_READ_UID_CMD_LENGTH] = {
		(uint8_t) (LR11XX_SYSTEM_READ_UID_OC >> 8),
		(uint8_t) (LR11XX_SYSTEM_READ_UID_OC >> 0),
	};

	return (lr11xx_status_t) lr11xx_hal_read(context, cbuffer, LR11XX_SYSTEM_READ_UID_CMD_LENGTH, unique_identifier,
			LR11XX_SYSTEM_UID_LENGTH);
}

lr1110_modem_response_code_t lr1110_modem_system_reboot(const void* context, const bool stay_in_bootloader)
{
    uint8_t cbuffer[LR1110_MODEM_SYSTEM_REBOOT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_SYSTEM;
    cbuffer[1] = LR1110_MODEM_SYSTEM_REBOOT_CMD;

    cbuffer[2] = ( stay_in_bootloader == true ) ? 0x03 : 0x00;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write_without_rc(
		            context, cbuffer, LR1110_MODEM_SYSTEM_REBOOT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_version(const void* context, lr1110_modem_version_t* version)
{
	uint8_t                      cbuffer[LR1110_MODEM_GET_VERSION_CMD_LENGTH];
	uint8_t                      rbuffer[LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH] = { 0x00 };
	lr1110_modem_response_code_t rc;

	cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
	cbuffer[1] = LR1110_MODEM_GET_VERSION_CMD;

	rc = (lr1110_modem_response_code_t) lr1110_modem_hal_read(context, cbuffer, LR1110_MODEM_GET_VERSION_CMD_LENGTH,
			rbuffer, LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH);

	version->bootloader = ((uint32_t) rbuffer[0] << 24) + ((uint32_t) rbuffer[1] << 16) +
		((uint32_t) rbuffer[2] << 8) + ((uint32_t) rbuffer[3]);
	version->functionality = (lr1110_modem_functionality_t) rbuffer[4];
	version->firmware =
		((uint32_t) rbuffer[5] << 16) + ((uint32_t) rbuffer[6] << 8) + ((uint32_t) rbuffer[7]);
	version->lorawan = ((uint16_t) rbuffer[8] << 8) + rbuffer[9];

	return rc;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint8_t lr11xx_bootloader_get_min_from_operand_and_max_block_size(uint32_t operand) {
	if (operand > LR11XX_FLASH_DATA_MAX_LENGTH_UINT32)
	{
		return LR11XX_FLASH_DATA_MAX_LENGTH_UINT32;
	}
	else
	{
		return (uint8_t) operand;
	}
}

/* --- EOF ------------------------------------------------------------------ */
