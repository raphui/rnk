#ifndef LR1110_UPDATE_H
#define LR1110_UPDATE_H

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#define LR11XX_TYPE_PRODUCTION_MODE 0xDF
#define LR11XX_BL_VERSION_LENGTH ( 4 )
#define LR11XX_BL_PIN_LENGTH ( 4 )
#define LR11XX_BL_CHIP_EUI_LENGTH ( 8 )
#define LR11XX_BL_JOIN_EUI_LENGTH ( 8 )
#define LR11XX_SYSTEM_GET_VERSION_OC 0x0101
#define LR11XX_SYSTEM_GET_VERSION_CMD_LENGTH (2)
#define LR11XX_SYSTEM_VERSION_LENGTH (4)
#define LR11XX_SYSTEM_READ_UID_OC 0x0125
#define LR11XX_SYSTEM_READ_UID_CMD_LENGTH ( 2 )
#define LR11XX_SYSTEM_UID_LENGTH ( 8 )
#define LR1110_MODEM_SYSTEM_REBOOT_CMD 0x18
#define LR1110_MODEM_SYSTEM_REBOOT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GROUP_ID_SYSTEM 0x1
#define LR1110_MODEM_GET_VERSION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH ( 10 )
#define LR1110_MODEM_GROUP_ID_MODEM 0x06
#define LR1110_MODEM_GET_VERSION_CMD 0x01

typedef uint8_t lr11xx_bootloader_pin_t[LR11XX_BL_PIN_LENGTH];
typedef uint8_t lr11xx_bootloader_chip_eui_t[LR11XX_BL_CHIP_EUI_LENGTH];
typedef uint8_t lr11xx_bootloader_join_eui_t[LR11XX_BL_JOIN_EUI_LENGTH];
typedef uint8_t lr11xx_system_uid_t[LR11XX_SYSTEM_UID_LENGTH];

typedef enum lr11xx_system_version_type_e {
    LR11XX_SYSTEM_VERSION_TYPE_LR1110 = 0x01,
    LR11XX_SYSTEM_VERSION_TYPE_LR1120 = 0x02,
    LR11XX_SYSTEM_VERSION_TYPE_LR1121 = 0x03,
} lr11xx_system_version_type_t;

typedef enum lr11xx_bootloader_chip_modes_e {
	LR11XX_BOOTLOADER_CHIP_MODE_SLEEP     = 0x00,
	LR11XX_BOOTLOADER_CHIP_MODE_STBY_RC   = 0x01,
	LR11XX_BOOTLOADER_CHIP_MODE_STBY_XOSC = 0x02,
	LR11XX_BOOTLOADER_CHIP_MODE_FS        = 0x03,
	LR11XX_BOOTLOADER_CHIP_MODE_RX        = 0x04,
	LR11XX_BOOTLOADER_CHIP_MODE_TX        = 0x05,
	LR11XX_BOOTLOADER_CHIP_MODE_LOC       = 0x06,
} lr11xx_bootloader_chip_modes_t;

typedef enum lr11xx_bootloader_reset_status_e {
	LR11XX_BOOTLOADER_RESET_STATUS_CLEARED      = 0x00,
	LR11XX_BOOTLOADER_RESET_STATUS_ANALOG       = 0x01,
	LR11XX_BOOTLOADER_RESET_STATUS_EXTERNAL     = 0x02,
	LR11XX_BOOTLOADER_RESET_STATUS_SYSTEM       = 0x03,
	LR11XX_BOOTLOADER_RESET_STATUS_WATCHDOG     = 0x04,
	LR11XX_BOOTLOADER_RESET_STATUS_RTC_RESTART  = 0x06,
} lr11xx_bootloader_reset_status_t;

typedef enum lr11xx_bootloader_command_status_e {
	LR11XX_BOOTLOADER_CMD_STATUS_FAIL = 0x00,
	LR11XX_BOOTLOADER_CMD_STATUS_PERR = 0x01,
	LR11XX_BOOTLOADER_CMD_STATUS_OK   = 0x02,
	LR11XX_BOOTLOADER_CMD_STATUS_DATA = 0x03,
} lr11xx_bootloader_command_status_t;

typedef struct lr11xx_bootloader_stat1_s {
	lr11xx_bootloader_command_status_t command_status;
	bool                               is_interrupt_active;
} lr11xx_bootloader_stat1_t;

typedef struct lr11xx_bootloader_stat2_s {
	lr11xx_bootloader_reset_status_t reset_status;
	lr11xx_bootloader_chip_modes_t   chip_mode;
	bool                             is_running_from_flash;
} lr11xx_bootloader_stat2_t;

typedef struct lr11xx_bootloader_version_s {
	uint8_t  hw;
	uint8_t  type;
	uint16_t fw;
} lr11xx_bootloader_version_t;

typedef enum {
	LR1110_FIRMWARE_UPDATE_TO_TRX,
	LR1110_FIRMWARE_UPDATE_TO_MODEM,
	LR1120_FIRMWARE_UPDATE_TO_TRX,
	LR1121_FIRMWARE_UPDATE_TO_TRX,
} lr11xx_fw_update_t;

typedef enum {
	LR11XX_FW_UPDATE_OK              = 0,
	LR11XX_FW_UPDATE_WRONG_CHIP_TYPE = 1,
	LR11XX_FW_UPDATE_ERROR           = 2,
} lr11xx_fw_update_status_t;

typedef enum lr11xx_status_e {
    LR11XX_STATUS_OK    = 0,
    LR11XX_STATUS_ERROR = 3,
} lr11xx_status_t;

typedef enum {
    LR1110_MODEM_RESPONSE_CODE_OK                  = 0x00,  //!< Driver command executed successfully
    LR1110_MODEM_RESPONSE_CODE_UNKOWN              = 0x01,  //!< Command code unknown
    LR1110_MODEM_RESPONSE_CODE_NOT_INITIALIZED     = 0x03,  //!< Command not initialized
    LR1110_MODEM_RESPONSE_CODE_INVALID             = 0x04,  //!< Invalid command parameters
    LR1110_MODEM_RESPONSE_CODE_BUSY                = 0x05,  //!< Command cannot be executed now
    LR1110_MODEM_RESPONSE_CODE_FAIL                = 0x06,  //!< Command execution failed
    LR1110_MODEM_RESPONSE_CODE_BAD_FILE_UPLOAD_CRC = 0x08,  //!< File upload CRC check failed
    LR1110_MODEM_RESPONSE_CODE_BAD_SIZE            = 0x0A,  //!< Size check failed
    LR1110_MODEM_RESPONSE_CODE_BAD_FRAME           = 0x0F,  //!< SPI command checksum failed or CRC failed
    LR1110_MODEM_RESPONSE_CODE_NO_TIME             = 0x10,  //!< GNSS time synchronisation lost
} lr1110_modem_response_code_t;

typedef enum {
    LR1110_MODEM_FUNCTIONALITY_TRX                   = 0x01,
    LR1110_MODEM_FUNCTIONALITY_MODEM_WIFI            = 0x02,
    LR1110_MODEM_FUNCTIONALITY_MODEM_WIFI_GPS        = 0x03,
    LR1110_MODEM_FUNCTIONALITY_MODEM_WIFI_GPS_BEIDOU = 0x04,
} lr1110_modem_functionality_t;

typedef struct lr11xx_system_version_s {
    uint8_t                      hw;
    lr11xx_system_version_type_t type;
    uint16_t                     fw;
} lr11xx_system_version_t;

typedef struct {
    uint32_t                     bootloader;     //!< Bootloader version
    lr1110_modem_functionality_t functionality;  //!< Functionality identifier
    uint32_t                     firmware;       //!< Firmware version
    uint16_t                     lorawan;        //!< LoRaWAN version
} lr1110_modem_version_t;

typedef enum lr11xx_hal_status_e {
    LR11XX_HAL_STATUS_OK    = 0,
    LR11XX_HAL_STATUS_ERROR = 3,
} lr11xx_hal_status_t;

typedef enum lr1110_modem_hal_status_e {
    LR1110_MODEM_HAL_STATUS_OK           = 0x00,  //!< Operation terminated successfully
    LR1110_MODEM_HAL_STATUS_ERROR        = 0x01,  //!< Operation terminated with error
    LR1110_MODEM_HAL_STATUS_BAD_FRAME    = 0x0F,  //!< Bad frame detected in the exchange
    LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT = 0xFF,  //!< Timeout occured while waiting for Busy line state
} lr1110_modem_hal_status_t;

typedef uint32_t lr11xx_bootloader_irq_mask_t;

struct lr1110_update {
	int spi_fd;
	bool is_updated;
	struct pio_desc *busy;
	struct pio_desc *radio_event;
};


int lr11xx_update_init(void *context);
int lr11xx_update_finalize(void *context, uint32_t fw_expected);
lr11xx_fw_update_status_t lr11xx_update_firmware(void* context, lr11xx_fw_update_t fw_update_direction, uint32_t fw_expected, const uint32_t* buffer, uint32_t length);
lr11xx_status_t lr11xx_bootloader_get_status(const void* context, lr11xx_bootloader_stat1_t* stat1, lr11xx_bootloader_stat2_t* stat2, lr11xx_bootloader_irq_mask_t* irq_status);
lr11xx_status_t lr11xx_bootloader_clear_reset_status_info(const void* context);
lr11xx_status_t lr11xx_bootloader_get_version(const void* context, lr11xx_bootloader_version_t* version);
lr11xx_status_t lr11xx_bootloader_erase_flash(const void* context);
lr11xx_status_t lr11xx_bootloader_write_flash_encrypted(const void* context, const uint32_t offset_in_byte, const uint32_t* data, uint8_t length_in_word);
lr11xx_status_t lr11xx_bootloader_write_flash_encrypted_full(const void* context, const uint32_t offset_in_byte, const uint32_t* buffer, const uint32_t length_in_word);
lr11xx_status_t lr11xx_bootloader_reboot(const void* context, const bool stay_in_bootloader);
lr11xx_status_t lr11xx_bootloader_read_pin(const void* context, lr11xx_bootloader_pin_t pin);
lr11xx_status_t lr11xx_bootloader_read_chip_eui(const void* context, lr11xx_bootloader_chip_eui_t chip_eui);
lr11xx_status_t lr11xx_bootloader_read_join_eui(const void* context, lr11xx_bootloader_join_eui_t join_eui);
lr11xx_status_t lr11xx_system_get_version(const void* context, lr11xx_system_version_t* version);
lr11xx_status_t lr11xx_system_read_uid(const void* context, lr11xx_system_uid_t unique_identifier);
lr1110_modem_response_code_t lr1110_modem_get_version(const void* context, lr1110_modem_version_t* version);
lr1110_modem_response_code_t lr1110_modem_system_reboot(const void* context, const bool stay_in_bootloader);

#endif /* LR1110_UPDATE_H */
