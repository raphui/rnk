/*!
 * @file      tracker_utility.h
 *
 * @brief     Tracker utility definition
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TRACKER_UTILITY_H
#define TRACKER_UTILITY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include <stdbool.h>
#include "wifi_scan.h"
#include "gnss_scan.h"
#include "modem/lr1110_modem_lorawan.h"
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/* Internal Log Tag */
#define TAG_GNSS 0x01
#define TAG_WIFI 0x03
#define TAG_NEXT_SCAN 0x04

/* Internal Log Len */
#define WIFI_SINGLE_BEACON_LEN 0x08
#define WIFI_TIMING_LEN 0x10
#define GNSS_TIMING_LEN 0x08

/* Internal Log Activation */
#define GNSS_LOG_ACTIVATED 1
#define WIFI_LOG_ACTIVATED 1

/* Internal Log display settings */
#define GNSS_DISPLAY_LOG_ACTIVATED 1
#define WIFI_DISPLAY_LOG_ACTIVATED 1
#define PAYLOAD_DISPLAY_LOG_ACTIVATED 0

/* Tracker application commands */

/* Application & Board */
#define GET_FW_VERSION_CMD 0x01
#define GET_FW_VERSION_LEN 0x00
#define GET_FW_VERSION_ANSWER_LEN 0x03
#define GET_STACK_VERSION_CMD 0x33
#define GET_STACK_VERSION_LEN 0x00
#define GET_STACK_VERSION_ANSWER_LEN 0x02
#define GET_MODEM_VERSION_CMD 0x34
#define GET_MODEM_VERSION_LEN 0x00
#define GET_MODEM_VERSION_ANSWER_LEN 0x03
#define GET_MODEM_STATUS_CMD 0x45
#define GET_MODEM_STATUS_LEN 0x00
#define GET_MODEM_STATUS_ANSWER_LEN 0x02
#define GET_MODEM_DATE_CMD 0x46
#define GET_MODEM_DATE_LEN 0x00
#define GET_MODEM_DATE_ANSWER_LEN 0x04

/* LoRaWAN */
#define SET_LORAWAN_DEVEUI_CMD 0x02
#define SET_LORAWAN_DEVEUI_LEN 0x08
#define GET_LORAWAN_DEVEUI_CMD 0x03
#define GET_LORAWAN_DEVEUI_LEN 0x00
#define GET_LORAWAN_DEVEUI_ANSWER_LEN 0x08
#define SET_LORAWAN_JOINEUI_CMD 0x04
#define SET_LORAWAN_JOINEUI_LEN 0x08
#define GET_LORAWAN_JOINEUI_CMD 0x05
#define GET_LORAWAN_JOINEUI_LEN 0x00
#define GET_LORAWAN_JOINEUI_ANSWER_LEN 0x08
#define SET_LORAWAN_APPKEY_CMD 0x06
#define SET_LORAWAN_APPKEY_LEN 0x10
#define GET_LORAWAN_APPKEY_CMD 0x07
#define GET_LORAWAN_APPKEY_LEN 0x00
#define GET_LORAWAN_APPKEY_ANSWER_LEN 0x10
#define SET_LORAWAN_REGION_CMD 0x35
#define SET_LORAWAN_REGION_LEN 0x01
#define GET_LORAWAN_REGION_CMD 0x36
#define GET_LORAWAN_REGION_LEN 0x00
#define GET_LORAWAN_REGION_ANSWER_LEN 0x01
#define GET_LORAWAN_PIN_CMD 0x39
#define GET_LORAWAN_PIN_LEN 0x00
#define GET_LORAWAN_PIN_ANSWER_LEN 0x04
#define SET_LORAWAN_JOIN_SERVER_CMD 0x3A
#define SET_LORAWAN_JOIN_SERVER_LEN 0x01
#define GET_LORAWAN_JOIN_SERVER_CMD 0x3B
#define GET_LORAWAN_JOIN_SERVER_LEN 0x00
#define GET_LORAWAN_JOIN_SERVER_ANSWER_LEN 0x01
#define SET_LORAWAN_ADR_PROFILE_CMD 0x3E
#define SET_LORAWAN_ADR_PROFILE_LEN 0x01
#define GET_LORAWAN_ADR_PROFILE_CMD 0x3F
#define GET_LORAWAN_ADR_PROFILE_LEN 0x00
#define GET_LORAWAN_ADR_PROFILE_ANSWER_LEN 0x01
#define GET_LORAWAN_CHIP_EUI_CMD 0x44
#define GET_LORAWAN_CHIP_EUI_LEN 0x00
#define GET_LORAWAN_CHIP_EUI_ANSWER_LEN 0x08
#define SET_LORAWAN_DUTY_CYCLE_CMD 0x47
#define SET_LORAWAN_DUTY_CYCLE_LEN 0x01
#define GET_LORAWAN_DUTY_CYCLE_CMD 0x48
#define GET_LORAWAN_DUTY_CYCLE_LEN 0x00
#define GET_LORAWAN_DUTY_CYCLE_ANSWER_LEN 0x01
#define GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD 0x50
#define GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_LEN 0x00
#define GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_ANSWER_LEN 0x02

/* GNSS */
#define SET_GNSS_ENABLE_CMD 0x08
#define SET_GNSS_ENABLE_LEN 0x01
#define GET_GNSS_ENABLE_CMD 0x09
#define GET_GNSS_ENABLE_LEN 0x00
#define GET_GNSS_ENABLE_ANSWER_LEN 0x01
#define SET_GNSS_CONSTELLATION_CMD 0x0A
#define SET_GNSS_CONSTELLATION_LEN 0x01
#define GET_GNSS_CONSTELLATION_CMD 0x0B
#define GET_GNSS_CONSTELLATION_LEN 0x00
#define GET_GNSS_CONSTELLATION_ANSWER_LEN 0x01

#define SET_GNSS_SCAN_TYPE_CMD 0x10
#define SET_GNSS_SCAN_TYPE_LEN 0x01
#define GET_GNSS_SCAN_TYPE_CMD 0x11
#define GET_GNSS_SCAN_TYPE_LEN 0x00
#define GET_GNSS_SCAN_TYPE_ANSWER_LEN 0x01
#define SET_GNSS_SCAN_MODE_CMD 0x12
#define SET_GNSS_SCAN_MODE_LEN 0x01
#define GET_GNSS_SCAN_MODE_CMD 0x13
#define GET_GNSS_SCAN_MODE_LEN 0x00
#define GET_GNSS_SCAN_MODE_ANSWER_LEN 0x01
#define SET_GNSS_SEARCH_MODE_CMD 0x14
#define SET_GNSS_SEARCH_MODE_LEN 0x01
#define GET_GNSS_SEARCH_MODE_CMD 0x15
#define GET_GNSS_SEARCH_MODE_LEN 0x00
#define GET_GNSS_SEARCH_MODE_ANSWER_LEN 0x01
#define GET_GNSS_LAST_ALMANAC_UPDATE_CMD 0x2D
#define GET_GNSS_LAST_ALMANAC_UPDATE_LEN 0x00
#define GET_GNSS_LAST_ALMANAC_UPDATE_ANSWER_LEN 0x04
#define GET_GNSS_LAST_NB_SV_CMD 0x51
#define GET_GNSS_LAST_NB_SV_LEN 0x00
#define GET_GNSS_LAST_NB_SV_ANSWER_LEN 0x01

/* WiFi */
#define SET_WIFI_ENABLE_CMD 0x16
#define SET_WIFI_ENABLE_LEN 0x01
#define GET_WIFI_ENABLE_CMD 0x17
#define GET_WIFI_ENABLE_LEN 0x00
#define GET_WIFI_ENABLE_ANSWER_LEN 0x01
#define SET_WIFI_CHANNELS_CMD 0x18
#define SET_WIFI_CHANNELS_LEN 0x02
#define GET_WIFI_CHANNELS_CMD 0x19
#define GET_WIFI_CHANNELS_LEN 0x00
#define GET_WIFI_CHANNELS_ANSWER_LEN 0x02
#define SET_WIFI_TYPE_CMD 0x1A
#define SET_WIFI_TYPE_LEN 0x01
#define GET_WIFI_TYPE_CMD 0x1B
#define GET_WIFI_TYPE_LEN 0x00
#define GET_WIFI_TYPE_ANSWER_LEN 0x01
#define SET_WIFI_SCAN_MODE_CMD 0x1C
#define SET_WIFI_SCAN_MODE_LEN 0x01
#define GET_WIFI_SCAN_MODE_CMD 0x1D
#define GET_WIFI_SCAN_MODE_LEN 0x00
#define GET_WIFI_SCAN_MODE_ANSWER_LEN 0x01
#define SET_WIFI_RETRIALS_CMD 0x1E
#define SET_WIFI_RETRIALS_LEN 0x01
#define GET_WIFI_RETRIALS_CMD 0x1F
#define GET_WIFI_RETRIALS_LEN 0x00
#define GET_WIFI_RETRIALS_ANSWER_LEN 0x01
#define SET_WIFI_MAX_RESULTS_CMD 0x20
#define SET_WIFI_MAX_RESULTS_LEN 0x01
#define GET_WIFI_MAX_RESULTS_CMD 0x21
#define GET_WIFI_MAX_RESULTS_LEN 0x00
#define GET_WIFI_MAX_RESULTS_ANSWER_LEN 0x01
#define SET_WIFI_TIMEOUT_CMD 0x22
#define SET_WIFI_TIMEOUT_LEN 0x02
#define GET_WIFI_TIMEOUT_CMD 0x23
#define GET_WIFI_TIMEOUT_LEN 0x00
#define GET_WIFI_TIMEOUT_ANSWER_LEN 0x02
#define GET_WIFI_LAST_NB_MAC_ADDRESS_CMD 0x52
#define GET_WIFI_LAST_NB_MAC_ADDRESS_LEN 0x00
#define GET_WIFI_LAST_NB_MAC_ADDRESS_ANSWER_LEN 0x01

/* Tracker Application */
#define SET_USE_ACCELEROMETER_CMD 0x24
#define SET_USE_ACCELEROMETER_LEN 0x01
#define GET_USE_ACCELEROMETER_CMD 0x25
#define GET_USE_ACCELEROMETER_LEN 0x00
#define GET_USE_ACCELEROMETER_ANSWER_LEN 0x01
#define SET_APP_SCAN_INTERVAL_CMD 0x26
#define SET_APP_SCAN_INTERVAL_LEN 0x02
#define GET_APP_SCAN_INTERVAL_CMD 0x27
#define GET_APP_SCAN_INTERVAL_LEN 0x00
#define GET_APP_SCAN_INTERVAL_ANSWER_LEN 0x02
#define SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD 0x28
#define SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN 0x02
#define GET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD 0x29
#define GET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN 0x00
#define GET_APP_KEEP_ALINE_FRAME_INTERVAL_ANSWER_LEN 0x02
#define SET_APP_RESET_CMD 0x2B
#define SET_APP_RESET_LEN 0x00
#define SET_SCAN_PRIORITY_CMD 0x3C
#define SET_SCAN_PRIORITY_LEN 0x01
#define GET_SCAN_PRIORITY_CMD 0x3D
#define GET_SCAN_PRIORITY_LEN 0x00
#define GET_SCAN_PRIORITY_ANSWER_LEN 0x01
#define SET_APP_FLUSH_INTERNAL_LOG_CMD 0x2A
#define SET_APP_FLUSH_INTERNAL_LOG_LEN 0x00
#define SET_APP_INTERNAL_LOG_CMD 0x41
#define SET_APP_INTERNAL_LOG_LEN 0x01
#define GET_APP_INTERNAL_LOG_CMD 0x42
#define GET_APP_INTERNAL_LOG_LEN 0x00
#define GET_APP_INTERNAL_LOG_ANSWER_LEN 0x01
#define GET_APP_INTERNAL_LOG_REMANING_SPACE_CMD 0x49
#define GET_APP_INTERNAL_LOG_REMANING_SPACE_LEN 0x00
#define GET_APP_INTERNAL_LOG_REMANING_SPACE_ANSWER_LEN 0x01
#define GET_APP_ACCUMULATED_CHARGE_CMD 0x4A
#define GET_APP_ACCUMULATED_CHARGE_LEN 0x00
#define GET_APP_ACCUMULATED_CHARGE_ANSWER_LEN 0x04
#define RESET_APP_ACCUMULATED_CHARGE_CMD 0x4B
#define RESET_APP_ACCUMULATED_CHARGE_LEN 0x00
#define GET_APP_TRACKER_SETTINGS_CMD 0x4C
#define GET_APP_TRACKER_SETTINGS_LEN 0x00
#define GET_APP_TRACKER_SETTINGS_ANSWER_LEN 0x37
#define GET_APP_TRACKER_SETTINGS_VERSION 0x01
#define GET_APP_RESET_COUNTER_CMD 0x4D
#define GET_APP_RESET_COUNTER_LEN 0x00
#define GET_APP_RESET_COUNTER_ANSWER_LEN 0x06
#define RESET_APP_RESET_COUNTER_CMD 0x4E
#define RESET_APP_RESET_COUNTER_LEN 0x00
#define GET_APP_SYSTEM_SANITY_CHECK_CMD 0x53
#define GET_APP_SYSTEM_SANITY_CHECK_LEN 0x00
#define GET_APP_SYSTEM_SANITY_CHECK_ANSWER_LEN 0x01

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Tracker system sanity check mask
 */
typedef enum
{
    TRACKER_GNSS_SCAN_SUCCESSFUL_ONCE = 0x01,  //!< Means that at least once GNSS scan has been successful
    TRACKER_WIFI_SCAN_SUCCESSFUL_ONCE = 0x02,  //!< Means that at least once Wi-Fi scan has been successful
    TRACKER_DOWNLINK_SUCCESSFUL_ONCE  = 0x04,  //!< Means that at least once Applicative downlink has been received
} tracker_system_sanity_check_mask_t;

/*!
 * @brief Tracker system sanity check mask
 */
typedef enum
{
    TRACKER_GNSS_PRIORITY = 0x00,  //!< Means that GNSS scan has the priority
    TRACKER_WIFI_PRIORITY = 0x01,  //!< Means that Wi-Fi scan has the priority
    TRACKER_NO_PRIORITY   = 0x02,  //!< Means no priority, GNSS and Wi-Fi are performed
} tracker_scan_priority_t;

/*!
 * @brief Tracker context structure
 */
typedef struct
{
    /* Time variables */
    uint32_t timestamp;

    /* Indicates if a date has been set */
    bool has_date;

    /* Context */
    uint8_t tracker_context_empty;

    /* Board */
    uint16_t voltage;

    /* LoRaWAN Parameters */
    uint8_t                     dev_eui[8];
    uint8_t                     join_eui[8];
    uint8_t                     app_key[16];
    uint8_t                     chip_eui[8];
    uint32_t                    lorawan_pin;
    bool                        lorawan_parameters_have_changed;
    lr1110_modem_regions_t      lorawan_region;
    lr1110_modem_adr_profiles_t lorawan_adr_profile;
    bool                        use_semtech_join_server;
    bool                        duty_cycle_enable;

    /* Modem version information */
    lr1110_modem_version_t modem_version;

    /* GNSS Parameters */
    gnss_settings_t gnss_settings;
    uint32_t        last_almanac_update;

    /* WiFi Parameters */
    wifi_settings_t wifi_settings;

    /* Application Parameters */
    bool                               accelerometer_used;
    uint32_t                           app_scan_interval;
    uint32_t                           app_keep_alive_frame_interval;
    uint8_t                            accelerometer_move_history;
    bool                               send_alive_frame;
    bool                               stream_done;
    bool                               airplane_mode;
    tracker_scan_priority_t            scan_priority;
    bool                               internal_log_enable;
    uint8_t                            tracker_settings_payload_len;
    uint8_t                            tracker_settings_payload[242];
    uint16_t                           host_reset_cnt;
    uint16_t                           modem_reset_by_itself_cnt;
    bool                               reset_cnt_sent;
    tracker_system_sanity_check_mask_t system_sanity_check;

    bool new_value_to_set;

    /* Results values */
    uint16_t                    lorawan_payload_len;
    uint8_t                     lorawan_payload[500];
    uint32_t                    next_frame_ctn;
    gnss_scan_single_result_t   gnss_scan_result;
    uint8_t                     last_nb_detected_satellites;
    wifi_scan_selected_result_t wifi_result;
    uint8_t                     last_nb_detected_mac_address;
    int16_t                     accelerometer_x;
    int16_t                     accelerometer_y;
    int16_t                     accelerometer_z;
    int16_t                     temperature;
    uint32_t                    charge;
    uint32_t                    accumulated_charge;

    /* parameters for flash read/write operations */
    bool     internal_log_flush_request;
    uint8_t  internal_log_empty;
    uint16_t nb_scan;
    uint32_t flash_addr_start;
    uint32_t flash_addr_end;
    uint32_t flash_addr_current;
    uint32_t flash_remaining_space;
} tracker_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init and store the tracker internal log context in the flash memory in the dedicated memory zone
 *
 * @param [out] SMTC_SUCESS/SMTC_FAIL
 */
uint8_t tracker_init_internal_log_ctx( void );

/*!
 * @brief Store the internal log context in the flash memory in the dedicated memory zone
 */
void tracker_store_internal_log_ctx( void );

/*!
 * @brief Restore the internal log context from the flash memory and set in /ref FieldTest_t structure
 *
 * @note if SMTC_FAIL is returned call init_field_test_ctx
 *
 * @returns SMTC_SUCESS/SMTC_FAIL
 */
uint8_t tracker_restore_internal_log_ctx( void );

/*!
 * @brief Store the Tracker context in the flash memory in the dedicated memory zone
 *
 */
void tracker_store_app_ctx( void );

/*!
 * @brief Init the Tracker context
 *
 * @param [in] dev_eui LoRaWAN Device Eui
 * @param [in] join_eui LoRaWAN Join Eui
 * @param [in] app_key LoRaWAN Application Key
 * @param [in] store_in_flash store the context in flash
 */
void tracker_init_app_ctx( uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key, bool store_in_flash );

/*!
 * @brief Restore the Tracker context from the flash memory and set in /ref Tracker_t structure
 *
 * @param [out] SMTC_SUCESS/SMTC_FAIL
 */
uint8_t tracker_restore_app_ctx( void );

/*!
 * @brief Store the scan result in the flash memory in the dedicated user memory zone
 */
void tracker_store_internal_log( void );

/*!
 * @brief Restore the logs results from the flash memory, parse and display it
 */
void tracker_restore_internal_log( void );

/*!
 * @brief Erase the scan results and the internal log context from the flash memory
 */
void tracker_erase_internal_log( void );

/*!
 * @brief Erase the internal log et create a new internal log context.
 */
void tracker_reset_internal_log( void );

/*!
 * @brief return the user memory flash remaning space.
 *
 * @returns the user memory flash remaning space in percentage
 */
uint8_t tracker_get_remaining_memory_space( void );

/*!
 * @brief Parse the commands coming from outside.
 *
 * @param [in] payload payload to parse
 * @param [in] buffer_out answer output buffer
 * @param [in] all_command_enable activate or no all the commands
 *
 * @returns size of buffer_out
 */
uint8_t tracker_parse_cmd( uint8_t* payload, uint8_t* buffer_out, bool all_commands_enable );

/*!
 * @brief store reset counters and reset the tracker
 *
 * @param [in] host_reset_cnt_to_add value to add to host_reset_cnt_to_add paramater
 */
void tracker_store_and_reset( uint8_t host_reset_cnt_to_add );

#ifdef __cplusplus
}
#endif

#endif  // TRACKER_UTILITY_H

/* --- EOF ------------------------------------------------------------------ */
