/*!
 * @file      lr1110_modem_lorawan.c
 *
 * @brief     LoRaWAN driver implementation for LR1110 modem
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr1110_modem_lorawan.h"
#include "lr1110_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_MODEM_GET_EVENT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_VERSION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_RESET_CMD_LENGTH ( 2 )
#define LR1110_MODEM_RESET_CHARGE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_CHARGE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_START_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_CMD_LENGTH ( 2 + 10 )
#define LR1110_MODEM_TEST_MODE_TST_TX_CONT_CMD_LENGTH ( 2 + 10 )
#define LR1110_MODEM_TEST_MODE_TST_CW_CMD_LENGTH ( 2 + 6 )
#define LR1110_MODEM_TEST_MODE_TST_RX_CONT_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_RSSI_SUBGHZ_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM_CMD_LENGTH ( 2 + 12 )
#define LR1110_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_RSSI_2G4_CMD_LENGTH ( 2 + 5 )
#define LR1110_MODEM_TEST_MODE_TST_RSSI_GNSS_CMD_LENGTH ( 2 + 5 )
#define LR1110_MODEM_GET_GPS_TIME_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_STATUS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_ALARM_TIMER_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_PIN_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_CHIP_EUI_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_JOIN_EUI_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_JOIN_EUI_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_GET_DEV_EUI_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DEV_EUI_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_SET_APP_KEY_CMD_LENGTH ( 2 + LR1110_MODEM_APP_KEY_LENGTH )
#define LR1110_MODEM_GET_CLASS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_CLASS_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_REGION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_REGION_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_LIST_REGION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_ADR_PROFILE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_ADR_PROFILE_CMD_LENGTH ( 2 + 1 )  // +16 in case of adr custom
#define LR1110_MODEM_GET_DM_PORT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DM_PORT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_DM_INTERVAL_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DM_INTERVAL_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_DM_FIELDS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DM_FIELDS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SEND_DM_STATUS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_APP_STATUS_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_JOIN_CMD_LENGTH ( 2 )
#define LR1110_MODEM_LEAVE_NETWORK_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SUSPEND_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD_LENGTH ( 2 )
#define LR1110_MODEM_REQUEST_TX_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_EMERGENCY_TX_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_UPLOAD_INIT_CMD_LENGTH ( 2 + 6 )
#define LR1110_MODEM_UPLOAD_DATA_CMD_LENGTH ( 2 )
#define LR1110_MODEM_UPLOAD_START_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_STREAM_INIT_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_SEND_STREAM_DATA_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_STREAM_STATUS_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_CMD_RSP_SIZE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_SET_GPS_TIME_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_EVENT_SIZE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_DERIVE_KEYS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_MANAGE_RF_OUTPUT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_SET_ALC_SYNC_PORT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_ALC_SYNC_PORT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_ALC_SYNC_MODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_ALC_SYNC_MODE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_LORAWAN_STATE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_SET_CERTIFICATION_MODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_CERTIFICATION_MODE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_CONNECTION_TIMEOUT_GENERIC_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_AVAILABLE_DATA_RATE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_OUTPUT_POWER_CONFIG_CMD_LENGTH ( 2 + 30 )
#define LR1110_MODEM_GET_OUTPUT_POWER_CONFIG_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_NETWORK_TYPE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_NETWORK_TYPE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_ACTIVATE_LBT_CMD_LENGTH ( 2 + 11 )
#define LR1110_MODEM_SET_NB_TRANS_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_NB_TRANS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_STREAM_REDUNDANCY_RATE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_STREAM_REDUNDANCY_RATE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_CRYSTAL_ERROR_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_CRYSTAL_ERROR_CMD_LENGTH ( 2 )

#define LR1110_MODEM_INFO_FIELDS_RBUFFER_MAX_LENGTH ( 20 )
#define LR1110_MODEM_CHIP_EUI_RBUFFER_LENGTH ( 8 )
#define LR1110_MODEM_JOIN_EUI_BUFFER_LENGTH ( 8 )
#define LR1110_MODEM_DEV_EUI_BUFFER_LENGTH ( 8 )
#define LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH ( 10 )
#define LR1110_MODEM_GET_STREAM_STATUS_RBUFFER_LENGTH ( 4 )
#define LR1110_MODEM_EVENT_HEADER_LENGTH ( 2 )
#define LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_RBUFFER_LENGTH ( 4 )
#define LR1110_MODEM_MODEM_GET_OUTPUT_POWER_CONFIG_RBUFFER_LENGTH ( 30 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief Operation code command
 */
enum
{
    LR1110_MODEM_GET_EVENT_CMD                         = 0x00,
    LR1110_MODEM_GET_VERSION_CMD                       = 0x01,
    LR1110_MODEM_RESET_CMD                             = 0x02,
    LR1110_MODEM_RESET_CHARGE_CMD                      = 0x04,
    LR1110_MODEM_GET_CHARGE_CMD                        = 0x05,
    LR1110_MODEM_GET_TX_POWER_OFFSET_CMD               = 0x06,
    LR1110_MODEM_SET_TX_POWER_OFFSET_CMD               = 0x07,
    LR1110_MODEM_TEST_CMD                              = 0x08,
    LR1110_MODEM_GET_GPS_TIME_CMD                      = 0x0A,
    LR1110_MODEM_GET_STATUS_CMD                        = 0x0B,
    LR1110_MODEM_SET_ALARM_TIMER_CMD                   = 0x0C,
    LR1110_MODEM_GET_PIN_CMD                           = 0x0E,
    LR1110_MODEM_GET_CHIP_EUI_CMD                      = 0x0F,
    LR1110_MODEM_GET_JOIN_EUI_CMD                      = 0x10,
    LR1110_MODEM_SET_JOIN_EUI_CMD                      = 0x11,
    LR1110_MODEM_GET_DEV_EUI_CMD                       = 0x12,
    LR1110_MODEM_SET_DEV_EUI_CMD                       = 0x13,
    LR1110_MODEM_SET_APP_KEY_CMD                       = 0x14,
    LR1110_MODEM_GET_CLASS_CMD                         = 0x15,
    LR1110_MODEM_SET_CLASS_CMD                         = 0x16,
    LR1110_MODEM_GET_REGION_CMD                        = 0x18,
    LR1110_MODEM_SET_REGION_CMD                        = 0x19,
    LR1110_MODEM_LIST_REGIONS_CMD                      = 0x1A,
    LR1110_MODEM_GET_ADR_PROFILE_CMD                   = 0x1B,
    LR1110_MODEM_SET_ADR_PROFILE_CMD                   = 0x1C,
    LR1110_MODEM_GET_DM_PORT_CMD                       = 0x1D,
    LR1110_MODEM_SET_DM_PORT_CMD                       = 0x1E,
    LR1110_MODEM_GET_DM_INFO_INTERVAL_CMD              = 0x1F,
    LR1110_MODEM_SET_DM_INFO_INTERVAL_CMD              = 0x20,
    LR1110_MODEM_GET_DM_INFO_FIELDS_CMD                = 0x21,
    LR1110_MODEM_SET_DM_INFO_FIELDS_CMD                = 0x22,
    LR1110_MODEM_DM_STATUS_CMD                         = 0x23,
    LR1110_MODEM_APP_STATUS_CMD                        = 0x24,
    LR1110_MODEM_JOIN_CMD                              = 0x25,
    LR1110_MODEM_LEAVE_NETWORK_CMD                     = 0x26,
    LR1110_MODEM_SUSPEND_MODEM_COM_CMD                 = 0x27,
    LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD           = 0x28,
    LR1110_MODEM_REQUEST_TX_CMD                        = 0x29,
    LR1110_MODEM_EMERGENCY_TX_CMD                      = 0x2A,
    LR1110_MODEM_UPLOAD_INIT_CMD                       = 0x2B,
    LR1110_MODEM_UPLOAD_DATA_CMD                       = 0x2C,
    LR1110_MODEM_UPLOAD_START_CMD                      = 0x2D,
    LR1110_MODEM_STREAM_INIT_CMD                       = 0x2E,
    LR1110_MODEM_SEND_STREAM_DATA_CMD                  = 0x2F,
    LR1110_MODEM_STREAM_STATUS_CMD                     = 0x30,
    LR1110_MODEM_GET_RSP_SIZE_CMD                      = 0x31,
    LR1110_MODEM_SET_GPS_TIME_CMD                      = 0x32,
    LR1110_MODEM_GET_EVENT_SIZE_CMD                    = 0x33,
    LR1110_MODEM_DERIVE_KEYS_CMD                       = 0x34,
    LR1110_MODEM_MANAGE_RF_OUTPUT_CMD                  = 0x36,
    LR1110_MODEM_SET_ALC_SYNC_PORT_CMD                 = 0x37,
    LR1110_MODEM_GET_ALC_SYNC_PORT_CMD                 = 0x38,
    LR1110_MODEM_SET_ALC_SYNC_MODE_CMD                 = 0x39,
    LR1110_MODEM_GET_ALC_SYNC_MODE_CMD                 = 0x3A,
    LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD            = 0x3C,
    LR1110_MODEM_GET_CONNECTION_TIMEOUT_CMD            = 0x3D,
    LR1110_MODEM_SET_CERTIFICATION_MODE_CMD            = 0x3E,
    LR1110_MODEM_GET_CERTIFICATION_MODE_CMD            = 0x3F,
    LR1110_MODEM_GET_LORAWAN_STATUS_CMD                = 0x40,
    LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD = 0x41,
    LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD  = 0x42,
    LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD              = 0x43,
    LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD             = 0x44,
    LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD               = 0x45,
    LR1110_MODEM_GET_CONNECTION_TIMEOUT_STATUS_CMD     = 0x48,
    LR1110_MODEM_GET_AVAILABLE_DATA_RATE_CMD           = 0x49,
    LR1110_MODEM_SET_OUTPUT_POWER_CONFIG_CMD           = 0x4A,
    LR1110_MODEM_GET_OUTPUT_POWER_CONFIG_CMD           = 0x4B,
    LR1110_MODEM_GET_NETWORK_TYPE_CMD                  = 0x4C,
    LR1110_MODEM_SET_NETWORK_TYPE_CMD                  = 0x4D,
    LR1110_MODEM_ACTIVATE_LBT_CMD                      = 0x4E,
    LR1110_MODEM_SET_NB_TRANS_CMD                      = 0x4F,
    LR1110_MODEM_GET_NB_TRANS_CMD                      = 0x50,
    LR1110_MODEM_SET_STREAM_REDUNDANCY_RATE_CMD        = 0x51,
    LR1110_MODEM_GET_STREAM_REDUNDANCY_RATE_CMD        = 0x52,
    LR1110_MODEM_SET_CRYSTAL_ERROR_CMD                 = 0x53,
    LR1110_MODEM_GET_CRYSTAL_ERROR_CMD                 = 0x54,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief This command returns the number of bytes of next Event stored in LR1110 Modem. The returned value is the
 * length encoded in two bytes.
 *
 * @param [in] context Chip implementation context
 *
 * @param [out] event_size Event size encoded in two bytes
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_event_size( const void* context, uint16_t* event_size );

/*!
 * @brief This command returns size of command response to read
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] cmd Command ID of the read command to be analyzed
 *
 * @param [out] cmd_rsp_size Response size
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_cmd_rsp_size( const void* context, uint8_t cmd, uint8_t* cmd_rsp_size );

/*!
 * @brief Generic command for modem timeout getters
 *
 * It is to be used for both get connection timeout and get connection timeout status command.
 *
 * @param [in] opcode The opcode of the command to call
 *
 * @param [in] context Chip implementation context
 *
 * @param [out] nb_uplink_mobile_static number of uplink without ack from network before modem adr profile switch from
 * mobile to static
 *
 * @param [out] nb_uplink_reset number of uplink without ack from network before modem reset
 *
 * @returns Operation status
 */
static lr1110_modem_response_code_t lr1110_modem_get_timeout_generic( const uint8_t opcode, const void* context,
                                                                      uint16_t* nb_uplink_mobile_static,
                                                                      uint16_t* nb_uplink_reset );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr1110_modem_response_code_t lr1110_modem_get_event( const void* context, lr1110_modem_event_fields_t* event_fields )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_EVENT_CMD_LENGTH];
    uint16_t                     event_size;
    lr1110_modem_response_code_t rc;

    /* Read the event size */
    rc = lr1110_modem_get_event_size( context, &event_size );

    if( ( event_size >= LR1110_MODEM_EVENT_HEADER_LENGTH ) && ( rc == LR1110_MODEM_RESPONSE_CODE_OK ) )
    {
        cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
        cbuffer[1] = LR1110_MODEM_GET_EVENT_CMD;

        rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
            context, cbuffer, LR1110_MODEM_GET_EVENT_CMD_LENGTH, event_fields->buffer, event_size );

        event_fields->event_type          = ( lr1110_modem_lorawan_event_type_t ) event_fields->buffer[0];
        event_fields->missed_events_count = event_fields->buffer[1];
        event_fields->buffer_len          = event_size - LR1110_MODEM_EVENT_HEADER_LENGTH;  // Remove type and count

        for( uint16_t i = 0; i < event_fields->buffer_len; i++ )
        {
            event_fields->buffer[i] = event_fields->buffer[i + LR1110_MODEM_EVENT_HEADER_LENGTH];
        }
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_version( const void* context, lr1110_modem_version_t* version )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_VERSION_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_VERSION_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_VERSION_CMD_LENGTH,
                                                                 rbuffer, LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH );

    version->bootloader = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                          ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );
    version->functionality = ( lr1110_modem_functionality_t ) rbuffer[4];
    version->firmware =
        ( ( uint32_t ) rbuffer[5] << 16 ) + ( ( uint32_t ) rbuffer[6] << 8 ) + ( ( uint32_t ) rbuffer[7] );
    version->lorawan = ( ( uint16_t ) rbuffer[8] << 8 ) + rbuffer[9];

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_reset( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_RESET_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_RESET_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write_without_rc( context, cbuffer,
                                                                               LR1110_MODEM_RESET_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_reset_charge( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_RESET_CHARGE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_RESET_CHARGE_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_RESET_CHARGE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_charge( const void* context, uint32_t* charge )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_CHARGE_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint32_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_CHARGE_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_CHARGE_CMD_LENGTH,
                                                                 rbuffer, sizeof( uint32_t ) );

    *charge = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) + ( ( uint32_t ) rbuffer[2] << 8 ) +
              ( ( uint32_t ) rbuffer[3] );

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_tx_power_offset( const void* context, int8_t* tx_power_offset )
{
    uint8_t cbuffer[LR1110_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_TX_POWER_OFFSET_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH, ( uint8_t* ) tx_power_offset, sizeof( int8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_tx_power_offset( const void* context, const int8_t tx_power_offset )
{
    uint8_t cbuffer[LR1110_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_TX_POWER_OFFSET_CMD;

    cbuffer[2] = ( uint8_t ) tx_power_offset;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_mode_start( const void* context )
{
    uint8_t       cbuffer[LR1110_MODEM_TEST_MODE_TST_START_CMD_LENGTH];
    const uint8_t test_msg[] = { 'T', 'E', 'S', 'T', 'T', 'E', 'S', 'T' };

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_START;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_START_CMD_LENGTH, test_msg, 8 );
}

lr1110_modem_response_code_t lr1110_modem_test_nop( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_NOP;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_tx_single( const void* context, uint32_t frequency, int8_t tx_power,
                                                          lr1110_modem_tst_mode_sf_t sf, lr1110_modem_tst_mode_bw_t bw,
                                                          lr1110_modem_tst_mode_cr_t cr, uint8_t payload_length )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_CMD_LENGTH];

    cbuffer[0]  = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1]  = LR1110_MODEM_TEST_CMD;
    cbuffer[2]  = LR1110_MODEM_TEST_MODE_TST_TX_SINGLE;
    cbuffer[3]  = ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 );
    cbuffer[4]  = ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 );
    cbuffer[5]  = ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 );
    cbuffer[6]  = ( uint8_t )( frequency & 0x000000FF );
    cbuffer[7]  = ( uint8_t ) tx_power;
    cbuffer[8]  = ( uint8_t ) sf;
    cbuffer[9]  = ( uint8_t ) bw;
    cbuffer[10] = ( uint8_t ) cr;
    cbuffer[11] = payload_length;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_tx_cont( const void* context, uint32_t frequency, int8_t tx_power,
                                                        lr1110_modem_tst_mode_sf_t sf, lr1110_modem_tst_mode_bw_t bw,
                                                        lr1110_modem_tst_mode_cr_t cr, uint8_t payload_length )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_TX_CONT_CMD_LENGTH];

    cbuffer[0]  = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1]  = LR1110_MODEM_TEST_CMD;
    cbuffer[2]  = LR1110_MODEM_TEST_MODE_TST_TX_CONT;
    cbuffer[3]  = ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 );
    cbuffer[4]  = ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 );
    cbuffer[5]  = ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 );
    cbuffer[6]  = ( uint8_t )( frequency & 0x000000FF );
    cbuffer[7]  = ( uint8_t ) tx_power;
    cbuffer[8]  = ( uint8_t ) sf;
    cbuffer[9]  = ( uint8_t ) bw;
    cbuffer[10] = ( uint8_t ) cr;
    cbuffer[11] = payload_length;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_TX_CONT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_tx_cw( const void* context, uint32_t frequency, int8_t tx_power )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_CW_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_CW;
    cbuffer[3] = ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 );
    cbuffer[4] = ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 );
    cbuffer[5] = ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 );
    cbuffer[6] = ( uint8_t )( frequency & 0x000000FF );
    cbuffer[7] = ( uint8_t ) tx_power;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_TEST_MODE_TST_CW_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_rx_cont( const void* context, uint32_t frequency,
                                                        lr1110_modem_tst_mode_sf_t sf, lr1110_modem_tst_mode_bw_t bw,
                                                        lr1110_modem_tst_mode_cr_t cr )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_RX_CONT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_RX_CONT;
    cbuffer[3] = ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 );
    cbuffer[4] = ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 );
    cbuffer[5] = ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 );
    cbuffer[6] = ( uint8_t )( frequency & 0x000000FF );
    cbuffer[7] = ( uint8_t ) sf;
    cbuffer[8] = ( uint8_t ) bw;
    cbuffer[9] = ( uint8_t ) cr;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_RX_CONT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_read_packet_counter_rx_cont( const void* context,
                                                                            uint32_t*   rx_packet_counter )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH];
    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_READ_RX_PKT_COUNTER_RX_CONT;

    uint8_t rbuffer[LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_RBUFFER_LENGTH] = { 0 };

    const lr1110_modem_response_code_t response_code = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH, rbuffer,
        LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_RBUFFER_LENGTH );

    *rx_packet_counter = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                         ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );

    return response_code;
}

lr1110_modem_response_code_t lr1110_modem_test_rssi_subghz( const void* context, uint32_t frequency, uint16_t time_ms,
                                                            lr1110_modem_tst_mode_bw_t bw )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_RSSI_SUBGHZ_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_RSSI_SUBGHZ;
    cbuffer[3] = ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 );
    cbuffer[4] = ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 );
    cbuffer[5] = ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 );
    cbuffer[6] = ( uint8_t )( frequency & 0x000000FF );
    cbuffer[7] = ( uint8_t )( time_ms >> 8 );
    cbuffer[8] = ( uint8_t )( time_ms );
    cbuffer[9] = bw;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_RSSI_SUBGHZ_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_radio_rst( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_RADIO_RST;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_exit( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_EXIT;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_tx_single_preamble( const void* context, uint32_t frequency,
                                                                   int8_t tx_power, lr1110_modem_tst_mode_sf_t sf,
                                                                   lr1110_modem_tst_mode_bw_t bw,
                                                                   lr1110_modem_tst_mode_cr_t cr,
                                                                   uint8_t payload_length, uint16_t preamble_length )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM_CMD_LENGTH];

    cbuffer[0]  = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1]  = LR1110_MODEM_TEST_CMD;
    cbuffer[2]  = LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM;
    cbuffer[3]  = ( uint8_t )( ( frequency & 0xFF000000 ) >> 24 );
    cbuffer[4]  = ( uint8_t )( ( frequency & 0x00FF0000 ) >> 16 );
    cbuffer[5]  = ( uint8_t )( ( frequency & 0x0000FF00 ) >> 8 );
    cbuffer[6]  = ( uint8_t )( frequency & 0x000000FF );
    cbuffer[7]  = ( uint8_t ) tx_power;
    cbuffer[8]  = ( uint8_t ) sf;
    cbuffer[9]  = ( uint8_t ) bw;
    cbuffer[10] = ( uint8_t ) cr;
    cbuffer[11] = payload_length;
    cbuffer[12] = ( uint8_t )( preamble_length >> 8 );
    cbuffer[13] = ( uint8_t )( preamble_length );

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_read_rssi( const void* context, int8_t* rssi )
{
    uint8_t                      cbuffer[LR1110_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint8_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_READ_RSSI;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH, rbuffer, sizeof( uint8_t ) );

    *rssi = ( int8_t )( rbuffer[0] - 64 );

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_test_rssi_2g4( const void* context, uint8_t channel, uint16_t time_ms,
                                                         lr1110_modem_tst_mode_bw_t bw )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_RSSI_2G4_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_RSSI_2G4;

    cbuffer[3] = channel;
    cbuffer[4] = ( uint8_t )( time_ms >> 8 );
    cbuffer[5] = ( uint8_t ) time_ms;
    cbuffer[6] = ( uint8_t ) bw;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_RSSI_2G4_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_test_rssi_gnss(
    const void* context, lr1110_modem_tst_mode_constellation_t constellation_modulation, uint16_t time_ms,
    lr1110_modem_tst_mode_bw_t bw )
{
    uint8_t cbuffer[LR1110_MODEM_TEST_MODE_TST_RSSI_GNSS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_TEST_CMD;
    cbuffer[2] = LR1110_MODEM_TEST_MODE_TST_RSSI_GNSS;

    cbuffer[3] = constellation_modulation;
    cbuffer[4] = ( uint8_t )( time_ms >> 8 );
    cbuffer[5] = ( uint8_t ) time_ms;
    cbuffer[6] = ( uint8_t ) bw;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_TEST_MODE_TST_RSSI_GNSS_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_gps_time( const void* context, uint32_t* time )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_GPS_TIME_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint32_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_GPS_TIME_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_GPS_TIME_CMD_LENGTH,
                                                                 rbuffer, sizeof( uint32_t ) );

    *time = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) + ( ( uint32_t ) rbuffer[2] << 8 ) +
            ( ( uint32_t ) rbuffer[3] );

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_status( const void* context, lr1110_modem_status_t* status )
{
    uint8_t cbuffer[LR1110_MODEM_GET_STATUS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_STATUS_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_STATUS_CMD_LENGTH,
                                                                   ( uint8_t* ) status, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_alarm_timer( const void* context, uint32_t seconds )
{
    uint8_t cbuffer[LR1110_MODEM_SET_ALARM_TIMER_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_ALARM_TIMER_CMD;

    cbuffer[2] = ( uint8_t )( seconds >> 24 );
    cbuffer[3] = ( uint8_t )( seconds >> 16 );
    cbuffer[4] = ( uint8_t )( seconds >> 8 );
    cbuffer[5] = ( uint8_t ) seconds;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_ALARM_TIMER_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_pin( const void* context, uint32_t* pin )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_PIN_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint32_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_PIN_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_PIN_CMD_LENGTH,
                                                                 rbuffer, sizeof( uint32_t ) );

    *pin = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) + ( ( uint32_t ) rbuffer[2] << 8 ) +
           ( ( uint32_t ) rbuffer[3] );

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_chip_eui( const void* context, lr1110_modem_chip_eui_t chip_eui )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_CHIP_EUI_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_CHIP_EUI_RBUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_CHIP_EUI_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_CHIP_EUI_CMD_LENGTH,
                                                                 rbuffer, LR1110_MODEM_CHIP_EUI_RBUFFER_LENGTH );

    for( uint8_t i = 0; i < LR1110_MODEM_CHIP_EUI_RBUFFER_LENGTH; i++ )
    {
        chip_eui[i] = rbuffer[i];
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_join_eui( const void* context, lr1110_modem_join_eui_t join_eui )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_JOIN_EUI_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_JOIN_EUI_BUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_JOIN_EUI_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_JOIN_EUI_CMD_LENGTH,
                                                                 rbuffer, LR1110_MODEM_JOIN_EUI_BUFFER_LENGTH );

    for( uint8_t i = 0; i < LR1110_MODEM_JOIN_EUI_BUFFER_LENGTH; i++ )
    {
        join_eui[i] = rbuffer[i];
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_set_join_eui( const void* context, const lr1110_modem_join_eui_t join_eui )
{
    uint8_t cbuffer[LR1110_MODEM_SET_JOIN_EUI_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_JOIN_EUI_CMD;

    for( uint8_t i = 0; i < LR1110_MODEM_JOIN_EUI_BUFFER_LENGTH; i++ )
    {
        cbuffer[2 + i] = join_eui[i];
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_JOIN_EUI_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_dev_eui( const void* context, lr1110_modem_dev_eui_t dev_eui )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_DEV_EUI_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_DEV_EUI_BUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_DEV_EUI_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_DEV_EUI_CMD_LENGTH,
                                                                 rbuffer, LR1110_MODEM_DEV_EUI_BUFFER_LENGTH );

    for( uint8_t i = 0; i < LR1110_MODEM_DEV_EUI_BUFFER_LENGTH; i++ )
    {
        dev_eui[i] = rbuffer[i];
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_set_dev_eui( const void* context, const lr1110_modem_dev_eui_t dev_eui )
{
    uint8_t cbuffer[LR1110_MODEM_SET_DEV_EUI_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_DEV_EUI_CMD;

    for( uint8_t i = 0; i < LR1110_MODEM_DEV_EUI_BUFFER_LENGTH; i++ )
    {
        cbuffer[2 + i] = dev_eui[i];
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_DEV_EUI_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_set_app_key( const void* context, const lr1110_modem_app_key_t app_key )
{
    uint8_t cbuffer[LR1110_MODEM_SET_APP_KEY_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_APP_KEY_CMD;

    for( uint8_t i = 0; i < LR1110_MODEM_APP_KEY_LENGTH; i++ )
    {
        cbuffer[2 + i] = app_key[i];
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_APP_KEY_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_class( const void* context, lr1110_modem_classes_t* modem_class )
{
    uint8_t cbuffer[LR1110_MODEM_GET_CLASS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_CLASS_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_CLASS_CMD_LENGTH,
                                                                   ( uint8_t* ) modem_class, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_class( const void* context, const lr1110_modem_classes_t modem_class )
{
    uint8_t cbuffer[LR1110_MODEM_SET_CLASS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_CLASS_CMD;

    cbuffer[2] = ( uint8_t ) modem_class;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer, LR1110_MODEM_SET_CLASS_CMD_LENGTH,
                                                                    0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_region( const void* context, lr1110_modem_regions_t* region )
{
    uint8_t cbuffer[LR1110_MODEM_GET_REGION_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_REGION_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_REGION_CMD_LENGTH,
                                                                   ( uint8_t* ) region, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_region( const void* context, const lr1110_modem_regions_t region )
{
    uint8_t cbuffer[LR1110_MODEM_SET_REGION_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_REGION_CMD;

    cbuffer[2] = ( uint8_t ) region;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_REGION_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_list_regions( const void* context, lr1110_modem_regions_list_t region_codes,
                                                        uint8_t* region_codes_size )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_LIST_REGION_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_REGIONS_NUMBER] = { 0 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_LIST_REGIONS_CMD;

    rc = lr1110_modem_get_cmd_rsp_size( context, LR1110_MODEM_LIST_REGIONS_CMD, region_codes_size );

    if( rc == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        if( *region_codes_size <= LR1110_MODEM_REGIONS_NUMBER )
        {
            rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
                context, cbuffer, LR1110_MODEM_GET_LIST_REGION_CMD_LENGTH, rbuffer, *region_codes_size );

            if( rc == LR1110_MODEM_RESPONSE_CODE_OK )
            {
                for( uint8_t i = 0; i < *region_codes_size; i++ )
                {
                    region_codes[i] = ( lr1110_modem_regions_t ) rbuffer[i];
                }
            }
        }
        else
        {
            rc = LR1110_MODEM_RESPONSE_CODE_INVALID;
        }
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_adr_profile( const void*                  context,
                                                           lr1110_modem_adr_profiles_t* adr_profile )
{
    uint8_t cbuffer[LR1110_MODEM_GET_ADR_PROFILE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_ADR_PROFILE_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_ADR_PROFILE_CMD_LENGTH, ( uint8_t* ) adr_profile, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_adr_profile( const void*                       context,
                                                           const lr1110_modem_adr_profiles_t adr_profile,
                                                           const uint8_t*                    adr_custom_list )
{
    uint8_t cbuffer[LR1110_MODEM_SET_ADR_PROFILE_CMD_LENGTH];
    uint8_t data_length = 0;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_ADR_PROFILE_CMD;

    cbuffer[2] = ( uint8_t ) adr_profile;

    if( adr_profile == LR1110_MODEM_ADR_PROFILE_CUSTOM )
    {
        data_length = 16;
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SET_ADR_PROFILE_CMD_LENGTH, adr_custom_list, data_length );
}

lr1110_modem_response_code_t lr1110_modem_get_dm_port( const void* context, uint8_t* port )
{
    uint8_t cbuffer[LR1110_MODEM_GET_DM_PORT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_DM_PORT_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_DM_PORT_CMD_LENGTH, port, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_dm_port( const void* context, const uint8_t port )
{
    uint8_t cbuffer[LR1110_MODEM_SET_DM_PORT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_DM_PORT_CMD;

    cbuffer[2] = ( uint8_t ) port;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_DM_PORT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_dm_info_interval( const void*                               context,
                                                                lr1110_modem_reporting_interval_format_t* format,
                                                                uint8_t*                                  interval )
{
    lr1110_modem_response_code_t rc;

    uint8_t cbuffer[LR1110_MODEM_GET_DM_INTERVAL_CMD_LENGTH];
    uint8_t rbuffer[1] = { 0x00 };

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_DM_INFO_INTERVAL_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_DM_INTERVAL_CMD_LENGTH, rbuffer, sizeof( uint8_t ) );

    *format   = ( lr1110_modem_reporting_interval_format_t )( rbuffer[0] >> 6 );
    *interval = rbuffer[0] & 0x3F;

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_set_dm_info_interval( const void*                                    context,
                                                                const lr1110_modem_reporting_interval_format_t format,
                                                                const uint8_t interval )
{
    uint8_t cbuffer[LR1110_MODEM_SET_DM_INTERVAL_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_DM_INFO_INTERVAL_CMD;

    cbuffer[2] = ( ( uint8_t ) format << 6 ) + ( interval & 0x3F );

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_DM_INTERVAL_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_dm_info_field( const void*                    context,
                                                             lr1110_modem_dm_info_fields_t* dm_info_fields )
{
    lr1110_modem_response_code_t rc;

    uint8_t cbuffer[LR1110_MODEM_GET_DM_FIELDS_CMD_LENGTH];
    uint8_t cmd_rsp_size;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_DM_INFO_FIELDS_CMD;

    rc = lr1110_modem_get_cmd_rsp_size( context, LR1110_MODEM_GET_DM_INFO_FIELDS_CMD, &cmd_rsp_size );

    dm_info_fields->dm_info_length = cmd_rsp_size;

    if( ( dm_info_fields->dm_info_length > 0 ) && ( rc == LR1110_MODEM_RESPONSE_CODE_OK ) )
    {
        rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
            context, cbuffer, LR1110_MODEM_GET_DM_FIELDS_CMD_LENGTH, ( uint8_t* ) dm_info_fields->dm_info_field,
            dm_info_fields->dm_info_length );
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_set_dm_info_field( const void*                          context,
                                                             const lr1110_modem_dm_info_fields_t* dm_info_fields )
{
    uint8_t cbuffer[LR1110_MODEM_SET_DM_FIELDS_CMD_LENGTH + LR1110_MODEM_INFO_FIELDS_RBUFFER_MAX_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_DM_INFO_FIELDS_CMD;

    for( uint8_t i = 0; i < dm_info_fields->dm_info_length; i++ )
    {
        cbuffer[2 + i] = dm_info_fields->dm_info_field[i];
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SET_DM_FIELDS_CMD_LENGTH + dm_info_fields->dm_info_length, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_send_dm_status( const void*                    context,
                                                          lr1110_modem_dm_info_fields_t* dm_info_fields )
{
    uint8_t cbuffer[LR1110_MODEM_SEND_DM_STATUS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_DM_STATUS_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SEND_DM_STATUS_CMD_LENGTH, dm_info_fields->dm_info_field,
        dm_info_fields->dm_info_length );
}

lr1110_modem_response_code_t lr1110_modem_set_app_status( const void* context, uint8_t* app_status )
{
    uint8_t cbuffer[LR1110_MODEM_SET_APP_STATUS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_APP_STATUS_CMD;

    for( uint8_t i = 0; i < 8; i++ )
    {
        cbuffer[2 + i] = app_status[i];
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_APP_STATUS_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_join( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_JOIN_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_JOIN_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer, LR1110_MODEM_JOIN_CMD_LENGTH, 0,
                                                                    0 );
}

lr1110_modem_response_code_t lr1110_modem_leave_network( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_LEAVE_NETWORK_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_LEAVE_NETWORK_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_LEAVE_NETWORK_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_suspend( const void* context, const lr1110_modem_suspend_t suspend )
{
    uint8_t cbuffer[LR1110_MODEM_SUSPEND_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SUSPEND_MODEM_COM_CMD;

    cbuffer[2] = suspend;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer, LR1110_MODEM_SUSPEND_CMD_LENGTH,
                                                                    0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_next_tx_max_payload( const void* context, uint8_t* tx_max_payload )
{
    uint8_t cbuffer[LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD_LENGTH, tx_max_payload, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_request_tx( const void* context, const uint8_t port,
                                                      const lr1110_modem_uplink_type_t uplink_type, const uint8_t* data,
                                                      const uint8_t length )
{
    uint8_t cbuffer[LR1110_MODEM_REQUEST_TX_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_REQUEST_TX_CMD;

    cbuffer[2] = port;
    cbuffer[3] = ( uint8_t ) uplink_type;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_REQUEST_TX_CMD_LENGTH, data, length );
}

lr1110_modem_response_code_t lr1110_modem_emergency_tx( const void* context, const uint8_t port,
                                                        const lr1110_modem_uplink_type_t uplink_type,
                                                        const uint8_t* data, const uint8_t length )
{
    uint8_t cbuffer[LR1110_MODEM_EMERGENCY_TX_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_EMERGENCY_TX_CMD;

    cbuffer[2] = port;
    cbuffer[3] = ( uint8_t ) uplink_type;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_EMERGENCY_TX_CMD_LENGTH, data, length );
}

lr1110_modem_response_code_t lr1110_modem_upload_init( const void* context, const uint8_t port,
                                                       const lr1110_modem_encryption_mode_t encryption_mode,
                                                       const uint16_t size, const uint16_t interval )
{
    uint8_t cbuffer[LR1110_MODEM_UPLOAD_INIT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_UPLOAD_INIT_CMD;

    cbuffer[2] = port;
    cbuffer[3] = ( uint8_t ) encryption_mode;
    cbuffer[4] = ( uint8_t )( size >> 8 );
    cbuffer[5] = ( uint8_t ) size;
    cbuffer[6] = ( uint8_t )( interval >> 8 );
    cbuffer[7] = ( uint8_t ) interval;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_UPLOAD_INIT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_upload_data( const void* context, const uint8_t* data, const uint8_t length )
{
    uint8_t cbuffer[LR1110_MODEM_UPLOAD_DATA_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_UPLOAD_DATA_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_UPLOAD_DATA_CMD_LENGTH, data, length );
}

lr1110_modem_response_code_t lr1110_modem_upload_start( const void* context, const uint32_t crc )
{
    uint8_t cbuffer[LR1110_MODEM_UPLOAD_START_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_UPLOAD_START_CMD;

    cbuffer[2] = ( uint8_t )( crc >> 24 );
    cbuffer[3] = ( uint8_t )( crc >> 16 );
    cbuffer[4] = ( uint8_t )( crc >> 8 );
    cbuffer[5] = ( uint8_t ) crc;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_UPLOAD_START_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_stream_init( const void* context, const uint8_t port,
                                                       const lr1110_modem_encryption_mode_t encryption_mode )
{
    uint8_t cbuffer[LR1110_MODEM_STREAM_INIT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_STREAM_INIT_CMD;

    cbuffer[2] = port;
    cbuffer[3] = ( uint8_t ) encryption_mode;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_STREAM_INIT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_send_stream_data( const void* context, const uint8_t port,
                                                            const uint8_t* record, const uint8_t length )
{
    uint8_t cbuffer[LR1110_MODEM_SEND_STREAM_DATA_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SEND_STREAM_DATA_CMD;

    cbuffer[2] = port;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SEND_STREAM_DATA_CMD_LENGTH, record, length );
}

lr1110_modem_response_code_t lr1110_modem_stream_status( const void* context, const uint8_t port,
                                                         lr1110_modem_stream_status_t* stream_status )
{
    uint8_t                      cbuffer[LR1110_MODEM_STREAM_STATUS_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_GET_STREAM_STATUS_RBUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_STREAM_STATUS_CMD;

    cbuffer[2] = port;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer,
                                                                 LR1110_MODEM_STREAM_STATUS_CMD_LENGTH, rbuffer,
                                                                 LR1110_MODEM_GET_STREAM_STATUS_RBUFFER_LENGTH );

    stream_status->pending = ( rbuffer[0] << 8 ) + rbuffer[1];
    stream_status->free    = ( rbuffer[2] << 8 ) + rbuffer[3];

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_cmd_rsp_size( const void* context, uint8_t cmd, uint8_t* cmd_rsp_size )
{
    uint8_t cbuffer[LR1110_MODEM_GET_CMD_RSP_SIZE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_RSP_SIZE_CMD;

    cbuffer[2] = cmd;
    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_CMD_RSP_SIZE_CMD_LENGTH, cmd_rsp_size, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_gps_time( const void* context, const uint32_t gps_time )
{
    uint8_t cbuffer[LR1110_MODEM_SET_GPS_TIME_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_GPS_TIME_CMD;

    cbuffer[2] = ( uint8_t )( gps_time >> 24 );
    cbuffer[3] = ( uint8_t )( gps_time >> 16 );
    cbuffer[4] = ( uint8_t )( gps_time >> 8 );
    cbuffer[5] = ( uint8_t ) gps_time;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_GPS_TIME_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_event_size( const void* context, uint16_t* event_size )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_EVENT_SIZE_CMD_LENGTH];
    uint8_t                      rbuffer[2] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_EVENT_SIZE_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer,
                                                                 LR1110_MODEM_GET_EVENT_SIZE_CMD_LENGTH, rbuffer, 2 );

    *event_size = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_derive_keys( const void* context )
{
    uint8_t cbuffer[LR1110_MODEM_DERIVE_KEYS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_DERIVE_KEYS_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_DERIVE_KEYS_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_set_rf_output( const void*                             context,
                                                         const lr1110_modem_radio_pa_selection_t output )
{
    uint8_t cbuffer[LR1110_MODEM_MANAGE_RF_OUTPUT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_MANAGE_RF_OUTPUT_CMD;

    cbuffer[2] = ( uint8_t ) output;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_MANAGE_RF_OUTPUT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_set_alc_sync_port( const void* context, const uint8_t port )
{
    uint8_t cbuffer[LR1110_MODEM_SET_ALC_SYNC_PORT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_ALC_SYNC_PORT_CMD;

    cbuffer[2] = port;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_ALC_SYNC_PORT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_alc_sync_port( const void* context, uint8_t* port )
{
    uint8_t cbuffer[LR1110_MODEM_GET_ALC_SYNC_PORT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_ALC_SYNC_PORT_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_ALC_SYNC_PORT_CMD_LENGTH, port, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_alc_sync_mode( const void*                        context,
                                                             const lr1110_modem_alc_sync_mode_t mode )
{
    uint8_t cbuffer[LR1110_MODEM_SET_ALC_SYNC_MODE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_ALC_SYNC_MODE_CMD;

    cbuffer[2] = ( uint8_t ) mode;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_ALC_SYNC_MODE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_alc_sync_mode( const void* context, lr1110_modem_alc_sync_mode_t* mode )
{
    uint8_t cbuffer[LR1110_MODEM_GET_ALC_SYNC_MODE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_ALC_SYNC_MODE_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_ALC_SYNC_MODE_CMD_LENGTH, ( uint8_t* ) mode,
        sizeof( lr1110_modem_alc_sync_mode_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_connection_timeout( const void*    context,
                                                                  const uint16_t nb_uplink_mobile_static,
                                                                  const uint16_t nb_uplink_reset )
{
    uint8_t cbuffer[LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD;

    cbuffer[2] = ( uint8_t )( nb_uplink_mobile_static >> 8 );
    cbuffer[3] = ( uint8_t ) nb_uplink_mobile_static;
    cbuffer[4] = ( uint8_t )( nb_uplink_reset >> 8 );
    cbuffer[5] = ( uint8_t ) nb_uplink_reset;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_connection_timeout( const void* context,
                                                                  uint16_t*   nb_uplink_mobile_static,
                                                                  uint16_t*   nb_uplink_reset )
{
    return lr1110_modem_get_timeout_generic( LR1110_MODEM_GET_CONNECTION_TIMEOUT_CMD, context, nb_uplink_mobile_static,
                                             nb_uplink_reset );
}

lr1110_modem_response_code_t lr1110_modem_get_connection_timeout_status( const void* context,
                                                                         uint16_t*   nb_uplink_mobile_static,
                                                                         uint16_t*   nb_uplink_reset )
{
    return lr1110_modem_get_timeout_generic( LR1110_MODEM_GET_CONNECTION_TIMEOUT_STATUS_CMD, context,
                                             nb_uplink_mobile_static, nb_uplink_reset );
}

lr1110_modem_response_code_t lr1110_modem_get_lorawan_state( const void*                   context,
                                                             lr1110_modem_lorawan_state_t* lorawan_state )
{
    uint8_t cbuffer[LR1110_MODEM_GET_LORAWAN_STATE_CMD_LENGTH];
    uint8_t state = 0;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_LORAWAN_STATUS_CMD;

    const lr1110_modem_hal_status_t hal_status =
        lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_LORAWAN_STATE_CMD_LENGTH, &state, 1 );

    if( hal_status == LR1110_MODEM_HAL_STATUS_OK )
    {
        *lorawan_state = ( state == 0 ) ? LR1110_MODEM_LORAWAN_IDLE : LR1110_MODEM_LORAWAN_BUSY;
    }

    return ( lr1110_modem_response_code_t ) hal_status;
}

lr1110_modem_response_code_t lr1110_modem_write_user_defined_charge_counter( const void*    context,
                                                                             const uint16_t charge )
{
    uint8_t cbuffer[LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD;

    cbuffer[2] = ( uint8_t )( charge >> 8 );
    cbuffer[3] = ( uint8_t ) charge;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_read_user_defined_charge_counter( const void* context, uint16_t* charge )
{
    uint8_t                      cbuffer[LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint16_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    *charge = ( ( uint16_t ) rbuffer[0] << 8 ) + rbuffer[1];

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_select_charge_uplink( const void*                      context,
                                                                const lr1110_modem_charge_type_t charge_type )
{
    uint8_t cbuffer[LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD;

    cbuffer[2] = ( uint8_t ) charge_type;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_duty_cycle_status( const void* context, int32_t* duty_cycle )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( int32_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD_LENGTH, rbuffer, sizeof( int32_t ) );

    *duty_cycle = ( ( int32_t ) rbuffer[0] << 24 ) + ( ( int32_t ) rbuffer[1] << 16 ) +
                  ( ( int32_t ) rbuffer[2] << 8 ) + ( int32_t ) rbuffer[3];

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_activate_duty_cycle( const void*                     context,
                                                               const lr1110_modem_duty_cycle_t duty_cycle )
{
    uint8_t cbuffer[LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD;

    cbuffer[2] = ( uint8_t ) duty_cycle;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_set_certification_mode( const void*                             context,
                                                                  const lr1110_modem_certification_mode_t enable )
{
    uint8_t cbuffer[LR1110_MODEM_SET_CERTIFICATION_MODE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_CERTIFICATION_MODE_CMD;

    cbuffer[2] = ( uint8_t ) enable;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SET_CERTIFICATION_MODE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_certification_mode( const void*                        context,
                                                                  lr1110_modem_certification_mode_t* enable )
{
    uint8_t cbuffer[LR1110_MODEM_GET_CERTIFICATION_MODE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_CERTIFICATION_MODE_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_CERTIFICATION_MODE_CMD_LENGTH, ( uint8_t* ) enable, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_get_available_data_rate( const void* context, uint16_t* available_data_rate )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_AVAILABLE_DATA_RATE_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint16_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_AVAILABLE_DATA_RATE_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_AVAILABLE_DATA_RATE_CMD_LENGTH, rbuffer, sizeof( uint16_t ) );

    *available_data_rate = ( ( uint16_t ) rbuffer[0] << 8 ) + rbuffer[1];

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_set_output_power_config(
    const void* context, const lr1110_modem_output_power_config_list_t output_power_config )
{
    uint8_t cbuffer[LR1110_MODEM_SET_OUTPUT_POWER_CONFIG_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_OUTPUT_POWER_CONFIG_CMD;

    for( uint8_t i = 0; i < LR1110_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS; i++ )
    {
        cbuffer[2 + ( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH )] = output_power_config[i].expected_power;
        cbuffer[2 + ( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 1] =
            output_power_config[i].configured_power;
        cbuffer[2 + ( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 2] =
            ( ( output_power_config[i].pa_sel & 0x0F ) << 4 ) | ( ( uint8_t ) output_power_config[i].pa_supply & 0x0F );
        cbuffer[2 + ( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 3] =
            ( ( output_power_config[i].pa_duty_cycle & 0x0F ) << 4 ) | ( output_power_config[i].pa_hp_sel & 0x0F );
        cbuffer[2 + ( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 4] =
            ( uint8_t ) output_power_config[i].pa_ramp_time;
    }

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SET_OUTPUT_POWER_CONFIG_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_output_power_config(
    const void* context, lr1110_modem_output_power_config_list_t output_power_config )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_OUTPUT_POWER_CONFIG_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_MODEM_GET_OUTPUT_POWER_CONFIG_RBUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_OUTPUT_POWER_CONFIG_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_OUTPUT_POWER_CONFIG_CMD_LENGTH, rbuffer,
        LR1110_MODEM_MODEM_GET_OUTPUT_POWER_CONFIG_RBUFFER_LENGTH );

    for( uint8_t i = 0; i < LR1110_MODEM_NB_OUTPUT_POWER_CONFIG_BLOCKS; i++ )
    {
        output_power_config[i].expected_power   = rbuffer[i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH];
        output_power_config[i].configured_power = rbuffer[( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 1];
        output_power_config[i].pa_sel =
            ( rbuffer[( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 2] & 0xF0 ) >> 4;
        output_power_config[i].pa_supply = ( lr1110_modem_pa_reg_supply_t )(
            rbuffer[( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 2] & 0x0F );
        output_power_config[i].pa_duty_cycle =
            ( rbuffer[( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 3] & 0xF0 ) >> 4;
        output_power_config[i].pa_hp_sel = rbuffer[( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 3] & 0x0F;
        output_power_config[i].pa_ramp_time =
            ( lr1110_modem_ramp_time_t )( rbuffer[( i * LR1110_MODEM_OUTPUT_POWER_CONFIG_BLOCK_LENGTH ) + 4] );
    }

    return rc;
}

lr1110_modem_response_code_t lr1110_modem_get_network_type( const void*                  context,
                                                            lr1110_modem_network_type_t* network_type )
{
    uint8_t cbuffer[LR1110_MODEM_GET_NETWORK_TYPE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_NETWORK_TYPE_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_NETWORK_TYPE_CMD_LENGTH, ( uint8_t* ) network_type, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_network_type( const void*                       context,
                                                            const lr1110_modem_network_type_t network_type )
{
    uint8_t cbuffer[LR1110_MODEM_SET_NETWORK_TYPE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_NETWORK_TYPE_CMD;

    cbuffer[2] = ( uint8_t ) network_type;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_NETWORK_TYPE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_activate_lbt( const void* context, const lr1110_modem_lbt_mode_t enable,
                                                        const int16_t threshold, const uint32_t duration,
                                                        const uint32_t bandwidth )
{
    uint8_t cbuffer[LR1110_MODEM_ACTIVATE_LBT_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_ACTIVATE_LBT_CMD;

    cbuffer[2]  = ( uint8_t ) enable;
    cbuffer[3]  = ( uint8_t )( threshold >> 8 );
    cbuffer[4]  = ( uint8_t )( threshold );
    cbuffer[5]  = ( uint8_t )( duration >> 24 );
    cbuffer[6]  = ( uint8_t )( duration >> 16 );
    cbuffer[7]  = ( uint8_t )( duration >> 8 );
    cbuffer[8]  = ( uint8_t )( duration );
    cbuffer[9]  = ( uint8_t )( bandwidth >> 24 );
    cbuffer[10] = ( uint8_t )( bandwidth >> 16 );
    cbuffer[11] = ( uint8_t )( bandwidth >> 8 );
    cbuffer[12] = ( uint8_t )( bandwidth );

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_ACTIVATE_LBT_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_set_nb_trans( const void* context, const uint8_t nb_trans )
{
    uint8_t cbuffer[LR1110_MODEM_SET_NB_TRANS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_NB_TRANS_CMD;

    cbuffer[2] = nb_trans;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_NB_TRANS_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_nb_trans( const void* context, uint8_t* nb_trans )
{
    uint8_t cbuffer[LR1110_MODEM_GET_NB_TRANS_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_NB_TRANS_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_NB_TRANS_CMD_LENGTH, ( uint8_t* ) nb_trans, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_stream_redundancy_rate( const void*   context,
                                                                      const uint8_t stream_redundancy_rate )
{
    uint8_t cbuffer[LR1110_MODEM_SET_STREAM_REDUNDANCY_RATE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_STREAM_REDUNDANCY_RATE_CMD;

    cbuffer[2] = stream_redundancy_rate;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write(
        context, cbuffer, LR1110_MODEM_SET_STREAM_REDUNDANCY_RATE_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_stream_redundancy_rate( const void* context,
                                                                      uint8_t*    stream_redundancy_rate )
{
    uint8_t cbuffer[LR1110_MODEM_GET_STREAM_REDUNDANCY_RATE_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_STREAM_REDUNDANCY_RATE_CMD;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer,
                                                                   LR1110_MODEM_GET_STREAM_REDUNDANCY_RATE_CMD_LENGTH,
                                                                   stream_redundancy_rate, sizeof( uint8_t ) );
}

lr1110_modem_response_code_t lr1110_modem_set_crystal_error( const void* context, const uint32_t crystal_error )
{
    uint8_t cbuffer[LR1110_MODEM_SET_CRYSTAL_ERROR_CMD_LENGTH];

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_SET_CRYSTAL_ERROR_CMD;

    cbuffer[2] = ( uint8_t )( crystal_error >> 24 );
    cbuffer[3] = ( uint8_t )( crystal_error >> 16 );
    cbuffer[4] = ( uint8_t )( crystal_error >> 8 );
    cbuffer[5] = ( uint8_t ) crystal_error;

    return ( lr1110_modem_response_code_t ) lr1110_modem_hal_write( context, cbuffer,
                                                                    LR1110_MODEM_SET_CRYSTAL_ERROR_CMD_LENGTH, 0, 0 );
}

lr1110_modem_response_code_t lr1110_modem_get_crystal_error( const void* context, uint32_t* crystal_error )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_CRYSTAL_ERROR_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint32_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_CRYSTAL_ERROR_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_CRYSTAL_ERROR_CMD_LENGTH, rbuffer, sizeof( uint32_t ) );

    *crystal_error = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                     ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );

    return rc;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

lr1110_modem_response_code_t lr1110_modem_get_timeout_generic( const uint8_t opcode, const void* context,
                                                               uint16_t* nb_uplink_mobile_static,
                                                               uint16_t* nb_uplink_reset )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_CONNECTION_TIMEOUT_GENERIC_CMD_LENGTH];
    uint8_t                      rbuffer[sizeof( uint32_t )] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = opcode;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read(
        context, cbuffer, LR1110_MODEM_GET_CONNECTION_TIMEOUT_GENERIC_CMD_LENGTH, rbuffer, sizeof( uint32_t ) );

    *nb_uplink_mobile_static = ( ( uint16_t ) rbuffer[0] << 8 ) + ( uint16_t ) rbuffer[1];

    *nb_uplink_reset = ( ( uint16_t ) rbuffer[2] << 8 ) + ( uint16_t ) rbuffer[3];

    return rc;
}

/* --- EOF ------------------------------------------------------------------ */
