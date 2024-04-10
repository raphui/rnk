/*!
 * @file      tracker_utility.c
 *
 * @brief     tracker utility implementation.
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

#include <string.h>
#include <unistd.h>
#include <time.h>
#include "modem/lr1110_modem_board.h"
#include "tracker.h"
#include "tracker_utility.h"
//#include "main_tracker.h"
//#include "lorawan_commissioning.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Tracker context structure
 */
tracker_ctx_t tracker_ctx;

/*!
 * @brief Internal Log tracker settings send status
 */
bool internal_log_tracker_settings_sent = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void tracker_print_device_settings(struct tracker *tracker);

/*!
 *@brief Read 4 bytes from array buffer at index and interpret it as uint32_t LSB. Increment index so that
 * it is position on the byte just after the uint32 read (so possibly after the end of array if the
 * uint32 that is being read is at the end of the buffer).
 * @param [in] dev_eui LoRaWAN Device Eui
 * @param [in] join_eui LoRaWAN Join Eui
 *
 * @returns uint32_t value
 */
uint32_t get_uint32_from_array_at_index_and_inc(const uint8_t* array, uint16_t* index);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint8_t tracker_init_internal_log_ctx(struct tracker *tracker)
{
    if(tracker->tracker_ctx.internal_log_empty == FLASH_BYTE_EMPTY_CONTENT)
    {
        tracker->tracker_ctx.nb_scan               = 0;
        tracker->tracker_ctx.flash_addr_start      = tracker->internal_log_start_addr;
        tracker->tracker_ctx.flash_addr_current    = tracker_ctx.flash_addr_start;
        tracker->tracker_ctx.flash_addr_end        = TRACKER_INTERNAL_LOG_END;
        tracker->tracker_ctx.flash_remaining_space = tracker_ctx.flash_addr_end - tracker_ctx.flash_addr_current;
        tracker_store_internal_log_ctx(tracker);
    }
    else
    {
        return 0;
    }

    return 1;
}

uint8_t tracker_restore_internal_log_ctx(struct tracker *tracker)
{
    uint8_t ctx_buf[32];
    uint8_t index = 0;

    lseek(tracker->lr1110.mtd_id, TRACKER_INTERNAL_LOG_CTX_START, SEEK_SET);
    read(tracker->lr1110.mtd_id, ctx_buf, 32);

    tracker->tracker_ctx.internal_log_flush_request = false;
    tracker->tracker_ctx.internal_log_empty = ctx_buf[0];
    index = 1;

    if(tracker->tracker_ctx.internal_log_empty == FLASH_BYTE_EMPTY_CONTENT)
    {
        return 0;
    }
    else
    {
        tracker->tracker_ctx.nb_scan = ctx_buf[index++];
        tracker->tracker_ctx.nb_scan += (uint32_t) ctx_buf[index++] << 8;

        tracker->tracker_ctx.flash_addr_start = ctx_buf[index++];
        tracker->tracker_ctx.flash_addr_start += (uint32_t) ctx_buf[index++] << 8;
        tracker->tracker_ctx.flash_addr_start += (uint32_t) ctx_buf[index++] << 16;
        tracker->tracker_ctx.flash_addr_start += (uint32_t) ctx_buf[index++] << 24;

        tracker->internal_log_start_addr = tracker->tracker_ctx.flash_addr_start;

        tracker->tracker_ctx.flash_addr_end = ctx_buf[index++];
        tracker->tracker_ctx.flash_addr_end += (uint32_t) ctx_buf[index++] << 8;
        tracker->tracker_ctx.flash_addr_end += (uint32_t) ctx_buf[index++] << 16;
        tracker->tracker_ctx.flash_addr_end += (uint32_t) ctx_buf[index++] << 24;

        tracker->tracker_ctx.flash_addr_current = ctx_buf[index++];
        tracker->tracker_ctx.flash_addr_current += (uint32_t) ctx_buf[index++] << 8;
        tracker->tracker_ctx.flash_addr_current += (uint32_t) ctx_buf[index++] << 16;
        tracker->tracker_ctx.flash_addr_current += (uint32_t) ctx_buf[index++] << 24;

        tracker->tracker_ctx.flash_remaining_space = ctx_buf[index++];
        tracker->tracker_ctx.flash_remaining_space += (uint32_t) ctx_buf[index++] << 8;
        tracker->tracker_ctx.flash_remaining_space += (uint32_t) ctx_buf[index++] << 16;
        tracker->tracker_ctx.flash_remaining_space += (uint32_t) ctx_buf[index++] << 24;
    }

    return 1;
}

void tracker_store_internal_log_ctx(struct tracker *tracker)
{
    uint8_t ctx_buf[32];
    uint8_t index = 0;

    if(tracker->tracker_ctx.internal_log_empty != FLASH_BYTE_EMPTY_CONTENT)
    {
	ioctl(tracker->lr1110.mtd_id, IOCTL_ERASE, (char *)TRACKER_INTERNAL_LOG_CTX_START);
    }
    else
    {
        tracker->tracker_ctx.internal_log_empty = 1;
    }

    ctx_buf[index++] = tracker->tracker_ctx.internal_log_empty;

    ctx_buf[index++] = tracker->tracker_ctx.nb_scan;
    ctx_buf[index++] = tracker->tracker_ctx.nb_scan >> 8;

    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_start;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_start >> 8;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_start >> 16;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_start >> 24;

    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_end;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_end >> 8;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_end >> 16;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_end >> 24;

    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_current;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_current >> 8;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_current >> 16;
    ctx_buf[index++] = tracker->tracker_ctx.flash_addr_current >> 24;

    tracker->tracker_ctx.flash_remaining_space = tracker->tracker_ctx.flash_addr_end - tracker->tracker_ctx.flash_addr_current;

    ctx_buf[index++] = tracker->tracker_ctx.flash_remaining_space;
    ctx_buf[index++] = tracker->tracker_ctx.flash_remaining_space >> 8;
    ctx_buf[index++] = tracker->tracker_ctx.flash_remaining_space >> 16;
    ctx_buf[index++] = tracker->tracker_ctx.flash_remaining_space >> 24;

    lseek(tracker->lr1110.mtd_id, TRACKER_INTERNAL_LOG_CTX_START, SEEK_SET);
    write(tracker->lr1110.mtd_id, ctx_buf, index);
}

void tracker_erase_internal_log(struct tracker *tracker)
{
    uint8_t nb_page_to_erase = 0;

    if(tracker->tracker_ctx.nb_scan > 0)
    {
        nb_page_to_erase =
            ((tracker->tracker_ctx.flash_addr_current - tracker->tracker_ctx.flash_addr_start) / PAGE_SIZE) + 1;
        /* Erase scan results */

	for (int i = 0; i < nb_page_to_erase; i++) {
  	    ioctl(tracker->lr1110.mtd_id, IOCTL_ERASE,
		(char *)(tracker->internal_log_start_addr + i * PAGE_SIZE));
	}
    }
    /* Erase ctx */
    ioctl(tracker->lr1110.mtd_id, IOCTL_ERASE, (char *)TRACKER_INTERNAL_LOG_CTX_START);
}

void tracker_reset_internal_log(struct tracker *tracker)
{
    if(tracker->tracker_ctx.internal_log_empty != FLASH_BYTE_EMPTY_CONTENT)
    {
        tracker_erase_internal_log(tracker);

        tracker->tracker_ctx.internal_log_empty = FLASH_BYTE_EMPTY_CONTENT;
    }

    /* Reinit the internal log context */
    tracker_init_internal_log_ctx(tracker);
}

uint8_t tracker_get_remaining_memory_space(struct tracker *tracker)
{
    return (tracker->tracker_ctx.flash_remaining_space * 100) / (tracker->tracker_ctx.flash_addr_end - tracker->tracker_ctx.flash_addr_start);
}

uint8_t tracker_restore_app_ctx(struct tracker *tracker)
{
    uint8_t tracker_ctx_buf[255];

    lseek(tracker->lr1110.mtd_id, TRACKER_APP_CTX_START, SEEK_SET);
    read(tracker->lr1110.mtd_id, tracker_ctx_buf, 255);

    tracker->tracker_ctx.tracker_context_empty = tracker_ctx_buf[0];

    if(tracker->tracker_ctx.tracker_context_empty == FLASH_BYTE_EMPTY_CONTENT)
    {
        return 0;
    }
    else
    {
        uint8_t tracker_ctx_buf_idx = 1;
        int32_t latitude = 0, longitude = 0;

        memcpy(tracker->tracker_ctx.dev_eui, tracker_ctx_buf + tracker_ctx_buf_idx, SET_LORAWAN_DEVEUI_LEN);
        tracker_ctx_buf_idx += SET_LORAWAN_DEVEUI_LEN;
        memcpy(tracker->tracker_ctx.join_eui, tracker_ctx_buf + tracker_ctx_buf_idx, SET_LORAWAN_JOINEUI_LEN);
        tracker_ctx_buf_idx += SET_LORAWAN_JOINEUI_LEN;
        memcpy(tracker->tracker_ctx.app_key, tracker_ctx_buf + tracker_ctx_buf_idx, SET_LORAWAN_APPKEY_LEN);
        tracker_ctx_buf_idx += SET_LORAWAN_APPKEY_LEN;

        /* GNSS Parameters */
        tracker->tracker_ctx.gnss_settings.enabled              = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.gnss_settings.constellation_to_use = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.gnss_settings.scan_type            = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.gnss_settings.search_mode =
            (lr1110_modem_gnss_search_mode_t) tracker_ctx_buf[tracker_ctx_buf_idx++];

        latitude = tracker_ctx_buf[tracker_ctx_buf_idx++];
        latitude += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        latitude += tracker_ctx_buf[tracker_ctx_buf_idx++] << 16;
        latitude += tracker_ctx_buf[tracker_ctx_buf_idx++] << 24;
        tracker->tracker_ctx.gnss_settings.assistance_position.latitude = (float) latitude / 10000000;

        longitude = tracker_ctx_buf[tracker_ctx_buf_idx++];
        longitude += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        longitude += tracker_ctx_buf[tracker_ctx_buf_idx++] << 16;
        longitude += tracker_ctx_buf[tracker_ctx_buf_idx++] << 24;
        tracker->tracker_ctx.gnss_settings.assistance_position.longitude = (float) longitude / 10000000;

        tracker->tracker_ctx.last_almanac_update = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.last_almanac_update += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.last_almanac_update += tracker_ctx_buf[tracker_ctx_buf_idx++] << 16;
        tracker->tracker_ctx.last_almanac_update += tracker_ctx_buf[tracker_ctx_buf_idx++] << 24;

        /* WiFi Parameters */
        tracker->tracker_ctx.wifi_settings.enabled  = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.channels = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.channels += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.wifi_settings.types =
            (lr1110_modem_wifi_signal_type_scan_t) tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.scan_mode    = (lr1110_modem_wifi_mode_t) tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.nbr_retrials = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.max_results  = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.timeout      = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.wifi_settings.timeout += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.wifi_settings.result_format =
            (lr1110_modem_wifi_result_format_t) tracker_ctx_buf[tracker_ctx_buf_idx++];

        /* Application Parameters */
        tracker->tracker_ctx.accelerometer_used = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.app_scan_interval  = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.app_scan_interval += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.app_scan_interval += tracker_ctx_buf[tracker_ctx_buf_idx++] << 16;
        tracker->tracker_ctx.app_scan_interval += tracker_ctx_buf[tracker_ctx_buf_idx++] << 24;

        tracker->tracker_ctx.app_keep_alive_frame_interval = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.app_keep_alive_frame_interval += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.app_keep_alive_frame_interval += tracker_ctx_buf[tracker_ctx_buf_idx++] << 16;
        tracker->tracker_ctx.app_keep_alive_frame_interval += tracker_ctx_buf[tracker_ctx_buf_idx++] << 24;

        tracker->tracker_ctx.lorawan_region          = (lr1110_modem_regions_t) tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.use_semtech_join_server = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.airplane_mode           = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.scan_priority           = (tracker_scan_priority_t) tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.lorawan_adr_profile     = (lr1110_modem_adr_profiles_t) tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.internal_log_enable     = tracker_ctx_buf[tracker_ctx_buf_idx++];

        tracker->tracker_ctx.accumulated_charge = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.accumulated_charge += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.accumulated_charge += tracker_ctx_buf[tracker_ctx_buf_idx++] << 16;
        tracker->tracker_ctx.accumulated_charge += tracker_ctx_buf[tracker_ctx_buf_idx++] << 24;

        tracker->tracker_ctx.host_reset_cnt = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.host_reset_cnt += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
        tracker->tracker_ctx.modem_reset_by_itself_cnt = tracker_ctx_buf[tracker_ctx_buf_idx++];
        tracker->tracker_ctx.modem_reset_by_itself_cnt += tracker_ctx_buf[tracker_ctx_buf_idx++] << 8;
    }
    return 1;
}

void tracker_store_app_ctx(struct tracker *tracker)
{
    uint8_t tracker_ctx_buf[255];
    uint8_t tracker_ctx_buf_idx = 0;
    int32_t latitude = 0, longitude = 0;

    if(tracker->tracker_ctx.tracker_context_empty != FLASH_BYTE_EMPTY_CONTENT)
    {
	ioctl(tracker->lr1110.mtd_id, IOCTL_ERASE, (char *)TRACKER_APP_CTX_START);
    }

    /* Context exists */
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.tracker_context_empty;

    /* LoRaWAN Parameter */
    memcpy(tracker_ctx_buf + tracker_ctx_buf_idx, tracker->tracker_ctx.dev_eui, SET_LORAWAN_DEVEUI_LEN);
    tracker_ctx_buf_idx += SET_LORAWAN_DEVEUI_LEN;
    memcpy(tracker_ctx_buf + tracker_ctx_buf_idx, tracker->tracker_ctx.join_eui, SET_LORAWAN_JOINEUI_LEN);
    tracker_ctx_buf_idx += SET_LORAWAN_JOINEUI_LEN;
    memcpy(tracker_ctx_buf + tracker_ctx_buf_idx, tracker->tracker_ctx.app_key, SET_LORAWAN_APPKEY_LEN);
    tracker_ctx_buf_idx += SET_LORAWAN_APPKEY_LEN;

    /* GNSS Parameters */
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.gnss_settings.enabled;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.gnss_settings.constellation_to_use;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.gnss_settings.scan_type;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.gnss_settings.search_mode;

    latitude                               = tracker->tracker_ctx.gnss_settings.assistance_position.latitude * 10000000;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = latitude;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = latitude >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = latitude >> 16;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = latitude >> 24;

    longitude                              = tracker->tracker_ctx.gnss_settings.assistance_position.longitude * 10000000;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = longitude;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = longitude >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = longitude >> 16;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = longitude >> 24;

    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.last_almanac_update;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.last_almanac_update >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.last_almanac_update >> 16;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.last_almanac_update >> 24;

    /* WiFi Parameters */
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.enabled;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.channels;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.channels >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.types;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.scan_mode;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.nbr_retrials;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.max_results;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.timeout;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.timeout >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.wifi_settings.result_format;

    /* Application Parameters */
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.accelerometer_used;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_scan_interval;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_scan_interval >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_scan_interval >> 16;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_scan_interval >> 24;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_keep_alive_frame_interval;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_keep_alive_frame_interval >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_keep_alive_frame_interval >> 16;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.app_keep_alive_frame_interval >> 24;

    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.lorawan_region;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.use_semtech_join_server;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.airplane_mode;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.scan_priority;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.lorawan_adr_profile;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.internal_log_enable;

    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.accumulated_charge;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.accumulated_charge >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.accumulated_charge >> 16;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.accumulated_charge >> 24;

    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.host_reset_cnt;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.host_reset_cnt >> 8;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.modem_reset_by_itself_cnt;
    tracker_ctx_buf[tracker_ctx_buf_idx++] = tracker->tracker_ctx.modem_reset_by_itself_cnt >> 8;

    lseek(tracker->lr1110.mtd_id, TRACKER_APP_CTX_START, SEEK_SET);
    write(tracker->lr1110.mtd_id, tracker_ctx_buf, tracker_ctx_buf_idx);
}

void tracker_init_app_ctx(struct tracker *tracker, uint8_t* dev_eui, uint8_t* join_eui, uint8_t* app_key, bool store_in_flash)
{
    /* Context exists */
    tracker->tracker_ctx.tracker_context_empty = 1;

    /* LoRaWAN Parameter */
    memcpy(tracker->tracker_ctx.dev_eui, dev_eui, 8);
    memcpy(tracker->tracker_ctx.join_eui, join_eui, 8);
    memcpy(tracker->tracker_ctx.app_key, app_key, 16);
    tracker->tracker_ctx.lorawan_region          = LORAWAN_REGION_USED;
    tracker->tracker_ctx.use_semtech_join_server = USE_SEMTECH_JOIN_SERVER;
    if((LORAWAN_REGION_USED == LR1110_LORAWAN_REGION_EU868) ||
        (LORAWAN_REGION_USED == LR1110_LORAWAN_REGION_RU864))
    {
        tracker->tracker_ctx.lorawan_adr_profile = LR1110_MODEM_ADR_PROFILE_CUSTOM;
    }
    else
    {
        tracker->tracker_ctx.lorawan_adr_profile = LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE;
    }

    /* GNSS Parameters */
    tracker->tracker_ctx.gnss_settings.enabled              = true;
    tracker->tracker_ctx.gnss_settings.constellation_to_use = LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK;
    tracker->tracker_ctx.gnss_settings.scan_type            = ASSISTED_MODE;
    tracker->tracker_ctx.gnss_settings.search_mode          = LR1110_MODEM_GNSS_OPTION_BEST_EFFORT;
    /* Set default position to Semtech France */
    tracker->tracker_ctx.gnss_settings.assistance_position.latitude  = 45.208;
    tracker->tracker_ctx.gnss_settings.assistance_position.longitude = 5.781;
    tracker->tracker_ctx.scan_priority                               = TRACKER_GNSS_PRIORITY;

    /* Wi-Fi Parameters */
    tracker->tracker_ctx.wifi_settings.enabled       = true;
    tracker->tracker_ctx.wifi_settings.channels      = 0x421;  // by default enable 1/6/1 channels
    tracker->tracker_ctx.wifi_settings.types         = LR1110_MODEM_WIFI_TYPE_SCAN_B;
    tracker->tracker_ctx.wifi_settings.scan_mode     = LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PKT;
    tracker->tracker_ctx.wifi_settings.nbr_retrials  = WIFI_NBR_RETRIALS_DEFAULT;
    tracker->tracker_ctx.wifi_settings.max_results   = WIFI_MAX_RESULTS_DEFAULT;
    tracker->tracker_ctx.wifi_settings.timeout       = WIFI_TIMEOUT_IN_MS_DEFAULT;
    tracker->tracker_ctx.wifi_settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;

    /* Application Parameters */
    tracker->tracker_ctx.accelerometer_used            = true;
    tracker->tracker_ctx.app_scan_interval             = TRACKER_SCAN_INTERVAL;
    tracker->tracker_ctx.app_keep_alive_frame_interval = TRACKER_KEEP_ALIVE_FRAME_INTERVAL;
    tracker->tracker_ctx.airplane_mode                 = false;
    tracker->tracker_ctx.internal_log_enable           = true;
    tracker->tracker_ctx.accumulated_charge            = 0;
    tracker->tracker_ctx.host_reset_cnt                = 0;
    tracker->tracker_ctx.modem_reset_by_itself_cnt     = 0;

    if(store_in_flash == true)
    {
        tracker_store_app_ctx(tracker);
    }
}

void tracker_store_internal_log(struct tracker *tracker)
{
    uint8_t  scan_buf[512];
    uint8_t  nb_variable_elements = 0;
    uint16_t index                = 3;  // index 0 1 and 2 are reserved for the scan length and the number of elements
    uint16_t index_next_addr      = 0;
    uint32_t next_scan_addr       = 0;

    memset(scan_buf, 0, 512);

    if(tracker->tracker_ctx.flash_remaining_space > 512)
    {
        /* Scan number */
        tracker->tracker_ctx.nb_scan++;  // Increase the nb_scan
        scan_buf[index++] = tracker->tracker_ctx.nb_scan;
        scan_buf[index++] = tracker->tracker_ctx.nb_scan >> 8;

        /* Scan Timestamp */
        scan_buf[index++] = tracker->tracker_ctx.timestamp;
        scan_buf[index++] = tracker->tracker_ctx.timestamp >> 8;
        scan_buf[index++] = tracker->tracker_ctx.timestamp >> 16;
        scan_buf[index++] = tracker->tracker_ctx.timestamp >> 24;

        /* Accelerometer data */
        scan_buf[index++] = tracker->tracker_ctx.accelerometer_x;
        scan_buf[index++] = tracker->tracker_ctx.accelerometer_x >> 8;
        scan_buf[index++] = tracker->tracker_ctx.accelerometer_y;
        scan_buf[index++] = tracker->tracker_ctx.accelerometer_y >> 8;
        scan_buf[index++] = tracker->tracker_ctx.accelerometer_z;
        scan_buf[index++] = tracker->tracker_ctx.accelerometer_z >> 8;

        /* Temperature durring scan */
        scan_buf[index++] = tracker->tracker_ctx.temperature;
        scan_buf[index++] = tracker->tracker_ctx.temperature >> 8;

        /* GNSS scan */
        if((tracker->tracker_ctx.gnss_scan_result.nav_message_size > 0) && (GNSS_LOG_ACTIVATED == 1))
        {
            scan_buf[index++] = TAG_GNSS;
            scan_buf[index++] = GNSS_TIMING_LEN + tracker->tracker_ctx.gnss_scan_result.nav_message_size;

            /* Add timings */
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.radio_ms;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.radio_ms >> 8;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.radio_ms >> 16;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.radio_ms >> 24;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.computation_ms;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.computation_ms >> 8;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.computation_ms >> 16;
            scan_buf[index++] = tracker->tracker_ctx.gnss_scan_result.timings.computation_ms >> 24;

            memcpy(&scan_buf[index], tracker->tracker_ctx.gnss_scan_result.nav_message,
                    tracker->tracker_ctx.gnss_scan_result.nav_message_size);
            index += tracker->tracker_ctx.gnss_scan_result.nav_message_size;
            nb_variable_elements++;
        }

        /* WiFi scan */
        if((tracker->tracker_ctx.wifi_result.nbr_results > 0) && (WIFI_LOG_ACTIVATED == 1))
        {
            scan_buf[index++] = TAG_WIFI;
            scan_buf[index++] = WIFI_TIMING_LEN + WIFI_SINGLE_BEACON_LEN * tracker->tracker_ctx.wifi_result.nbr_results;

            /* Add timings */
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.demodulation_us;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.demodulation_us >> 8;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.demodulation_us >> 16;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.demodulation_us >> 24;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_capture_us;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_capture_us >> 8;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_capture_us >> 16;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_capture_us >> 24;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_correlation_us;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_correlation_us >> 8;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_correlation_us >> 16;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_correlation_us >> 24;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_detection_us;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_detection_us >> 8;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_detection_us >> 16;
            scan_buf[index++] = tracker->tracker_ctx.wifi_result.timings.rx_detection_us >> 24;

            for(uint8_t i = 0; i < tracker->tracker_ctx.wifi_result.nbr_results; i++)
            {
                scan_buf[index]     = tracker->tracker_ctx.wifi_result.results[i].rssi;
                scan_buf[index + 1] = ((tracker->tracker_ctx.wifi_result.results[i].channel & 0x0F) |
                                        ((tracker->tracker_ctx.wifi_result.results[i].type & 0x03) << 4));
                memcpy(&scan_buf[index + 2], tracker->tracker_ctx.wifi_result.results[i].mac_address, 6);
                index += WIFI_SINGLE_BEACON_LEN;
            }
            nb_variable_elements++;
        }

        /* Next scan addr */
        scan_buf[index++] = TAG_NEXT_SCAN;
        scan_buf[index++] = 4;
        /* Complete index for FLASH_TYPEPROGRAM_DOUBLEWORD operation and define the next addr */
        index_next_addr = index;
        index += 4;
        if((index % 8) != 0)  // 4: anticipate the buffer increment
        {
            index = index + (8 - (index % 8));
        }
        next_scan_addr                = tracker->tracker_ctx.flash_addr_current + index;
        scan_buf[index_next_addr]     = next_scan_addr;
        scan_buf[index_next_addr + 1] = next_scan_addr >> 8;
        scan_buf[index_next_addr + 2] = next_scan_addr >> 16;
        scan_buf[index_next_addr + 3] = next_scan_addr >> 24;

        /* Scan Len */
        scan_buf[0] = index;
        scan_buf[1] = index >> 8;

        /* nb elements */
        scan_buf[2] = nb_variable_elements + 1;  // +1 because of the next address scan

	lseek(tracker->lr1110.mtd_id, tracker->tracker_ctx.flash_addr_start, SEEK_SET);
	write(tracker->lr1110.mtd_id, scan_buf, index);

        tracker->tracker_ctx.flash_addr_current = next_scan_addr;
        tracker_store_internal_log_ctx(tracker);
    }
}

void tracker_restore_internal_log(struct tracker *tracker)
{
    uint8_t   scan_buf[512];
    uint16_t  scan_buf_index    = 0;
    uint8_t   nb_elements       = 0;
    uint8_t   nb_elements_index = 0;
    uint8_t   tag_element       = 0;
    uint16_t  scan_len          = 0;
    uint16_t  nb_scan_index     = 1;
    uint16_t  scan_number;
    int16_t   acc_x, acc_y, acc_z;
    int16_t   temperature    = 0;
    uint32_t  next_scan_addr = tracker->tracker_ctx.flash_addr_start;
    time_t    scan_timestamp = 0;
//FIXME
#if 0
    struct tm epoch_time;
#endif
    uint32_t  job_counter = 0;

    tracker_print_device_settings(tracker);

    while(nb_scan_index <= tracker->tracker_ctx.nb_scan)
    {
        /* read the scan lentgh */
	lseek(tracker->lr1110.mtd_id, next_scan_addr, SEEK_SET);
	read(tracker->lr1110.mtd_id, scan_buf, 2);

        scan_len = scan_buf[0];
        scan_len += (uint32_t) scan_buf[1] << 8;

        /* read the rest of the scan */
	read(tracker->lr1110.mtd_id, scan_buf, scan_len - 2);

        nb_elements = scan_buf[scan_buf_index++];

        /* Scan number */
        scan_number = scan_buf[scan_buf_index++];
        scan_number += (uint16_t) scan_buf[scan_buf_index++] << 8;

        /* Scan Timestamp */
        scan_timestamp = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);
//FIXME
#if 0
        memcpy(&epoch_time, localtime(&scan_timestamp), sizeof(struct tm));
#endif

        /* Acceleromter data */
        acc_x = scan_buf[scan_buf_index++];
        acc_x += (uint16_t) scan_buf[scan_buf_index++] << 8;
        acc_y = scan_buf[scan_buf_index++];
        acc_y += (uint16_t) scan_buf[scan_buf_index++] << 8;
        acc_z = scan_buf[scan_buf_index++];
        acc_z += (uint16_t) scan_buf[scan_buf_index++] << 8;
//FIXME
#if 0
        HAL_DBG_TRACE_PRINTF("[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                              epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec);
#endif
        HAL_DBG_TRACE_PRINTF("[%d - %d] ", job_counter++, 4);
        HAL_DBG_TRACE_PRINTF("%d,%d,%d\r\n", acc_x, acc_y, acc_z);

        temperature = scan_buf[scan_buf_index++];
        temperature += (uint16_t) scan_buf[scan_buf_index++] << 8;
//FIXME
#if 0
        HAL_DBG_TRACE_PRINTF("[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                              epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min, epoch_time.tm_sec);
#endif
        HAL_DBG_TRACE_PRINTF("[%d - %d] ", job_counter++, 5);
        HAL_DBG_TRACE_PRINTF("%2.2f\r\n", ((float) temperature) / 100);

        while(nb_elements_index < nb_elements)
        {
            uint8_t len = 0;
            tag_element = scan_buf[scan_buf_index++];  // get the element
            len         = scan_buf[scan_buf_index++];  // get the size element

            switch(tag_element)
            {
            case TAG_GNSS:
                if(GNSS_DISPLAY_LOG_ACTIVATED)
                {
                    uint32_t radio_ms;
                    uint32_t computation_ms;
                    uint16_t nav_len = len - GNSS_TIMING_LEN;

                    /* Get Timings */
                    radio_ms       = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);
                    computation_ms = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);

                    /* Display Raw NAV Message*/
//FIXME
#if 0
                    HAL_DBG_TRACE_PRINTF("[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900, epoch_time.tm_mon + 1,
                                          epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min,
                                          epoch_time.tm_sec);
#endif
                    HAL_DBG_TRACE_PRINTF("[%d - %d] ", job_counter++, tag_element);

                    for(uint8_t i = 0; i < nav_len; i++)
                    {
                        HAL_DBG_TRACE_PRINTF("%02X", scan_buf[scan_buf_index++]);
                    }
                    HAL_DBG_TRACE_PRINTF(",0,%d,%d\r\n", radio_ms, computation_ms);

                    if(PAYLOAD_DISPLAY_LOG_ACTIVATED)
                    {
                        /* Display in Formatted Payload */
//FIXME
#if 0
                        HAL_DBG_TRACE_PRINTF("[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900,
                                              epoch_time.tm_mon + 1, epoch_time.tm_mday, epoch_time.tm_hour,
                                              epoch_time.tm_min, epoch_time.tm_sec);
#endif
                        HAL_DBG_TRACE_PRINTF("[%d - %d] ", job_counter++, TLV_GNSS_NAV_TAG);

                        HAL_DBG_TRACE_PRINTF("%02X%02X", TLV_GNSS_NAV_TAG, len);
                        scan_buf_index -= len;
                        for(uint8_t i = 0; i < len; i++)
                        {
                            HAL_DBG_TRACE_PRINTF("%02X", scan_buf[scan_buf_index++]);
                        }
                        HAL_DBG_TRACE_MSG("\r\n");
                    }
                }
                else
                {
                    scan_buf_index += len;
                }
                break;
            case TAG_WIFI:
                if(WIFI_DISPLAY_LOG_ACTIVATED)
                {
                    int8_t                                 wifi_rssi;
                    uint8_t                                wifi_data;
                    lr1110_modem_wifi_channel_t            wifi_channel;
                    lr1110_modem_wifi_signal_type_result_t wifi_type;
                    char                                   wifi_type_char = 'B';
                    lr1110_modem_wifi_cumulative_timings_t timings;

                    /* Get Timings */
                    timings.demodulation_us   = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);
                    timings.rx_capture_us     = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);
                    timings.rx_correlation_us = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);
                    timings.rx_detection_us   = get_uint32_from_array_at_index_and_inc(scan_buf, &scan_buf_index);

                    for(uint8_t i = 0; i < ((len - WIFI_TIMING_LEN) / WIFI_SINGLE_BEACON_LEN); i++)
                    {
//FIXME
#if 0
                        HAL_DBG_TRACE_PRINTF("[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900,
                                              epoch_time.tm_mon + 1, epoch_time.tm_mday, epoch_time.tm_hour,
                                              epoch_time.tm_min, epoch_time.tm_sec);
#endif
                        HAL_DBG_TRACE_PRINTF("[%d - %d] ", job_counter, tag_element);

                        wifi_rssi = scan_buf[scan_buf_index++];

                        wifi_data    = scan_buf[scan_buf_index++];
                        wifi_channel = (lr1110_modem_wifi_channel_t)(wifi_data & 0x0F);
                        wifi_type    = (lr1110_modem_wifi_signal_type_result_t)((wifi_data & 0x30) >> 4);

                        switch(wifi_type)
                        {
                        case LR1110_MODEM_WIFI_TYPE_RESULT_B:
                            wifi_type_char = 'B';
                            break;
                        case LR1110_MODEM_WIFI_TYPE_RESULT_G:
                            wifi_type_char = 'G';
                            break;
                        case LR1110_MODEM_WIFI_TYPE_RESULT_N:
                            wifi_type_char = 'N';
                            break;
                        default:
                            break;
                        }

                        /* Display MAC address */
                        for(uint8_t i = 0; i < 5; i++)
                        {
                            HAL_DBG_TRACE_PRINTF("%02X:", scan_buf[scan_buf_index++]);
                        }
                        HAL_DBG_TRACE_PRINTF("%02X,", scan_buf[scan_buf_index++]);

                        /* Display RSSI */
                        HAL_DBG_TRACE_PRINTF("CHANNEL_%d,TYPE_%c,%d,%d,%d,%d,%d\r\n", wifi_channel, wifi_type_char,
                                              wifi_rssi, timings.demodulation_us, timings.rx_capture_us,
                                              timings.rx_correlation_us, timings.rx_detection_us);
                    }
                    if(PAYLOAD_DISPLAY_LOG_ACTIVATED)
                    {
                        /* Display in Formatted Payload */
                        job_counter++;
                        scan_buf_index -= len;
//FIXME
#if 0
                        HAL_DBG_TRACE_PRINTF("[%d-%d-%d %d:%d:%d.000] ", epoch_time.tm_year + 1900,
                                              epoch_time.tm_mon + 1, epoch_time.tm_mday, epoch_time.tm_hour,
                                              epoch_time.tm_min, epoch_time.tm_sec);
#endif
                        HAL_DBG_TRACE_PRINTF("[%d - %d] ", job_counter, TLV_WIFI_SCAN_TAG);
                        HAL_DBG_TRACE_PRINTF("%02X%02X", TLV_WIFI_SCAN_TAG, len);
                        for(uint8_t i = 0; i < len / WIFI_SINGLE_BEACON_LEN; i++)
                        {
                            HAL_DBG_TRACE_PRINTF("%02X", scan_buf[scan_buf_index++]);
                            for(uint8_t i = 0; i < 6; i++)
                            {
                                HAL_DBG_TRACE_PRINTF("%02X", scan_buf[scan_buf_index++]);
                            }
                        }
                        HAL_DBG_TRACE_MSG("\r\n");
                    }
                    job_counter++;  // incremente for next job
                }
                else
                {
                    scan_buf_index += len;
                }
                break;
            case TAG_NEXT_SCAN:
                next_scan_addr = scan_buf[scan_buf_index++];
                next_scan_addr += (uint16_t) scan_buf[scan_buf_index++] << 8;
                next_scan_addr += (uint32_t) scan_buf[scan_buf_index++] << 16;
                next_scan_addr += (uint32_t) scan_buf[scan_buf_index++] << 24;
                break;
            default:
            {
                if((tag_element < 1) && (tag_element > 4))
                {
                    scan_buf_index += len;
                }
            }
            break;
            }
            nb_elements_index++;
        }
        nb_scan_index++;
        nb_elements_index = 0;
        scan_buf_index    = 0;  // reset the index;
    }
}

uint8_t tracker_parse_cmd(struct tracker *tracker, uint8_t* payload, uint8_t* buffer_out, bool all_commands_enable)
{
    uint8_t nb_elements         = 0;
    uint8_t nb_elements_index   = 0;
    uint8_t payload_index       = 0;
    uint8_t output_buffer_index = 1;
    uint8_t tag                 = 0;
    uint8_t len                 = 0;
    uint8_t res_size            = 0;
    bool    reset_board_asked   = false;

    nb_elements = payload[payload_index++];

    buffer_out[0] = 0;  // ensure that byte 0 is set to 0 at the beggining.

    if(nb_elements > 0)
    {
        while(nb_elements_index < nb_elements)
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            switch(tag)
            {
            case GET_FW_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_FW_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_FW_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = TRACKER_MAJOR_APP_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_MINOR_APP_VERSION;
                buffer_out[output_buffer_index++] = TRACKER_SUB_MINOR_APP_VERSION;

                payload_index += GET_FW_VERSION_LEN;
                break;
            }

            case GET_STACK_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_STACK_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_STACK_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.lorawan >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.lorawan;

                payload_index += GET_STACK_VERSION_LEN;
                break;
            }

            case GET_MODEM_VERSION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_VERSION_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_VERSION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.firmware >> 16;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.firmware >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.firmware;

                payload_index += GET_MODEM_VERSION_LEN;
                break;
            }

            case GET_MODEM_STATUS_CMD:
            {
                lr1110_modem_status_t modem_status;

                lr1110_modem_get_status(&tracker->lr1110, &modem_status);

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_STATUS_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_STATUS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.has_date;
                buffer_out[output_buffer_index++] = modem_status;

                payload_index += GET_MODEM_STATUS_LEN;
                break;
            }

            case GET_MODEM_DATE_CMD:
            {
                uint32_t date = lr1110_modem_board_get_systime_from_gps(&tracker->lr1110);

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_MODEM_DATE_CMD;
                buffer_out[output_buffer_index++] = GET_MODEM_DATE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = date >> 24;
                buffer_out[output_buffer_index++] = date >> 16;
                buffer_out[output_buffer_index++] = date >> 8;
                buffer_out[output_buffer_index++] = date;

                payload_index += GET_MODEM_DATE_LEN;
                break;
            }

            case GET_LORAWAN_PIN_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_PIN_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_PIN_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_pin >> 24;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_pin >> 16;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_pin >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_pin;

                payload_index += GET_LORAWAN_PIN_LEN;
                break;
            }

            case SET_LORAWAN_DEVEUI_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                memcpy(tracker->tracker_ctx.dev_eui, payload + payload_index, SET_LORAWAN_DEVEUI_LEN);
                tracker->tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_DEVEUI_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_DEVEUI_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.dev_eui, SET_LORAWAN_DEVEUI_LEN);
                output_buffer_index += SET_LORAWAN_DEVEUI_LEN;

                lr1110_modem_set_dev_eui(&tracker->lr1110, tracker->tracker_ctx.dev_eui);
                /* do a derive key to have the new pin code */
                lr1110_modem_derive_keys(&tracker->lr1110);
                lr1110_modem_get_pin(&tracker->lr1110, &tracker->tracker_ctx.lorawan_pin);

                payload_index += SET_LORAWAN_DEVEUI_LEN;
                break;
            }

            case GET_LORAWAN_DEVEUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_DEVEUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_DEVEUI_ANSWER_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.dev_eui, GET_LORAWAN_DEVEUI_ANSWER_LEN);
                output_buffer_index += GET_LORAWAN_DEVEUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_DEVEUI_LEN;
                break;
            }

            case GET_LORAWAN_CHIP_EUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_CHIP_EUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_CHIP_EUI_ANSWER_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.chip_eui, GET_LORAWAN_CHIP_EUI_ANSWER_LEN);
                output_buffer_index += GET_LORAWAN_CHIP_EUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_CHIP_EUI_LEN;
                break;
            }

            case SET_LORAWAN_JOINEUI_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                memcpy(tracker->tracker_ctx.join_eui, payload + payload_index, SET_LORAWAN_JOINEUI_LEN);
                tracker->tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOINEUI_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_JOINEUI_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.join_eui, SET_LORAWAN_JOINEUI_LEN);
                output_buffer_index += SET_LORAWAN_JOINEUI_LEN;

                /* do a derive key to have the new pin code */
                lr1110_modem_set_join_eui(&tracker->lr1110, tracker->tracker_ctx.join_eui);
                lr1110_modem_derive_keys(&tracker->lr1110);
                lr1110_modem_get_pin(&tracker->lr1110, &tracker->tracker_ctx.lorawan_pin);

                payload_index += SET_LORAWAN_JOINEUI_LEN;
                break;
            }

            case GET_LORAWAN_JOINEUI_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOINEUI_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOINEUI_ANSWER_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.join_eui, GET_LORAWAN_JOINEUI_ANSWER_LEN);
                output_buffer_index += GET_LORAWAN_JOINEUI_ANSWER_LEN;

                payload_index += GET_LORAWAN_JOINEUI_LEN;
                break;
            }

            case SET_LORAWAN_APPKEY_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                memcpy(tracker->tracker_ctx.app_key, payload + payload_index, SET_LORAWAN_APPKEY_LEN);
                tracker->tracker_ctx.lorawan_parameters_have_changed = true;

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_LORAWAN_APPKEY_CMD;
                buffer_out[output_buffer_index++] = SET_LORAWAN_APPKEY_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.app_key, SET_LORAWAN_APPKEY_LEN);
                output_buffer_index += SET_LORAWAN_APPKEY_LEN;

                payload_index += SET_LORAWAN_APPKEY_LEN;
                break;
            }

            case GET_LORAWAN_APPKEY_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_APPKEY_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_APPKEY_ANSWER_LEN;
                memcpy(buffer_out + output_buffer_index, tracker->tracker_ctx.app_key, GET_LORAWAN_APPKEY_ANSWER_LEN);
                output_buffer_index += GET_LORAWAN_APPKEY_ANSWER_LEN;

                payload_index += GET_LORAWAN_APPKEY_LEN;
                break;
            }

            case GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD:
            {
                uint16_t nb_uplink_mobile_static;
                uint16_t nb_uplink_reset;

                lr1110_modem_get_connection_timeout_status(&tracker->lr1110, &nb_uplink_mobile_static, &nb_uplink_reset);

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_ANSWER_LEN;
                buffer_out[output_buffer_index++] = nb_uplink_reset >> 8;
                buffer_out[output_buffer_index++] = nb_uplink_reset;

                payload_index += GET_LORAWAN_NB_UPLINK_SINCE_LAST_DOWNLINK_LEN;
                break;
            }

            case SET_GNSS_ENABLE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.gnss_settings.enabled = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.enabled;
                }
                else
                {
                    tracker->tracker_ctx.gnss_settings.enabled = 1;
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.enabled;
                }
                payload_index += SET_GNSS_ENABLE_LEN;
                break;
            }

            case GET_GNSS_ENABLE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_ENABLE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_ENABLE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.enabled;

                payload_index += GET_GNSS_ENABLE_LEN;
                break;
            }

            case SET_GNSS_CONSTELLATION_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 2)
                {
                    tracker->tracker_ctx.gnss_settings.constellation_to_use = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.constellation_to_use;
                }
                else
                {
                    tracker->tracker_ctx.gnss_settings.constellation_to_use = 3;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_CONSTELLATION_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.constellation_to_use;
                }

                payload_index += SET_GNSS_CONSTELLATION_LEN;
                break;
            }

            case GET_GNSS_CONSTELLATION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_CONSTELLATION_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_CONSTELLATION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.constellation_to_use;

                payload_index += GET_GNSS_CONSTELLATION_LEN;
                break;
            }

            case SET_GNSS_SCAN_TYPE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if((payload[payload_index] >= 1) && (payload[payload_index] <= 3))
                {
                    tracker->tracker_ctx.gnss_settings.scan_type = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.scan_type;
                }
                else
                {
                    /* NAck the CMD */
                    /* Clip the value */
                    if(payload[payload_index] < 1)
                    {
                        tracker->tracker_ctx.gnss_settings.scan_type = 1;
                    }
                    else
                    {
                        tracker->tracker_ctx.gnss_settings.scan_type = 3;
                    }
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SCAN_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.scan_type;
                }
                payload_index += SET_GNSS_SCAN_TYPE_LEN;
                break;
            }

            case GET_GNSS_SCAN_TYPE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_SCAN_TYPE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_SCAN_TYPE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.scan_type;

                payload_index += GET_GNSS_SCAN_TYPE_LEN;
                break;
            }

            case SET_GNSS_SEARCH_MODE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.gnss_settings.search_mode = (lr1110_modem_gnss_search_mode_t) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.search_mode;
                }
                else
                {
                    tracker->tracker_ctx.gnss_settings.search_mode = (lr1110_modem_gnss_search_mode_t) 1;
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_GNSS_SEARCH_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.search_mode;
                }

                payload_index += SET_GNSS_SEARCH_MODE_LEN;
                break;
            }

            case GET_GNSS_SEARCH_MODE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_SEARCH_MODE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_SEARCH_MODE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.search_mode;

                payload_index += GET_GNSS_SEARCH_MODE_LEN;
                break;
            }

            case GET_GNSS_LAST_ALMANAC_UPDATE_CMD:
            {
                uint32_t oldest_almanac_date = 0;
                uint32_t newest_almanac_date = 0;

                /* get the dates form the Modem-E */
                lr1110_modem_get_almanac_dates(&tracker->lr1110, &oldest_almanac_date, &newest_almanac_date);

                if(oldest_almanac_date > tracker->tracker_ctx.last_almanac_update)
                {
                    tracker->tracker_ctx.last_almanac_update = oldest_almanac_date;
                }

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_ALMANAC_UPDATE_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_ALMANAC_UPDATE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_almanac_update >> 24;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_almanac_update >> 16;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_almanac_update >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_almanac_update;

                payload_index += GET_GNSS_LAST_ALMANAC_UPDATE_LEN;
                break;
            }

            case GET_GNSS_LAST_NB_SV_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_NB_SV_CMD;
                buffer_out[output_buffer_index++] = GET_GNSS_LAST_NB_SV_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_nb_detected_satellites;

                payload_index += GET_GNSS_LAST_NB_SV_LEN;
                break;
            }

            case SET_WIFI_ENABLE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.wifi_settings.enabled = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.enabled;
                }
                else
                {
                    tracker->tracker_ctx.wifi_settings.enabled = 1;
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_ENABLE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.enabled;
                }

                payload_index += SET_WIFI_ENABLE_LEN;
                break;
            }

            case GET_WIFI_ENABLE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_ENABLE_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_ENABLE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.enabled;

                payload_index += GET_WIFI_ENABLE_LEN;
                break;
            }

            case SET_WIFI_CHANNELS_CMD:
            {
                uint16_t wifi_channels;
                tracker->tracker_ctx.new_value_to_set = true;

                wifi_channels = (uint16_t) payload[payload_index] << 8;
                wifi_channels += payload[payload_index + 1];

                if(wifi_channels <= 0x3FFF)
                {
                    tracker->tracker_ctx.wifi_settings.channels = wifi_channels;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels >> 8;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels;
                }
                else
                {
                    tracker->tracker_ctx.wifi_settings.channels = 0x3FFF;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_CHANNELS_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels >> 8;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels;
                }

                payload_index += SET_WIFI_CHANNELS_LEN;
                break;
            }

            case GET_WIFI_CHANNELS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_CHANNELS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_CHANNELS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels;

                payload_index += GET_WIFI_CHANNELS_LEN;
                break;
            }

            case SET_WIFI_TYPE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if((payload[payload_index] >= 1) && (payload[payload_index] <= 2))
                {
                    tracker->tracker_ctx.wifi_settings.types = (lr1110_modem_wifi_signal_type_scan_t) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.types;
                }
                else
                {
                    /* Clip the value */
                    if(payload[payload_index] < 1)
                    {
                        tracker->tracker_ctx.wifi_settings.types = (lr1110_modem_wifi_signal_type_scan_t) 1;
                    }
                    else
                    {
                        tracker->tracker_ctx.wifi_settings.types = (lr1110_modem_wifi_signal_type_scan_t) 2;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TYPE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.types;
                }
                payload_index += SET_WIFI_TYPE_LEN;
                break;
            }

            case GET_WIFI_TYPE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_TYPE_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_TYPE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.types;

                payload_index += GET_WIFI_TYPE_LEN;
                break;
            }

            case SET_WIFI_SCAN_MODE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if((payload[payload_index] >= 1) && (payload[payload_index] <= 2))
                {
                    tracker->tracker_ctx.wifi_settings.scan_mode = (lr1110_modem_wifi_mode_t) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.scan_mode;
                }
                else
                {
                    /* Clip the value */
                    if(payload[payload_index] < 1)
                    {
                        tracker->tracker_ctx.wifi_settings.scan_mode = (lr1110_modem_wifi_mode_t) 1;
                    }
                    else
                    {
                        tracker->tracker_ctx.wifi_settings.scan_mode = (lr1110_modem_wifi_mode_t) 2;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_SCAN_MODE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.scan_mode;
                }
                payload_index += SET_WIFI_SCAN_MODE_LEN;
                break;
            }

            case GET_WIFI_SCAN_MODE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_SCAN_MODE_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_SCAN_MODE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.scan_mode;

                payload_index += GET_WIFI_SCAN_MODE_LEN;
                break;
            }

            case SET_WIFI_RETRIALS_CMD:
            {
                tracker->tracker_ctx.new_value_to_set           = true;
                tracker->tracker_ctx.wifi_settings.nbr_retrials = payload[payload_index];

                /* Ack the CMD */
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_WIFI_RETRIALS_CMD;
                buffer_out[output_buffer_index++] = SET_WIFI_RETRIALS_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.nbr_retrials;

                payload_index += SET_WIFI_RETRIALS_LEN;
                break;
            }

            case GET_WIFI_RETRIALS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_RETRIALS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_RETRIALS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.nbr_retrials;

                payload_index += GET_WIFI_RETRIALS_LEN;
                break;
            }

            case SET_WIFI_MAX_RESULTS_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if((payload[payload_index] >= 1) && (payload[payload_index] <= 32))
                {
                    tracker->tracker_ctx.wifi_settings.max_results = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.max_results;
                }
                else
                {
                    /* Clip the value */
                    if(payload[payload_index] < 1)
                    {
                        tracker->tracker_ctx.wifi_settings.max_results = 1;
                    }
                    else
                    {
                        tracker->tracker_ctx.wifi_settings.max_results = 32;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_MAX_RESULTS_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.max_results;
                }

                payload_index += SET_WIFI_MAX_RESULTS_LEN;
                break;
            }

            case GET_WIFI_MAX_RESULTS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_MAX_RESULTS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_MAX_RESULTS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.max_results;

                payload_index += GET_WIFI_MAX_RESULTS_LEN;
                break;
            }

            case SET_WIFI_TIMEOUT_CMD:
            {
                uint16_t wifi_timeout = 0;

                tracker->tracker_ctx.new_value_to_set = true;
                wifi_timeout                 = (uint16_t) payload[payload_index] << 8;
                wifi_timeout += payload[payload_index + 1];

                if((wifi_timeout >= 20) && (wifi_timeout <= 5000))
                {
                    tracker->tracker_ctx.wifi_settings.timeout = wifi_timeout;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout >> 8;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout;
                }
                else
                {
                    /* Clip the value */
                    if(payload[payload_index] < 20)
                    {
                        tracker->tracker_ctx.wifi_settings.timeout = 20;
                    }
                    else
                    {
                        tracker->tracker_ctx.wifi_settings.timeout = 5000;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_CMD;
                    buffer_out[output_buffer_index++] = SET_WIFI_TIMEOUT_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout >> 8;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout;
                }
                payload_index += SET_WIFI_TIMEOUT_LEN;
                break;
            }

            case GET_WIFI_TIMEOUT_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_TIMEOUT_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_TIMEOUT_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout;

                payload_index += GET_WIFI_TIMEOUT_LEN;
                break;
            }

            case GET_WIFI_LAST_NB_MAC_ADDRESS_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_WIFI_LAST_NB_MAC_ADDRESS_CMD;
                buffer_out[output_buffer_index++] = GET_WIFI_LAST_NB_MAC_ADDRESS_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_nb_detected_mac_address;

                payload_index += GET_WIFI_LAST_NB_MAC_ADDRESS_LEN;
                break;
            }

            case SET_USE_ACCELEROMETER_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.accelerometer_used = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_CMD;
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.accelerometer_used;
                }
                else
                {
                    /* Clip the value */
                    tracker->tracker_ctx.accelerometer_used = 1;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_CMD;
                    buffer_out[output_buffer_index++] = SET_USE_ACCELEROMETER_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.accelerometer_used;
                }

                payload_index += SET_USE_ACCELEROMETER_LEN;
                break;
            }

            case GET_USE_ACCELEROMETER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_USE_ACCELEROMETER_CMD;
                buffer_out[output_buffer_index++] = GET_USE_ACCELEROMETER_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.accelerometer_used;

                payload_index += GET_USE_ACCELEROMETER_LEN;
                break;
            }

            case SET_APP_SCAN_INTERVAL_CMD:
            {
                uint16_t app_duty_cycle = 0;

                tracker->tracker_ctx.new_value_to_set = true;

                app_duty_cycle = (uint16_t) payload[payload_index] << 8;
                app_duty_cycle += payload[payload_index + 1];

                if((app_duty_cycle >= 10) && (app_duty_cycle <= 1800))
                {
                    tracker->tracker_ctx.app_scan_interval = app_duty_cycle * 1000;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000) >> 8;
                    buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000);
                }
                else
                {
                    /* Clip the value */
                    if(payload[payload_index] < 10)
                    {
                        tracker->tracker_ctx.app_scan_interval = 10 * 1000;
                    }
                    else
                    {
                        tracker->tracker_ctx.app_scan_interval = 1800 * 1000;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_SCAN_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000) >> 8;
                    buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000);
                }

                payload_index += SET_APP_SCAN_INTERVAL_LEN;
                break;
            }

            case GET_APP_SCAN_INTERVAL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_SCAN_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = GET_APP_SCAN_INTERVAL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000) >> 8;
                buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000);

                payload_index += GET_APP_SCAN_INTERVAL_LEN;
                break;
            }

            case SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD:
            {
                uint16_t app_low_duty_cycle = 0;

                tracker->tracker_ctx.new_value_to_set = true;

                app_low_duty_cycle = (uint16_t) payload[payload_index] << 8;
                app_low_duty_cycle += payload[payload_index + 1];

                if((app_low_duty_cycle >= 10) && (app_low_duty_cycle <= 1440))
                {
                    tracker->tracker_ctx.app_keep_alive_frame_interval = app_low_duty_cycle * 60 * 1000;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = ((tracker->tracker_ctx.app_keep_alive_frame_interval / 60000) >> 8);
                    buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_keep_alive_frame_interval / 60000);
                }
                else
                {
                    /* Clip the value */
                    if(payload[payload_index] < 10)
                    {
                        tracker->tracker_ctx.app_keep_alive_frame_interval = 10 * 60 * 1000;
                    }
                    else
                    {
                        tracker->tracker_ctx.app_keep_alive_frame_interval = 1440 * 60 * 1000;
                    }
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                    buffer_out[output_buffer_index++] = ((tracker->tracker_ctx.app_keep_alive_frame_interval / 60000) >> 8);
                    buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_keep_alive_frame_interval / 60000);
                }

                payload_index += SET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                break;
            }

            case GET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_KEEP_ALINE_FRAME_INTERVAL_CMD;
                buffer_out[output_buffer_index++] = GET_APP_KEEP_ALINE_FRAME_INTERVAL_ANSWER_LEN;
                buffer_out[output_buffer_index++] = ((tracker->tracker_ctx.app_keep_alive_frame_interval / 60000) >> 8);
                buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_keep_alive_frame_interval / 60000);

                payload_index += GET_APP_KEEP_ALINE_FRAME_INTERVAL_LEN;
                break;
            }

            case SET_LORAWAN_REGION_CMD:
            {
                if((payload[payload_index] == 1) || (payload[payload_index] == 3))
                {
                    tracker->tracker_ctx.new_value_to_set                = true;
                    tracker->tracker_ctx.lorawan_region                  = (lr1110_modem_regions_t) payload[payload_index];
                    tracker->tracker_ctx.lorawan_parameters_have_changed = true;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_region;
                }
                else
                {
                    /* Don't change the value of the region in this case */
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_REGION_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_region;
                }

                payload_index += SET_LORAWAN_REGION_LEN;
                break;
            }

            case GET_LORAWAN_REGION_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_REGION_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_REGION_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_region;

                payload_index += GET_LORAWAN_REGION_LEN;
                break;
            }

            case SET_LORAWAN_JOIN_SERVER_CMD:
            {
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.new_value_to_set                = true;
                    tracker->tracker_ctx.use_semtech_join_server         = payload[payload_index];
                    tracker->tracker_ctx.lorawan_parameters_have_changed = true;

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.use_semtech_join_server;
                }
                else
                {
                    /* Don't change the value of the usage of the usage of join server in this case */
                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_JOIN_SERVER_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.use_semtech_join_server;
                }

                payload_index += SET_LORAWAN_JOIN_SERVER_LEN;
                break;
            }

            case GET_LORAWAN_JOIN_SERVER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOIN_SERVER_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_JOIN_SERVER_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.use_semtech_join_server;

                payload_index += GET_LORAWAN_JOIN_SERVER_LEN;
                break;
            }

            case SET_LORAWAN_DUTY_CYCLE_CMD:
            {
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.duty_cycle_enable = payload[payload_index];

                    lr1110_modem_activate_duty_cycle(&tracker->lr1110, (lr1110_modem_duty_cycle_t) tracker->tracker_ctx.duty_cycle_enable);

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_DUTY_CYCLE_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_DUTY_CYCLE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.duty_cycle_enable;
                }
                else
                {
                    /* NAck the CMD */
                    tracker->tracker_ctx.duty_cycle_enable = true;
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_DUTY_CYCLE_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_DUTY_CYCLE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.duty_cycle_enable;
                }

                payload_index += SET_LORAWAN_DUTY_CYCLE_LEN;
                break;
            }

            case GET_LORAWAN_DUTY_CYCLE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_DUTY_CYCLE_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_DUTY_CYCLE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.duty_cycle_enable;

                payload_index += GET_LORAWAN_DUTY_CYCLE_LEN;
                break;
            }
            case SET_SCAN_PRIORITY_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 2)
                {
                    tracker->tracker_ctx.scan_priority = (tracker_scan_priority_t) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_SCAN_PRIORITY_CMD;
                    buffer_out[output_buffer_index++] = SET_SCAN_PRIORITY_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.scan_priority;
                }
                else
                {
                    /* Clip the value */
                    tracker->tracker_ctx.scan_priority = TRACKER_GNSS_PRIORITY;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_SCAN_PRIORITY_CMD;
                    buffer_out[output_buffer_index++] = SET_SCAN_PRIORITY_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.scan_priority;
                }

                payload_index += SET_SCAN_PRIORITY_LEN;
                break;
            }

            case GET_SCAN_PRIORITY_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_SCAN_PRIORITY_CMD;
                buffer_out[output_buffer_index++] = GET_SCAN_PRIORITY_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.scan_priority;

                payload_index += GET_SCAN_PRIORITY_LEN;
                break;
            }

            case SET_LORAWAN_ADR_PROFILE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 3)
                {
                    tracker->tracker_ctx.lorawan_adr_profile = (lr1110_modem_adr_profiles_t) payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_adr_profile;
                }
                else
                {
                    /* Clip the value */
                    tracker->tracker_ctx.lorawan_adr_profile = LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_CMD;
                    buffer_out[output_buffer_index++] = SET_LORAWAN_ADR_PROFILE_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_adr_profile;
                }

                payload_index += SET_LORAWAN_ADR_PROFILE_LEN;
                break;
            }

            case GET_LORAWAN_ADR_PROFILE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_LORAWAN_ADR_PROFILE_CMD;
                buffer_out[output_buffer_index++] = GET_LORAWAN_ADR_PROFILE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_adr_profile;

                payload_index += GET_LORAWAN_ADR_PROFILE_LEN;
                break;
            }

            case GET_APP_ACCUMULATED_CHARGE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_ACCUMULATED_CHARGE_CMD;
                buffer_out[output_buffer_index++] = GET_APP_ACCUMULATED_CHARGE_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.accumulated_charge >> 24;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.accumulated_charge >> 16;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.accumulated_charge >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.accumulated_charge;

                payload_index += GET_APP_ACCUMULATED_CHARGE_ANSWER_LEN;
                break;
            }

            case RESET_APP_ACCUMULATED_CHARGE_CMD:
            {
                tracker->tracker_ctx.new_value_to_set   = true;
                tracker->tracker_ctx.accumulated_charge = 0;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = RESET_APP_ACCUMULATED_CHARGE_CMD;
                buffer_out[output_buffer_index++] = RESET_APP_ACCUMULATED_CHARGE_LEN;

                payload_index += RESET_APP_ACCUMULATED_CHARGE_LEN;
                break;
            }

            case GET_APP_RESET_COUNTER_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_RESET_COUNTER_CMD;
                buffer_out[output_buffer_index++] = GET_APP_RESET_COUNTER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.host_reset_cnt >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.host_reset_cnt;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_reset_by_itself_cnt >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_reset_by_itself_cnt;

                payload_index += GET_APP_RESET_COUNTER_ANSWER_LEN;
                break;
            }

            case RESET_APP_RESET_COUNTER_CMD:
            {
                tracker->tracker_ctx.new_value_to_set          = true;
                tracker->tracker_ctx.host_reset_cnt            = 0;
                tracker->tracker_ctx.modem_reset_by_itself_cnt = 0;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = RESET_APP_RESET_COUNTER_CMD;
                buffer_out[output_buffer_index++] = RESET_APP_RESET_COUNTER_LEN;

                payload_index += RESET_APP_RESET_COUNTER_LEN;
                break;
            }

            case GET_APP_SYSTEM_SANITY_CHECK_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_SYSTEM_SANITY_CHECK_CMD;
                buffer_out[output_buffer_index++] = GET_APP_SYSTEM_SANITY_CHECK_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.system_sanity_check;

                payload_index += GET_APP_SYSTEM_SANITY_CHECK_LEN;
                break;
            }

            case SET_APP_INTERNAL_LOG_CMD:
            {
                tracker->tracker_ctx.new_value_to_set = true;
                if(payload[payload_index] <= 1)
                {
                    tracker->tracker_ctx.internal_log_enable = payload[payload_index];

                    /* Ack the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_INTERNAL_LOG_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_INTERNAL_LOG_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.internal_log_enable;
                }
                else
                {
                    /* Clip the value */
                    tracker->tracker_ctx.internal_log_enable = 0;

                    /* NAck the CMD */
                    buffer_out[0] += 1;  // Add the element in the output buffer
                    buffer_out[output_buffer_index++] = SET_APP_INTERNAL_LOG_CMD;
                    buffer_out[output_buffer_index++] = SET_APP_INTERNAL_LOG_LEN;
                    buffer_out[output_buffer_index++] = tracker->tracker_ctx.internal_log_enable;
                }

                payload_index += SET_USE_ACCELEROMETER_LEN;
                break;
            }

            case GET_APP_INTERNAL_LOG_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_CMD;
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.internal_log_enable;

                payload_index += GET_APP_INTERNAL_LOG_LEN;
                break;
            }

            case GET_APP_INTERNAL_LOG_REMANING_SPACE_CMD:
            {
                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_REMANING_SPACE_CMD;
                buffer_out[output_buffer_index++] = GET_APP_INTERNAL_LOG_REMANING_SPACE_ANSWER_LEN;
                buffer_out[output_buffer_index++] = tracker_get_remaining_memory_space(tracker);

                payload_index += GET_APP_INTERNAL_LOG_REMANING_SPACE_LEN;
                break;
            }

            case SET_APP_FLUSH_INTERNAL_LOG_CMD:
            {
                tracker->tracker_ctx.internal_log_flush_request = true;

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = SET_APP_FLUSH_INTERNAL_LOG_CMD;
                buffer_out[output_buffer_index++] = SET_APP_FLUSH_INTERNAL_LOG_LEN;

                payload_index += SET_APP_FLUSH_INTERNAL_LOG_LEN;

                break;
            }

            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint32_t oldest_almanac_date = 0;
                uint32_t newest_almanac_date = 0;
                uint16_t nb_uplink_mobile_static;
                uint16_t nb_uplink_reset;

                /* get the dates form the Modem-E */
                lr1110_modem_get_almanac_dates(&tracker->lr1110, &oldest_almanac_date, &newest_almanac_date);
                lr1110_modem_get_connection_timeout_status(&tracker->lr1110, &nb_uplink_mobile_static, &nb_uplink_reset);

                buffer_out[0] += 1;  // Add the element in the output buffer
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_CMD;
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_ANSWER_LEN;

                /* All settings pattern version */
                buffer_out[output_buffer_index++] = GET_APP_TRACKER_SETTINGS_VERSION;
                /* Modem-E firmware version */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.firmware >> 16;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.firmware >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_version.firmware;
                /* LoRaWAN ADR profile */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.lorawan_adr_profile;
                /* LoRaWAN Join Sever usage */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.use_semtech_join_server;
                /* GNSS enable */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.enabled;
                /* GNSS Assistance position */
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.latitude * 10000000)) >> 24;
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.latitude * 10000000)) >> 16;
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.latitude * 10000000)) >> 8;
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.latitude * 10000000));
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.longitude * 10000000)) >> 24;
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.longitude * 10000000)) >> 16;
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.longitude * 10000000)) >> 8;
                buffer_out[output_buffer_index++] =
                    ((int32_t)(tracker->tracker_ctx.gnss_settings.assistance_position.longitude * 10000000));
                /* GNSS Constellation to use */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.constellation_to_use;
                /* GNSS Search mode */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.gnss_settings.search_mode;
                /* GNSS oldest almanac */
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 24;
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 16;
                buffer_out[output_buffer_index++] = oldest_almanac_date >> 8;
                buffer_out[output_buffer_index++] = oldest_almanac_date;
                /* GNSS newest almanac */
                buffer_out[output_buffer_index++] = newest_almanac_date >> 24;
                buffer_out[output_buffer_index++] = newest_almanac_date >> 16;
                buffer_out[output_buffer_index++] = newest_almanac_date >> 8;
                buffer_out[output_buffer_index++] = newest_almanac_date;
                /* Scan priority */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.scan_priority;
                /* Wi-Fi enable */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.enabled;
                /* Wi-Fi channles */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.channels;
                /* Wi-Fi max results */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.max_results;
                /* Wi-Fi scan mode */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.scan_mode;
                /* Wi-Fi timeout */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.timeout;
                /* Wi-Fi types */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.types;
                /* Wi-Fi nbr_retrials */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.wifi_settings.nbr_retrials;
                /* App accelerometer used */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.accelerometer_used;
                /* App scan interval */
                buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000) >> 8;
                buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_scan_interval / 1000);
                /* App keep alive interval */
                buffer_out[output_buffer_index++] = ((tracker->tracker_ctx.app_keep_alive_frame_interval / 60000) >> 8);
                buffer_out[output_buffer_index++] = (tracker->tracker_ctx.app_keep_alive_frame_interval / 60000);
                /* App internal log enable */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.internal_log_enable;
                /* App internal log remaining memory space */
                buffer_out[output_buffer_index++] = tracker_get_remaining_memory_space(tracker);
                /* App Reset counter */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.host_reset_cnt >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.host_reset_cnt;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_reset_by_itself_cnt >> 8;
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.modem_reset_by_itself_cnt;
                /* Last sv number detected */
                buffer_out[output_buffer_index++] = nb_uplink_reset >> 8;
                buffer_out[output_buffer_index++] = nb_uplink_reset;
                /* Last sv number detected */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_nb_detected_satellites;
                /* Last mac address number detected */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.last_nb_detected_mac_address;
                /* Sanity check bit mask */
                buffer_out[output_buffer_index++] = tracker->tracker_ctx.system_sanity_check;

                payload_index += GET_APP_TRACKER_SETTINGS_LEN;

                break;
            }

            case SET_APP_RESET_CMD:
            {
                reset_board_asked = true;
                payload_index += SET_APP_RESET_LEN;

                break;
            }

            default:
                payload_index += len;

                break;
            }

            nb_elements_index++;
        }
    }

    /* Store the new values here if it's asked */
    if(((tracker->tracker_ctx.new_value_to_set) == true) && (reset_board_asked == true))
    {
        tracker_store_app_ctx(tracker);
    }

    if(reset_board_asked == true)
    {
//FIXME
#if 0
        hal_mcu_reset();
#endif
    }

    if(output_buffer_index > 1)  // if > 1 it means there is something to send
    {
        res_size = output_buffer_index;
    }

    return res_size;
}

void tracker_store_and_reset(struct tracker *tracker, uint8_t host_reset_cnt_to_add)
{
    tracker->tracker_ctx.host_reset_cnt += host_reset_cnt_to_add;

    tracker_store_app_ctx(tracker);

    /* System reset */
//FIXME
#if 0
    hal_mcu_reset();
#endif
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void tracker_print_device_settings(struct tracker *tracker)
{
//FIXME
#if 0
    struct tm epoch_time;
#endif
    uint8_t   nb_trans;

    HAL_DBG_TRACE_MSG("#Device Settings :\r\n\r\n");

    HAL_DBG_TRACE_MSG("#LoRaWAN Settings :\r\n");
    HAL_DBG_TRACE_PRINTF("#\tLR1110 MODEM-E VERSION : LORAWAN : %#04X | FIRMWARE : %#02X | BOOTLOADER : %#02X\r\n",
                          tracker->tracker_ctx.modem_version.lorawan, tracker->tracker_ctx.modem_version.firmware,
                          tracker->tracker_ctx.modem_version.bootloader);

    /* Device EUI */
    HAL_DBG_TRACE_PRINTF("#\tDevice Eui : %02X", tracker->tracker_ctx.dev_eui[0]);
    for(int i = 1; i < 8; i++)
    {
        HAL_DBG_TRACE_PRINTF("-%02X", tracker->tracker_ctx.dev_eui[i]);
    }
    HAL_DBG_TRACE_MSG("\r\n");

    /* Join EUI */
    HAL_DBG_TRACE_PRINTF("#\tAppEui      : %02X", tracker->tracker_ctx.join_eui[0]);
    for(int i = 1; i < 8; i++)
    {
        HAL_DBG_TRACE_PRINTF("-%02X", tracker->tracker_ctx.join_eui[i]);
    }
    HAL_DBG_TRACE_PRINTF("\r\n");

    /* AppKey / Semtech JS */
    if(tracker->tracker_ctx.use_semtech_join_server)
    {
        HAL_DBG_TRACE_MSG("#\tAppKey      : Semtech join server used\r\n");
    }
    else
    {
        HAL_DBG_TRACE_PRINTF("#\tAppKey      : %02X", tracker->tracker_ctx.app_key[0]);
        for(int i = 1; i < 16; i++)
        {
            HAL_DBG_TRACE_PRINTF("-%02X", tracker->tracker_ctx.app_key[i]);
        }
        HAL_DBG_TRACE_PRINTF("\r\n");
    }

    /* LoRaWAN settings */
    HAL_DBG_TRACE_PRINTF("#\tADR profile : %d\r\n", tracker->tracker_ctx.lorawan_adr_profile);

    lr1110_modem_get_nb_trans(&tracker->lr1110, &nb_trans);

    HAL_DBG_TRACE_PRINTF("#\tlorawan nb trans : %d\r\n", nb_trans);
    HAL_DBG_TRACE_PRINTF("#\tlorawan_region : %d\r\n", tracker->tracker_ctx.lorawan_region);

    /* GNSS settings */
    HAL_DBG_TRACE_MSG("#GNSS Settings:\r\n");
    HAL_DBG_TRACE_PRINTF("#\tenabled : %d\r\n", tracker->tracker_ctx.gnss_settings.enabled);
    HAL_DBG_TRACE_PRINTF("#\tassistance position latitude : %f\r\n",
                          tracker->tracker_ctx.gnss_settings.assistance_position.latitude);
    HAL_DBG_TRACE_PRINTF("#\tassistance position longitude : %f\r\n",
                          tracker->tracker_ctx.gnss_settings.assistance_position.longitude);
    HAL_DBG_TRACE_PRINTF("#\tconstellation_to_use : %d\r\n", tracker->tracker_ctx.gnss_settings.constellation_to_use);
    HAL_DBG_TRACE_PRINTF("#\tsearch_mode : %d\r\n", tracker->tracker_ctx.gnss_settings.search_mode);
    HAL_DBG_TRACE_PRINTF("#\tinput_parameters : %d\r\n", tracker->tracker_ctx.gnss_settings.input_parameters);
    HAL_DBG_TRACE_PRINTF("#\tnb_sat : %d\r\n", tracker->tracker_ctx.gnss_settings.nb_sat);
//FIXME
#if 0
    memcpy(&epoch_time, localtime(&tracker->tracker_ctx.last_almanac_update), sizeof(struct tm));
    HAL_DBG_TRACE_PRINTF("#\tlast_almanac_update : [%d-%d-%d %d:%d:%d.000]\r\n", epoch_time.tm_year + 1900,
                          epoch_time.tm_mon + 1, epoch_time.tm_mday, epoch_time.tm_hour, epoch_time.tm_min,
                          epoch_time.tm_sec);
#endif

    /* Wi-Fi settings */
    HAL_DBG_TRACE_MSG("#Wi-Fi Settings:\r\n");
    HAL_DBG_TRACE_PRINTF("#\tenabled : %d\r\n", tracker->tracker_ctx.wifi_settings.enabled);
    HAL_DBG_TRACE_PRINTF("#\tchannels : %02X\r\n", tracker->tracker_ctx.wifi_settings.channels);
    HAL_DBG_TRACE_PRINTF("#\tmax_results : %d\r\n", tracker->tracker_ctx.wifi_settings.max_results);
    HAL_DBG_TRACE_PRINTF("#\tscan_mode : %d\r\n", tracker->tracker_ctx.wifi_settings.scan_mode);
    HAL_DBG_TRACE_PRINTF("#\ttimeout : %d ms\r\n", tracker->tracker_ctx.wifi_settings.timeout);
    HAL_DBG_TRACE_PRINTF("#\ttypes : %d\r\n", tracker->tracker_ctx.wifi_settings.types);
    HAL_DBG_TRACE_PRINTF("#\tnbr_retrials : %d\r\n", tracker->tracker_ctx.wifi_settings.nbr_retrials);

    /* Application settings */
    HAL_DBG_TRACE_MSG("#Application settings:\r\n");
    HAL_DBG_TRACE_PRINTF("#\taccelerometer_used : %d\r\n", tracker->tracker_ctx.accelerometer_used);
    HAL_DBG_TRACE_PRINTF("#\tapp_scan_interval : %d s\r\n", tracker->tracker_ctx.app_scan_interval / 1000);
    HAL_DBG_TRACE_PRINTF("#\tapp_keep_alive_frame_interval : %d min\r\n",
                          tracker->tracker_ctx.app_keep_alive_frame_interval / 60000);
    HAL_DBG_TRACE_PRINTF("#\tscan_priority : %d\r\n", tracker->tracker_ctx.scan_priority);
    HAL_DBG_TRACE_PRINTF("#\tairplane_mode : %d\r\n", tracker->tracker_ctx.airplane_mode);
    HAL_DBG_TRACE_PRINTF("#\tinternal_log_enable : %d\r\n", tracker->tracker_ctx.internal_log_enable);
    HAL_DBG_TRACE_PRINTF("#\ttracker_fw_version : %d.%d.%d\r\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION,
                          TRACKER_SUB_MINOR_APP_VERSION);
    HAL_DBG_TRACE_PRINTF("#\thost_reset_cnt : %d\r\n", tracker->tracker_ctx.host_reset_cnt);
    HAL_DBG_TRACE_PRINTF("#\tmodem_reset_by_itself_cnt : %d\r\n", tracker->tracker_ctx.modem_reset_by_itself_cnt);
}

uint32_t get_uint32_from_array_at_index_and_inc(const uint8_t* array, uint16_t* index)
{
    const uint16_t local_index = *index;
    const uint32_t value = array[local_index] + (array[local_index + 1] << 8) + (array[local_index + 2] << 16) +
                           (array[local_index + 3] << 24);
    *index = local_index + 4;
    return value;
}

/* --- EOF ------------------------------------------------------------------ */
