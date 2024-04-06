/*!
 * @file      wifi_scan.h
 *
 * @brief     Wi-Fi scan definition
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

#ifndef WIFI_SCAN_H
#define WIFI_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "modem/lr1110_modem_wifi.h"
#include "modem/lr1110_modem_system.h"
#include <stdint.h>

struct tracker;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define WIFI_NBR_RETRIALS_DEFAULT 5
#define WIFI_MAX_RESULTS_DEFAULT 5
#define WIFI_TIMEOUT_IN_MS_DEFAULT 90
#define WIFI_MAX_BASIC_RESULTS_PER_SCAN 32
#define WIFI_MAX_EXTENDED_RESULTS_PER_SCAN 12

#define WIFI_SCAN_SUCCESS 1
#define WIFI_SCAN_FAIL 0

#define WIFI_BUFFER_MAX_SIZE 948  // 12 results * LR1110_WIFI_EXTENDED_FULL_RESULT_SIZE

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Wi-Fi scan result type
 */
typedef uint8_t wifi_scan_result_t;

/*!
 * @brief Wi-Fi state used in the state machine
 */
typedef enum
{
    WIFI_INIT,
    WIFI_SCAN,
    WIFI_WAIT_FOR_SCAN,
    WIFI_GET_RESULTS,
} wifi_state_t;

/*!
 * @brief Wi-Fi single scan result structure
 */
typedef struct
{
    lr1110_modem_wifi_mac_address_t        mac_address;
    lr1110_modem_wifi_channel_t            channel;
    lr1110_modem_wifi_signal_type_result_t type;
    int8_t                                 rssi;
} wifi_scan_single_result_t;

/*!
 * @brief Wi-Fi scan all result structure
 */
typedef struct
{
    uint8_t                                nbr_results;
    wifi_scan_single_result_t              results[WIFI_MAX_BASIC_RESULTS_PER_SCAN];
    lr1110_modem_wifi_cumulative_timings_t timings;
} wifi_scan_selected_result_t;

/*!
 * @brief Wi-Fi scan all result structure
 */
typedef struct
{
    lr1110_modem_wifi_mode_t                          scan_mode;
    lr1110_modem_wifi_result_format_t                 result_format;
    uint8_t                                           nbr_results;
    lr1110_modem_wifi_basic_mac_type_channel_result_t basic_mac_type_channel_results[WIFI_MAX_BASIC_RESULTS_PER_SCAN];
    lr1110_modem_wifi_basic_complete_result_t         basic_complete_results[WIFI_MAX_BASIC_RESULTS_PER_SCAN];
    lr1110_modem_wifi_extended_full_result_t          extended_full_results[WIFI_MAX_EXTENDED_RESULTS_PER_SCAN];
    lr1110_modem_wifi_cumulative_timings_t            timings;
    uint32_t                                          global_consumption_uas;
    bool                                              error;
} wifi_scan_all_results_t;

/*!
 * @brief Wi-Fi settings stucture parameters
 */
typedef struct
{
    bool                                 enabled;
    lr1110_modem_wifi_channel_mask_t     channels;
    lr1110_modem_wifi_signal_type_scan_t types;
    lr1110_modem_wifi_mode_t             scan_mode;
    uint8_t                              nbr_retrials;
    uint8_t                              max_results;
    uint32_t                             timeout;
    lr1110_modem_wifi_result_format_t    result_format;
    lr1110_modem_system_reg_mode_t       reg_mode;
} wifi_settings_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Function executed on Wifi Scan done event
 *
 * @param [in] buffer Buffer containing the raw data
 * @param [in] size Size of the raw data buffer
 */
void lr1110_modem_wifi_scan_done(struct tracker *tracker, uint8_t* buffer, uint16_t size);

/*!
 * @brief Display the last Wi-Fi scan results
 *
 * @param [in] capture_result Structure containing the capture results, \ref wifi_scan_all_results_t
 */
void lr1110_modem_display_wifi_scan_results(const wifi_scan_all_results_t* capture_result);

/*!
 * @brief Select and copy results from Wi-Fi scan to another stucture containing only the necessary result for user
 *
 * @param [in] capture_result Structure containing the capture results, \ref wifi_scan_all_results_t
 * @param [out] selected_results Structure containing the only necessary results, \ref
 * wifi_scan_selected_result_t
 */
void lr1110_modem_wifi_scan_select_results(const wifi_scan_all_results_t* capture_result,
                                            wifi_scan_selected_result_t*   selected_results);

/*!
 * @brief execute the wifi scan state machine
 *
 * @param [in] context Radio abstraction
 * @param [in] wifi_settings Wi-Fi settings structure to apply \ref wifi_settings_t
 * @param [out] capture_result Structure containing the capture results, \ref wifi_scan_all_results_t
 *
 * @returns Wi-Fi scan result operation
 */
wifi_scan_result_t wifi_execute_scan(struct tracker *tracker, const void* context, const wifi_settings_t* wifi_settings,
                                      wifi_scan_all_results_t* capture_result);

#ifdef __cplusplus
}
#endif

#endif  // WIFI_SCAN_H

/* --- EOF ------------------------------------------------------------------ */
