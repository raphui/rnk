/*!
 * @file      wifi_scan.c
 *
 * @brief     Wifi scan implementation.
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

#include <time.h>
#include <string.h>

#include "tracker.h"
#include "wifi_scan.h"
#include "modem/lr1110_modem_board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define WIFI_SCAN_ABORT_ON_TIMEOUT (true)
#define WIFI_CONSUMPTION_DCDC_CORRELATION_MA (12)
#define WIFI_CONSUMPTION_DCDC_DEMODULATION_MA (4)
#define WIFI_CONSUMPTION_LDO_CORRELATION_MA (24)
#define WIFI_CONSUMPTION_LDO_DEMODULATION_MA (8)

/*!
 * @brief Wi-Fi scan state machine timeout
 */
#define WIFI_SCAN_TIMEOUT (10000 * 1000)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Wi-Fi state \ref wifi_state_t
 */
static wifi_state_t wifi_state;

/*!
 * @brief Buffer scan result buffer
 */
static uint8_t wifi_scan_result_buffer[WIFI_BUFFER_MAX_SIZE];

/*!
 * @brief Buffer scan result buffer size
 */
static uint16_t wifi_scan_result_buffer_size;

/*!
 * @brief Wi-Fi scan timeout flag
 */
static bool wifi_scan_timeout = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief init the wifi scan state machine
 *
 * @param [in] context Radio abstraction
 * @param [in] settings structure containing the Wi-Fi parameter \see wifi_settings_t
 */
static void wifi_init(const void* context, const wifi_settings_t* settings);

/*!
 * @brief Compute consumption based on cumulative timings
 *
 * @param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 * @param [in] timing \ref lr1110_modem_wifi_cumulative_timings_t
 *
 * @returns Consumption in micro ampere second (uas)
 */
static uint32_t wifi_compute_consumption(const lr1110_modem_system_reg_mode_t         reg_mode,
                                          const lr1110_modem_wifi_cumulative_timings_t timing);

/*!
 * @brief copy complete mac results into wi-fi result structure
 *
 * @param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 * @param [out] results structure containing the results \ref wifi_scan_all_results_t
 * @param [in] scan_result to copy \ref lr1110_modem_wifi_basic_complete_result_t
 * @param [in] nbr_results nb result to copy
 * @param [in] timing scan timing
 */
static void wifi_add_complete_mac_to_results(lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_results_t* results,
                                              const lr1110_modem_wifi_basic_complete_result_t* scan_result,
                                              uint8_t nbr_results, lr1110_modem_wifi_cumulative_timings_t timing);

/*!
 * @brief copy basic mac results into wi-fi result structure
 *
 * @param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 * @param [out] results structure containing the results \ref wifi_scan_all_results_t
 * @param [in] scan_result to copy \ref lr1110_modem_wifi_basic_mac_type_channel_result_t
 * @param [in] nbr_results nb result to copy
 * @param [in] timing scan timing
 */
static void wifi_add_basic_mac_to_results(lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_results_t* results,
                                           const lr1110_modem_wifi_basic_mac_type_channel_result_t* scan_result,
                                           uint8_t nbr_results, lr1110_modem_wifi_cumulative_timings_t timing);

/*!
 * @brief copy extendent results into wi-fi result structure
 *
 * @param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 * @param [out] results structure containing the results \ref wifi_scan_all_results_t
 * @param [in] scan_result to copy \ref lr1110_modem_wifi_basic_mac_type_channel_result_t
 * @param [in] nbr_results nb result to copy
 * @param [in] timing scan timing
 */
static void wifi_add_extended_mac_to_results(lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_results_t* results,
                                              const lr1110_modem_wifi_extended_full_result_t* scan_result,
                                              uint8_t nbr_results, lr1110_modem_wifi_cumulative_timings_t timing);

/*!
 * @brief Function executed on Wi-Fi scan timeout event
 */
static void on_wifi_scan_timeout_event(void* context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_wifi_scan_done(struct tracker *tracker, uint8_t* buffer, uint16_t size)
{
    memcpy(wifi_scan_result_buffer, buffer, size);
    wifi_scan_result_buffer_size = size;

    wifi_state = WIFI_GET_RESULTS;
}

void wifi_init(const void* context, const wifi_settings_t* settings)
{
    wifi_state                   = WIFI_INIT;
    wifi_scan_result_buffer_size = 0;
}

wifi_scan_result_t wifi_execute_scan(struct tracker *tracker, const void* context, const wifi_settings_t* wifi_settings,
                                      wifi_scan_all_results_t* capture_result)
{
    bool                         wifi_scan_done      = false;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    wifi_scan_result_t           scan_result         = WIFI_SCAN_SUCCESS;
    uint8_t                      wifi_max_results    = 0;

    /* Reset result parameters */
    wifi_scan_timeout           = false;
    capture_result->nbr_results = 0;

    /* Init the Wi-Fi parameters */
    wifi_init(context, wifi_settings);

    /* if format is LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE with the acquisition
     * LR1110_MODEM_WIFI_SCAN_MODE_FULL_BEACON and  max result available is 12 otherwise it's 32. */
    if((wifi_settings->result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE) &&
        (wifi_settings->scan_mode == LR1110_MODEM_WIFI_SCAN_MODE_FULL_BEACON) && (wifi_settings->max_results > 12))
    {
        HAL_DBG_TRACE_WARNING("max_results too high, limit to 12 \r\n");
        wifi_max_results = 12;
    }
    else if(wifi_settings->max_results > 32)
    {
        HAL_DBG_TRACE_WARNING("max_results too high, limit to 32 \r\n");
        wifi_max_results = 32;
    }
    else
    {
        wifi_max_results = wifi_settings->max_results;
    }

    time_oneshot(&((lr1110_t *)context)->wifi_scan_timeout_timer, WIFI_SCAN_TIMEOUT, on_wifi_scan_timeout_event, NULL);

    while((wifi_scan_done != true) && (wifi_scan_timeout != true))
    {
        /* Process Event */
        if(((lr1110_t*) context)->radio_event->irq.handler != NULL)
        {
            lr1110_modem_event_process(tracker, context);
        }

        switch(wifi_state)
        {
        case WIFI_INIT:
        {
            wifi_state          = WIFI_SCAN;
            modem_response_code = lr1110_modem_wifi_reset_cumulative_timing(context);
            break;
        }

        case WIFI_SCAN:
        {
//FIXME
#if 0
            /* Turn on the scan led during the scan */
            leds_on(LED_SCAN_MASK);
#endif

            modem_response_code = lr1110_modem_wifi_passive_scan(
                context, wifi_settings->types, wifi_settings->channels, wifi_settings->scan_mode, wifi_max_results,
                wifi_settings->nbr_retrials, wifi_settings->timeout, WIFI_SCAN_ABORT_ON_TIMEOUT,
                wifi_settings->result_format);

            /* If response code different than RESPONSE_CODE_OK leave */
            if(modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK)
            {
                wifi_scan_done = true;
                scan_result    = WIFI_SCAN_FAIL;
            }

            wifi_state = WIFI_WAIT_FOR_SCAN;
            break;
        }

        case WIFI_GET_RESULTS:
        {
            lr1110_modem_wifi_cumulative_timings_t wifi_results_timings = { 0 };
            uint8_t                                nb_results;

            lr1110_modem_wifi_read_cumulative_timing(context, &wifi_results_timings);

            if(wifi_settings->scan_mode == LR1110_MODEM_WIFI_SCAN_MODE_FULL_BEACON)
            {
                lr1110_modem_wifi_extended_full_result_t wifi_results_mac_addr[WIFI_MAX_EXTENDED_RESULTS_PER_SCAN] = {
                    0
                };

                /* Parse result buffer */
                lr1110_modem_wifi_read_extended_full_results(wifi_scan_result_buffer, wifi_scan_result_buffer_size,
                                                              wifi_results_mac_addr, &nb_results);

                /* Copy results in the result structure */
                wifi_add_extended_mac_to_results(LR1110_MODEM_SYSTEM_REG_MODE_DCDC, capture_result,
                                                  wifi_results_mac_addr, nb_results, wifi_results_timings);
            }
            else  // LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PKT and LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PKT
            {
                if(wifi_settings->result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL)
                {
                    lr1110_modem_wifi_basic_mac_type_channel_result_t
                        wifi_results_mac_addr[WIFI_MAX_BASIC_RESULTS_PER_SCAN] = { 0 };

                    /* Parse result buffer */
                    lr1110_modem_wifi_read_basic_mac_type_channel_results(
                        wifi_scan_result_buffer, wifi_scan_result_buffer_size, wifi_results_mac_addr, &nb_results);

                    /* Copy results in the result structure */
                    wifi_add_basic_mac_to_results(LR1110_MODEM_SYSTEM_REG_MODE_DCDC, capture_result,
                                                   wifi_results_mac_addr, nb_results, wifi_results_timings);
                }
                else if(wifi_settings->result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE)
                {
                    lr1110_modem_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_BASIC_RESULTS_PER_SCAN] = {
                        0
                    };

                    /* Parse result buffer */
                    lr1110_modem_wifi_read_basic_complete_results(
                        wifi_scan_result_buffer, wifi_scan_result_buffer_size, wifi_results_mac_addr, &nb_results);

                    /* Copy results in the result structure */
                    wifi_add_complete_mac_to_results(LR1110_MODEM_SYSTEM_REG_MODE_DCDC, capture_result,
                                                      wifi_results_mac_addr, nb_results, wifi_results_timings);
                }
            }

            /* Store both following settings in the result */
            capture_result->result_format = wifi_settings->result_format;
            capture_result->scan_mode     = wifi_settings->scan_mode;

            wifi_scan_done = true;
            wifi_state     = WIFI_INIT;

            break;
        }

        case WIFI_WAIT_FOR_SCAN:
        {
            /* go in low power */
            if(lr1110_modem_board_read_event_line(context) == false)
            {
//FIXME
#if 0
                hal_mcu_low_power_handler();
#endif
            }
            break;
        }
        }
    }

//FIXME
#if 0
    /* Turn off the scan led at the end of the scan */
    leds_off(LED_SCAN_MASK);

#endif

    time_oneshot_cancel(&((lr1110_t *)context)->wifi_scan_timeout_timer);

    if(wifi_scan_timeout == true)
    {
        scan_result = WIFI_SCAN_FAIL;
    }

    return scan_result;
}

void lr1110_modem_wifi_scan_select_results(const wifi_scan_all_results_t* capture_result,
                                            wifi_scan_selected_result_t*   selected_results)
{
    selected_results->nbr_results = capture_result->nbr_results;

    if(capture_result->nbr_results != 0)
    {
        if(capture_result->scan_mode == LR1110_MODEM_WIFI_SCAN_MODE_FULL_BEACON)
        {
            /* Copy results from extended full results to selected results */
            for(uint8_t i = 0; i < capture_result->nbr_results; i++)
            {
                memcpy(selected_results->results[i].mac_address,
                        capture_result->extended_full_results[i].mac_address_2, LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
                selected_results->results[i].channel = capture_result->extended_full_results[i].current_channel;
                selected_results->results[i].rssi    = capture_result->extended_full_results[i].rssi;
                selected_results->results[i].type =
                    (lr1110_modem_wifi_signal_type_result_t) capture_result->extended_full_results[i]
                        .data_rate_info_byte;
            }
        }
        else
        {
            if(capture_result->result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE)
            {
                /* Copy results from basic complet results to selected results */
                for(uint8_t i = 0; i < capture_result->nbr_results; i++)
                {
                    memcpy(selected_results->results[i].mac_address,
                            capture_result->basic_complete_results[i].mac_address,
                            LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
                    selected_results->results[i].channel =
                        (lr1110_modem_wifi_channel_t) capture_result->basic_complete_results[i].channel_info_byte;
                    selected_results->results[i].rssi = capture_result->basic_complete_results[i].rssi;
                    selected_results->results[i].type =
                        (lr1110_modem_wifi_signal_type_result_t) capture_result->basic_complete_results[i]
                            .data_rate_info_byte;
                }
            }
            else  // LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL
            {
                /* Copy results from basic mac type channel results to selected results */
                for(uint8_t i = 0; i < capture_result->nbr_results; i++)
                {
                    memcpy(selected_results->results[i].mac_address,
                            capture_result->basic_mac_type_channel_results[i].mac_address,
                            LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
                    selected_results->results[i].channel =
                        (lr1110_modem_wifi_channel_t) capture_result->basic_mac_type_channel_results[i]
                            .channel_info_byte;
                    selected_results->results[i].rssi = capture_result->basic_mac_type_channel_results[i].rssi;
                    selected_results->results[i].type =
                        (lr1110_modem_wifi_signal_type_result_t) capture_result->basic_mac_type_channel_results[i]
                            .data_rate_info_byte;
                }
            }
        }
        /* Copy timings */
        selected_results->timings = capture_result->timings;
    }
}

void lr1110_modem_display_wifi_scan_results(const wifi_scan_all_results_t* capture_result)
{
    if(capture_result->nbr_results != 0)
    {
        HAL_DBG_TRACE_PRINTF("Nb access points scanned : %d \r\n", capture_result->nbr_results);

        if(capture_result->scan_mode == LR1110_MODEM_WIFI_SCAN_MODE_FULL_BEACON)
        {
            HAL_DBG_TRACE_PRINTF("Wi-Fi Acquisition mode : FULL_BEACON\r\n");

            for(uint8_t i = 0; i < capture_result->nbr_results; i++)
            {
                HAL_DBG_TRACE_MSG("MAC addr 1: ");
                for(uint8_t j = 0; j < LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH; j++)
                {
                    HAL_DBG_TRACE_PRINTF("%#04X ", capture_result->extended_full_results[i].mac_address_1[j]);
                }
                HAL_DBG_TRACE_MSG("\r\n");
                HAL_DBG_TRACE_MSG("MAC addr 2: ");
                for(uint8_t j = 0; j < LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH; j++)
                {
                    HAL_DBG_TRACE_PRINTF("%#04X ", capture_result->extended_full_results[i].mac_address_2[j]);
                }
                HAL_DBG_TRACE_MSG("\r\n");
                HAL_DBG_TRACE_MSG("MAC addr 3: ");
                for(uint8_t j = 0; j < LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH; j++)
                {
                    HAL_DBG_TRACE_PRINTF("%#04X ", capture_result->extended_full_results[i].mac_address_3[j]);
                }
                HAL_DBG_TRACE_MSG("\r\n");
                HAL_DBG_TRACE_MSG("SSID : ");
                for(uint8_t j = 0; j < LR1110_MODEM_WIFI_RESULT_SSID_LENGTH; j++)
                {
                    HAL_DBG_TRACE_PRINTF("%c", capture_result->extended_full_results[i].ssid_bytes[j]);
                }
                HAL_DBG_TRACE_MSG("\r\n");

                HAL_DBG_TRACE_PRINTF("\t\t\t\t\t - RSSI : %d dBm", capture_result->extended_full_results[i].rssi);
                HAL_DBG_TRACE_PRINTF(" - Channel : %d", capture_result->extended_full_results[i].channel_info_byte);
                HAL_DBG_TRACE_PRINTF(" - Signal type : %d",
                                      capture_result->extended_full_results[i].data_rate_info_byte);
                HAL_DBG_TRACE_PRINTF(" - Phi Offset : %d\r\n", capture_result->extended_full_results[i].phi_offset);
                HAL_DBG_TRACE_PRINTF("\t\t\t\t\t - Beacon period : %d tu",
                                      capture_result->extended_full_results[i].beacon_period_tu);
                HAL_DBG_TRACE_PRINTF(" - timestamp : %lld us", capture_result->extended_full_results[i].timestamp_us);
                HAL_DBG_TRACE_PRINTF(" - rate : %d MB/s\r\n", capture_result->extended_full_results[i].rate / 10);
                HAL_DBG_TRACE_PRINTF("\t\t\t\t\t - service : %d", capture_result->extended_full_results[i].service);
                HAL_DBG_TRACE_PRINTF(" - length : %d", capture_result->extended_full_results[i].length);
                HAL_DBG_TRACE_PRINTF(" - frame_control : %d", capture_result->extended_full_results[i].frame_control);
                HAL_DBG_TRACE_PRINTF(" - seq_control : %d\r\n", capture_result->extended_full_results[i].seq_control);
                HAL_DBG_TRACE_PRINTF("\t\t\t\t\t - current_channel : %d",
                                      capture_result->extended_full_results[i].current_channel);
                HAL_DBG_TRACE_PRINTF(" - country_code : %c%c ",
                                      (capture_result->extended_full_results[i].country_code >> 8),
                                      capture_result->extended_full_results[i].country_code);
                HAL_DBG_TRACE_PRINTF(" - io_regulation : %d", capture_result->extended_full_results[i].io_regulation);
                HAL_DBG_TRACE_PRINTF(" - fcs_check_byte : %d\r\n",
                                      capture_result->extended_full_results[i].fcs_check_byte);
            }
        }
        else
        {
            if(capture_result->scan_mode == LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PKT)
            {
                HAL_DBG_TRACE_PRINTF("Wi-Fi Acquisition mode : BEACON_AND_PKT\r\n");
            }
            else
            {
                HAL_DBG_TRACE_PRINTF("Wi-Fi Acquisition mode : BEACON\r\n");
            }

            if(capture_result->result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE)
            {
                HAL_DBG_TRACE_PRINTF("Wi-Fi result format : BASIC_COMPLETE\r\n");

                for(uint8_t i = 0; i < capture_result->nbr_results; i++)
                {
                    HAL_DBG_TRACE_MSG("MAC addr : ");
                    for(uint8_t j = 0; j < LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH; j++)
                    {
                        HAL_DBG_TRACE_PRINTF("%#04X ", capture_result->basic_complete_results[i].mac_address[j]);
                    }
                    HAL_DBG_TRACE_PRINTF(" - RSSI : %d dBm", capture_result->basic_complete_results[i].rssi);
                    HAL_DBG_TRACE_PRINTF(" - Channel : %d",
                                          capture_result->basic_complete_results[i].channel_info_byte);
                    HAL_DBG_TRACE_PRINTF(" - Type : %d",
                                          capture_result->basic_complete_results[i].frame_type_info_byte);
                    HAL_DBG_TRACE_PRINTF(" - Signal Type : %d\r\n",
                                          capture_result->basic_complete_results[i].data_rate_info_byte);
                    HAL_DBG_TRACE_PRINTF("\t\t\t\t\t  - Phi Offset : %d",
                                          capture_result->basic_complete_results[i].phi_offset);
                    HAL_DBG_TRACE_PRINTF(" - Beacon period : %d tu",
                                          capture_result->basic_complete_results[i].beacon_period_tu);
                    HAL_DBG_TRACE_PRINTF(" - timestamp : %lld us\r\n",
                                          capture_result->basic_complete_results[i].timestamp_us);
                }
            }
            else  // LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL
            {
                HAL_DBG_TRACE_PRINTF("Wi-Fi result format : BASIC_MAC_TYPE_CHANNEL\r\n");

                for(uint8_t i = 0; i < capture_result->nbr_results; i++)
                {
                    HAL_DBG_TRACE_MSG("MAC addr : ");
                    for(uint8_t j = 0; j < LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH; j++)
                    {
                        HAL_DBG_TRACE_PRINTF("%#04X ",
                                              capture_result->basic_mac_type_channel_results[i].mac_address[j]);
                    }
                    HAL_DBG_TRACE_PRINTF("- RSSI : %d dBm", capture_result->basic_mac_type_channel_results[i].rssi);
                    HAL_DBG_TRACE_PRINTF(" - Channel : %d",
                                          capture_result->basic_mac_type_channel_results[i].channel_info_byte);
                    HAL_DBG_TRACE_PRINTF(" - Signal Type : %d\r\n",
                                          capture_result->basic_mac_type_channel_results[i].data_rate_info_byte);
                }
            }
        }
        HAL_DBG_TRACE_PRINTF("\r\nScan Timing : %d ms\r\n",
                              (capture_result->timings.demodulation_us + capture_result->timings.rx_capture_us +
                                capture_result->timings.rx_correlation_us + capture_result->timings.rx_detection_us) /
                                  1000);
        HAL_DBG_TRACE_PRINTF("Scan Consumption : %d uas\r\n", capture_result->global_consumption_uas);
        HAL_DBG_TRACE_MSG("\r\n");
    }
    else
    {
        HAL_DBG_TRACE_MSG("No MAC address found \r\n\r\n");
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_wifi_scan_timeout_event(void* context) { wifi_scan_timeout = true; }

static uint32_t wifi_compute_consumption(const lr1110_modem_system_reg_mode_t         reg_mode,
                                          const lr1110_modem_wifi_cumulative_timings_t timing)
{
    uint32_t consumption_uas = 0;

    switch(reg_mode)
    {
    case LR1110_MODEM_SYSTEM_REG_MODE_LDO:
        consumption_uas = ((timing.rx_correlation_us + timing.rx_capture_us) * WIFI_CONSUMPTION_LDO_CORRELATION_MA +
                            timing.demodulation_us * WIFI_CONSUMPTION_LDO_DEMODULATION_MA) /
                          1000;
        break;
    case LR1110_MODEM_SYSTEM_REG_MODE_DCDC:
        consumption_uas = ((timing.rx_correlation_us + timing.rx_capture_us) * WIFI_CONSUMPTION_DCDC_CORRELATION_MA +
                            timing.demodulation_us * WIFI_CONSUMPTION_DCDC_DEMODULATION_MA) /
                          1000;
        break;
    }
    return consumption_uas;
}

static void wifi_add_basic_mac_to_results(lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_results_t* results,
                                           const lr1110_modem_wifi_basic_mac_type_channel_result_t* scan_result,
                                           uint8_t nbr_results, lr1110_modem_wifi_cumulative_timings_t timing)
{
    for(uint8_t index = 0; index < nbr_results; index++)
    {
        results->basic_mac_type_channel_results[index].channel_info_byte =
            lr1110_modem_extract_channel_from_info_byte(scan_result[index].channel_info_byte);
        results->basic_mac_type_channel_results[index].data_rate_info_byte =
            lr1110_modem_extract_signal_type_from_data_rate_info(scan_result[index].data_rate_info_byte);
        memcpy(results->basic_mac_type_channel_results[index].mac_address, scan_result[index].mac_address,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
        results->basic_mac_type_channel_results[index].rssi = scan_result[index].rssi;
    }

    results->timings                = timing;
    results->global_consumption_uas = wifi_compute_consumption(reg_mode, timing);
    results->nbr_results            = nbr_results;
}

static void wifi_add_complete_mac_to_results(lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_results_t* results,
                                              const lr1110_modem_wifi_basic_complete_result_t* scan_result,
                                              uint8_t nbr_results, lr1110_modem_wifi_cumulative_timings_t timing)
{
    for(uint8_t index = 0; index < nbr_results; index++)
    {
        results->basic_complete_results[index].channel_info_byte =
            lr1110_modem_extract_channel_from_info_byte(scan_result[index].channel_info_byte);
        results->basic_complete_results[index].data_rate_info_byte =
            lr1110_modem_extract_signal_type_from_data_rate_info(scan_result[index].data_rate_info_byte);
        results->basic_complete_results[index].frame_type_info_byte = scan_result[index].frame_type_info_byte;
        memcpy(results->basic_complete_results[index].mac_address, scan_result[index].mac_address,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
        results->basic_complete_results[index].rssi             = scan_result[index].rssi;
        results->basic_complete_results[index].phi_offset       = scan_result[index].phi_offset;
        results->basic_complete_results[index].timestamp_us     = scan_result[index].timestamp_us;
        results->basic_complete_results[index].beacon_period_tu = scan_result[index].beacon_period_tu;
    }

    results->timings                = timing;
    results->global_consumption_uas = wifi_compute_consumption(reg_mode, timing);
    results->nbr_results            = nbr_results;
}

static void wifi_add_extended_mac_to_results(lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_results_t* results,
                                              const lr1110_modem_wifi_extended_full_result_t* scan_result,
                                              uint8_t nbr_results, lr1110_modem_wifi_cumulative_timings_t timing)
{
    for(uint8_t index = 0; index < nbr_results; index++)
    {
        results->extended_full_results[index].channel_info_byte =
            lr1110_modem_extract_channel_from_info_byte(scan_result[index].channel_info_byte);
        results->extended_full_results[index].data_rate_info_byte =
            lr1110_modem_extract_signal_type_from_data_rate_info(scan_result[index].data_rate_info_byte);
        memcpy(results->extended_full_results[index].mac_address_1, scan_result[index].mac_address_1,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
        memcpy(results->extended_full_results[index].mac_address_2, scan_result[index].mac_address_2,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
        memcpy(results->extended_full_results[index].mac_address_3, scan_result[index].mac_address_3,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH);
        memcpy(results->extended_full_results[index].ssid_bytes, scan_result[index].ssid_bytes,
                LR1110_MODEM_WIFI_RESULT_SSID_LENGTH);
        results->extended_full_results[index].rssi             = scan_result[index].rssi;
        results->extended_full_results[index].rate             = scan_result[index].rate;
        results->extended_full_results[index].service          = scan_result[index].service;
        results->extended_full_results[index].length           = scan_result[index].length;
        results->extended_full_results[index].frame_control    = scan_result[index].frame_control;
        results->extended_full_results[index].rssi             = scan_result[index].rssi;
        results->extended_full_results[index].seq_control      = scan_result[index].seq_control;
        results->extended_full_results[index].current_channel  = scan_result[index].current_channel;
        results->extended_full_results[index].country_code     = scan_result[index].country_code;
        results->extended_full_results[index].io_regulation    = scan_result[index].io_regulation;
        results->extended_full_results[index].fcs_check_byte   = scan_result[index].fcs_check_byte;
        results->extended_full_results[index].phi_offset       = scan_result[index].phi_offset;
        results->extended_full_results[index].timestamp_us     = scan_result[index].timestamp_us;
        results->extended_full_results[index].beacon_period_tu = scan_result[index].beacon_period_tu;
    }

    results->timings                = timing;
    results->global_consumption_uas = wifi_compute_consumption(reg_mode, timing);
    results->nbr_results            = nbr_results;
}

/* --- EOF ------------------------------------------------------------------ */
