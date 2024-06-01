/*!
 * @file      gnss_scan.c
 *
 * @brief     GNSS scan implementation.
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
#include "gnss_scan.h"
#include "modem/lr1110_modem_board.h"
#include "modem/lr1110_modem_helper.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief GNSS scan state machine timeout
 */
#define GNSS_SCAN_TIMEOUT (15000 * 1000)

/*!
 * @brief GNSS scan timing description
 *
 * Assisted mode
          |                                               |<----- GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL --------->|                                            |
          |<-- ALMANAC_CRC_CHECK -->|<-- TCXCO_STARTUP -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|<-- IDLE -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|
          |<--------------------(lna off)---------------->|<-------(lna on)-------->|<-------------(lna off)------->|<-------(lna on)-------->|<---(lna off)---->|
          |                                               |                         |                               |                         |                  |
  Scan command Received                             Start Scan GPS      End Scan GPS/Start Comput GPS         Start Scan Beidou      End Scan Beidou/Start Comput Beidou
 *
 * Autonomous mode
          |                     |<----- GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL --------->|                                            |
          |<-- TCXCO_STARTUP -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|<-- IDLE -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|
          |<-------(lna off)--->|<-------(lna on)-------->|<-------------(lna off)------->|<-------(lna on)-------->|<---(lna off)---->|
          |                     |                         |                               |                         |                  |
  Scan command Received   Start Scan GPS      End Scan GPS/Start Comput GPS         Start Scan Beidou      End Scan Beidou/Start Comput Beidou
 *
 * For the very first scan after a reset of the modem, an additional "CALIBRATION" step is added before the first "RADIO_ACQUISITION".
 * The LNA must be on during the "CALIBRATION" step, which lasts 5ms plus the TCXO startup time.
 * Note that the "CALIBRATION" steps only occurs on the first scan after reset (even in case of double constellation scan), and not for other scans.
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief GNSS state used in the state machine
 */
typedef enum
{
    GNSS_START_SCAN,
    GNSS_GET_RESULTS,
    GNSS_TERMINATED,
    GNSS_LOW_POWER,
} gnss_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief GNSS state \ref gnss_state_t
 */
static gnss_state_t gnss_state;

/*!
 * @brief Buffer scan result buffer
 */
static uint8_t gnss_scan_result_buffer[GNSS_BUFFER_MAX_SIZE];

/*!
 * @brief Buffer scan result buffer size
 */
static uint16_t gnss_scan_result_buffer_size;

/*!
 * @brief GNSS scan type parameter
 */
static uint8_t scan_type = ASSISTED_MODE;

/*!
 * @brief GNSS scan timeout flag
 */
static bool gnss_scan_timeout = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Analyze the received NAV message and determine if it's a valid one
 *
 * @param [in] settings Gnss settings used \ref gnss_settings_t
 * @param [in] capture_result Structure containing the capture result
 * @param [out] is_valid_nav_message If true, the NAV message is valid if false, it's not valid
 * @param [out] average_cn Average CN of the detected satellites
 */
static void gnss_analyse_nav_message(const gnss_settings_t* settings, const gnss_scan_single_result_t* capture_result,
                                      bool* is_valid_nav_message, uint8_t* average_cn);

/*!
 * @brief configure by default the gnss scanner
 *
 * @param [in] context Radio abstraction
 * @param [in] settings gnss settings to apply \ref gnss_settings_t
 */
static void gnss_scan_configure(const void* context, const gnss_settings_t* settings);

/*!
 * @brief Choose scan mode between assisted and autonomous
 *
 * @param [in] type between
    ASSISTED_MODE or AUTONOMOUS_MODE
 */
static void gnss_scan_set_type(uint8_t type);
/*!
 * @brief init the gnss state machine
 *
 * @param [in] context Radio abstraction
 * @param [in] settings GNSS settings to apply \ref gnss_settings_t
 */
static void gnss_scan_init(const void* context, const gnss_settings_t* settings);

/*!
 * @brief Check if the setting mask indicates a single constellation GNSS scan
 *
 * @param [in] settings GNSS settings used \ref gnss_settings_t
 */
static bool is_single_constellation_setting(const gnss_settings_t* settings);
/*!
 * @brief Function executed on gnss scan timeout event
 *
 * @param [in] context Radio abstraction
 */
static void on_gnss_scan_timeout_event(void* context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_gnss_scan_done(struct tracker *tracker, uint8_t* buffer, uint16_t size)
{
    lr1110_modem_gnss_destination_t     destination;
    lr1110_modem_gnss_scan_done_event_t event_type;

    lr1110_modem_helper_gnss_get_result_destination(buffer, size, &destination);

    if(destination == LR1110_MODEM_GNSS_DESTINATION_HOST)
    {
        lr1110_modem_helper_gnss_get_event_type(buffer, size, &event_type);
    }
    memcpy(gnss_scan_result_buffer, buffer, size);
    gnss_scan_result_buffer_size = size;
    gnss_state                   = GNSS_GET_RESULTS;
}

gnss_scan_result_t gnss_scan_execute(struct tracker *tracker, const void* context, const gnss_settings_t* settings,
                                      gnss_scan_single_result_t* capture_result)
{
    bool                         gnss_scan_done      = false;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    gnss_scan_result_t           scan_result         = GNSS_SCAN_SUCCESS;

    /* Reset parameters */
    gnss_scan_timeout                    = false;
    capture_result->is_valid_nav_message = false;

    /* Init the GNSS parameters */
    gnss_scan_init(context, settings);

    time_oneshot(&((lr1110_t *)context)->gnss_scan_timeout_timer, GNSS_SCAN_TIMEOUT, on_gnss_scan_timeout_event, NULL);

    while((gnss_scan_done != true) && (gnss_scan_timeout != true))
    {
        switch(gnss_state)
        {
        case GNSS_START_SCAN:
        {
// FIXME
#if 0
            /* Turn on the scan led during the scan */
            leds_on(LED_SCAN_MASK);

            /* Switch on the LNA */
            lr1110_modem_board_lna_on();
#endif

            if(scan_type == AUTONOMOUS_MODE)
            {
                modem_response_code = lr1110_modem_gnss_scan_autonomous(
                    context, settings->search_mode, LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK, settings->nb_sat);
            }
            else
            {
                modem_response_code = lr1110_modem_gnss_scan_assisted(
                    context, settings->search_mode, LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK, settings->nb_sat);
            }

            /* If response code different than RESPONSE_CODE_OK leave */
            if(modem_response_code == LR1110_MODEM_RESPONSE_CODE_NO_TIME)
            {
                gnss_scan_done = true;
                scan_result    = GNSS_SCAN_NO_TIME;
            }
            else if(modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK)
            {
                gnss_scan_done = true;
                scan_result    = GNSS_SCAN_FAIL;
            }

            gnss_state = GNSS_LOW_POWER;
            break;
        }

        case GNSS_GET_RESULTS:
        {
            /* Store the NAV message */
            memcpy(capture_result->nav_message, gnss_scan_result_buffer, gnss_scan_result_buffer_size);
            capture_result->nav_message_size = gnss_scan_result_buffer_size;

            modem_response_code =
                lr1110_modem_gnss_get_nb_detected_satellites(context, &capture_result->nb_detected_satellites);
            modem_response_code = lr1110_modem_gnss_get_detected_satellites(
                context, capture_result->nb_detected_satellites, capture_result->detected_satellites);
            lr1110_modem_gnss_get_timings(context, &capture_result->timings);

            /* Analyze the received NAV message */
            gnss_analyse_nav_message(settings, capture_result, &capture_result->is_valid_nav_message,
                                      &capture_result->average_cn);

            gnss_state = GNSS_TERMINATED;

            break;
        }

        case GNSS_TERMINATED:
        {
            gnss_state     = GNSS_START_SCAN;
            gnss_scan_done = true;
            break;
        }

        case GNSS_LOW_POWER:
        {
	    sem_wait(&tracker->lr1110.event_processed_sem);

            break;
        }
        }
    }

// FIXME
#if 0
    /* Switch off the LNA */
    lr1110_modem_board_lna_off();

    /* Turn off the scan led at the end of the scan */
    leds_off(LED_SCAN_MASK);
#endif
    time_oneshot_cancel(&((lr1110_t *)context)->gnss_scan_timeout_timer);

    if(gnss_scan_timeout == true)
    {
        scan_result = GNSS_SCAN_FAIL;
    }

    return scan_result;
}

void gnss_scan_display_results(const gnss_scan_single_result_t* capture_result)
{
    uint8_t i = 0;

    /* Satellites infos */

    HAL_DBG_TRACE_PRINTF("Nb Detected satellites : %d \r\n", capture_result->nb_detected_satellites);

    if(capture_result->nb_detected_satellites > 0)
    {
        HAL_DBG_TRACE_MSG("Satellites infos : \r\n");

        for(i = 0; i < capture_result->nb_detected_satellites; i++)
        {
            HAL_DBG_TRACE_PRINTF("ID = %d -- CN = %d \r\n", capture_result->detected_satellites[i].satellite_id,
                                  capture_result->detected_satellites[i].cnr);
        }
    }

    /* Scan Timings */
    HAL_DBG_TRACE_PRINTF("Scan timing radio_ms : %d\r\n", capture_result->timings.radio_ms);
    HAL_DBG_TRACE_PRINTF("Scan timing computation_ms : %d\r\n", capture_result->timings.computation_ms);

    /* NAV Message */

    HAL_DBG_TRACE_MSG("NAV = ");

    for(i = 0; i < capture_result->nav_message_size; i++)
    {
        HAL_DBG_TRACE_PRINTF("%02X", capture_result->nav_message[i]);
    }

    HAL_DBG_TRACE_PRINTF("\r\nIs NAV message valid : %d\r\n", capture_result->is_valid_nav_message);

    if(capture_result->is_valid_nav_message == true)
    {
        /* Average CN */
        HAL_DBG_TRACE_PRINTF("Average CN : %d\r\n", capture_result->average_cn);
    }

    HAL_DBG_TRACE_MSG("\r\n");
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_gnss_scan_timeout_event(void* context)
{
	gnss_scan_timeout = true;
}

static void gnss_scan_configure(const void* context, const gnss_settings_t* settings)
{
    lr1110_modem_gnss_set_constellations_to_use(context, settings->constellation_to_use);
    gnss_scan_set_type(settings->scan_type);
}

static bool is_single_constellation_setting(const gnss_settings_t* settings)
{
    return settings->constellation_to_use != (LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK);
}

static void gnss_analyse_nav_message(const gnss_settings_t* settings, const gnss_scan_single_result_t* capture_result,
                                      bool* is_valid_nav_message, uint8_t* average_cn)
{
    uint16_t average_cn_tmp = 0;

    /* Analyse the NAV message:
    Check the validity which is defined by having :
    at least 2 sv per constellation (BEIDOU and GNSS only) and 6 detected satellites in the case of double
    constellation. if there are 5 sv in a same constellation in case of double constellation the NAV message is valid
    5 detected satellites (BEIDOU and GNSS only) in the case of single constellation.
    GPS satellites ID [0 31], SBAS satellites ID [32 63] but not used, BEIDOU satellites ID [64 128].
    Calcul the average CN.
    */
    if(capture_result->nb_detected_satellites >= 5)
    {
        uint8_t gps_sv_cnt    = 0;
        uint8_t beidou_sv_cnt = 0;

        for(uint8_t i = 0; i < capture_result->nb_detected_satellites; i++)
        {
            average_cn_tmp += capture_result->detected_satellites[i].cnr;

            /* Remove the SBAS from the count */
            /* Check if it's a GPS satellite */
            if(capture_result->detected_satellites[i].satellite_id <= 31)
            {
                gps_sv_cnt++;
            }
            /* Check if it's a BEIDOU satellite */
            if((capture_result->detected_satellites[i].satellite_id >= 64) &&
                (capture_result->detected_satellites[i].satellite_id <= 128))
            {
                beidou_sv_cnt++;
            }
        }

        /* Calcul the average CN */
        *average_cn = average_cn_tmp / capture_result->nb_detected_satellites;

        /* Check if the NAV message is valid */
        if((is_single_constellation_setting(settings) == true))
        {
            if((gps_sv_cnt >= 5) || (beidou_sv_cnt >= 5))
            {
                *is_valid_nav_message = true;
            }
            else
            {
                *is_valid_nav_message = false;
            }
        }
        else
        {
            if(((gps_sv_cnt >= 2) && (beidou_sv_cnt >= 2) && ((gps_sv_cnt + beidou_sv_cnt) >= 6)) ||
                (((gps_sv_cnt >= 5) || (beidou_sv_cnt >= 5)) && ((gps_sv_cnt + beidou_sv_cnt) >= 5)))
            {
                *is_valid_nav_message = true;
            }
            else
            {
                *is_valid_nav_message = false;
            }
        }
    }
    else
    {
        *is_valid_nav_message = false;
    }
}

void gnss_scan_set_type(uint8_t type)
{
    if(type == ASSISTED_MODE)
    {
        scan_type = ASSISTED_MODE;
    }
    else
    {
        scan_type = AUTONOMOUS_MODE;
    }
}

void gnss_scan_init(const void* context, const gnss_settings_t* settings)
{
    gnss_state                   = GNSS_START_SCAN;
    gnss_scan_result_buffer_size = 0;

    gnss_scan_configure(context, settings);
}


/* --- EOF ------------------------------------------------------------------ */
