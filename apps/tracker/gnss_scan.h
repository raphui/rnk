/*!
 * @file      gnss_scan.h
 *
 * @brief     GNSS scan definition
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

#ifndef GNSS_SCAN_H
#define GNSS_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include "modem/lr1110_modem_gnss.h"
#include "modem/lr1110_modem_system.h"

struct tracker;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define GNSS_BUFFER_MAX_SIZE 255

#define GNSS_LEAP_SECONDS_OFFSET 18
#define GNSS_EPOCH_SECONDS 315964800  // 6/01/1980
#define SECS_PER_WEEK 604800          //(60L*60*24*7)

#define ASSISTED_MODE 1
#define AUTONOMOUS_MODE 2

#define GNSS_SCAN_NO_TIME 2
#define GNSS_SCAN_SUCCESS 1
#define GNSS_SCAN_FAIL 0

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief GNSS scan result type
 */
typedef uint8_t gnss_scan_result_t;

/*!
 * @brief GNSS general settings
 */
typedef struct
{
    bool                                           enabled;
    uint8_t                                        scan_type;
    lr1110_modem_gnss_search_mode_t                search_mode;
    uint8_t                                        input_parameters;
    uint8_t                                        constellation_to_use;
    lr1110_modem_gnss_solver_assistance_position_t assistance_position;
    uint8_t                                        nb_sat;
} gnss_settings_t;

/*!
 * @brief GNSS scan results
 */
typedef struct
{
    bool                                   is_valid_nav_message;
    uint16_t                               nav_message_size;
    uint8_t                                nav_message[GNSS_BUFFER_MAX_SIZE];
    uint8_t                                nb_detected_satellites;
    lr1110_modem_gnss_detected_satellite_t detected_satellites[32];
    lr1110_modem_gnss_timings_t            timings;
    uint8_t                                average_cn;
} gnss_scan_single_result_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Function executed on GNSS Scan done event
 *
 * @param [in] buffer Buffer containing the NAV Message
 * @param [in] size Size of the NAV message
 */
void lr1110_modem_gnss_scan_done(struct tracker *tracker, uint8_t* buffer, uint16_t size);

/*!
 * @brief Display the last scan results
 *
 * @param [in] capture_result Structure containing the capture result, \ref gnss_scan_single_result_t
 */
void gnss_scan_display_results(const gnss_scan_single_result_t* capture_result);

/*!
 * @brief start the gnss capture state machine
 *
 * @param [in] context Radio abstraction
 * @param [in] settings GNSS settings to apply \ref gnss_settings_t
 * @param [out] capture_result Structure containing the capture result, \ref gnss_scan_single_result_t
 *
 * @returns GNSS scan result operation
 */
gnss_scan_result_t gnss_scan_execute(struct tracker *tracker, const void* context, const gnss_settings_t* settings,
                                      gnss_scan_single_result_t* capture_result);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_SCAN_H

/* --- EOF ------------------------------------------------------------------ */
