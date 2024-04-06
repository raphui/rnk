/*!
 * @file      lr1110_modem_board.h
 *
 * @brief     Target LR1110 EVK Modem board driver definition
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

#ifndef LR1110_MODEM_BOARD_H
#define LR1110_MODEM_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "../tracker.h"
#include "lr1110_modem_common.h"
#include "lr1110.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initializes the radio I/Os pins context
 *
 * @param [in] context Radio abstraction
 */
void lr1110_modem_board_init_io_context( void* context );

/*!
 * @brief Initializes the radio I/Os pins interface
 *
 * @param [in] context Radio abstraction
 */
void lr1110_modem_board_init_io( const void* context );

/*!
 * @brief De-initializes the radio I/Os pins interface.
 *
 * @param [in] context Radio abstraction
 *
 * @remark Useful when going in MCU low power modes
 */
void lr1110_modem_board_deinit_io( const void* context );

/*!
 * @brief De-initializes the radio I/Os pins interface for deep sleep purpose --> switch Busy and DIO in analog input.
 *
 * @param [in] context Radio abstraction
 *
 * @remark Useful when going in MCU low power modes
 */
void lr1110_modem_board_analog_deinit_io( const void* context );

/*!
 * @brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * @returns time Board TCXO wakeup time in ms.
 */
uint32_t lr1110_modem_board_get_tcxo_wakeup_time( void );

/*!
 * @brief Initializes the radio driver
 *
 * @param [in] context Radio abstraction
 * @param [in] event Pointeur to the event callbacks \see lr1110_modem_event_callback_t
 *
 * @returns Status of the init
 */
lr1110_modem_response_code_t lr1110_modem_board_init(struct tracker *tracker, const void* context, lr1110_modem_event_callback_t* event);

/*!
 * @brief Flush the modem event queue
 *
 * @param [in] context Radio abstraction
 *
 * @returns Modem-E response code
 */
lr1110_modem_response_code_t lr1110_modem_board_event_flush( const void* context );

/*!
 * @brief Read the event line value used to process the event queue
 *
 * @param [in] context Radio abstraction
 */
bool lr1110_modem_board_read_event_line( const void* context );

/*!
 * @brief turn on the LNA
 */
void lr1110_modem_board_lna_on( void );

/*!
 * @brief turn off the LNA
 */
void lr1110_modem_board_lna_off( void );

/*!
 * @brief convert the GPS time in unix time \note assume that GPS time is right
 *
 * @param [in] context Radio abstraction
 *
 * @returns Unix time in s.
 */
uint32_t lr1110_modem_board_get_systime_from_gps( const void* context );

/*!
 * @brief Get the oldest and the newest almanac date from the Modem-E
 *
 * @param [in] context Radio abstraction
 * @param [out] oldest_almanac_date oldest sv date
 * @param [out] newest_almanac_date newest sv date
 *
 * @returns Modem-E response code
 */
lr1110_modem_response_code_t lr1110_modem_get_almanac_dates( const void* context, uint32_t* oldest_almanac_date,
                                                             uint32_t* newest_almanac_date );

/*!
 * @brief notify the user is the modem is ready
 *
 * @returns Modem ready state.
 */
bool lr1110_modem_board_is_ready( void );

/*!
 * @brief set the modem is ready flag
 *
 * @param [in] ready ready state
 */
void lr1110_modem_board_set_ready( bool ready );

/*!
 * @brief Measure the dropout voltage when the board drains batteries current
 *
 * @param [in] context Radio abstraction
 * @param [out] drop Voltage drop measured during the TX
 * @param [out] time_recovery  time taken to the supply rail to reach Vnom after th TX shutdown
 *
 * @returns Modem-E response code
 */
lr1110_modem_response_code_t lr1110_modem_board_measure_battery_drop( const void* context, int32_t* drop,
                                                                      uint32_t* time_recovery );

/*!
 * @brief Turn on/off the requested LED(s)
 *
 * @param [in] led_mask Mask representing the list of the LEDs to turn on/off
 * @param [in] turn_on If true, the requested LEDs are turned on, else they are turned off
 */
void lr1110_modem_board_led_set( uint32_t led_mask, bool turn_on );

/*!
 * @brief Turn on/off the requested LED(s) for a given duration
 *
 * @param [in] led_mask Mask representing the list of the LEDs to turn on/off
 * @param [in] turn_on If true, the requested LEDs are turned on, else they are turned off
 * @param [in] duration_ms Duration of the pulse, in milliseconds
 */
void lr1110_modem_board_led_pulse( uint32_t led_mask, bool turn_on, uint32_t duration_ms );

#ifdef __cplusplus
}
#endif

#endif  // LR1110_MODEM_BOARD_H

/* --- EOF ------------------------------------------------------------------ */
