/*!
 * @file      lr1110_modem_gnss.h
 *
 * @brief     GNSS scan driver definition for LR1110 modem
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

#ifndef LR1110_MODEM_GNSS_H
#define LR1110_MODEM_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1110_modem_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * @brief Number of almanacs in full update payload
 */
#define LR1110_MODEM_GNSS_FULL_UPDATE_N_ALMANACS ( 128 )

/*!
 * @brief Size of the almanac of a single satellite when reading
 */
#define LR1110_MODEM_GNSS_SINGLE_ALMANAC_READ_SIZE ( 22 )

/*!
 * @brief Size of the almanac of a single satellite when writing
 */
#define LR1110_MODEM_GNSS_SINGLE_ALMANAC_WRITE_SIZE ( 20 )

/*!
 * @brief Size of the almanac for all satellites when writing
 */
#define LR1110_MODEM_GNSS_FULL_ALMANAC_WRITE_BUFFER_SIZE \
    ( ( LR1110_MODEM_GNSS_FULL_UPDATE_N_ALMANACS * LR1110_MODEM_GNSS_SINGLE_ALMANAC_WRITE_SIZE ) + 20 )

/*!
 * @brief Position of the destination ID in the scan result buffer
 */
#define LR1110_MODEM_GNSS_SCAN_RESULT_DESTINATION_INDEX ( 0 )

/*!
 * @brief Position of the scan done event type in the scan result buffer, \note a event type exists only if the
 * destination is LR1110_MODEM_GNSS_DESTINATION_HOST
 */
#define LR1110_MODEM_GNSS_SCAN_RESULT_EVENT_TYPE_INDEX ( 1 )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief GNSS Event value for Host destinated message
 *
 * These values are the meaning of the second byte of a message destinated to host obtained when reading
 * LR1110_MODEM_LORAWAN_EVENT_GNSS_SCAN_DONE buffer
 */
typedef enum
{
    LR1110_MODEM_GNSS_SCAN_DONE_PROCESS_OK                                 = 0x00,
    LR1110_MODEM_GNSS_SCAN_DONE_IQ_FAILS                                   = 0x05,
    LR1110_MODEM_GNSS_SCAN_DONE_NO_TIME                                    = 0x06,
    LR1110_MODEM_GNSS_SCAN_DONE_NO_SATELLITE_DETECTED                      = 0x07,
    LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_TOO_OLD                            = 0x08,
    LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_UPDATE_FAILS_CRC_ERROR             = 0x09,
    LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR = 0x0A,
    LR1110_MODEM_GNSS_SCAN_DONE_GLOBAL_ALMANAC_CRC_ERROR                   = 0x0D,
    LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_VERSION_NOT_SUPPORTED              = 0x0E,
} lr1110_modem_gnss_scan_done_event_t;

/*!
 * @brief GNSS response type indicates the destination: Host MCU or GNSS solver
 */
typedef enum
{
    LR1110_MODEM_GNSS_DESTINATION_HOST   = 0x00,  //!< Host MCU
    LR1110_MODEM_GNSS_DESTINATION_SOLVER = 0x01,  //!< GNSS Solver
} lr1110_modem_gnss_destination_t;

/*!
 * @brief Search mode for GNSS scan
 */
typedef enum
{
    LR1110_MODEM_GNSS_OPTION_DEFAULT     = 0x00,  //!< Search all requested satellites or fail
    LR1110_MODEM_GNSS_OPTION_BEST_EFFORT = 0x01,  //!< Add additional search if not all satellites are found
} lr1110_modem_gnss_search_mode_t;

/*!
 * @brief bit mask indicating which information is added in the output payload
 */
enum lr1110_modem_gnss_result_mask_e
{
    LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK = ( 1 << 0 ),
    LR1110_MODEM_GNSS_DOPPLER_MASK      = ( 1 << 1 ),
    LR1110_MODEM_GNSS_BIT_CHANGE_MASK   = ( 1 << 2 ),
};

/*!
 * @brief Constellation identifiers
 */
typedef enum
{
    LR1110_MODEM_GNSS_GPS_MASK    = 0x01,
    LR1110_MODEM_GNSS_BEIDOU_MASK = 0x02,
} lr1110_modem_gnss_constellation_t;

/*!
 * @brief Almanac Constellation identifiers
 */
typedef enum
{
    LR1110_MODEM_GNSS_ALMANAC_CONSTELLATION_GPS       = 0x01,
    LR1110_MODEM_GNSS_ALMANAC_CONSTELLATION_BEIDOU    = 0x02,
    LR1110_MODEM_GNSS_ALMANAC_CONSTELLATION_UNDEFINED = 0x08,
} lr1110_modem_gnss_almanac_constellation_id_t;

/*!
 * @brief Frequency search space around the Doppler frequency
 */
typedef enum
{
    LR1110_MODEM_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ = 0x00,
    LR1110_MODEM_GNSS_FREQUENCY_SEARCH_SPACE_500_HZ = 0x01,
    LR1110_MODEM_GNSS_FREQUENCY_SEARCH_SPACE_1_KHZ  = 0x02,
    LR1110_MODEM_GNSS_FREQUENCY_SEARCH_SPACE_2_KHZ  = 0x03,
} lr1110_modem_gnss_frequency_search_space_t;

/*!
 * @brief Context status error code
 */
typedef enum
{
    LR1110_MODEM_GNSS_CONTEXT_STATUS_NO_ERROR                         = 0x00,  //!< No error
    LR1110_MODEM_GNSS_CONTEXT_STATUS_ALMANAC_TOO_OLD                  = 0x01,  //!< Almanac too old
    LR1110_MODEM_GNSS_CONTEXT_STATUS_LAST_ALMANAC_UPDATE_CRC_MISMATCH = 0x02,  //!< Last almanac update CRC mismatch
    LR1110_MODEM_GNSS_CONTEXT_STATUS_FLASH_MEMORY_INTEGRITY_ERROR     = 0x03,  //!< Flash memory integrity error
    LR1110_MODEM_GNSS_CONTEXT_STATUS_LAST_ALMANAC_UPDATE_TOO_OLD =
        0x04,  //!< Last almanac update time difference more than 1 month
} lr1110_modem_gnss_context_status_error_code_t;

/*!
 * @brief Satellite ID type
 */
typedef uint8_t lr1110_modem_gnss_satellite_id_t;

/*!
 * @brief Bit mask of constellation configurations
 *
 * @see lr1110_modem_gnss_constellation_t
 */
typedef uint8_t lr1110_modem_gnss_constellation_mask_t;

/*!
 * @brief Bit mask of frequency search space configurations
 *
 * @see lr1110_modem_gnss_frequency_search_space_t
 */
typedef uint8_t lr1110_modem_gnss_frequency_search_space_mask_t;

/*!
 * @brief Buffer that holds data for all almanacs full update
 */
typedef uint8_t lr1110_modem_gnss_almanac_full_update_bytestream_t[LR1110_MODEM_GNSS_FULL_ALMANAC_WRITE_BUFFER_SIZE];

/*!
 * @brief Buffer that hold one chunk of almanac for update
 */
typedef uint8_t lr1110_modem_gnss_almanac_one_chunk_bytestream_t[LR1110_MODEM_GNSS_SINGLE_ALMANAC_WRITE_SIZE];

/*!
 * @brief Assistance position.
 */
typedef struct
{
    float latitude;   //!< Latitude 12 bits (latitude in degree * 2048/90) with resolution 0.044°
    float longitude;  //!< Longitude 12 bits (longitude in degree * 2048/180) with resolution 0.088°
} lr1110_modem_gnss_solver_assistance_position_t;

/*!
 * @brief Detected satellite structure
 */
typedef struct
{
    lr1110_modem_gnss_satellite_id_t satellite_id;  //!< Satellite ID
    int8_t                           cnr;           //!< Carrier-to-noise ration (C/N) in dB
} lr1110_modem_gnss_detected_satellite_t;

/*!
 * @brief GNSS timings of the LR1110 modem
 */
typedef struct
{
    uint32_t radio_ms;        //!< Duration with radio on
    uint32_t computation_ms;  //!< Duration of computation
} lr1110_modem_gnss_timings_t;

/*!
 * @brief Version structure of the LR1110 GNSS firmware
 */
typedef struct
{
    uint8_t gnss_firmware;  //!< Version of the firmware
    uint8_t gnss_almanac;   //!< Version of the almanac format
} lr1110_modem_gnss_version_t;

/*!
 * @brief Status message struct in case of operation code = 0x18 (Status Message)
 */
typedef struct
{
    uint8_t  gnss_firmware_version;  //!< GNSS firmware version
    uint32_t global_almanac_crc;  //!< The global CRC is the 32 bit CRC computed on all the flash memory content, on all
                                  //!< 128 satellites. Per sv 21 bytes
    lr1110_modem_gnss_context_status_error_code_t error_code;               //!< Error code
    uint8_t                                       almanac_update_bit_mask;  //!< Almanac update bit mask
    lr1110_modem_gnss_frequency_search_space_t    frequency_search_space;   //!< Frequency search space
} lr1110_modem_gnss_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Activate the GNSS scan constellation
 *
 * @param [in] context Chip implementation context
 * @param [in] constellation_mask Bit mask of the constellations to use. See @ref lr1110_modem_gnss_constellation_mask_t
 * for the possible values
 *
 * @see lr1110_gnss_read_used_constellations
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_set_constellations_to_use(
    const void* context, const lr1110_modem_gnss_constellation_mask_t constellation_mask );

/*!
 * @brief Read constellation used by the GNSS scanner from the almanac update configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] constellations_used Bit mask of the constellations used. See @ref lr1110_modem_gnss_constellation_mask_t
 * for the possible values
 *
 * @see lr1110_modem_gnss_set_constellations_to_use
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_used_constellations(
    const void* context, lr1110_modem_gnss_constellation_mask_t* constellations_used );

/*!
 * @brief Activate the almanac update
 *
 * @param [in] context Chip implementation context
 * @param [in] constellations_to_update Bit mask of the constellations to mark to update. See @ref
 * lr1110_modem_gnss_constellation_t for the possible values
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_set_almanac_update(
    const void* context, const lr1110_modem_gnss_constellation_mask_t constellations_to_update );

/*!
 * @brief Function to read the almanac update configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] constellations_to_update Bit mask of the constellations to mark to update. See @ref
 * lr1110_modem_gnss_constellation_t for the possible values
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_almanac_update(
    const void* context, lr1110_modem_gnss_constellation_mask_t* constellations_to_update );

/*!
 * @brief Set the GNSS frequency search
 *
 * @param [in] context Chip implementation context
 * @param [in] frequency_search_space Bit mask of the frequency search space to use. See @ref
 * lr1110_modem_gnss_frequency_search_space_mask_t for the possible values
 *
 * @see lr1110_modem_gnss_frequency_search_space_mask
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_set_frequency_search(
    const void* context, const lr1110_modem_gnss_frequency_search_space_mask_t frequency_search_space );

/*!
 * @brief Read constellation used by the GNSS scanner from the almanac update configuration
 *
 * @param [in] context Chip implementation context
 * @param [out] frequency_search_space Bit mask of the frequency search space used. See @ref
 * lr1110_modem_gnss_frequency_search_space_mask_t for the possible values
 *
 * @see lr1110_modem_gnss_frequency_search_space_mask
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_frequency_search(
    const void* context, lr1110_modem_gnss_frequency_search_space_mask_t* frequency_search_space );

/*!
 * @brief Function to read the GNSS firmware version
 *
 * @param [in] context Chip implementation context
 * @param [out] version GNSS Firmware version currently running on the chip
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_firmware_version( const void*                  context,
                                                                      lr1110_modem_gnss_version_t* version );

/*!
 * @brief Function to read the supported constellation, GPS or BEIDOU other constellations
 *
 * @param [in] context Chip implementation context
 * @param [out] supported_constellations Bit mask of the constellations used. See @ref
 * lr1110_modem_gnss_constellation_mask_t for the possible values
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_supported_constellations(
    const void* context, lr1110_modem_gnss_constellation_mask_t* supported_constellations );

/*!
 * @brief Update full almanac for all satellites
 *
 * This function is to be used to update satellites almanac. Note that all 128 satellite almanacs must be update in a
 * row. Therefore, this function must be called 128 times in a row without any other calls in between.
 *
 * @param [in] context Chip implementation context
 * @param [in] almanac_bytestream Almanac buffer to update all almanac of the LR1110. It is up to the application to
 * ensure that the buffer almanac_bytestream is indeed of size LR1110_MODEM_GNSS_FULL_ALMANAC_WRITE_BUFFER_SIZE
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_almanac_full_update(
    const void* context, const lr1110_modem_gnss_almanac_full_update_bytestream_t almanac_bytestream );

/*!
 * @brief One chunk almanac update
 *
 * This function is to be used to update a single chunk almanac. Note that all 129 chunks almanacs must be updated in a
 * row. Therefore, this function must be called 129 times in a row without any other calls in between.
 *
 * On the contrary, lr1110_gnss_almanac_full_update can be used to update all almanacs in one call, but the application
 * must be able to provide a buffer that holds all almanac (>2kB).
 *
 * @param [in] context Chip implementation context
 * @param [in] almanac_one_chunk_bytestream Almanac buffer to update one chunk almanac of the LR1110. It is up to the
 * application to ensure that bytestream is at least LR1110_MODEM_GNSS_SINGLE_ALMANAC_WRITE_SIZE long.
 *
 * @returns Status of the driver call
 */
lr1110_modem_response_code_t lr1110_modem_gnss_one_chunk_almanac_update(
    const void* context, const lr1110_modem_gnss_almanac_one_chunk_bytestream_t almanac_one_chunk_bytestream );

/*!
 * @brief Function to set the assistance position.
 *
 * @param [in] context Chip implementation context
 * @param [in] assistance_position latitude 12 bits and longitude 12 bits @ref
 * lr1110_modem_gnss_solver_assistance_position_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_set_assistance_position(
    const void* context, const lr1110_modem_gnss_solver_assistance_position_t* assistance_position );

/*!
 * @brief Function to read the assistance position.
 *
 * The assistance position read may be different from the one set beforehand
 * with lr1110_modem_gnss_set_assistance_position due to a scaling computation.
 *
 * @param [in] context Chip implementation context
 * @param [out] assistance_position latitude 12 bits and longitude 12 bits @ref
 * lr1110_modem_gnss_solver_assistance_position_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_assistance_position(
    const void* context, lr1110_modem_gnss_solver_assistance_position_t* assistance_position );

/*!
 * @brief Function to set the Xtal error.
 *
 * @param [in] context Chip implementation context
 * @param [in] xtal_error_in_ppm value in +/-40ppm
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_set_xtal_error( const void* context, const float xtal_error_in_ppm );

/*!
 * @brief Function to read the Xtal error.
 *
 * @param [in] context Chip implementation context
 * @param [out] xtal_error_in_ppm value returned between +/-30ppm
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_read_xtal_error( const void* context, float* xtal_error_in_ppm );

/*!
 * @brief Function to read context status.
 *
 * @param [in] context Chip implementation context
 * @param [out] gnss_context @ref lr1110_modem_gnss_context_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_get_context( const void*                  context,
                                                            lr1110_modem_gnss_context_t* gnss_context );

/*!
 * @brief Get the number of detected satellites during last scan
 *
 * @param [in] context Chip implementation context
 * @param [out] nb_detected_satellites Number of satellites detected
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_get_nb_detected_satellites( const void* context,
                                                                           uint8_t*    nb_detected_satellites );

/*!
 * @brief Get the satellites detected on last scan with their IDs and C/N (aka.
 * CNR)
 *
 * @param [in] context Chip implementation context
 * @param [in] nb_detected_satellites Number of detected satellites on last scan (obtained by calling
 * lr1110_gnss_get_nb_detected_satellites)
 * @param [out] detected_satellite_id_snr Pointer to an array of structures of size big enough to contain
 * nb_detected_satellites elements
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_get_detected_satellites(
    const void* context, const uint8_t nb_detected_satellites,
    lr1110_modem_gnss_detected_satellite_t* detected_satellite_id_snr );

/*!
 * @brief Get the timings spent in signal acquisition and signal analyzis
 *
 * These timings allow to compute the current timings of the last GNSS scan.
 *
 * @param [in] context Chip implementation context
 * @param [out] timings GNSS timings of last GNSS scan
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_get_timings( const void* context, lr1110_modem_gnss_timings_t* timings );

/*!
 * @brief Read at maximum 11 sv’s Almanac, starting from sv id
 *
 * @param [in] context Chip implementation context
 * @param [in] sv_id Satellite ID
 * @param [in] nb_sv Number of satellites @note the maximum nb sv readable is 11.
 * @param [out] almanac_read The almanac buffer @note the minimal size of the buffer to provide is nb_sv * 22 bytes
 * @param [in] buffer_len Buffer len of the almanac_read buffer.
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_almanac_read_by_index( const void* context, uint8_t sv_id, uint8_t nb_sv,
                                                                      uint8_t* almanac_read, uint8_t buffer_len );

/*!
 * @brief GNSS scan with no assisted parameters needed
 *
 * @warning Parameter effort_mode can only be set to LR1110_MODEM_GNSS_OPTION_DEFAULT
 *
 * @param [in] context Chip implementation context
 * @param [in] effort_mode Effort mode @ref lr1110_modem_gnss_search_mode_t
 * @param [in] gnss_result_mask Bit mask indicating which information is added in the output payload @ref
 * lr1110_modem_gnss_result_mask_e
 * @param [in] nb_sat The expected number of satellite to provide. This value must be in the range [0:128]
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_scan_autonomous( const void*                           context,
                                                                const lr1110_modem_gnss_search_mode_t effort_mode,
                                                                const uint8_t gnss_result_mask,
                                                                const uint8_t nb_sat );

/*!
 * @brief GNSS scan with assisted parameters.
 *
 * @param [in] context Chip implementation context
 * @param [in] effort_mode Effort mode @ref lr1110_modem_gnss_search_mode_t
 * @param [in] gnss_result_mask Bit mask indicating which information is added in the output payload @ref
 * lr1110_modem_gnss_result_mask_e
 * @param [in] nb_sat The expected number of satellite to provide. This value must be in the range [0:128]
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_scan_assisted( const void*                           context,
                                                              const lr1110_modem_gnss_search_mode_t effort_mode,
                                                              const uint8_t gnss_result_mask,
                                                              const uint8_t nb_sat );

/*!
 * @brief Push data received from solver to LR1110 modem
 *
 * @param [in] context Chip implementation context
 * @param [in] payload Payload received from solver
 * @param [in] payload_size Size of the payload received from solver (in bytes)
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_gnss_push_solver_msg( const void* context, const uint8_t* payload,
                                                                const uint16_t payload_size );

#ifdef __cplusplus
}
#endif

#endif  // LR1110_MODEM_GNSS_H

/* --- EOF ------------------------------------------------------------------ */
