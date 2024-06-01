/*!
 * @file      lr1110_modem_hal.c
 *
 * @brief     Hardware Abstraction Layer (HAL) implementation for LR1110
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

#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "lr1110_hal.h"
#include "lr1110_modem_hal.h"
#include "lr1110_modem_system.h"
#include "lr1110_modem_board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_MODEM_RESET_TIMEOUT (3000 * 1000)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

struct lr1110_transfer {
	const uint8_t *command;
	uint16_t command_length;
	const uint8_t *data;
	uint16_t data_length;
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief LR1110 modem-e reset timeout flag
 */
static bool lr1110_modem_reset_timeout = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Function to wait that the lr1110 transceiver busy line raise to high
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1110_hal_status_t
 */
static lr1110_hal_status_t lr1110_hal_wait_on_busy(const void* context, uint32_t timeout_ms);

/*!
 * @brief Function to wait that the lr1110 modem-e busy line fall to low
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1110_hal_status_t
 */
static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_busy(const void* context, uint32_t timeout_ms);

/*!
 * @brief Function to wait the that lr1110 modem-e busy line raise to high
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1110_hal_status_t
 */
static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_unbusy(const void* context, uint32_t timeout_ms);

/*!
 * @brief Function executed on lr1110 modem-e reset timeout event
 */
static void on_lr1110_modem_reset_timeout_event(void* context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*!
 * @brief lr1110_modem_hal.h API implementation
 */

lr1110_modem_hal_status_t lr1110_modem_hal_write(const void* context, const uint8_t* command,
                                                  const uint16_t command_length, const uint8_t* data,
                                                  const uint16_t data_length)
{
    uint8_t crc = 0;
    uint8_t crc_received = 0;
    struct lr1110_transfer transfer;
    lr1110_modem_hal_status_t status;

    transfer.command = command;
    transfer.command_length = command_length;
    transfer.data = data;
    transfer.data_length = data_length;

    if (lr1110_modem_hal_wakeup(context) == LR1110_MODEM_HAL_STATUS_OK) {
	write(((lr1110_t *)context)->spi_id, &transfer, sizeof(struct lr1110_transfer));

	/* Wait on busy pin up to 1000 ms */
        if (lr1110_modem_hal_wait_on_busy(context, 1000) != LR1110_MODEM_HAL_STATUS_OK) {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        /* Send dummy byte to retrieve RC & CRC */

        /* NSS low */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_SET_NSS, NULL);

        /* read RC */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&status);

	ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&crc_received);

        /* Compute response crc */
        crc = lr1110_modem_compute_crc(0xFF, (uint8_t*) &status, 1);

        /* NSS high */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_CLEAR_NSS, NULL);

        if (crc != crc_received) {
            /* change the response code */
            status = LR1110_MODEM_HAL_STATUS_BAD_FRAME;
        }

        /* Wait on busy pin up to 1000 ms */
        if (lr1110_modem_hal_wait_on_unbusy(context, 1000) != LR1110_MODEM_HAL_STATUS_OK) {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

	return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_write_without_rc(const void* context, const uint8_t* command,
                                                             const uint16_t command_length, const uint8_t* data,
                                                             const uint16_t data_length)
{
    struct lr1110_transfer transfer;
    lr1110_modem_hal_status_t status = LR1110_MODEM_HAL_STATUS_OK;

    transfer.command = command;
    transfer.command_length = command_length;
    transfer.data = data;
    transfer.data_length = data_length;

    if (lr1110_modem_hal_wakeup(context) == LR1110_MODEM_HAL_STATUS_OK) {
	write(((lr1110_t *)context)->spi_id, &transfer, sizeof(struct lr1110_transfer));

        return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_read(const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length)
{
    uint8_t crc = 0;
    uint8_t crc_received = 0;
    struct lr1110_transfer transfer;
    lr1110_modem_hal_status_t status;

    transfer.command = command;
    transfer.command_length = command_length;
    transfer.data = data;
    transfer.data_length = data_length;

    if (lr1110_modem_hal_wakeup(context) == LR1110_MODEM_HAL_STATUS_OK) {
	read(((lr1110_t *)context)->spi_id, &transfer, sizeof(struct lr1110_transfer));

        /* Wait on busy pin up to 1000 ms */
        if (lr1110_modem_hal_wait_on_busy(context, 1000) != LR1110_MODEM_HAL_STATUS_OK) {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        /* Send dummy byte to retrieve RC & CRC */

        /* NSS low */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_SET_NSS, NULL);

        /* read RC */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&status);

        if (status == LR1110_MODEM_HAL_STATUS_OK) {
            for (uint16_t i = 0; i < data_length; i++) {
		ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&data[i]);

            }
        }

	ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&crc_received);

        /* NSS high */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_CLEAR_NSS, NULL);

        /* Compute response crc */
        crc = lr1110_modem_compute_crc(0xFF, (uint8_t*) &status, 1);

        if (status == LR1110_MODEM_HAL_STATUS_OK) {
            crc = lr1110_modem_compute_crc(crc, data, data_length);
        }

        if (crc != crc_received) {
            /* change the response code */
            status = LR1110_MODEM_HAL_STATUS_BAD_FRAME;
        }

        /* Wait on busy pin up to 1000 ms */
        if (lr1110_modem_hal_wait_on_unbusy(context, 1000) != LR1110_MODEM_HAL_STATUS_OK) {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_reset(struct tracker *tracker, const void* context)
{
    int ret = 0;
    lr1110_modem_board_set_ready(false);
    

    time_oneshot(&((lr1110_t *)context)->reset_timeout_timer, LR1110_MODEM_RESET_TIMEOUT, on_lr1110_modem_reset_timeout_event, NULL);
    lr1110_modem_reset_timeout = false;

    ioctl(((lr1110_t *)context)->spi_id, IOCTL_RESET, NULL);

    ret = sem_timedwait(&tracker->lr1110.event_processed_sem, LR1110_MODEM_RESET_TIMEOUT);;

    if(ret < 0)
    {
        return LR1110_MODEM_HAL_STATUS_ERROR;
    }
    else
    {
        return LR1110_MODEM_HAL_STATUS_OK;
    }
}

void lr1110_modem_hal_enter_dfu(const void* context)
{
    /* Force dio0 to 0 */
    gpiolib_set_output(((lr1110_t *)context)->busy, 0);

    ioctl(((lr1110_t *)context)->spi_id, IOCTL_RESET, NULL);

    /* wait 250ms */
    time_usleep(250 * 1000);

    /* reinit dio0 */
    gpiolib_set_input(((lr1110_t *)context)->busy);
}

lr1110_modem_hal_status_t lr1110_modem_hal_wakeup(const void* context)
{
    if(lr1110_modem_hal_wait_on_busy(context, 10000) == LR1110_MODEM_HAL_STATUS_OK)
    {
        /* Wakeup radio */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_WAKEUP, NULL);
    }
    else
    {
        return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /* Wait on busy pin for 1000 ms */
    return lr1110_modem_hal_wait_on_unbusy(context, 1000);
}

/*!
 * @brief Bootstrap bootloader and SPI bootloader API implementation
 */

lr1110_hal_status_t lr1110_hal_write(const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length)
{
    if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {

        /* NSS high */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_SET_NSS, NULL);

        for (uint16_t i = 0; i < command_length; i++) {
	    ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&command[i]);
        }
        for(uint16_t i = 0; i < data_length; i++) {
	    ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&data[i]);
        }

        /* NSS low */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_CLEAR_NSS, NULL);

        return lr1110_hal_wait_on_busy(context, 5000);
    }

    return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_read(const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length)
{
    char dummy;

    if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {

        /* NSS high */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_SET_NSS, NULL);

        for (uint16_t i = 0; i < command_length; i++) {
	    ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&command[i]);
        }

        /* NSS low */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_CLEAR_NSS, NULL);

        if(lr1110_hal_wait_on_busy(context, 5000) != LR1110_HAL_STATUS_OK)
        {
            return LR1110_HAL_STATUS_ERROR;
        }

        /* Send dummy byte */
	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_SET_NSS, NULL);

	ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&dummy);

        for (uint16_t i = 0; i < data_length; i++) {
	    ioctl(((lr1110_t *)context)->spi_id, IOCTL_WRITE_CHAR, (char *)&data[i]);
        }

	ioctl(((lr1110_t *)context)->spi_id, IOCTL_SPI_CLEAR_NSS, NULL);

        return lr1110_hal_wait_on_busy(context, 5000);
    }

    return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_wakeup(const void* context)
{
    /* Wakeup radio */
    ioctl(((lr1110_t *)context)->spi_id, IOCTL_WAKEUP, NULL);

    /* Wait on busy pin for 5000 ms */
    return lr1110_hal_wait_on_busy(context, 5000);
}

lr1110_hal_status_t lr1110_hal_reset(const void* context)
{
    ioctl(((lr1110_t *)context)->spi_id, IOCTL_RESET, NULL);

    return LR1110_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_lr1110_modem_reset_timeout_event(void* context)
{
	lr1110_modem_reset_timeout = true;
}

static lr1110_hal_status_t lr1110_hal_wait_on_busy(const void* context, uint32_t timeout_ms)
{
#if 0
    while(gpiolib_get_value(((lr1110_t*)context)->busy) == 1)
    {
        ;
    }
#else
    uint32_t start = time_get_ticks();
    while(gpiolib_get_value(((lr1110_t*)context)->busy) == 1)
    {
        if((int32_t)(time_get_ticks() - start) > (int32_t) timeout_ms)
        {
            return LR1110_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_HAL_STATUS_OK;
}

static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_busy(const void* context, uint32_t timeout_ms)
{
#if 0
    while(gpiolib_get_value(((lr1110_t*)context)->busy) == 0)
    {
        ;
    }
#else
    uint32_t start = time_get_ticks();
    while(gpiolib_get_value(((lr1110_t*)context)->busy) == 0)
    {
        if((int32_t)(time_get_ticks() - start) > (int32_t) timeout_ms)
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_MODEM_HAL_STATUS_OK;
}

static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_unbusy(const void* context, uint32_t timeout_ms)
{
#if 0
    while(gpiolib_get_value(((lr1110_t*)context)->busy) == 1)
    {
        ;
    }
#else
    uint32_t start = time_get_ticks();
    while(gpiolib_get_value(((lr1110_t*)context)->busy) == 1)
    {
        if((int32_t)(time_get_ticks() - start) > (int32_t) timeout_ms)
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_MODEM_HAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
