/*!
 * @file      lr1110_modem_board.c
 *
 * @brief     Target board LR1110 EVK Modem board driver implementation
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
#include <stdlib.h>
#include <gpiolib.h>
#include "../tracker.h"
#include "lr1110_modem_hal.h"
#include "lr1110_modem_system.h"
#include "lr1110_modem_lorawan.h"
#include "lr1110_modem_board.h"
#include "lr1110_modem_helper.h"
#include "lr1110.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define GNSS_WEEK_NUMBER_ROLLOVER_2019_2038 2

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief LR1110 EVK LED context
 */
typedef struct
{
//FIXME
#if 0
    timer_event_t led_timer;         /*!< @brief Pulse timer */
#endif
    bool          timer_initialized; /*!< @brief True if the pulse timer has been initialized, false otherwise */
} lr1110_modem_board_led_ctx_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief modem ready flag
 */
static bool modem_is_ready = false;

/*!
 * @brief LED1110 EVK LED context array
 */
//FIXME
#if 0
static lr1110_modem_board_led_ctx_t lr1110_modem_board_leds[LR1110_EVK_LED_COUNT] = { { .timer_initialized = false },
                                                                                      { .timer_initialized = false },
                                                                                      { .timer_initialized = false } };
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief initialize the TCXO
 *
 * @param [in] context Chip implementation context
 */
static lr1110_modem_response_code_t lr1110_modem_board_init_tcxo_io(const void* context);

/*!
 * @brief Pulse timer timeout callback
 *
 * @param context Context used to retrieve the index of the relevant LED.
 */
static void on_led_timer_event(void* context);

uint16_t hal_mcu_get_vref_level(void)
{
	return 0;
}

int16_t hal_mcu_get_temperature(void)
{
	return 0;
}

void lna_on(void)
{
	return;
}

void lna_off(void)
{
	return;
}


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_board_init_io_context(void* context)
{
}

void lr1110_modem_board_init_io(const void* context)
{
}

void lr1110_modem_board_deinit_io(const void* context)
{
}

void lr1110_modem_board_analog_deinit_io(const void* context)
{
}

uint32_t lr1110_modem_board_get_tcxo_wakeup_time(void)
{
	return BOARD_TCXO_WAKEUP_TIME;
}

lr1110_modem_response_code_t lr1110_modem_board_init(struct tracker *tracker, const void* context, lr1110_modem_event_callback_t* event)
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_hal_status_t    modem_hal_status    = LR1110_MODEM_HAL_STATUS_OK;

    radio_event_init(tracker, event);

    modem_hal_status = lr1110_modem_hal_reset(tracker, context);

    if(modem_hal_status != LR1110_MODEM_HAL_STATUS_OK)
    {
        /* Something goes wrong with the lr1110 modem-e */
        return LR1110_MODEM_RESPONSE_CODE_FAIL;
    }

    /* Initialize TCXO control */
    //modem_response_code |= lr1110_modem_board_init_tcxo_io(context);

    /* Initialize RF switch control */
    lr1110_modem_system_rf_switch_cfg_t rf_switch_cfg;
    rf_switch_cfg.enable = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH |
                           LR1110_MODEM_SYSTEM_RFSW2_HIGH | LR1110_MODEM_SYSTEM_RFSW3_HIGH;
    rf_switch_cfg.standby = 0;
    /* LoRa SPDT */
    rf_switch_cfg.rx = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW3_HIGH;
    rf_switch_cfg.tx = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH | LR1110_MODEM_SYSTEM_RFSW3_HIGH;
    rf_switch_cfg.tx_hp = LR1110_MODEM_SYSTEM_RFSW1_HIGH | LR1110_MODEM_SYSTEM_RFSW3_HIGH;
    /* GNSS LNA ON */
    rf_switch_cfg.gnss = LR1110_MODEM_SYSTEM_RFSW2_HIGH;

    modem_response_code |= lr1110_modem_system_set_dio_as_rf_switch(context, &rf_switch_cfg);

    /* Set Pa Config */
    modem_response_code |= lr1110_modem_set_rf_output(context, LR1110_MODEM_RADIO_PA_SEL_LP_HP_LF);

    modem_response_code |= lr1110_modem_system_set_reg_mode(context, LR1110_MODEM_SYSTEM_REG_MODE_LDO);

    /* XXX: Add low clock speed to use 32kHz */
    modem_response_code |= lr1110_modem_system_cfg_lfclk(&tracker->lr1110, LR1110_MODEM_SYSTEM_LFCLK_RC, 1);

    return modem_response_code;
}

uint32_t lr1110_modem_board_get_systime_from_gps(const void* context)
{
    uint32_t gps_time = 0;

    lr1110_modem_helper_get_utc_time(context, &gps_time);

    return gps_time;
}

lr1110_modem_response_code_t  lr1110_modem_get_almanac_dates(const void* context, uint32_t* oldest_almanac_date, uint32_t* newest_almanac_date)
{
    uint8_t                      i                   = 0;
    uint32_t                     almanac_date        = 0;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    *oldest_almanac_date = 0;
    *newest_almanac_date = 0;

    for(i = 0; i < 127; i++)
    {
        modem_response_code = lr1110_modem_helper_gnss_get_almanac_date_by_index(context, i, &almanac_date, GNSS_WEEK_NUMBER_ROLLOVER_2019_2038);
        if(modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK)
        {
            return modem_response_code;
        }

        if(almanac_date > 0)
        {
            if((*oldest_almanac_date == 0) && (*newest_almanac_date == 0))
            {
                *oldest_almanac_date = almanac_date;
                *newest_almanac_date = almanac_date;
            }
            else
            {
                if(almanac_date < *oldest_almanac_date)
                {
                    *oldest_almanac_date = almanac_date;
                }
                if(almanac_date > *newest_almanac_date)
                {
                    *newest_almanac_date = almanac_date;
                }
            }
        }
    }

    return modem_response_code;
}

void lr1110_modem_board_lna_on(void) 
{
	lna_on();
}

void lr1110_modem_board_lna_off(void)
{
	lna_off();
}

lr1110_modem_response_code_t lr1110_modem_board_event_flush(const void* context)
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_event_fields_t  event_fields;

    do
    {
        modem_response_code = lr1110_modem_get_event(context, &event_fields);
    } while(lr1110_modem_board_read_event_line(context) == 1);

    return modem_response_code;
}

bool lr1110_modem_board_read_event_line(const void* context)
{
    return (gpiolib_get_value(((lr1110_t *)context)->radio_event) ? true : false);
}

bool lr1110_modem_board_is_ready(void) { return modem_is_ready; }

void lr1110_modem_board_set_ready(bool ready) { modem_is_ready = ready; }

lr1110_modem_response_code_t lr1110_modem_board_measure_battery_drop(const void* context, int32_t* drop,
                                                                      uint32_t* time_recovery)
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_regions_t       region;
    uint32_t                     relaxed_voltage = 0;
    uint32_t                     tick_vdrop      = 0;

    relaxed_voltage = hal_mcu_get_vref_level();

    modem_response_code |= lr1110_modem_get_region(context, &region);

    /* Enter in test mode */
    modem_response_code |= lr1110_modem_test_mode_start(context);

    switch(region)
    {
    case LR1110_LORAWAN_REGION_EU868:
    case LR1110_LORAWAN_REGION_IN865:
    case LR1110_LORAWAN_REGION_RU864:
    {
        modem_response_code |= lr1110_modem_test_tx_cw(context, 865500000, 14);
        break;
    }
    case LR1110_LORAWAN_REGION_US915:
    case LR1110_LORAWAN_REGION_AU915:
    case LR1110_LORAWAN_REGION_AS923_GRP1:
    case LR1110_LORAWAN_REGION_AS923_GRP2:
    case LR1110_LORAWAN_REGION_AS923_GRP3:
    case LR1110_LORAWAN_REGION_KR920:
    {
        modem_response_code |= lr1110_modem_test_tx_cw(context, 920900000, 14);
        break;
    }
    default:
    {
        HAL_DBG_TRACE_ERROR("This region is not covered by this test\n\r");
        break;
    }
    }

    /* Wait the drop */
    time_usleep(2000 * 1000);

    /* Measure the drop */
    *drop = relaxed_voltage - hal_mcu_get_vref_level();

    /* Leave the test mode */
    lr1110_modem_test_nop(context);
    lr1110_modem_test_exit(context);

    HAL_DBG_TRACE_PRINTF("Battery voltage drop = %d mV\n\r", *drop);

    if(*drop > 0)
    {
        *time_recovery = 0;
        /* Get Start Tick*/
        tick_vdrop = time_get_ticks();
        /* Wait 66% of drop recovery */
        while((hal_mcu_get_vref_level() < relaxed_voltage - (*drop / 3)) && (*time_recovery < 10000))
        {
            *time_recovery = time_get_ticks() - tick_vdrop;
        }

        HAL_DBG_TRACE_PRINTF("Voltage recovery time = %d ms\n\r", *time_recovery);
    }

    return modem_response_code;
}

void lr1110_modem_board_led_set(uint32_t led_mask, bool turn_on)
{
    /* If a pulse timer is running on one of the requested LEDs, it
     *  must be stopped to avoid conflicting with the requested LED state. */
//FIXME
#if 0
    lr1110_evk_led_t led = LR1110_EVK_LED_TX;
    for(led = LR1110_EVK_LED_TX; led < LR1110_EVK_LED_COUNT; led++)
    {
        if(led_mask & (1 << led))
        {
            if((lr1110_modem_board_leds[led].timer_initialized) &&
                (timer_is_started(&lr1110_modem_board_leds[led].led_timer)))
            {
                timer_stop(&lr1110_modem_board_leds[led].led_timer);
            }
        }
    }
    if(turn_on)
    {
        leds_on(led_mask);
    }
    else
    {
        leds_off(led_mask);
    }
#endif
}

void lr1110_modem_board_led_pulse(uint32_t led_mask, bool turn_on, uint32_t duration_ms)
{
//FIXME
#if 0
    lr1110_evk_led_t led = LR1110_EVK_LED_TX;
    for(led = LR1110_EVK_LED_TX; led < LR1110_EVK_LED_COUNT; led++)
    {
        if(led_mask & (1 << led))
        {
            if(lr1110_modem_board_leds[led].timer_initialized)
            {
                if(timer_is_started(&lr1110_modem_board_leds[led].led_timer))
                {
                    timer_stop(&lr1110_modem_board_leds[led].led_timer);
                }
            }
            else
            {
                timer_init(&lr1110_modem_board_leds[led].led_timer, on_led_timer_event);
                timer_set_context(&lr1110_modem_board_leds[led].led_timer, (void*) led);
                lr1110_modem_board_leds[led].timer_initialized = true;
            }
            timer_set_value(&lr1110_modem_board_leds[led].led_timer, duration_ms);
            timer_start(&lr1110_modem_board_leds[led].led_timer); 
        }
    }
    if(turn_on)
    {
        leds_on(led_mask);
    }
    else
    {
        leds_off(led_mask);
    }
#endif
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr1110_modem_response_code_t lr1110_modem_board_init_tcxo_io(const void* context)
{
    return lr1110_modem_system_set_tcxo_mode(context, LR1110_MODEM_SYSTEM_TCXO_CTRL_1_8V,
                                              (lr1110_modem_board_get_tcxo_wakeup_time() * 1000) / 30.52);
}

void on_led_timer_event(void* context)
{
//FIXME
#if 0
    lr1110_evk_led_t led      = (lr1110_evk_led_t) context;
    uint32_t         led_mask = 1 << led;
    leds_toggle(led_mask);
    timer_stop(&lr1110_modem_board_leds[led].led_timer);
#endif
}

/* --- EOF ------------------------------------------------------------------ */
