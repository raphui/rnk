#include <errno.h>
#include <string.h>

#include "../tracker.h"
#include "lr1110_modem_board.h"

void print_hex_buffer( const uint8_t* buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            HAL_DBG_TRACE_PRINTF( "\r\n" );
            newline = 0;
        }

        HAL_DBG_TRACE_PRINTF( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
}

void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key, uint32_t pin,
                         const bool use_semtech_join_server )
{
    HAL_DBG_TRACE_PRINTF( "DevEui      : %02X", dev_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", dev_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    HAL_DBG_TRACE_PRINTF( "AppEui      : %02X", join_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", join_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    if( use_semtech_join_server )
    {
        HAL_DBG_TRACE_MSG( "AppKey      : Semtech join server used\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "AppKey      : %02X", app_key[0] );
        for( int i = 1; i < 16; i++ )
        {
            HAL_DBG_TRACE_PRINTF( "-%02X", app_key[i] );
        }
        HAL_DBG_TRACE_PRINTF( "\r\n" );
    }

    HAL_DBG_TRACE_PRINTF( "Pin         : %08X\r\n\r\n", pin );
}

void modem_status_to_string( lr1110_modem_status_t modem_status )
{
    HAL_DBG_TRACE_MSG( "Modem status : " );

    if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
    {
        HAL_DBG_TRACE_MSG( "CRASH " );
    }
    if( ( modem_status & LR1110_LORAWAN_MUTE ) == LR1110_LORAWAN_MUTE )
    {
        HAL_DBG_TRACE_MSG( "MUTE " );
    }
    if( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED )
    {
        HAL_DBG_TRACE_MSG( "JOINED " );
    }
    if( ( modem_status & LR1110_LORAWAN_SUSPEND ) == LR1110_LORAWAN_SUSPEND )
    {
        HAL_DBG_TRACE_MSG( "SUSPEND " );
    }
    if( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD )
    {
        HAL_DBG_TRACE_MSG( "UPLOAD " );
    }
    if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
    {
        HAL_DBG_TRACE_MSG( "JOINING " );
    }
    if( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM )
    {
        HAL_DBG_TRACE_MSG( "STREAM " );
    }

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
}


void lr1110_modem_reset_event(struct tracker *tracker, uint16_t reset_count)
{
    HAL_DBG_TRACE_INFO("###### ===== LR1110 MODEM-E RESET %lu ==== ######\r\n\r\n", reset_count);

    if (lr1110_modem_board_is_ready() == true) {
        tracker->tracker_ctx.modem_reset_by_itself_cnt++;
        tracker_store_and_reset(1 + lr1110_modem_board_read_event_line(&tracker->lr1110));
    } else {
        lr1110_modem_board_set_ready(true);
    }
}

void lr1110_modem_network_joined(struct tracker *tracker)
{
    HAL_DBG_TRACE_INFO("###### ===== JOINED ==== ######\r\n\r\n");

    /* Set the ADR profile once joined */
    lr1110_modem_set_adr_profile(&tracker->lr1110, (lr1110_modem_adr_profiles_t)tracker->tracker_ctx.lorawan_adr_profile, tracker->adr_custom_list);
    if (tracker->tracker_ctx.lorawan_adr_profile == LR1110_MODEM_ADR_PROFILE_CUSTOM) {
        lr1110_modem_set_nb_trans(&tracker->lr1110, 2);
    }

    /* Init the stream once joined */
    lr1110_modem_stream_init(&tracker->lr1110, LORAWAN_STREAM_APP_PORT, LR1110_MODEM_SERVICES_ENCRYPTION_DISABLE);
    /* Set the stream redundancy rate */
    lr1110_modem_set_stream_redundancy_rate(&tracker->lr1110, TRACKER_STREAM_REDUNDANCY_RATE);
}

void lr1110_modem_join_fail(struct tracker *tracker)
{
	HAL_DBG_TRACE_INFO("###### ===== JOIN FAIL ==== ######\r\n\r\n");
}

void lr1110_modem_alarm(struct tracker *tracker)
{
    lr1110_modem_status_t        modem_status;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO("###### ===== LR1110 ALARM ==== ######\r\n\r\n");

    modem_response_code = lr1110_modem_get_status(&tracker->lr1110, &modem_status);

    if(modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK) {
        modem_status_to_string(modem_status);

        if (((modem_status & LR1110_LORAWAN_JOINED) == LR1110_LORAWAN_JOINED) ||
            ((modem_status & LR1110_LORAWAN_STREAM) == LR1110_LORAWAN_STREAM) ||
            ((modem_status & LR1110_LORAWAN_UPLOAD) == LR1110_LORAWAN_UPLOAD))
        {
            tracker->device_state = DEVICE_COLLECT_DATA;
        } else if ((modem_status & LR1110_LORAWAN_JOINING) == LR1110_LORAWAN_JOINING) {
            /* Network not joined yet. Wait */
            tracker->device_state = DEVICE_STATE_CYCLE;
        } else {
            HAL_DBG_TRACE_WARNING("Unknow modem status %d\r\n\r\n", modem_status);
            tracker->device_state = DEVICE_STATE_CYCLE;
        }
    }
}

void lr1110_modem_down_data(struct tracker *tracker, int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port, const uint8_t* payload, uint8_t size)
{
    static uint32_t downlink_cnt = 0;
    HAL_DBG_TRACE_INFO("\r\n###### ===== DOWNLINK FRAME %lu ==== ######\r\n\r\n", downlink_cnt++);

    HAL_DBG_TRACE_PRINTF("RX WINDOW   : %d\r\n", flags);

    HAL_DBG_TRACE_PRINTF("RX PORT     : %d\r\n", port);

    if (size != 0) {
        HAL_DBG_TRACE_MSG("RX DATA     : ");
        print_hex_buffer(payload, size);
    }

    HAL_DBG_TRACE_PRINTF("RX RSSI     : %d\r\n", rssi);
    HAL_DBG_TRACE_PRINTF("RX SNR      : %d\r\n\r\n", snr);

    /* Update the system_sanity_check bit field */
    tracker->tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

//FIXME
#if 0
    leds_on(LED_RX_MASK);
    timer_start(&led_rx_timer);
#endif

    tracker_app_parse_downlink_frame(tracker, port, payload, size);
}

void lr1110_modem_mute(struct tracker *tracker, lr1110_modem_mute_t mute)
{
    if(mute == LR1110_MODEM_UNMUTED) {
        HAL_DBG_TRACE_INFO("###### ===== MODEM UNMUTED ==== ######\r\n\r\n");
    } else {
        HAL_DBG_TRACE_INFO("###### ===== MODEM MUTED ==== ######\r\n\r\n");
    }
}

void lr1110_modem_set_conf(struct tracker *tracker, lr1110_modem_event_setconf_tag_t tag)
{
    HAL_DBG_TRACE_INFO("###### ===== MODEM SET CONF %02X ==== ######\r\n\r\n", tag);
}

void lr1110_modem_stream_done(struct tracker *tracker)
{
    static uint32_t stream_cnt = 0;
    HAL_DBG_TRACE_INFO("###### ===== STREAM DONE nb %d ==== ######\r\n\r\n", stream_cnt++);

    tracker->tracker_ctx.stream_done = true;
}

void lr1110_modem_time_updated_alc_sync(struct tracker *tracker, lr1110_modem_alc_sync_state_t alc_sync_state)
{
    HAL_DBG_TRACE_INFO("###### ===== APPLICATION LAYER CLOCK SYNC EVENT ==== ######\r\n\r\n");

    /* Update the system_sanity_check bit field */
    tracker->tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

    if (alc_sync_state == LR1110_MODEM_ALC_SYNC_SYNCHRONIZED) {
        HAL_DBG_TRACE_MSG("CLOCK SYNC STATE SYNCHRONIZED\r\n\r\n");
        /* Notify user that the date has been received */
//FIXME
#if 0
        leds_blink(LED_RX_MASK, 100, 4, true);
#endif
        tracker->tracker_ctx.has_date = true;
    } else {
        HAL_DBG_TRACE_MSG("CLOCK SYNC STATE DESYNCHRONIZED\r\n\r\n");
        /* Notify user that the date has been received */
//FIXME
#if 0
        leds_blink(LED_TX_MASK, 100, 4, true);
#endif
        tracker->tracker_ctx.has_date = false;
    }
}

void lr1110_modem_adr_mobile_to_static(struct tracker *tracker)
{
    HAL_DBG_TRACE_INFO("###### ===== ADR HAS SWITCHED FROM MOBILE TO STATIC ==== ######\r\n\r\n");
}

void lr1110_modem_new_link_adr(struct tracker *tracker)
{
	HAL_DBG_TRACE_INFO("###### ===== NEW LINK ADR ==== ######\r\n\r\n");
}

void lr1110_modem_no_event(struct tracker *tracker)
{
	HAL_DBG_TRACE_INFO("###### ===== NO EVENT ==== ######\r\n\r\n");
}
