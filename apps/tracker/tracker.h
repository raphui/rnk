#ifndef TRACKER_H
#define TRACKER_H

#include <stdio.h>
#include <stdint.h>

#include "cmath.h"
#include "tracker_utility.h"
#include "modem/lr1110.h"
#include "modem/lr1110_modem_lorawan.h"

/*!
 * @brief Use or not the Semtech join server
 */
#define USE_SEMTECH_JOIN_SERVER 1

/*!
 * @brief Use or not the LoRaWAN production Keys. If not, keys are described below
 */
#define USE_PRODUCTION_KEYS 1

/*!
 * @brief IEEE Organizationally Unique Identifier ( OUI ) (big endian)
 * @remark This is unique to a company or organization
 */
#define IEEE_OUI 0x00, 0x00, 0x00

/*!
 * @brief Device IEEE EUI (big endian)
 *
 * @remark In this application the value is automatically generated by calling
 *         lr1110_modem_get_dev_eui function
 */
#define LORAWAN_DEVICE_EUI                     \
    {                                          \
        IEEE_OUI, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#define LORAWAN_DEVICE_EUI_LEN 8

/*!
 * @brief App/Join server IEEE EUI (big endian)
 */
#define LORAWAN_JOIN_EUI                               \
    {                                                  \
        0x00, 0x16, 0xC0, 0x01, 0xFF, 0xFE, 0x00, 0x01 \
    }
#define LORAWAN_JOIN_EUI_LEN 8

/*!
 * @brief loRaWAN Application Key (big endian)
 */
#define LORAWAN_APP_KEY                                                                                \
    {                                                                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#define LORAWAN_APP_KEY_LEN 16



/*!
 * @brief Defines the duty cycle threshold where the application will not send the results, results will be only logged
 * into flash memory if internal log is enable, 4.5s , value in [ms].
 */
#define TRACKER_DUTY_CYCLE_THRESHOLD 4500

/*!
 * @brief Defines the stream redundancy rate of the tracker application
 */
#define TRACKER_STREAM_REDUNDANCY_RATE 110

/*!
 * @brief Defines the application scan interval
 * when device has moved, 120s (2min), value in [ms].
 */
#define TRACKER_SCAN_INTERVAL 120000

/*!
 * @brief Defines the application keep alive frame interval
 * when device doesn't move, 3600s, value in [ms].
 */
#define TRACKER_KEEP_ALIVE_FRAME_INTERVAL 3600000

/*!
 * @brief Defines the application data transmission duty cycle counter.
 * when device doesn't move.
 */
#define TRACKER_APP_TX_LOW_DUTYCYCLE_CTN TRACKER_KEEP_ALIVE_FRAME_INTERVAL / TRACKER_SCAN_INTERVAL

/*!
 * @brief Defines the application firmware version
 */
#define TRACKER_MAJOR_APP_VERSION 0
#define TRACKER_MINOR_APP_VERSION 1
#define TRACKER_SUB_MINOR_APP_VERSION 0

/*!
 * @brief Time during which an LED is turned on when a TX or RX event occurs, in milliseconds.
 */
#define LED_PERIOD_MS 250

/*!
 * @brief LoRaWAN application TLV Tag
 */
#define TLV_GNSS_NAV_TAG 0x07
#define TLV_WIFI_SCAN_TAG 0x0E
#define TLV_RESET_COUNTER_TAG 0x0C
#define TLV_SENSORS_TAG 0x0D
#define TLV_TRACKER_SETTINGS_TAG 0x4C

/*!
 * @brief LoRaWAN application TLV Len
 */
#define TLV_WIFI_SINGLE_BEACON_LEN 0x07
#define TLV_SENSOR_BASIC_VERSION_LEN 0x01
#define TLV_SENSOR_FULL_VERSION_LEN 0x07

/*!
 * @brief LoRaWAN application TLV Sensors version
 */
#define TLV_WIFI_VERSION 0x01
#define TLV_SENSOR_BASIC_VERSION 0x00
#define TLV_SENSOR_FULL_VERSION 0x01

/*!
 * @brief LoRaWAN stream application port
 */
#define LORAWAN_STREAM_APP_PORT 199

/*!
 * @brief LoRaWAN port used to the gnss push solver messages
 */
#define GNSS_PUSH_SOLVER_MSG_PORT 150

/*!
 * @brief LoRaWAN port used to trigger tracker events
 */
#define TRACKER_REQUEST_MSG_PORT 151

/*!
 * @brief TLV used to execute application events
 */
#define SET_RX_LED_CMD 0x4F
#define SET_RX_LED_LEN 0x01

/*!
 * @brief Define the number of time where the tracker send a scan result once static
 */
#define TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC 0x03
#define TRACKER_SEND_TWO_MORE_SCANS_ONCE_STATIC 0x07
#define TRACKER_SEND_THRE_MORE_SCANS_ONCE_STATIC 0x0F

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS (LoRaWAN configuration) --------------------------------
 */

/*!
 * @brief LoRaWAN regulatory region.
 * One of:
 * LR1110_LORAWAN_REGION_EU868
 * LR1110_LORAWAN_REGION_US915
 * LR1110_LORAWAN_REGION_AU915
 * LR1110_LORAWAN_REGION_AS923_GRP1
 * LR1110_LORAWAN_REGION_CN470
 * LR1110_LORAWAN_REGION_AS923_GRP2
 * LR1110_LORAWAN_REGION_AS923_GRP3
 * LR1110_LORAWAN_REGION_IN865
 * LR1110_LORAWAN_REGION_KR920
 * LR1110_LORAWAN_REGION_RU864
 */
#define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_EU868

/*!
 * @brief LoRaWAN regulatory region country. define LoRaWAN subregion countries to activate or not the LBT, 0 means disable, 1 means enable
 */
#define LORAWAN_COUNTRY_JAPAN 0

/*!
 * @brief LoRaWAN class.
 * One of:
 *  LR1110_LORAWAN_CLASS_A
 *  LR1110_LORAWAN_CLASS_C
 */
#define LORAWAN_CLASS_USED LR1110_LORAWAN_CLASS_A

/*!
 * @brief LoRaWAN ETSI duty cycle control enable/disable
 * Supported values:
 *  LR1110_MODEM_DUTY_CYCLE_ENABLE
 *  LR1110_MODEM_DUTY_CYCLE_DISABLE
 *
 * @remark Please note that ETSI mandates duty cycled transmissions. Set to false only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON LR1110_MODEM_DUTY_CYCLE_ENABLE

/*!
 * @brief Datarate when device is static and mobile
 * Supported values:
 *  LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED
 *  LR1110_MODEM_ADR_PROFILE_MOBILE_LONG_RANGE
 *  LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER
 *  LR1110_MODEM_ADR_PROFILE_CUSTOM
 */
#define LORAWAN_STATIC_DATARATE LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED
#define LORAWAN_MOBILE_DATARATE LR1110_MODEM_ADR_PROFILE_MOBILE_LOW_POWER

#define BOARD_TCXO_WAKEUP_TIME 5

#define FORCE_NEW_TRACKER_CONTEXT 0

#define HAL_DBG_TRACE_INFO(...) do{ printf(__VA_ARGS__); }while(0)
#define HAL_DBG_TRACE_PRINTF(...) do{ printf(__VA_ARGS__); }while(0)
#define HAL_DBG_TRACE_MSG(...) do{ printf(__VA_ARGS__); }while(0)
#define HAL_DBG_TRACE_ERROR(...) do{ printf(__VA_ARGS__); }while(0)
#define HAL_DBG_TRACE_WARNING(...) do{ printf(__VA_ARGS__); }while(0)

void tracker_wifi_run_scan(struct tracker *tracker, const wifi_settings_t* wifi_settings, wifi_scan_selected_result_t* wifi_result);
uint8_t tracker_gnss_get_next_nb_sat(struct tracker *tracker);
void tracker_gnss_run_scan(struct tracker *tracker, const gnss_settings_t* gnss_settings, gnss_scan_single_result_t* capture_result);
lr1110_modem_response_code_t tracker_gnss_init(struct tracker *tracker, const gnss_settings_t* gnss_settings);
void tracker_gnss_store_new_assistance_position(struct tracker *tracker);
lr1110_modem_response_code_t lorawan_init(struct tracker *tracker, lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class);
bool tracker_app_start_scan(struct tracker *tracker, const tracker_scan_priority_t scan_piority);
void tracker_app_reset_scan_results(struct tracker *tracker);
void tracker_app_build_and_stream_payload(struct tracker *tracker, bool send_complete_sensors);
void tracker_app_build_and_stream_tracker_settings(struct tracker *tracker, const uint8_t* buffer, uint8_t len);
void tracker_app_store_new_acculated_charge(struct tracker *tracker, uint32_t modem_charge);
void tracker_app_join_network(struct tracker *tracker);
void tracker_app_adapt_adr(struct tracker *tracker);
bool tracker_app_add_payload_in_streaming_fifo(struct tracker *tracker, const uint8_t* payload, uint16_t len);
bool tracker_app_is_next_scan_possible(struct tracker *tracker);
bool tracker_app_is_region_use_duty_cycle(struct tracker *tracker, lr1110_modem_regions_t region);
bool tracker_app_is_tracker_in_static_mode(struct tracker *tracker);
void tracker_app_parse_downlink_frame(struct tracker *tracker, uint8_t port, const uint8_t* payload, uint8_t size);

enum {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_COLLECT_DATA,
    DEVICE_STATE_SLEEP
};


struct tracker {
	uint8_t *adr_custom_list;
	int device_state;
	lr1110_t lr1110;
	lr1110_modem_event_callback_t lr1110_modem_event_callback;
	tracker_ctx_t tracker_ctx;
};

#endif /* TRACKER_H */
