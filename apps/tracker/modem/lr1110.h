#ifndef LR1110_H
#define LR1110_H

#include <time.h>
#include <stdint.h>
#include <gpiolib.h>
#include <drv/mtd.h>

#include "../lis2de12.h"
#include "lr1110_modem_lorawan.h"

struct tracker;

/*!
 * @brief LR1110 modem-e callback functions
 */
typedef struct
{
    /*!
     * @brief  Reset callback prototype.
     *
     * @param [in] reset_count
     */
    void (*reset)(struct tracker *tracker, uint16_t reset_count);
    /*!
     * @brief  Alarm timer expired callback prototype.
     */
    void (*alarm)(struct tracker *tracker);
    /*!
     * @brief  Attemp to join network failed callback prototype.
     */
    void (*joined)(struct tracker *tracker);
    /*!
     * @brief  Joined callback prototype.
     */
    void (*join_fail)(struct tracker *tracker);
    /*!
     * @brief  Tx done callback prototype.
     *
     * @param [in] status
     */
    void (*tx_done)(struct tracker *tracker, lr1110_modem_tx_done_event_t status);
    /*!
     * @brief Downlink data received callback prototype.
     *
     * @param [in] rssi    rssi in signed value in dBm + 64
     * @param [in] snr     snr signed value in 0.25 dB steps
     * @param [in] flags   rx flags \see down_data_flag_t
     * @param [in] port    LoRaWAN port
     * @param [in] payload Received buffer pointer
     * @param [in] size    Received buffer size
     */
    void (*down_data)(struct tracker *tracker, int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port,
                         const uint8_t* payload, uint8_t size);
    /*!
     * @brief  File upload completed callback prototype.
     *
     * @param [in] upload status \see lr1110_modem_upload_event_t
     */
    void (*upload_done)(struct tracker *tracker, lr1110_modem_upload_event_t upload_status);
    /*!
     * @brief  Set conf changed by DM callback prototype.
     *
     * @param [in] tag \see lr1110_modem_event_setconf_tag_t
     */
    void (*set_conf)(struct tracker *tracker, lr1110_modem_event_setconf_tag_t tag);
    /*!
     * @brief  Mute callback prototype.
     *
     * @param [in] mute
     */
    void (*mute)(struct tracker *tracker, lr1110_modem_mute_t mute);
    /*!
     * @brief  Data stream fragments sent callback prototype.
     */
    void (*stream_done)(struct tracker *tracker);
    /*!
     * @brief  Gnss Done Done callback prototype.
     *
     * @param [in] nav_message
     * @param [in] size
     */
    void (*gnss_scan_done)(struct tracker *tracker, uint8_t* nav_message, uint16_t size);
    /*!
     * @brief  Gnss Done Done callback prototype.
     *
     * @param [in] scan buffer containing the raw data coming from the scan
     * @param [in] size of the raw buffer
     */
    void (*wifi_scan_done)(struct tracker *tracker, uint8_t* scan, uint16_t size);
    /*!
     * @brief  Time Updated by application layer clock synchronization callback prototype.
     *
     * @param [in] alc_sync_state \ref lr1110_modem_alc_sync_state_t
     */
    void (*time_updated_alc_sync)(struct tracker *tracker, lr1110_modem_alc_sync_state_t alc_sync_state);
    /*!
     * @brief  Automatic switch from mobile to static ADR when connection timeout occurs callback prototype.
     */
    void (*adr_mobile_to_static)(struct tracker *tracker);
    /*!
     * @brief  New link ADR request callback prototype.
     */
    void (*new_link_adr)(struct tracker *tracker);
    /*!
     * @brief  No event exists callback prototype.
     */
    void (*no_event)(struct tracker *tracker);
} lr1110_modem_event_callback_t;

/*!
 * @brief Radio hardware and global parameters
 */
typedef struct lr1110_s
{
    timer_t reset_timeout_timer;
    timer_t gnss_scan_timeout_timer;
    timer_t wifi_scan_timeout_timer;
    timer_t led_timer;
    struct pio_desc *led_rx;
    struct pio_desc *led_tx;
    struct pio_desc *led_scan;
    struct pio_desc *radio_event;
    struct pio_desc *busy;
    struct pio_desc *acc_irq;
    bool accelerometer_irq1_state;
    uint8_t who_am_i;
    axis3bit16_t data_raw_acceleration;
    float acceleration_mg[3];
    uint32_t spi_id;
    uint32_t mtd_id;
    uint32_t i2c_id;
} lr1110_t;

/*!
 * @brief Hardware IO IRQ callback function definition
 */
typedef void (*lr1110_dio_irq_handler)(void* context);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init the LR1110 modem-e event callbacks
 *
 * @param [in] event lr1110 modem-e event callback \ref lr1110_modem_event_callback_t
 */
void radio_event_init(struct tracker *tracker, lr1110_modem_event_callback_t* event);

/*!
 * @brief Callback when event occurs
 */
void radio_event_callback(void* obj);

/*!
 * @brief Process the analysis of radio event and calls callback functions
 *        depending on event
 *
 * @param [in] context Radio abstraction
 *
 */
void lr1110_modem_event_process(struct tracker *tracker, const void* context);

#endif /* LR1110_H */
