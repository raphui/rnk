#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#include "tracker.h"
#include "lis2de12.h"
#include "modem/lr1110.h"
#include "modem/lr1110_modem.h"
#include "modem/lr1110_modem_board.h"
#include "modem/lr1110_modem_helper.h"

#ifdef CONFIG_STM32L476
/* PC0 */
#define LED_RX_PIN	33
/* PC1 */
#define LED_TX_PIN	34
/* PB5 */
#define LED_SCAN_PIN	22
/* PB4 */
#define RADIO_EVENT_PIN	21
/* PB3 */
#define BUSY_PIN	20
/* PA9 */
#define ACC_IRQ_PIN	10
/* PB0 */
#define LNA_PIN		17

/* PC13 */
#define USER_BUTTON_PIN	46
#else
/* PA8 */
#define LED_PIN		9
/* PB4 */
#define RADIO_EVENT_PIN	21
/* PA15 */
#define BUSY_PIN	16
/* PA2 */
#define ACC_IRQ_PIN	3
/* PB5 */
#define LNA_PIN		22
#endif

#define SPI_DEVICE	"/dev/spi1"
#define MTD_DEVICE	"/dev/mtd1"
#define I2C_DEVICE	"/dev/i2c1"

extern uint16_t hal_mcu_get_vref_level(void);
extern int16_t hal_mcu_get_temperature(void);
extern void print_lorawan_keys(const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key, uint32_t pin, const bool use_semtech_join_server);

uint8_t adr_custom_list[16] = { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
	0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01 };

static void lr1110_modem_init(struct tracker *tracker)
{
	unsigned char buffer[PAGE_SIZE / 0x10];
	uint32_t start_addr = TRACKER_INTERNAL_LOG_START;
	uint32_t end_addr = TRACKER_INTERNAL_LOG_END;
	int nb_empty_bytes = 0;

	lseek(tracker->lr1110.mtd_id, start_addr, SEEK_SET);

	while ((nb_empty_bytes != PAGE_SIZE) && (start_addr < end_addr)) {
		nb_empty_bytes = 0;

		for (int j = 0; j < 0x10; j++) {
			read(tracker->lr1110.mtd_id, buffer, sizeof(buffer));

			for (int i = 0; i < sizeof(buffer); i++) {
				if (buffer[i] == 0xFF) {
					nb_empty_bytes++;
				}
			}
		}
		start_addr += PAGE_SIZE;
	}

	/* XXX: (- PAGE_SIZE) is because in any case we leave the while loop by adding PAGE_SIZE to start_addr */
	tracker->internal_log_start_addr = start_addr - PAGE_SIZE;

	tracker->adr_custom_list = adr_custom_list;
	tracker->device_state = DEVICE_STATE_INIT;
#ifdef CONFIG_STM32L476
	tracker->lr1110.led_rx = gpiolib_export(LED_RX_PIN);
	tracker->lr1110.led_tx = gpiolib_export(LED_TX_PIN);
	tracker->lr1110.led_scan = gpiolib_export(LED_SCAN_PIN);
#else
	tracker->lr1110.led_rx = gpiolib_export(LED_PIN);
	tracker->lr1110.led_tx = tracker->lr1110.led_rx;
	tracker->lr1110.led_scan = tracker->lr1110.led_rx;
#endif
	tracker->lr1110.radio_event = gpiolib_export(RADIO_EVENT_PIN);
	tracker->lr1110.busy = gpiolib_export(BUSY_PIN);
	tracker->lr1110.acc_irq = gpiolib_export(ACC_IRQ_PIN);
	tracker->lr1110.lna = gpiolib_export(LNA_PIN);

	sem_init(&tracker->lr1110.radio_event_sem, 1);
	sem_init(&tracker->lr1110.event_processed_sem, 1);
	sem_init(&tracker->timer_sem, 1);

	gpiolib_set_output(tracker->lr1110.led_rx, 0);
	gpiolib_set_output(tracker->lr1110.led_tx, 0);
	gpiolib_set_output(tracker->lr1110.led_scan, 0);
	gpiolib_set_output(tracker->lr1110.lna, 0);
	gpiolib_set_input(tracker->lr1110.radio_event);
	gpiolib_set_input(tracker->lr1110.busy);
	gpiolib_set_input(tracker->lr1110.acc_irq);

	gpiolib_request_irq(tracker->lr1110.radio_event, radio_event_callback, IRQF_RISING, tracker);
	gpiolib_request_irq(tracker->lr1110.acc_irq, lis2de12_int1_irq_handler, IRQF_RISING, tracker);

	/* Init LR1110 modem-e event */
	memset(&tracker->lr1110_modem_event_callback, 0, sizeof(lr1110_modem_event_callback_t));
	tracker->lr1110_modem_event_callback.reset = lr1110_modem_reset_event;
	tracker->lr1110_modem_event_callback.alarm = lr1110_modem_alarm;
	tracker->lr1110_modem_event_callback.joined = lr1110_modem_network_joined;
	tracker->lr1110_modem_event_callback.join_fail = lr1110_modem_join_fail;
	tracker->lr1110_modem_event_callback.down_data = lr1110_modem_down_data;
	tracker->lr1110_modem_event_callback.set_conf = lr1110_modem_set_conf;
	tracker->lr1110_modem_event_callback.mute = lr1110_modem_mute;
	tracker->lr1110_modem_event_callback.gnss_scan_done = lr1110_modem_gnss_scan_done;
	tracker->lr1110_modem_event_callback.wifi_scan_done = lr1110_modem_wifi_scan_done;
	tracker->lr1110_modem_event_callback.stream_done = lr1110_modem_stream_done;
	tracker->lr1110_modem_event_callback.time_updated_alc_sync = lr1110_modem_time_updated_alc_sync;
	tracker->lr1110_modem_event_callback.adr_mobile_to_static = lr1110_modem_adr_mobile_to_static;
	tracker->lr1110_modem_event_callback.new_link_adr = lr1110_modem_new_link_adr;
	tracker->lr1110_modem_event_callback.no_event = lr1110_modem_no_event;

	pthread_create(&tracker->event_thread, lr1110_modem_event_process, tracker, 1);

	accelerometer_init(tracker, 1);

#if 0
	while (1) {
		printf("accelerometer: %d\n", is_accelerometer_detected_moved(tracker) ? 1 : 0);
	}
#endif
}

void radio_event_callback(void* obj)
{
	struct tracker *tracker = (struct tracker *)obj;

	sem_post(&tracker->lr1110.radio_event_sem);
}

void radio_event_init(struct tracker *tracker, lr1110_modem_event_callback_t* event)
{
	return;
}

void lr1110_modem_event_process(void *arg)
{
	struct tracker *tracker = (struct tracker *)arg;
	void *context = (void *)&tracker->lr1110;
	lr1110_modem_helper_status_t modem_response_code = LR1110_MODEM_HELPER_STATUS_OK;
        lr1110_modem_event_t modem_event;
	lr1110_modem_event_callback_t *lr1110_modem_event_callback = &tracker->lr1110_modem_event_callback;

	while (1) {
		sem_wait(&tracker->lr1110.radio_event_sem);

		do {
			modem_response_code = lr1110_modem_helper_get_event_data(context, &modem_event);

			if (modem_response_code == LR1110_MODEM_HELPER_STATUS_OK) {
				
				printf("%s: modem_event.event_type = %d\n", __func__, modem_event.event_type);
				switch(modem_event.event_type)
				{
				case LR1110_MODEM_LORAWAN_EVENT_RESET:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->reset != NULL)) {
						lr1110_modem_event_callback->reset(tracker, modem_event.event_data.reset.count);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_ALARM:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->alarm != NULL)) {
						lr1110_modem_event_callback->alarm(tracker);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_JOINED:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->joined != NULL)) {
						lr1110_modem_event_callback->joined(tracker);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_JOIN_FAIL:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->join_fail != NULL)) {
						lr1110_modem_event_callback->join_fail(tracker);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_TX_DONE:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->tx_done != NULL)) {
						lr1110_modem_event_callback->tx_done(tracker, modem_event.event_data.txdone.status);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_DOWN_DATA:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->down_data != NULL)) {
						lr1110_modem_event_callback->down_data(tracker,
							modem_event.event_data.downdata.rssi, modem_event.event_data.downdata.snr,
							modem_event.event_data.downdata.flag, modem_event.event_data.downdata.fport,
							modem_event.event_data.downdata.data, modem_event.event_data.downdata.length);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_UPLOAD_DONE:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->upload_done != NULL)) {
						lr1110_modem_event_callback->upload_done(tracker, modem_event.event_data.upload.status);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_SET_CONF:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->set_conf != NULL)) {
						lr1110_modem_event_callback->set_conf(tracker, modem_event.event_data.setconf.tag);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_MUTE:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->mute != NULL)) {
						lr1110_modem_event_callback->mute(tracker, modem_event.event_data.mute.status);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_STREAM_DONE:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->stream_done != NULL)) {
						lr1110_modem_event_callback->stream_done(tracker);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_WIFI_SCAN_DONE:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->wifi_scan_done != NULL)) {
						lr1110_modem_event_callback->wifi_scan_done(tracker, modem_event.event_data.wifi.buffer,
										     modem_event.event_data.wifi.len);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_GNSS_SCAN_DONE:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->gnss_scan_done != NULL)) {
						lr1110_modem_event_callback->gnss_scan_done(tracker, modem_event.event_data.gnss.nav_message,
											     modem_event.event_data.gnss.len);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_TIME_UPDATED_ALC_SYNC:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->time_updated_alc_sync != NULL)) {
						lr1110_modem_event_callback->time_updated_alc_sync(tracker, modem_event.event_data.time.status);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_ADR_MOBILE_TO_STATIC:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->adr_mobile_to_static != NULL)) {
						lr1110_modem_event_callback->adr_mobile_to_static(tracker);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_NEW_LINK_ADR:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->new_link_adr != NULL)) {
						lr1110_modem_event_callback->new_link_adr(tracker);
					}
					break;
				case LR1110_MODEM_LORAWAN_EVENT_NO_EVENT:
					if ((lr1110_modem_event_callback != NULL) && (lr1110_modem_event_callback->no_event != NULL)) {
						lr1110_modem_event_callback->no_event(tracker);
					}
					break;
				default:
					break;
				}

			} else {
				HAL_DBG_TRACE_ERROR("lr1110_modem_helper_get_event_data RC = %d\r\n\r\n", modem_response_code);
			}
		} while ((lr1110_modem_board_read_event_line(context) == 1) && (modem_response_code == LR1110_MODEM_HELPER_STATUS_OK));

		sem_post(&tracker->lr1110.event_processed_sem);
	}
}


int main(void)
{
	int ret = 0;
	int32_t duty_cycle;
	bool send_complete_sensors = false;
	struct tracker *tracker = NULL;
	lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
	lr1110_modem_event_callback_t lr1110_modem_event_callback;
	lr1110_modem_lorawan_state_t lorawan_state;

	uint8_t dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
	uint8_t join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
	uint8_t app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

	printf("Starting tracker app\n");
	printf("app version: %d.%d.%d\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION, TRACKER_SUB_MINOR_APP_VERSION);

	tracker = malloc(sizeof(*tracker));
	if (!tracker) {
		printf("failed to alloc tracker structure\n");
		ret = -ENOMEM;
		goto err;
	}

	memset(tracker, 0, sizeof(*tracker));

	tracker->lr1110.spi_id = open(SPI_DEVICE, O_RDWR);
	if (tracker->lr1110.spi_id < 0) {
		printf("failed to open: %s\n", SPI_DEVICE);
		goto err;
	}

	tracker->lr1110.mtd_id = open(MTD_DEVICE, O_RDWR);
	if (tracker->lr1110.mtd_id < 0) {
		printf("failed to open: %s\n", MTD_DEVICE);
		goto err_close;
	}
	
	tracker->lr1110.i2c_id = open(I2C_DEVICE, O_RDWR);
	if (tracker->lr1110.mtd_id < 0) {
		printf("failed to open: %s\n", I2C_DEVICE);
		goto err_close;
	}

	lr1110_modem_board_init_io_context(&tracker->lr1110);

	lr1110_modem_board_init_io(&tracker->lr1110);

	lr1110_modem_init(tracker);

	if (lr1110_modem_board_init(tracker, &tracker->lr1110, &lr1110_modem_event_callback) != LR1110_MODEM_RESPONSE_CODE_OK)
	{
		HAL_DBG_TRACE_ERROR("###### ===== LR1110 BOARD INIT FAIL ==== ######\r\n\r\n");
	}

	/* LR1110 modem-e version */
	lr1110_modem_get_version(&tracker->lr1110, &tracker->tracker_ctx.modem_version);
	HAL_DBG_TRACE_INFO("###### ===== LR1110 MODEM-E VERSION ==== ######\r\n\r\n");
	HAL_DBG_TRACE_PRINTF("LORAWAN     : %#04X\r\n", tracker->tracker_ctx.modem_version.lorawan);
	HAL_DBG_TRACE_PRINTF("FIRMWARE    : %#02X\r\n", tracker->tracker_ctx.modem_version.firmware);
	HAL_DBG_TRACE_PRINTF("BOOTLOADER  : %#02X\r\n", tracker->tracker_ctx.modem_version.bootloader);

	/* Store or restore the Tracker context */
	if ((tracker_restore_app_ctx(tracker) != 1) || (FORCE_NEW_TRACKER_CONTEXT == 1))
	{
		HAL_DBG_TRACE_INFO("###### ===== CREATE CONTEXT ==== ######\r\n\r\n");

#if (USE_PRODUCTION_KEYS == 1)
		/* When the production keys are used, DevEUI = ChipEUI and JoinEUI is the one defined in lorawan_comissioning.h
		*/
		modem_response_code = lr1110_modem_get_chip_eui(&tracker->lr1110, dev_eui);
#endif
		/* Init the LoRaWAN keys set in Commissioning_tracker->tracker_ctx.h or using the production keys and init the global
		 * context */
		tracker_init_app_ctx(tracker, dev_eui, join_eui, app_key, true);

		/* Init the tracker internal log context */
		tracker_reset_internal_log(tracker);
	}
	else
	{
		/* Restore the tracker internal log context */
		if (tracker_restore_internal_log_ctx(tracker) != 1)
		{
			tracker_init_internal_log_ctx(tracker);
		}

		/* Set the restored LoRaWAN Keys */
		memcpy(dev_eui, tracker->tracker_ctx.dev_eui, LORAWAN_DEVICE_EUI_LEN);
		memcpy(join_eui, tracker->tracker_ctx.join_eui, LORAWAN_JOIN_EUI_LEN);
		memcpy(app_key, tracker->tracker_ctx.app_key, LORAWAN_APP_KEY_LEN);
	}

	/* Basic LoRaWAN configuration */
	if (lorawan_init(tracker, tracker->tracker_ctx.lorawan_region, LORAWAN_CLASS_USED) != LR1110_MODEM_RESPONSE_CODE_OK)
	{
		HAL_DBG_TRACE_ERROR("###### ===== LORAWAN INIT ERROR ==== ######\r\n\r\n");
	}

	if (tracker_gnss_init(tracker, &tracker->tracker_ctx.gnss_settings) != LR1110_MODEM_RESPONSE_CODE_OK)
	{
		HAL_DBG_TRACE_ERROR("###### ===== GNSS INIT ERROR ==== ######\r\n\r\n");
	}

	/* Set Keys */
	modem_response_code = lr1110_modem_set_dev_eui(&tracker->lr1110, dev_eui);
	modem_response_code = lr1110_modem_set_join_eui(&tracker->lr1110, join_eui);

	/* do a derive keys to get the pin code */
	lr1110_modem_derive_keys(&tracker->lr1110);
	lr1110_modem_get_pin(&tracker->lr1110, &tracker->tracker_ctx.lorawan_pin);
	lr1110_modem_get_chip_eui(&tracker->lr1110, tracker->tracker_ctx.chip_eui);

//FIXME
#if 0
	/* Init the software watchdog */
	hal_mcu_init_software_watchdog(tracker->tracker_ctx.app_scan_interval * 3);

	/* Init Leds timer */
	timer_init(&led_tx_timer, on_led_tx_timer_event);
	timer_set_value(&led_tx_timer, LED_PERIOD_MS);
	timer_init(&led_rx_timer, on_led_rx_timer_event);
	timer_set_value(&led_rx_timer, LED_PERIOD_MS);
#endif

	/* Init tracker context volatile parameters */
	tracker->tracker_ctx.has_date                   = false;
	tracker->tracker_ctx.accelerometer_move_history = 1;
	tracker->tracker_ctx.stream_done                = true;
	tracker->tracker_ctx.voltage                    = hal_mcu_get_vref_level();
	tracker->tracker_ctx.reset_cnt_sent             = false;
	tracker->tracker_ctx.system_sanity_check        = (tracker_system_sanity_check_mask_t) 0;

	while (1) {
		/* Process Event */
		//sem_wait(&tracker->lr1110.event_processed_sem);

		switch (tracker->device_state)
		{
		case DEVICE_STATE_INIT:
			HAL_DBG_TRACE_INFO("###### ===== LR1110 MODEM-E INIT ==== ######\r\n\r\n");

			if (tracker->tracker_ctx.use_semtech_join_server == true) {
				modem_response_code = lr1110_modem_derive_keys(&tracker->lr1110);
			} else {
				modem_response_code = lr1110_modem_set_app_key(&tracker->lr1110, app_key);
			}

			tracker->device_state = DEVICE_STATE_JOIN;
			break;
		case DEVICE_STATE_JOIN:
			/* Display used keys */
			print_lorawan_keys(dev_eui, join_eui, app_key, tracker->tracker_ctx.lorawan_pin,
					tracker->tracker_ctx.use_semtech_join_server);

			tracker_app_join_network(tracker);

			tracker->device_state = DEVICE_STATE_CYCLE;
			break;
		case DEVICE_COLLECT_DATA:
			/* Create a movevment history on 8 bits and update this value only if the stream is done */
			if (tracker->tracker_ctx.stream_done == true) {
				tracker->tracker_ctx.accelerometer_move_history =
					(tracker->tracker_ctx.accelerometer_move_history << 1) + is_accelerometer_detected_moved(tracker);
				printf("tracker->tracker_ctx.accelerometer_move_history: 0x%x\n", tracker->tracker_ctx.accelerometer_move_history);
			}

			/* Check if scan can be launched */
			if (tracker_app_is_next_scan_possible(tracker) == true) {
//FIXME
#if 0
				/* Reload the software watchdog */
				hal_mcu_reset_software_watchdog();
#endif

				/* Reset previous scan results */
				tracker_app_reset_scan_results(tracker);

				/* Reset flag and counter */
				if (tracker->tracker_ctx.send_alive_frame == true) {
					send_complete_sensors = true;
				}
				tracker->tracker_ctx.send_alive_frame = false;
				tracker->tracker_ctx.next_frame_ctn   = 0;

				/* Adapt the ADR following the acceleromer movement */
				tracker_app_adapt_adr(tracker);

//FIXME
#if 0
				/* Led start for user notification */
				leds_on(LED_TX_MASK);
				timer_start(&led_tx_timer);

				/* Activate the partial low power mode */
				hal_mcu_partial_sleep_enable(true);
#endif

				/* Launch the scan according to the scan prioriy */
				if (tracker_app_start_scan(tracker, tracker->tracker_ctx.scan_priority) == false) {
					HAL_DBG_TRACE_MSG("No scan results good enough, send complete sensors values\r\n");
					send_complete_sensors = true;
				}

				/*  SENSORS DATA */
				HAL_DBG_TRACE_INFO("*** sensors collect ***\n\r\n\r");

				/* Acceleration */
				acc_read_raw_data(tracker);
				tracker->tracker_ctx.accelerometer_x = acc_get_raw_x(tracker);
				tracker->tracker_ctx.accelerometer_y = acc_get_raw_y(tracker);
				tracker->tracker_ctx.accelerometer_z = acc_get_raw_z(tracker);
				HAL_DBG_TRACE_PRINTF("Acceleration [mg]: X=%4.2f mg | Y=%4.2f mg | Z=%4.2f mg \r\n",
						(double) tracker->tracker_ctx.accelerometer_x, (double) tracker->tracker_ctx.accelerometer_y,
						(double) tracker->tracker_ctx.accelerometer_z);

				/* Move history */
				HAL_DBG_TRACE_PRINTF("Move history : %d\r\n", tracker->tracker_ctx.accelerometer_move_history);

				/* Temperature */
				//tracker->tracker_ctx.temperature = hal_mcu_get_temperature() * 100;
				tracker->tracker_ctx.temperature = acc_get_temperature(tracker) * 100;
				HAL_DBG_TRACE_PRINTF("Temperature : %d *C\r\n", tracker->tracker_ctx.temperature / 100);

				/* Modem charge */
				lr1110_modem_get_charge(&tracker->lr1110, &tracker->tracker_ctx.charge);
				HAL_DBG_TRACE_PRINTF("Charge value : %d mAh\r\n", tracker->tracker_ctx.charge);
				tracker_app_store_new_acculated_charge(tracker, tracker->tracker_ctx.charge);
				HAL_DBG_TRACE_PRINTF("Accumulated charge value : %d mAh\r\n", tracker->tracker_ctx.accumulated_charge);

				/* Board voltage charge */
				lr1110_modem_get_lorawan_state(&tracker->lr1110, &lorawan_state);
				if (lorawan_state == LR1110_MODEM_LORAWAN_IDLE) {
					tracker->tracker_ctx.voltage = hal_mcu_get_vref_level();
					HAL_DBG_TRACE_PRINTF("Board voltage : %d mV\r\n", tracker->tracker_ctx.voltage);
				} else {
					HAL_DBG_TRACE_WARNING("TX ongoing don't collect the board voltage\r\n");
				}

				if (tracker->tracker_ctx.internal_log_enable) {
					HAL_DBG_TRACE_PRINTF("Log results in the Internal Log memory\r\n", tracker->tracker_ctx.voltage);
					tracker_store_internal_log(tracker);
					HAL_DBG_TRACE_PRINTF("Internal Log memory space remaining: %d %%\r\n",
							tracker_get_remaining_memory_space(tracker));
				}

				/* Build the payload and stream it if we have enough time */
				lr1110_modem_get_duty_cycle_status(&tracker->lr1110, &duty_cycle);

				if ((tracker_app_is_region_use_duty_cycle(tracker, tracker->tracker_ctx.lorawan_region) == false) ||
						((duty_cycle >= TRACKER_DUTY_CYCLE_THRESHOLD) &&
						  ((tracker_app_is_region_use_duty_cycle(tracker, tracker->tracker_ctx.lorawan_region) == true)))) {
					HAL_DBG_TRACE_PRINTF("Remaining time (%d ms) to send data\r\n", duty_cycle);
					tracker_app_build_and_stream_payload(tracker, send_complete_sensors);
				} else {
					HAL_DBG_TRACE_WARNING(
							"Not enough remaining time (%d ms) to send data, results are just logged (if enable)\r\n",
							duty_cycle);
				}

				/* Send tracker setting if it has been asked by the application server */
				if (tracker->tracker_ctx.tracker_settings_payload_len > 0) {
					tracker_app_build_and_stream_tracker_settings(tracker, tracker->tracker_ctx.tracker_settings_payload,
							tracker->tracker_ctx.tracker_settings_payload_len);
					tracker->tracker_ctx.tracker_settings_payload_len = 0;
				}

				tracker->device_state = DEVICE_STATE_SEND;
			} else {
				if (tracker->tracker_ctx.stream_done == true) {
					if (tracker->tracker_ctx.next_frame_ctn >=
							(tracker->tracker_ctx.app_keep_alive_frame_interval / tracker->tracker_ctx.app_scan_interval)) {
						HAL_DBG_TRACE_INFO("Send a keep alive frame next time\r\n");
						tracker->tracker_ctx.send_alive_frame = true;
					} else {
						HAL_DBG_TRACE_PRINTF(
								"Device is static next keep alive frame in %d sec\r\n",
								((tracker->tracker_ctx.app_keep_alive_frame_interval / tracker->tracker_ctx.app_scan_interval) -
								  tracker->tracker_ctx.next_frame_ctn) *
								(tracker->tracker_ctx.app_scan_interval / 1000));
						tracker->tracker_ctx.next_frame_ctn++;
					}
					tracker->device_state = DEVICE_STATE_CYCLE;
				} else {
					tracker->device_state = DEVICE_STATE_SEND;
				}
			}

			break;
		case DEVICE_STATE_SEND:
			if (tracker->tracker_ctx.stream_done == false) {
				lr1110_modem_stream_status_t stream_status;

				/* Stream previous payload if it's not terminated */
				lr1110_modem_stream_status(&tracker->lr1110, LORAWAN_STREAM_APP_PORT, &stream_status);
				HAL_DBG_TRACE_PRINTF("Streaming ongoing %d bytes remaining %d bytes free \r\n", stream_status.pending,
						stream_status.free);
			}

			tracker->device_state = DEVICE_STATE_CYCLE;

			break;
		case DEVICE_STATE_CYCLE:
//FIXME
#if 0
			/* Reload the software watchdog */
			hal_mcu_reset_software_watchdog();
#endif

			tracker->device_state = DEVICE_STATE_SLEEP;

			/* Schedule next packet transmission */
			do
			{
				modem_response_code = lr1110_modem_set_alarm_timer(&tracker->lr1110, tracker->tracker_ctx.app_scan_interval / 1000);
			} while(modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK);

			HAL_DBG_TRACE_PRINTF("Set alarm timer : %d s\r\n\r\n", tracker->tracker_ctx.app_scan_interval / 1000);

			break;
		case DEVICE_STATE_SLEEP:
			sem_wait(&tracker->lr1110.event_processed_sem);

			/* Wake up from static mode thanks the accelerometer ? */
			if ((get_accelerometer_irq1_state(tracker) == true) && (tracker_app_is_tracker_in_static_mode(tracker) == true)) {
				/* Stop the LR1110 current modem alarm */
				do
				{
					modem_response_code = lr1110_modem_set_alarm_timer(&tracker->lr1110, 0);
				} while(modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK);

				tracker->device_state = DEVICE_COLLECT_DATA;
			}
			break;
		default:
			tracker->device_state = DEVICE_STATE_INIT;
			break;
		}
	}

err_close:
	close(tracker->lr1110.spi_id);
err:
	return ret;
}

void tracker_wifi_run_scan(struct tracker *tracker, const wifi_settings_t* wifi_settings, wifi_scan_selected_result_t* wifi_result)
{
	static wifi_scan_all_results_t capture_result;

	if (wifi_execute_scan(tracker, &tracker->lr1110, wifi_settings, &capture_result) == WIFI_SCAN_SUCCESS)
	{
		lr1110_modem_display_wifi_scan_results(&capture_result);

		lr1110_modem_wifi_scan_select_results(&capture_result, wifi_result);

		if (wifi_result->nbr_results != 0) {
			/* Update the system_sanity_check bit field */
			tracker->tracker_ctx.system_sanity_check |= TRACKER_WIFI_SCAN_SUCCESSFUL_ONCE;
		}
	} else {
		HAL_DBG_TRACE_ERROR("Wi-Fi Scan error\n\r");
		wifi_result->nbr_results = 0;  // reset MAC addr detected
	}
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER GNSS FUNCTION TYPES ---------------------------------------------
 */

uint8_t tracker_gnss_get_next_nb_sat(struct tracker *tracker)
{
	uint8_t next_tx_max_payload = 0;

	/* Adapt the nb_sat value in function of the number of byte available for the next lorawan payload */
	lr1110_modem_get_next_tx_max_payload(&tracker->lr1110, &next_tx_max_payload);

	if (next_tx_max_payload <= 51) {
		return 8;
	} else {
		return 12;
	}
}

void tracker_gnss_run_scan(struct tracker *tracker, const gnss_settings_t* gnss_settings, gnss_scan_single_result_t* capture_result)
{
	uint8_t gnss_status;

	gnss_status = gnss_scan_execute(tracker, &tracker->lr1110, gnss_settings, capture_result);
	if (gnss_status == GNSS_SCAN_SUCCESS) {
		gnss_scan_display_results(capture_result);

		if (capture_result->nb_detected_satellites != 0) {
			/* Update the system_sanity_check bit field */
			tracker->tracker_ctx.system_sanity_check |= TRACKER_GNSS_SCAN_SUCCESSFUL_ONCE;
		}
	} else {
		if (gnss_status == GNSS_SCAN_NO_TIME) {
			HAL_DBG_TRACE_ERROR("GNSS Scan error: No time\n\r");
		} else {
			HAL_DBG_TRACE_ERROR("GNSS Scan error\n\r");
		}
	}
}

lr1110_modem_response_code_t tracker_gnss_init(struct tracker *tracker, const gnss_settings_t* gnss_settings)
{
	lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

	modem_response_code = lr1110_modem_gnss_set_assistance_position(&tracker->lr1110, &gnss_settings->assistance_position);

	return modem_response_code;
}

void tracker_gnss_store_new_assistance_position(struct tracker *tracker)
{
	lr1110_modem_gnss_solver_assistance_position_t assistance_position;
	float latitude_dif, longitude_dif;

	lr1110_modem_gnss_read_assistance_position(&tracker->lr1110, &assistance_position);

	latitude_dif  = fabs(assistance_position.latitude - tracker->tracker_ctx.gnss_settings.assistance_position.latitude);
	longitude_dif = fabs(assistance_position.longitude - tracker->tracker_ctx.gnss_settings.assistance_position.longitude);
	longitude_dif = 0;
	latitude_dif = 0;

	/* Store the new assistance position only if the difference is greater than the conversion error */
	if ((latitude_dif > (float) 0.03)|| (longitude_dif > (float)0.03)) {
		HAL_DBG_TRACE_MSG("New assistance position stored\r\n");

		tracker->tracker_ctx.gnss_settings.assistance_position.latitude  = assistance_position.latitude;
		tracker->tracker_ctx.gnss_settings.assistance_position.longitude = assistance_position.longitude;

		tracker_store_app_ctx(tracker);
	}
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER LORAWAN FUNCTION TYPES ------------------------------------------
 */

lr1110_modem_response_code_t lorawan_init(struct tracker *tracker, lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class)
{
	lr1110_modem_dm_info_fields_t dm_info_fields;
	lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

	modem_response_code |= lr1110_modem_set_class(&tracker->lr1110, lorawan_class);

	if (lorawan_class == LR1110_LORAWAN_CLASS_A) {
		HAL_DBG_TRACE_MSG("CLASS       : A\r\n");
	}

	if (lorawan_class == LR1110_LORAWAN_CLASS_C) {
		HAL_DBG_TRACE_MSG("CLASS       : C\r\n");
	}

	modem_response_code |= lr1110_modem_set_region(&tracker->lr1110, region);

	switch (region)
	{
	case LR1110_LORAWAN_REGION_EU868:
		HAL_DBG_TRACE_MSG("REGION      : EU868\r\n\r\n");
		modem_response_code |= lr1110_modem_activate_duty_cycle(&tracker->lr1110, LORAWAN_DUTYCYCLE_ON);
		break;
	case LR1110_LORAWAN_REGION_US915:
		HAL_DBG_TRACE_MSG("REGION      : US915\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_AU915:
		HAL_DBG_TRACE_MSG("REGION      : AU915\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_AS923_GRP1:
		if (LORAWAN_COUNTRY_JAPAN == 1) {
			HAL_DBG_TRACE_MSG("LBT         : ACTIVATE LBT\r\n");
			/* Activate LBT for 5ms before each transmission with a threshold at -80 dBm */
			modem_response_code |= lr1110_modem_activate_lbt(&tracker->lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -80, 5, 1250000);
		}

		HAL_DBG_TRACE_MSG("REGION      : AS923_GRP1\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_CN470:
		HAL_DBG_TRACE_MSG("REGION      : CN470\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_AS923_GRP2:
		HAL_DBG_TRACE_MSG("REGION      : AS923_GRP2\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_AS923_GRP3:
		HAL_DBG_TRACE_MSG("REGION      : AS923_GRP3\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_IN865:
		HAL_DBG_TRACE_MSG("REGION      : IN865\r\n\r\n");
		break;
	case LR1110_LORAWAN_REGION_KR920:
		HAL_DBG_TRACE_MSG("LBT         : ACTIVATE LBT\r\n");
		HAL_DBG_TRACE_MSG("REGION      : KR920\r\n\r\n");

		/* Activate LBT for 5ms before each transmission with a threshold at -65 dBm */
		modem_response_code |= lr1110_modem_activate_lbt(&tracker->lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -65, 5, 1250000);
		break;
	case LR1110_LORAWAN_REGION_RU864:
		HAL_DBG_TRACE_MSG("REGION      : RU864\r\n\r\n");
		break;
	default:
		HAL_DBG_TRACE_ERROR("No supported region selected\r\n\r\n");
		break;
	}

	/* Set DM info field */
	dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_CHARGE;
	dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS;
	dm_info_fields.dm_info_field[2] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
	dm_info_fields.dm_info_length   = 3;

	modem_response_code |= lr1110_modem_set_dm_info_field(&tracker->lr1110, &dm_info_fields);
	modem_response_code |= lr1110_modem_set_dm_info_interval(&tracker->lr1110, LR1110_MODEM_REPORTING_INTERVAL_IN_DAY, 1);
	modem_response_code |= lr1110_modem_set_alc_sync_mode(&tracker->lr1110, LR1110_MODEM_ALC_SYNC_MODE_ENABLE);

	return modem_response_code;
}

/*
 * -------------------------------------------------------------------------
 * --- TRACKER APP FUNCTION TYPES ------------------------------------------
 */

bool tracker_app_start_scan(struct tracker *tracker, const tracker_scan_priority_t scan_piority)
{
	bool has_scan_result = true;

	/* Activate the partial low power mode */
	//FIXME
#if 0
	hal_mcu_partial_sleep_enable(true);
#endif

	/* Timestamp scan */
	tracker->tracker_ctx.timestamp = lr1110_modem_board_get_systime_from_gps(&tracker->lr1110);

	switch (scan_piority)
	{
	case TRACKER_GNSS_PRIORITY:
		HAL_DBG_TRACE_INFO("*** Tracker scan GNSS priority *** \n\r\n\r");

		/*  GNSS SCAN */
		if (tracker->tracker_ctx.gnss_settings.enabled == true) {
			HAL_DBG_TRACE_INFO("*** Gnss Scan ***\n\r\n\r");

			/* Check if a new assistance position is available */
			tracker_gnss_store_new_assistance_position(tracker);

			if (tracker->tracker_ctx.has_date == true) {
				/* Get the next nb sat value */
				tracker->tracker_ctx.gnss_settings.nb_sat = tracker_gnss_get_next_nb_sat(tracker);

				tracker_gnss_run_scan(tracker, &tracker->tracker_ctx.gnss_settings, &tracker->tracker_ctx.gnss_scan_result);
			} else {
				HAL_DBG_TRACE_MSG("Wait application layer clock synchronisation\r\n\r\n");
			}
		}

		/*  WIFI SCAN */
		if (tracker->tracker_ctx.wifi_settings.enabled == true) {
			/* Proceed to a Wi-Fi scan only if no GNSS data is available */
			if ((tracker->tracker_ctx.gnss_settings.enabled == false) || ((tracker->tracker_ctx.gnss_scan_result.is_valid_nav_message == false))) {
				HAL_DBG_TRACE_INFO("*** Wi-Fi Scan *** \n\r\n\r");

				tracker_wifi_run_scan(tracker, &tracker->tracker_ctx.wifi_settings, &tracker->tracker_ctx.wifi_result);

				tracker->tracker_ctx.last_nb_detected_mac_address = tracker->tracker_ctx.wifi_result.nbr_results;
			} else {
				gpiolib_set_output(tracker->lr1110.led_tx, 1);
				HAL_DBG_TRACE_PRINTF("GNSS scan good enough, drop Wi-Fi scan\r\n");
			}
		}

		break;
	case TRACKER_WIFI_PRIORITY:
		HAL_DBG_TRACE_INFO("*** Tracker scan Wi-Fi priority *** \n\r\n\r");

		/*  WIFI SCAN */
		if (tracker->tracker_ctx.wifi_settings.enabled == true) {
			HAL_DBG_TRACE_INFO("*** Wi-Fi Scan *** \n\r\n\r");

			tracker_wifi_run_scan(tracker, &tracker->tracker_ctx.wifi_settings, &tracker->tracker_ctx.wifi_result);

			tracker->tracker_ctx.last_nb_detected_mac_address = tracker->tracker_ctx.wifi_result.nbr_results;
		}

		/*  GNSS SCAN */
		if (tracker->tracker_ctx.gnss_settings.enabled == true) {
			/* Proceed to a GNSS scan only if no Wi-Fi data is available */
			if ((tracker->tracker_ctx.wifi_settings.enabled == false) || (tracker->tracker_ctx.wifi_result.nbr_results < 2)) {
				HAL_DBG_TRACE_INFO("*** Gnss Scan ***\n\r\n\r");

				/* Check if a new assistance position is available */
				tracker_gnss_store_new_assistance_position(tracker);

				if (tracker->tracker_ctx.has_date == true) {
					/* Get the next nb sat value */
					tracker->tracker_ctx.gnss_settings.nb_sat = tracker_gnss_get_next_nb_sat(tracker);

					tracker_gnss_run_scan(tracker, &tracker->tracker_ctx.gnss_settings, &tracker->tracker_ctx.gnss_scan_result);
				} else {
					HAL_DBG_TRACE_MSG("Wait application layer clock synchronisation\r\n\r\n");
				}
			} else {
				HAL_DBG_TRACE_PRINTF("Wi-Fi scan good enough, drop GNSS scan\r\n");
			}
			break;
		}
	case TRACKER_NO_PRIORITY:
		HAL_DBG_TRACE_INFO("*** Tracker scan no priority *** \n\r\n\r");

		/*  WIFI SCAN */
		if (tracker->tracker_ctx.wifi_settings.enabled == true) {
			HAL_DBG_TRACE_INFO("*** Wi-Fi Scan *** \n\r\n\r");

			tracker_wifi_run_scan(tracker, &tracker->tracker_ctx.wifi_settings, &tracker->tracker_ctx.wifi_result);

			tracker->tracker_ctx.last_nb_detected_mac_address = tracker->tracker_ctx.wifi_result.nbr_results;
		}

		/*  GNSS SCAN */
		if (tracker->tracker_ctx.gnss_settings.enabled == true) {
			HAL_DBG_TRACE_INFO("*** Gnss Scan ***\n\r\n\r");

			/* Check if a new assistance position is available */
			tracker_gnss_store_new_assistance_position(tracker);

			if (tracker->tracker_ctx.has_date == true) {
				/* Get the next nb sat value */
				tracker->tracker_ctx.gnss_settings.nb_sat = tracker_gnss_get_next_nb_sat(tracker);

				tracker_gnss_run_scan(tracker, &tracker->tracker_ctx.gnss_settings, &tracker->tracker_ctx.gnss_scan_result);
			} else {
				HAL_DBG_TRACE_MSG("Wait application layer clock synchronisation\r\n\r\n");
			}
		}

		break;
	default:
		break;
	}

	/* Deactivate the partial low power mode */
	//FIXME
#if 0
	hal_mcu_partial_sleep_enable(false);
#endif

	/* If no scan results available send sensors */
	if ((tracker->tracker_ctx.gnss_scan_result.is_valid_nav_message == false) && (tracker->tracker_ctx.wifi_result.nbr_results < 2)) {
		has_scan_result = false;
	}

	return has_scan_result;
}

void tracker_app_reset_scan_results(struct tracker *tracker)
{
	tracker->tracker_ctx.gnss_scan_result.nb_detected_satellites = 0;
	tracker->tracker_ctx.gnss_scan_result.nav_message_size       = 0;
	tracker->tracker_ctx.gnss_scan_result.is_valid_nav_message   = false;
	tracker->tracker_ctx.wifi_result.nbr_results                 = 0;
}

void tracker_app_build_and_stream_payload(struct tracker *tracker, bool send_complete_sensors)
{
	/* BUILD THE PAYLOAD IN TLV FORMAT */
	tracker->tracker_ctx.lorawan_payload_len = 0;  // reset the payload len

	HAL_DBG_TRACE_MSG("\r\nAdd in the FiFo stream:\r\n");

	if ((tracker->tracker_ctx.gnss_scan_result.is_valid_nav_message == true) && (tracker->tracker_ctx.gnss_settings.enabled == true)) {
		/* Add GNSS scan */

		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_GNSS_NAV_TAG;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] =
			tracker->tracker_ctx.gnss_scan_result.nav_message_size - 1;  // GNSS LEN
		memcpy(tracker->tracker_ctx.lorawan_payload + tracker->tracker_ctx.lorawan_payload_len,
				tracker->tracker_ctx.gnss_scan_result.nav_message + 1, tracker->tracker_ctx.gnss_scan_result.nav_message_size - 1);
		tracker->tracker_ctx.lorawan_payload_len += tracker->tracker_ctx.gnss_scan_result.nav_message_size - 1;

		HAL_DBG_TRACE_MSG(" - GNSS NAV message\r\n");
	}

	if ((tracker->tracker_ctx.wifi_result.nbr_results >= 2) && (tracker->tracker_ctx.wifi_settings.enabled == true)) {
		/* Add Wi-Fi scan */
		uint8_t wifi_index = 0;

		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_WIFI_SCAN_TAG;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] =
			tracker->tracker_ctx.wifi_result.nbr_results * TLV_WIFI_SINGLE_BEACON_LEN +
			5;  // Wi-Fi Len = nb AP * 7 bytes + version (4 bytes) + timestamp (1 byte)

		/* Add Version */
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_WIFI_VERSION;

		/* Add timestamp */
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.timestamp >> 24;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.timestamp >> 16;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.timestamp >> 8;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.timestamp;

		wifi_index = tracker->tracker_ctx.lorawan_payload_len;
		for (uint8_t i = 0; i < tracker->tracker_ctx.wifi_result.nbr_results; i++) {
			tracker->tracker_ctx.lorawan_payload[wifi_index] = tracker->tracker_ctx.wifi_result.results[i].rssi;
			memcpy(&tracker->tracker_ctx.lorawan_payload[wifi_index + 1], tracker->tracker_ctx.wifi_result.results[i].mac_address, 6);
			wifi_index += TLV_WIFI_SINGLE_BEACON_LEN;
		}
		tracker->tracker_ctx.lorawan_payload_len += tracker->tracker_ctx.wifi_result.nbr_results * TLV_WIFI_SINGLE_BEACON_LEN;

		HAL_DBG_TRACE_MSG(" - WiFi scan\r\n");
	}

	/* Add sensors value */
	tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_SENSORS_TAG;
	if (send_complete_sensors) {
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_SENSOR_FULL_VERSION_LEN;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] =
			((TLV_SENSOR_FULL_VERSION & 0x0F) << 4) | (tracker->tracker_ctx.accelerometer_move_history & 0x0F);

		/* Temperature */
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.temperature >> 8;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.temperature;

		/* Modem-E Charge */
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.accumulated_charge >> 8;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.accumulated_charge;

		/* Board Voltage */
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.voltage >> 8;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.voltage;
	}
	else
	{
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_SENSOR_BASIC_VERSION_LEN;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] =
			((TLV_SENSOR_BASIC_VERSION & 0x0F) << 4) | (tracker->tracker_ctx.accelerometer_move_history & 0x0F);
	}

	HAL_DBG_TRACE_MSG(" - Sensors value\r\n");

	/* send this information just once */
	if (tracker->tracker_ctx.reset_cnt_sent == false)
	{
		/* Add Reset counters value */
		tracker->tracker_ctx.reset_cnt_sent = true;

		/* Reset counter */
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_RESET_COUNTER_TAG;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = 4;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.host_reset_cnt >> 8;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.host_reset_cnt;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.modem_reset_by_itself_cnt >> 8;
		tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = tracker->tracker_ctx.modem_reset_by_itself_cnt;

		HAL_DBG_TRACE_MSG(" - Reset counters\r\n");
	}

	/* Push the Payload in the FiFo stream */
	tracker_app_add_payload_in_streaming_fifo(tracker, tracker->tracker_ctx.lorawan_payload, tracker->tracker_ctx.lorawan_payload_len);

	tracker->tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
}

void tracker_app_build_and_stream_tracker_settings(struct tracker *tracker, const uint8_t* buffer, uint8_t len)
{
	/* Add tracker settings value */
	tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = TLV_TRACKER_SETTINGS_TAG;  // Tracker settings TAG
	tracker->tracker_ctx.lorawan_payload[tracker->tracker_ctx.lorawan_payload_len++] = len;  // Tracker settings LEN is variable

	memcpy(tracker->tracker_ctx.lorawan_payload + 2, buffer, len);
	tracker->tracker_ctx.lorawan_payload_len += len;

	HAL_DBG_TRACE_PRINTF(" - Tracker settings (%d bytes) : ", tracker->tracker_ctx.lorawan_payload_len);

	/* Push the sensor values in the FiFo stream */
	tracker_app_add_payload_in_streaming_fifo(tracker, tracker->tracker_ctx.lorawan_payload, tracker->tracker_ctx.lorawan_payload_len);

	tracker->tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
}

void tracker_app_store_new_acculated_charge(struct tracker *tracker, uint32_t modem_charge)
{
	static uint32_t previous_modem_charge = 0;  // Previous modem charge before read it into the LR1110 Modem-E, keep
						    // the historic even after leave the function because of the static

	/* Store the new accumulated charge only if the modem charge has changed */
	if (modem_charge != previous_modem_charge) {
		tracker->tracker_ctx.accumulated_charge += modem_charge - previous_modem_charge;
		HAL_DBG_TRACE_MSG("New acculated charge stored\r\n");
		tracker_store_app_ctx(tracker);

		previous_modem_charge = modem_charge;
	}
}

void tracker_app_join_network(struct tracker *tracker)
{
	lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

	/* Starts the join procedure */
	modem_response_code = lr1110_modem_join(&tracker->lr1110);

	if (modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK)
	{
		HAL_DBG_TRACE_INFO("###### ===== JOINING ==== ######\r\n\r\n");
	}
	else
	{
		HAL_DBG_TRACE_ERROR("###### ===== JOINING CMD ERROR ==== ######\r\n\r\n");
	}
}

void tracker_app_adapt_adr(struct tracker *tracker)
{
	lr1110_modem_adr_profiles_t adr_profile;

	lr1110_modem_get_adr_profile(&tracker->lr1110, &adr_profile);

	//means the device is static
	if (tracker->tracker_ctx.send_alive_frame == true) {
		if (adr_profile != LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED) {
			HAL_DBG_TRACE_MSG("Set ADR to LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED\n\r\n\r");
			lr1110_modem_set_adr_profile(&tracker->lr1110, LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED, NULL);
		}
	} else { //means the device is mobile
		if (adr_profile != tracker->tracker_ctx.lorawan_adr_profile) {
			HAL_DBG_TRACE_PRINTF("Set ADR to %d\n\r\n\r", tracker->tracker_ctx.lorawan_adr_profile);
			lr1110_modem_set_adr_profile(&tracker->lr1110, (lr1110_modem_adr_profiles_t) tracker->tracker_ctx.lorawan_adr_profile, adr_custom_list);
			if (tracker->tracker_ctx.lorawan_adr_profile == LR1110_MODEM_ADR_PROFILE_CUSTOM) {
				lr1110_modem_set_nb_trans(&tracker->lr1110, 2);
			}
		}
	}
}

bool tracker_app_add_payload_in_streaming_fifo(struct tracker *tracker, const uint8_t* payload, uint16_t len)
{
	lr1110_modem_stream_status_t stream_status;

	/* Push the Payload in the FiFo stream */
	lr1110_modem_stream_status(&tracker->lr1110, LORAWAN_STREAM_APP_PORT, &stream_status);

	if (stream_status.free > len) {
		tracker->tracker_ctx.stream_done = false;
		HAL_DBG_TRACE_PRINTF("%d bytes added in streaming FiFo\r\n", len);
		lr1110_modem_send_stream_data(&tracker->lr1110, LORAWAN_STREAM_APP_PORT, payload, len);

		return true;
	} else {
		HAL_DBG_TRACE_PRINTF("Not enought space, need = %d bytes - free = %d bytes\r\n", len, stream_status.free);

		return false;
	}
}

bool tracker_app_is_next_scan_possible(struct tracker *tracker)
{
	if ((((tracker->tracker_ctx.accelerometer_move_history & TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC) != 0) ||
				(tracker->tracker_ctx.send_alive_frame == true) || (tracker->tracker_ctx.accelerometer_used == 0)) &&
			(tracker->tracker_ctx.stream_done == true)) {
		return true;
	} else {
		return false;
	}
}

bool tracker_app_is_region_use_duty_cycle(struct tracker *tracker, lr1110_modem_regions_t region)
{
	if ((region == LR1110_LORAWAN_REGION_EU868) || (region == LR1110_LORAWAN_REGION_RU864)) {
		return true;
	} else {
		return false;
	}
}

bool tracker_app_is_tracker_in_static_mode(struct tracker *tracker)
{
	if (((tracker->tracker_ctx.accelerometer_move_history & TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC) == 0) &&
			(tracker->tracker_ctx.accelerometer_used == 1)) {
		return true;
	} else {
		return false;
	}
}

void tracker_app_parse_downlink_frame(struct tracker *tracker, uint8_t port, const uint8_t* payload, uint8_t size)
{
	uint8_t tag           = 0;
	uint8_t len           = 0;
	uint8_t payload_index = 0;
	uint8_t settings_buffer[240];

	uint8_t buffer[6];
	const uint32_t modem_date = lr1110_modem_board_get_systime_from_gps(&tracker->lr1110);

	switch (port)
	{
	case GNSS_PUSH_SOLVER_MSG_PORT:
		HAL_DBG_TRACE_INFO("###### ===== GNSS PUSH SOLVER MSG ==== ######\r\n\r\n");
		lr1110_modem_gnss_push_solver_msg(&tracker->lr1110, payload, size);
		break;
	case TRACKER_REQUEST_MSG_PORT:
		while (payload_index < size) {
			tag = payload[payload_index++];
			len = payload[payload_index++];

			switch (tag)
			{
			case GET_APP_TRACKER_SETTINGS_CMD:
				memcpy(settings_buffer, payload + payload_index, len);

				HAL_DBG_TRACE_INFO(
						"###### ===== TRACKER CONFIGURATION SETTINGS PAYLOAD RECEIVED ==== ######\r\n\r\n");

				tracker->tracker_ctx.tracker_settings_payload_len =
					tracker_parse_cmd(tracker, settings_buffer, tracker->tracker_ctx.tracker_settings_payload, false);

				/* Store the new values here if it's asked */
				if ((tracker->tracker_ctx.new_value_to_set) == true) {
					tracker->tracker_ctx.new_value_to_set = false;
					tracker_store_app_ctx(tracker);
				}

				break;
			case GET_MODEM_DATE_CMD:
				buffer[0] = GET_MODEM_DATE_CMD;
				buffer[1] = GET_MODEM_DATE_ANSWER_LEN;
				buffer[2] = modem_date >> 24;
				buffer[3] = modem_date >> 16;
				buffer[4] = modem_date >> 8;
				buffer[5] = modem_date;

				/* Use the emergency TX to reduce the latency */
				lr1110_modem_emergency_tx(&tracker->lr1110, port, LR1110_MODEM_UPLINK_UNCONFIRMED, buffer, 6);

				HAL_DBG_TRACE_INFO("###### ===== SEND MODEM-E DATE IN EMERGENCY TX ==== ######\r\n\r\n");

				break;
			case SET_RX_LED_CMD:
				HAL_DBG_TRACE_INFO("###### ===== SET RX LED MSG ==== ######\r\n\r\n");
				/* Wait the end of the rx led timer */
				time_usleep(30 * 1000);

				if (payload[payload_index] == 1)
				{
					gpiolib_set_output(tracker->lr1110.led_rx, 1);
				}
				if (payload[payload_index] == 2)
				{
					gpiolib_set_output(tracker->lr1110.led_rx, 0);
				}

				payload_index += len;
				break;
			default:
				payload_index += len;
				break;
			}
		}
	default:
		break;
	}
}
