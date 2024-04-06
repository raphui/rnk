#ifndef LR1110_MODEM_H
#define LR1110_MODEM_H

#include "../tracker.h"

void lr1110_modem_reset_event(struct tracker *tracker, uint16_t reset_count);
void lr1110_modem_network_joined(struct tracker *tracker);
void lr1110_modem_join_fail(struct tracker *tracker);
void lr1110_modem_alarm(struct tracker *tracker);
void lr1110_modem_down_data(struct tracker *tracker, int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port, const uint8_t* payload, uint8_t size);
void lr1110_modem_mute(struct tracker *tracker, lr1110_modem_mute_t mute);
void lr1110_modem_set_conf(struct tracker *tracker, lr1110_modem_event_setconf_tag_t tag);
void lr1110_modem_stream_done(struct tracker *tracker);
void lr1110_modem_time_updated_alc_sync(struct tracker *tracker, lr1110_modem_alc_sync_state_t alc_sync_state);
void lr1110_modem_adr_mobile_to_static(struct tracker *tracker);
void lr1110_modem_new_link_adr(struct tracker *tracker);
void lr1110_modem_no_event(struct tracker *tracker);

#endif /* LR1110_MODEM_H */
