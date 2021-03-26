#ifndef FC_SENSOR_H
#define FC_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef void (*fc_rx_cb)(const char *topic,
                         const uint8_t *data,
                         uint32_t length_in_bytes);
typedef void (*fc_ad_cb)(const char *topic);

int fc_sensor_initialize(bool enable_debug_messages,
                         fc_rx_cb rx_cb,
                         fc_ad_cb ad_cb);
int fc_sensor_advertise(const char *topic);
int fc_sensor_unadvertise(const char *topic);
int fc_sensor_subscribe(const char *topic);
int fc_sensor_unsubscribe(const char *topic);
int fc_sensor_query_subscriber(const char *topic);
int fc_sensor_send_data(const char *topic,
                        const uint8_t *data,
                        uint32_t length_in_bytes);
#ifdef __cplusplus
}
#endif

#endif // FC_SENSOR_H
