#ifndef SEE_see_sensor_H
#define SEE_see_sensor_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct see_sensor_icm4x6xx_imu_data_t {
	float accl_ms2[3];         ///< XYZ acceleration in m/s^2
	float gyro_rad[3];         ///< XYZ gyro rotation in rad/s
    float temperature;         ///< temperature in degrees Celcius
	uint64_t timestamp_slpi_ticks;  ///< timestamp in 19.2 MHz SLPI ticks
	uint64_t timestamp_apps_real;   ///< timestamp in apps proc realtime nanosec
} __attribute__((packed)) see_sensor_icm4x6xx_imu_data_t;

void see_sensor_icm4x6xx_enable_debug_messages();
int  see_sensor_icm4x6xx_detect();
int  see_sensor_icm4x6xx_init();
int  see_sensor_icm4x6xx_read(see_sensor_icm4x6xx_imu_data_t* data);
int  see_sensor_icm4x6xx_close();

typedef struct see_sensor_icp101xx_barometer_data_t {
	float pressure;         ///< pressure in hPa (hectoPascal)
    float temperature;      ///< temperature in degrees Celcius
	uint64_t timestamp_slpi_ticks;  ///< timestamp in 19.2 MHz SLPI ticks
	uint64_t timestamp_apps_real;   ///< timestamp in apps proc realtime nanosec
} __attribute__((packed)) see_sensor_icp101xx_barometer_data_t;

void see_sensor_icp101xx_enable_debug_messages();
int  see_sensor_icp101xx_detect();
int  see_sensor_icp101xx_init();
int  see_sensor_icp101xx_read(see_sensor_icp101xx_barometer_data_t* data);
int  see_sensor_icp101xx_close();

typedef void (*fc_rx_cb)(const char *topic, const uint8_t *data, uint32_t length_in_bytes);
typedef void (*fc_ad_cb)(const char *topic);

int  see_sensor_flight_controller_initialize(bool enable_debug_messages,
                                             fc_rx_cb rx_cb,
                                             fc_ad_cb ad_cb);
int  see_sensor_flight_controller_advertise(const char *topic);
int  see_sensor_flight_controller_unadvertise(const char *topic);
int  see_sensor_flight_controller_subscribe(const char *topic);
int  see_sensor_flight_controller_unsubscribe(const char *topic);
int  see_sensor_flight_controller_query_subscriber(const char *topic);
int  see_sensor_flight_controller_send_data(const char *topic,
                                            const uint8_t *data,
                                            uint32_t length_in_bytes);
#ifdef __cplusplus
}
#endif

#endif // SEE_see_sensor_H
