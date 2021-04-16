
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/getopt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include "imu_thin_client.hpp"

static IMU_ThinClient *local_instance;

IMU_ThinClient::IMU_ThinClient() : _px4_accel(1), _px4_gyro(1), _px4_mag(1) {}

int imu_thin_client_main(int argc, char *argv[])
{
    if (local_instance == nullptr) local_instance = new IMU_ThinClient();

    if (local_instance == nullptr) {
        PX4_ERR("Couldn't instantiate IMU_ThinClient");
        return -1;
    }

	if ((argc == 2) && (strcmp(argv[1], "start") == 0)) {
        PX4_INFO("Starting IMU_ThinClient");
        local_instance->start();
    } else if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
        PX4_INFO("Stopping IMU_ThinClient");
        local_instance->stop();
    } else {
        PX4_ERR("Invalid command for IMU_ThinClient");
    }

    return 0;
}

void imu_thin_client_accel_data(float x, float y, float z) {
    if (local_instance) local_instance->update_accel(x, y, z);
}

void imu_thin_client_gyro_data(float x, float y, float z) {
    if (local_instance) local_instance->update_gyro(x, y, z);
}

void imu_thin_client_temperature_data(float temperature) {
    if (local_instance) local_instance->update_temperature(temperature);
}
