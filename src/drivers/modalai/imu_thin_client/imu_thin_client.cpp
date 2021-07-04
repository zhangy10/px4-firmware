
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

#include <px4_platform_common/getopt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include "imu_thin_client.hpp"

#include <qurt.h>

static IMU_ThinClient *local_instance = nullptr;

#define IMU_STACK_SIZE 4096
#define IMU_THREAD_PRIORITY (255 - 7)

static px4_task_t imu_thread_tid = -1;

static qurt_mutex_t imu_mutex;

static hrt_abstime accel_sample_time = 0;
static hrt_abstime gyro_sample_time = 0;

static float accel_x = 0.0;
static float accel_y = 0.0;
static float accel_z = 0.0;

static float gyro_x = 0.0;
static float gyro_y = 0.0;
static float gyro_z = 0.0;

static uint32_t loop_counter = 0;

static int imu_update_thread(int argc, char *argv[]) {

    static hrt_abstime accel_update_time = 0;
    static hrt_abstime gyro_update_time = 0;

    while (true) {
        loop_counter++;

        if (accel_sample_time != accel_update_time) {
            qurt_mutex_lock(&imu_mutex);
            hrt_abstime accel_sample_time_copy = accel_sample_time;
            float accel_x_copy = accel_x;
            float accel_y_copy = accel_y;
            float accel_z_copy = accel_z;
            qurt_mutex_unlock(&imu_mutex);
            if (local_instance) local_instance->update_accel(accel_sample_time_copy, accel_x_copy, accel_y_copy, accel_z_copy);
            accel_update_time = accel_sample_time_copy;
        }

        if (gyro_sample_time != gyro_update_time) {
            qurt_mutex_lock(&imu_mutex);
            hrt_abstime gyro_sample_time_copy = gyro_sample_time;
            float gyro_x_copy = gyro_x;
            float gyro_y_copy = gyro_y;
            float gyro_z_copy = gyro_z;
            qurt_mutex_unlock(&imu_mutex);
            if (local_instance) local_instance->update_gyro(gyro_sample_time_copy, gyro_x_copy, gyro_y_copy, gyro_z_copy);
            gyro_update_time = gyro_sample_time_copy;
        }

        qurt_timer_sleep(100);
    }

    return 0;
}

// TODO: Allow configuration of the rotation. It is hardcoded now to match M0051
IMU_ThinClient::IMU_ThinClient() : _px4_accel(1, ROTATION_YAW_90), _px4_gyro(1, ROTATION_YAW_90) {}

int imu_thin_client_main(int argc, char *argv[])
{
    if (local_instance == nullptr) local_instance = new IMU_ThinClient();

    if (local_instance == nullptr) {
        PX4_ERR("Couldn't instantiate IMU_ThinClient");
        return -1;
    }

    if (imu_thread_tid < 0) {
        qurt_mutex_init(&imu_mutex);
        imu_thread_tid = px4_task_spawn_cmd("imu_thin_client",
    				                        SCHED_DEFAULT,
    				                        IMU_THREAD_PRIORITY,
    				                        IMU_STACK_SIZE,
    				                        imu_update_thread,
    				                        NULL);
    }

    if (imu_thread_tid < 0) {
        PX4_ERR("Couldn't create IMU_ThinClient update thread");
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

    static uint32_t sample_msg_counter = 0;

    if (local_instance != nullptr) {
        if (local_instance->is_running()) {
            qurt_mutex_lock(&imu_mutex);
            accel_x = x;
            accel_y = y;
            accel_z = z;
            accel_sample_time = hrt_absolute_time();
            qurt_mutex_unlock(&imu_mutex);
        }
    }

    if (++sample_msg_counter == 1000) {
        PX4_INFO("ACCEL sample at %llu %u", accel_sample_time, loop_counter);
        sample_msg_counter = 0;
    }
}

void imu_thin_client_gyro_data(float x, float y, float z) {
    gyro_x = x;
    gyro_y = y;
    gyro_z = z;
    gyro_sample_time = hrt_absolute_time();
}

void imu_thin_client_temperature_data(float temperature) {
    if (local_instance) local_instance->update_temperature(temperature);
}
