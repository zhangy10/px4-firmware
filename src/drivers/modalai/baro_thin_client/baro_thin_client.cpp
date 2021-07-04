
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
#include "baro_thin_client.hpp"

#include <qurt.h>

#define BARO_STACK_SIZE 4096
#define BARO_THREAD_PRIORITY (255 - 32)

static BaroThinClient *local_instance = nullptr;

static px4_task_t baro_thread_tid = -1;

static qurt_mutex_t baro_mutex;

static hrt_abstime baro_sample_time = 0;

static float baro_pressure = 0.0;

static uint32_t loop_counter = 0;

static int baro_update_thread(int argc, char *argv[]) {

    static hrt_abstime baro_update_time = 0;

    while (true) {
        loop_counter++;
        if (baro_sample_time != baro_update_time) {
            qurt_mutex_lock(&baro_mutex);
            hrt_abstime baro_sample_time_copy = baro_sample_time;
            float baro_pressure_copy = baro_pressure;
            qurt_mutex_unlock(&baro_mutex);
            if (local_instance) local_instance->update_pressure(baro_sample_time_copy, baro_pressure_copy);
            baro_update_time = baro_sample_time_copy;
        }

        qurt_timer_sleep(10000);
    }

    return 0;
}

BaroThinClient::BaroThinClient() : _px4_baro(1) {}

int baro_thin_client_main(int argc, char *argv[])
{
    if (local_instance == nullptr) local_instance = new BaroThinClient();

    if (local_instance == nullptr) {
        PX4_ERR("Couldn't instantiate BaroThinClient");
        return -1;
    }

    if (baro_thread_tid < 0) {
        qurt_mutex_init(&baro_mutex);
        baro_thread_tid = px4_task_spawn_cmd("baro_thin_client",
    				                         SCHED_DEFAULT,
    				                         BARO_THREAD_PRIORITY,
    				                         BARO_STACK_SIZE,
    				                         baro_update_thread,
    				                         NULL);
    }

    if (baro_thread_tid < 0) {
        PX4_ERR("Couldn't create BaroThinClient update thread");
        return -1;
    }

	if ((argc == 2) && (strcmp(argv[1], "start") == 0)) {
        PX4_INFO("Starting BaroThinClient");
        local_instance->start();
    } else if ((argc == 2) && (strcmp(argv[1], "stop") == 0)) {
        PX4_INFO("Stopping BaroThinClient");
        local_instance->stop();
    } else {
        PX4_ERR("Invalid command for BaroThinClient");
    }

    return 0;
}

void baro_thin_client_pressure_data(float pressure) {

    static uint32_t sample_msg_counter = 0;

    if (local_instance != nullptr) {
        if (local_instance->is_running()) {
            qurt_mutex_lock(&baro_mutex);
            baro_pressure = pressure;
            baro_sample_time = hrt_absolute_time();
            qurt_mutex_unlock(&baro_mutex);
        }
    }

    if (++sample_msg_counter == 50) {
        PX4_INFO("BARO sample at %llu %u", baro_sample_time, loop_counter);
        sample_msg_counter = 0;
    }
}

void baro_thin_client_temperature_data(float temperature) {
    if (local_instance) local_instance->update_temperature(temperature);
}
