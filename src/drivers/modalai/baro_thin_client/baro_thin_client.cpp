
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

static BaroThinClient *local_instance;

BaroThinClient::BaroThinClient() : _px4_baro(1) {}

int baro_thin_client_main(int argc, char *argv[])
{
    if (local_instance == nullptr) local_instance = new BaroThinClient();

    if (local_instance == nullptr) {
        PX4_ERR("Couldn't instantiate BaroThinClient");
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
    if (local_instance) local_instance->update_pressure(pressure);
}

void baro_thin_client_temperature_data(float temperature) {
    if (local_instance) local_instance->update_temperature(temperature);
}
