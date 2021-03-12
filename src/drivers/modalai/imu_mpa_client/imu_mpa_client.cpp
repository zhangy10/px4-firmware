/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

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
#include <poll.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/getopt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include "imu_mpa_client.hpp"

int IMU_MPA_ClientModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int IMU_MPA_ClientModule::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int IMU_MPA_ClientModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("IMU_MPA_ClientModule",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

IMU_MPA_ClientModule *IMU_MPA_ClientModule::instantiate(int argc, char *argv[])
{
	// int example_param = 0;
	// bool example_flag = false;
	// bool error_flag = false;
    //
	// int myoptind = 1;
	// int ch;
	// const char *myoptarg = nullptr;
    //
	// // parse CLI arguments
	// while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
	// 	switch (ch) {
	// 	case 'p':
	// 		example_param = (int)strtol(myoptarg, nullptr, 10);
	// 		break;
    //
	// 	case 'f':
	// 		example_flag = true;
	// 		break;
    //
	// 	case '?':
	// 		error_flag = true;
	// 		break;
    //
	// 	default:
	// 		PX4_WARN("unrecognized flag");
	// 		error_flag = true;
	// 		break;
	// 	}
	// }
    //
	// if (error_flag) {
	// 	return nullptr;
	// }

	// IMU_MPA_ClientModule *instance = new IMU_MPA_ClientModule(example_param, example_flag);
	IMU_MPA_ClientModule *instance = new IMU_MPA_ClientModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

// IMU_MPA_ClientModule::IMU_MPA_ClientModule(int example_param, bool example_flag)
IMU_MPA_ClientModule::IMU_MPA_ClientModule()
	: ModuleParams(nullptr), _px4_accel(1), _px4_gyro(1)
{
}

void IMU_MPA_ClientModule::run()
{
	PX4_INFO("ModalAI IMU_MPA_ClientModule starting");

    const char *imu_fifo = "/dev/imu-pipe0";
    see_sensor_icm4x6xx_imu_data_t imu_data;

    struct stat pipe_stat;
    int stat_rc = lstat(imu_fifo, &pipe_stat);
    if (stat_rc == 0) {
        // The path already exists. Make sure it is a pipe.
        if ( ! S_ISFIFO(pipe_stat.st_mode)) {
            PX4_ERR("Error: %s exists but it is not a pipe", imu_fifo);
            return;
        } else {
            PX4_INFO("%s exists and it is a pipe", imu_fifo);
        }
    } else {
        // The pipe does not exist yet. Create it.
        int mkfifo_rc = mkfifo(imu_fifo, 0666);
        if (mkfifo_rc) {
            PX4_ERR("Error: Couldn't create pipe %s", imu_fifo);
            return;
        } else {
            PX4_INFO("Created pipe %s", imu_fifo);
        }
    }

    // The call to open will block until a writer attaches to the pipe
    int fifo_fd = open(imu_fifo, O_RDONLY);
    if (fifo_fd == -1) {
        PX4_ERR("Error: Couldn't open pipe %s", imu_fifo);
        return;
    } else {
        PX4_INFO("Opened pipe %s for reading", imu_fifo);
    }

    size_t data_len = sizeof(see_sensor_icm4x6xx_imu_data_t);
    ssize_t bytes_read = 0;
	while (!should_exit()) {
        bytes_read = read(fifo_fd, &imu_data, data_len);
        if ((bytes_read > 0) && (bytes_read == (ssize_t) data_len)) {

            // Convert 19.2MHz tick frequency to microseconds for PX4
            hrt_abstime slpi_time_ms = (hrt_abstime) (((double) imu_data.timestamp_slpi_ticks) / 19.2);

            _px4_accel.set_temperature(imu_data.temperature);
            _px4_accel.update(slpi_time_ms,
                              imu_data.accl_ms2[0],
                              imu_data.accl_ms2[1],
                              imu_data.accl_ms2[2]);

            _px4_gyro.set_temperature(imu_data.temperature);
            _px4_gyro.update(slpi_time_ms,
                             imu_data.gyro_rad[0],
                             imu_data.gyro_rad[1],
                             imu_data.gyro_rad[2]);

            // TODO: Incorporate into the status response
            // struct timespec rts;
            // clock_gettime(CLOCK_REALTIME, &rts);
            // struct timespec mts;
            // clock_gettime(CLOCK_MONOTONIC, &mts);
            // PX4_INFO("Read %ld bytes from %s", bytes_read, imu_fifo);
            // PX4_INFO("\tSLPI timestamp: %lu 19.2MHz ticks", imu_data.timestamp_slpi_ticks);
            // PX4_INFO("\tLocal timestamp: %lu ns", imu_data.timestamp_apps_real);
            // PX4_INFO("\tPX4 current timestamp: %lu ms", hrt_absolute_time());
            // PX4_INFO("\tlinux current RT timestamp: %lu ns", rts.tv_sec * 1000000000 + rts.tv_nsec);
            // PX4_INFO("\tlinux current MT timestamp: %lu ns", mts.tv_sec * 1000000000 + mts.tv_nsec);
            // PX4_INFO("\tTemperature: %f C", (double) imu_data.temperature);
            // for (unsigned int j = 0; j < 3; j++) {
            //     PX4_INFO("\tAccel[%u] = %f, Gyro[%u] = %f", j, (double) imu_data.accl_ms2[j], j, (double) imu_data.gyro_rad[j]);
            // }
        } else {
            PX4_ERR("Error: Couldn't read %lu bytes from the pipe", data_len);
            PX4_ERR("       read returned %ld", bytes_read);
            break;
        }

		parameters_update();
    }

    close(fifo_fd);

	PX4_INFO("IMU_MPA_ClientModule Exiting");
}

void IMU_MPA_ClientModule::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int IMU_MPA_ClientModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	// TODO: PRINT_MODULE_DESCRIPTION

	PRINT_MODULE_USAGE_NAME("imu_mpa_client", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int imu_mpa_client_main(int argc, char *argv[])
{
	return IMU_MPA_ClientModule::main(argc, argv);
}
