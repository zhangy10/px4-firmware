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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include "uart_esc_driver.hpp"

int UART_ESC_DriverModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int UART_ESC_DriverModule::custom_command(int argc, char *argv[])
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


int UART_ESC_DriverModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("UART_ESC_DriverModule",
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

UART_ESC_DriverModule *UART_ESC_DriverModule::instantiate(int argc, char *argv[])
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

	// UART_ESC_DriverModule *instance = new UART_ESC_DriverModule(example_param, example_flag);
	UART_ESC_DriverModule *instance = new UART_ESC_DriverModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

// UART_ESC_DriverModule::UART_ESC_DriverModule(int example_param, bool example_flag)
UART_ESC_DriverModule::UART_ESC_DriverModule()
	: ModuleParams(nullptr)
{
}

void UART_ESC_DriverModule::run()
{
	PX4_INFO("ModalAI UART_ESC_DriverModule starting");

	int	_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
    actuator_controls_s _controls;
    memset(&_controls, 0, sizeof(actuator_controls_s));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = _controls_sub;
	fds[0].events = POLLIN;
	//orb_set_interval(_controls_sub, 10);  // max actuator update period, ms

	while (!should_exit()) {

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		// Handle new actuator controls data
		if (fds[0].revents & POLLIN) {
			// Grab new controls data
			orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls);

            PX4_INFO("UART ESC: 0x%lx 0x%lx", _controls.timestamp, _controls.timestamp_sample);
            PX4_INFO("UART ESC: %f %f %f %f", (double) _controls.control[0], (double) _controls.control[1],
                                              (double) _controls.control[2], (double) _controls.control[3]);
        	// float control[8];
        }

		parameters_update();
    }

	PX4_INFO("UART_ESC_DriverModule Exiting");
}

void UART_ESC_DriverModule::parameters_update(bool force)
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

int UART_ESC_DriverModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	// TODO: PRINT_MODULE_DESCRIPTION

	PRINT_MODULE_USAGE_NAME("uart_esc_driver", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int uart_esc_driver_main(int argc, char *argv[])
{
	return UART_ESC_DriverModule::main(argc, argv);
}
