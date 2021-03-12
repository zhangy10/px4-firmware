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

#include "baro_mpa_client.hpp"

int Baro_MPA_ClientModule::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Baro_MPA_ClientModule::custom_command(int argc, char *argv[])
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


int Baro_MPA_ClientModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("Baro_MPA_ClientModule",
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

Baro_MPA_ClientModule *Baro_MPA_ClientModule::instantiate(int argc, char *argv[])
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

	// Baro_MPA_ClientModule *instance = new Baro_MPA_ClientModule(example_param, example_flag);
	Baro_MPA_ClientModule *instance = new Baro_MPA_ClientModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

// Baro_MPA_ClientModule::Baro_MPA_ClientModule(int example_param, bool example_flag)
Baro_MPA_ClientModule::Baro_MPA_ClientModule()
	: ModuleParams(nullptr), _px4_baro(1)
{
}

void Baro_MPA_ClientModule::run()
{
	PX4_INFO("ModalAI Baro_MPA_ClientModule starting");

	while (!should_exit()) {

        usleep(100000);

        _px4_baro.set_temperature(23.5);
        _px4_baro.update(hrt_absolute_time(), 1013.25);

		parameters_update();
    }

	PX4_INFO("Baro_MPA_ClientModule Exiting");
}

void Baro_MPA_ClientModule::parameters_update(bool force)
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

int Baro_MPA_ClientModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	// TODO: PRINT_MODULE_DESCRIPTION

	PRINT_MODULE_USAGE_NAME("rc_controller", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("rc");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int baro_mpa_client_main(int argc, char *argv[])
{
	return Baro_MPA_ClientModule::main(argc, argv);
}
