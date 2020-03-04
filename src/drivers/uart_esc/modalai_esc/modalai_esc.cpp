/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_getopt.h>

#include "modalai_esc.hpp"
#include "modalai_esc_serial.hpp"
#include "qc_esc_packet.h"

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"

const char *_device;

ModalaiEsc::ModalaiEsc() :
	CDev(MODALAI_ESC_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_device = MODALAI_ESC_DEFAULT_PORT;

	// modaltb TODO
	_mixing_output.setAllFailsafeValues(1);
	_mixing_output.setAllDisarmedValues(2);
	_mixing_output.setAllMinValues(3);
	_mixing_output.setAllMaxValues(100);
}

ModalaiEsc::~ModalaiEsc()
{
	// modaltb TODO
	/* make sure servos are off */
	//up_pwm_servo_deinit();

	if (_uart_port) {
		_uart_port->uart_close();
		_uart_port = nullptr;
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
}

int ModalaiEsc::init()
{
	/* do regular cdev init */
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	_class_instance = register_class_devname(MODALAI_ESC_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* lets not be too verbose */
	} else if (_class_instance < 0) {
		PX4_ERR("FAILED registering class device");
	}

	_mixing_output.setDriverInstance(_class_instance);

	/* Getting initial parameter values */
	update_params();

	_uart_port = new ModalaiEscSerial();

	ScheduleNow();

	return 0;
}

int ModalaiEsc::task_spawn(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_device = argv[myoptind];
			break;

		default:
			break;
		}
	}

	ModalaiEsc *instance = new ModalaiEsc();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ModalaiEsc::sendCommandThreadSafe(int esc_id)
{
	// Modal TB
	Command cmd;
	cmd.esc_id = esc_id;
	cmd.num_repetitions = 1;
	_new_command.store(&cmd);

	/* wait until main thread processed it */
	while (_new_command.load()) {
		px4_usleep(1000);
	}

	return 0;
}

int ModalaiEsc::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;
	int esc_id = -1;

	if (argc < 3) {
		return print_usage("unknown command");
	}

	const char *verb = argv[2];

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return ModalaiEsc::task_spawn(argc, argv);
		}
	}

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;

	}

	while ((ch = px4_getopt(argc, argv, "i", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			esc_id = atoi(myoptarg);
			break;

		default:
			print_usage("Unknown command");
			return 0;
		}
	}

	if (!strcmp(verb, "reset")) {
		return get_instance()->sendCommandThreadSafe(esc_id);
	}

	return print_usage("unknown command");
}

void ModalaiEsc::update_params()
{
	updateParams();

	// we use a minimum value of 1, since 0 is for disarmed
	//_mixing_output.setAllMinValues(math::constrain((int)(_param_dshot_min.get() * (float)DSHOT_MAX_THROTTLE),
	//			       DISARMED_VALUE + 1, DSHOT_MAX_THROTTLE));

}


int ModalaiEsc::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PX4_DEBUG("modalai_esc ioctl cmd: %d, arg: %ld", cmd, arg);

	switch (cmd) {
	case PWM_SERVO_ARM:
		PX4_INFO("PWM_SERVO_ARM");
		break;

	case PWM_SERVO_DISARM:
		PX4_INFO("PWM_SERVO_DISARM");
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _output_count;
		break;

	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
		}
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filp, cmd, arg);
	}

	return ret;
}

/* OutputModuleInterface */
void ModalaiEsc::mixerChanged()
{
	// modaltb TODO
	//updateTelemetryNumMotors();
}

/* OutputModuleInterface */
bool ModalaiEsc::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			       unsigned num_outputs, unsigned num_control_groups_updated)
{
	static int test = 0;

	if (!_outputs_on) {
		return false;
	}

	if (stop_motors) {
		for (int i = 0; i < (int)num_outputs; i++) {
			//PX4_INFO("%i - %i", i, outputs[i]);
		}
	} else {
		if (test++ > 50) {
			for (int i = 0; i < (int)num_outputs; i++) {
				PX4_INFO("%i - %i", i, outputs[i]);
			}

			test = 0;
		}

		/* clear commands when motors are running */
		_current_command.clear();
	}



	// modaltb TODO
	/*
	int requested_telemetry_index = -1;

	if (_telemetry) {
		// check for an ESC info request. We only process it when we're not expecting other telemetry data
		if (_request_esc_info.load() != nullptr && !_waiting_for_esc_info && stop_motors
		    && !_telemetry->handler.expectingData() && !_current_command.valid()) {
			requested_telemetry_index = requestESCInfo();

		} else {
			requested_telemetry_index = _mixing_output.reorderedMotorIndex(_telemetry->handler.getRequestMotorIndex());
		}
	}

	if (stop_motors) {

		// when motors are stopped we check if we have other commands to send
		for (int i = 0; i < (int)num_outputs; i++) {
			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				// for some reason we need to always request telemetry when sending a command
				up_dshot_motor_command(i, _current_command.command, true);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);
			}
		}

		if (_current_command.valid()) {
			--_current_command.num_repetitions;
		}

	} else {
		for (int i = 0; i < (int)num_outputs; i++) {
			if (outputs[i] == DISARMED_VALUE) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, i == requested_telemetry_index);

			} else {
				up_dshot_motor_data_set(i, math::min(outputs[i], (uint16_t)DSHOT_MAX_THROTTLE), i == requested_telemetry_index);
			}
		}

		// clear commands when motors are running
		_current_command.clear();
	}

	if (stop_motors || num_control_groups_updated > 0) {
		up_dshot_trigger();
	}
	*/

	return true;
}


void ModalaiEsc::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open(_device) == PX4_OK) {
			PX4_INFO("Opened UART ESC device");

		} else {
			PX4_ERR("Failed openening device");
			return;
		}
	}

	_mixing_output.update();

	/* update output status if armed or if mixer is loaded */
	bool armed = _mixing_output.armed().armed || _mixing_output.mixers();

	if (armed != _outputs_on) {
		_outputs_on = armed;
		PX4_INFO("modalai uart esc - armed changed");

		//update_pwm_out_state(pwm_on);
	}

	/* check for parameter updates */
	if (_parameter_update_sub.updated()) {
		/* clear update */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		/* update parameters from storage */
		update_params();
	}

	// modaltb TODO - clean this up
	/* new CLI command? */
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}

	} else {
		if (!_outputs_on) {
			uint8_t buf[128];
			int len = qc_esc_create_reset_packet(_current_command.esc_id, buf, sizeof(buf));

			if (_uart_port->uart_write(buf, len) == len) {
				PX4_INFO("Send reset request to ESC: %i", _current_command.esc_id);

			} else {
				PX4_ERR("Failed to reset: %i", errno);
			}

			--_current_command.num_repetitions;
		}
	}

	/* check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread) */
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}


int ModalaiEsc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for...

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ todo

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("modalai_esc", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");

	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Send reset request to ESC");
	PRINT_MODULE_USAGE_ARG("<id>", "ESC ID number (0-3)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ModalaiEsc::print_status()
{
	PX4_INFO("Max update rate: %i Hz", _current_update_rate);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	PX4_INFO("UART port: %s", _device);
	PX4_INFO("UART open: %s", _uart_port->is_open() ? "yes" : "no");

	perf_print_counter(_cycle_perf);
	_mixing_output.printStatus();

	return 0;
}


extern "C" __EXPORT int modalai_esc_main(int argc, char *argv[]);

int modalai_esc_main(int argc, char *argv[])
{
	return ModalaiEsc::main(argc, argv);
}
