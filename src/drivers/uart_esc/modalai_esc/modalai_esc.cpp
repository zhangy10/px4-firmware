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
#include "qc_esc_packet_types.h"

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"

const char *_device;

ModalaiEsc::ModalaiEsc() :
	CDev(MODALAI_ESC_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval"))
{
	_device = MODALAI_ESC_DEFAULT_PORT;

	// modaltb TODO
	//_mixing_output.setAllFailsafeValues(0);
	_mixing_output.setAllDisarmedValues(0);
	_mixing_output.setAllMinValues(1);
	_mixing_output.setAllMaxValues(2000);
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
	perf_free(_output_update_perf);
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

int ModalaiEsc::readResponse(Command *out_cmd)
{
	px4_usleep(_current_cmd.resp_delay_us);

	int res = _uart_port->uart_read(_current_cmd.buf, sizeof(_current_cmd.buf));

	if (res > 0) {
		if (parseResponse(_current_cmd.buf, res) < 0) {
			PX4_ERR("Error parsing response");
			return -1;
		}

	} else {
		PX4_ERR("Read error: %i", res);
		return -1;
	}

	_current_cmd.response = false;

	return 0;
}

int ModalaiEsc::parseResponse(uint8_t *buf, uint8_t len)
{
	if (len < 4) {
		PX4_ERR("Invalid packet length");
		return -1;
	}

	if (buf[0] != ESC_PACKET_HEADER) {
		PX4_ERR("Invalid packet start");
		return -1;
	}

	switch (buf[2]) {
	case ESC_PACKET_TYPE_VERSION_RESPONSE:
		if (len != sizeof(QC_ESC_VERSION_INFO)) {
			PX4_ERR("Invalid QC_ESC_VERSION_INFO length");
			return -1;

		} else {
			QC_ESC_VERSION_INFO ver;
			memcpy(&ver, buf, len);
			PX4_INFO("ESC ID: %i", ver.id);
			PX4_INFO("HW Version: %i", ver.hw_version);
			PX4_INFO("SW Version: %i", ver.sw_version);
			PX4_INFO("Unique ID: %i", ver.unique_id);
		}

		break;

	default:
		PX4_ERR("Unkown packet type: %i", buf[2]);
		return -1;
	}

	return 0;
}

int ModalaiEsc::sendCommandThreadSafe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}

int ModalaiEsc::custom_command(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	Command cmd;
	uint8_t esc_id = 255;
	uint8_t period = 0;
	uint8_t duration = 0;
	uint8_t power = 0;
	uint16_t led_mask = 0;
	int16_t rate = 0;

	if (argc < 3) {
		return print_usage("unknown command");
	}

	const char *verb = argv[argc - 1];

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

	while ((ch = px4_getopt(argc, argv, "i:p:d:v:l:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			esc_id = atoi(myoptarg);
			break;

		case 'p':
			period = atoi(myoptarg);
			break;

		case 'd':
			duration = atoi(myoptarg);
			break;

		case 'v':
			power = atoi(myoptarg);
			break;

		case 'l':
			led_mask = atoi(myoptarg);
			break;

		case 'r':
			rate = atoi(myoptarg);
			break;

		default:
			print_usage("Unknown command");
			return 0;
		}
	}

	if (!strcmp(verb, "reset")) {
		if (esc_id < 3) {
			PX4_INFO("Reset ESC: %i", esc_id);
			cmd.len = qc_esc_create_reset_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "version")) {
		if (esc_id < 3) {
			PX4_INFO("Request version for ESC: %i", esc_id);
			cmd.len = qc_esc_create_version_request_packet(esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = true;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC ID, use 0-3");
			return 0;
		}

	} else if (!strcmp(verb, "tone")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request tone for ESC mask: %i", esc_id);
			cmd.len = qc_esc_create_sound_packet(period, duration, power, esc_id, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "led")) {
		if (led_mask <= 0x0FFF) {
			PX4_INFO("Request LED control for ESCs with mask: %i", led_mask);
			uint8_t led_byte_1 = (led_mask & 0x00FF);
			uint8_t led_byte_2 = (led_mask & 0xFF00) >> 8;
			cmd.len = qc_esc_create_led_control_packet(led_byte_1, led_byte_2, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	}  else if (!strcmp(verb, "rpm")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request RPM for ESC mask: %i - RPM: %i", esc_id, rate);
			int16_t rpm0 = (esc_id & 1) ? rate : 0;
			int16_t rpm1 = (esc_id & 2) ? -rate : 0; // my hardware needs this reversed?
			int16_t rpm2 = (esc_id & 4) ? rate : 0;
			int16_t rpm3 = (esc_id & 8) ? rate : 0;

			cmd.len = qc_esc_create_rpm_packet4(rpm0, rpm1, rpm2, rpm3, 0, 0, 0, 0, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			cmd.repeats = 500;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}

	} else if (!strcmp(verb, "pwm")) {
		if (0 < esc_id && esc_id < 16) {
			PX4_INFO("Request PWM for ESC mask: %i - PWM: %i", esc_id, rate);
			int16_t pwm0 = (esc_id & 1) ? rate : 0;
			int16_t pwm1 = (esc_id & 2) ? -rate : 0; // my hardware needs this reversed?
			int16_t pwm2 = (esc_id & 4) ? rate : 0;
			int16_t pwm3 = (esc_id & 8) ? rate : 0;

			cmd.len = qc_esc_create_pwm_packet4(pwm0, pwm1, pwm2, pwm3, 0, 0, 0, 0, cmd.buf, sizeof(cmd.buf));
			cmd.response = false;
			cmd.repeats = 500;
			return get_instance()->sendCommandThreadSafe(&cmd);

		} else {
			print_usage("Invalid ESC mask, use 1-15");
			return 0;
		}
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
	if (!_outputs_on || num_outputs != MODALAI_ESC_OUTPUT_CHANNELS) {
		return false;
	}

	if (stop_motors) {
		_esc_chans.rate0 = 0;
		_esc_chans.rate1 = 0;
		_esc_chans.rate2 = 0;
		_esc_chans.rate3 = 0;

	} else {
		_esc_chans.rate0 = outputs[0] * 10; // max_rpm(outputs[0]*10);
		_esc_chans.rate1 = outputs[1] * 10; //max_rpm(outputs[1]*10);
		_esc_chans.rate2 = outputs[2] * 10; //max_rpm(outputs[2]*10);
		_esc_chans.rate3 = outputs[3] * 10; //max_rpm(outputs[3]*10);
		static int i = 0;

		if (i++ == 1000) {
			i = 0;
			PX4_INFO("%i %i %i %i", outputs[0], outputs[1], outputs[2], outputs[3]);
			PX4_INFO("-> %i %i %i %i", _esc_chans.rate0, _esc_chans.rate1, _esc_chans.rate2, _esc_chans.rate3);
		}
	}

	Command cmd;
	cmd.len = qc_esc_create_rpm_packet4(_esc_chans.rate0,
					    _esc_chans.rate1,
					    _esc_chans.rate2,
					    _esc_chans.rate3,
					    _esc_chans.led0,
					    _esc_chans.led1,
					    _esc_chans.led2,
					    _esc_chans.led3,
					    cmd.buf,
					    sizeof(cmd.buf));


	if (_uart_port->uart_write(cmd.buf, cmd.len) != cmd.len) {
		PX4_ERR("Failed to send packet");
		return false;
	}

	perf_count(_output_update_perf);

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
	bool armed = _mixing_output.armed().armed;

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

	if (_outputs_on) {

	} else {
		if (_current_cmd.valid()) {
			do {
				if (_uart_port->uart_write(_current_cmd.buf, _current_cmd.len) == _current_cmd.len) {
					if (_current_cmd.repeats == 0) {
						_current_cmd.clear();
					}

					if (_current_cmd.response) {
						readResponse(&_current_cmd);
					}

				} else {
					if (_current_cmd.retries == 0) {
						_current_cmd.clear();
						PX4_ERR("Failed to send command, errno: %i", errno);

					} else {
						_current_cmd.retries--;
						PX4_ERR("Failed to send command, errno: %i", errno);
					}
				}

				px4_usleep(_current_cmd.repeat_delay_us);
			} while (_current_cmd.repeats-- > 0);

		} else {
			Command *new_cmd = _pending_cmd.load();

			if (new_cmd) {
				_current_cmd = *new_cmd;
				_pending_cmd.store(nullptr);
			}
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
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("version", "Send version request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "ESC ID, 0-3", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("tone", "Send tone generation request to ESC");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 255, "Period of sound, inverse frequency, 0-255", false);
	PRINT_MODULE_USAGE_PARAM_INT('d', 0, 0, 255, "Duration of the sound, 0-255, 1LSB = 13ms", false);
	PRINT_MODULE_USAGE_PARAM_INT('v', 0, 0, 100, "Power (volume) of sound, 0-100", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "Send LED control request");
	PRINT_MODULE_USAGE_PARAM_INT('l', 0, 0, 4095, "Bitmask 0x0FFF (12 bits) - ESC0 (RGB) ESC1 (RGB) ESC2 (RGB) ESC3 (RGB)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("rpm", "Closed-Loop RPM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 3, "RPM, -32,7680 to 32,768", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("pwm", "Open-Loop PWM test control request");
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 15, "ESC ID bitmask, 1-15", false);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 2200, "PWM value, 0 to 2200", false);

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
	perf_print_counter(_output_update_perf);

	_mixing_output.printStatus();

	return 0;
}


extern "C" __EXPORT int modalai_esc_main(int argc, char *argv[]);

int modalai_esc_main(int argc, char *argv[])
{
	return ModalaiEsc::main(argc, argv);
}
