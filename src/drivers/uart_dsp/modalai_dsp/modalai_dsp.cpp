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

#include <px4_platform_common/getopt.h>

#include "modalai_dsp.hpp"
#include "modalai_dsp_serial.hpp"
#include "qc_esc_packet.h"
#include "qc_esc_packet_types.h"

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"

#ifdef __PX4_QURT
#define MODALAI_ESC_DEFAULT_PORT 	"2"
#else
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"
#endif

using matrix::wrap_2pi;

const char *_device_dsp;

ModalaiDSP::ModalaiDSP() :
	CDev(MODALAI_ESC_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_output_update_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval"))
{
	_device_dsp = MODALAI_ESC_DEFAULT_PORT;

	//_mixing_output.setAllFailsafeValues(0);
	//_mixing_output.setAllDisarmedValues(0);
}

ModalaiDSP::~ModalaiDSP()
{
	_outputs_on = false;

	if (_uart_port) {
		_uart_port->uart_close();
		_uart_port = nullptr;
	}

	/* clean up the alternate device node */
	unregister_class_devname(PWM_OUTPUT_BASE_DEVICE_PATH, _class_instance);

	perf_free(_cycle_perf);
	perf_free(_output_update_perf);
}

int ModalaiDSP::init()
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

	//_mixing_output.setDriverInstance(_class_instance);

	/* Getting initial parameter values */
	//ret = update_params();

	//if (ret != OK) {
	//	return ret;
	//}

	_uart_port = new ModalaiDSPSerial();
	//memset(&_esc_chans, 0x00, sizeof(_esc_chans));

	ScheduleNow();

	return 0;
}

int ModalaiDSP::load_params(uart_esc_params_t *params, ch_assign_t *map)
{
	int ret = PX4_OK;

	//param_get(param_find("UART_ESC_CONFIG"),  &params->config);
	//param_get(param_find("UART_ESC_BAUD"),    &params->baud_rate);
	//param_get(param_find("UART_ESC_MOTOR1"),  &params->motor_map[0]);
	//param_get(param_find("UART_ESC_MOTOR2"),  &params->motor_map[1]);
	//param_get(param_find("UART_ESC_MOTOR3"),  &params->motor_map[2]);
	//param_get(param_find("UART_ESC_MOTOR4"),  &params->motor_map[3]);
	//param_get(param_find("UART_ESC_RPM_MIN"), &params->rpm_min);
	//param_get(param_find("UART_ESC_RPM_MAX"), &params->rpm_max);

	//if (params->rpm_min >= params->rpm_max) {
	//	PX4_ERR("Invalid parameter UART_ESC_RPM_MIN.  Please verify parameters.");
	//	params->rpm_min = 0;
	//	ret = PX4_ERROR;
	//}

	//for (int i = 0; i < MODALAI_ESC_OUTPUT_CHANNELS; i++) {
	//	if (params->motor_map[i] == MODALAI_ESC_OUTPUT_DISABLED ||
	//	    params->motor_map[i] < -(MODALAI_ESC_OUTPUT_CHANNELS) ||
	//	    params->motor_map[i] > MODALAI_ESC_OUTPUT_CHANNELS) {
	//		PX4_ERR("Invalid parameter UART_ESC_MOTORX.  Please verify parameters.");
	//		params->motor_map[i] = 0;
	//		ret = PX4_ERROR;
	//	}

		/* Can map -4 to 4, 0 being disabled.  Negative represents reverse direction */
	//	map[i].number = abs(params->motor_map[i]);
	//	map[i].direction = (params->motor_map[i] > 0) ? 1 : -1;
	//}

	return ret;
}

int ModalaiDSP::task_spawn(int argc, char *argv[])
{
	int myoptind = 0;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_device_dsp = argv[myoptind];
			break;

		default:
			break;
		}
	}

	ModalaiDSP *instance = new ModalaiDSP();

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

int ModalaiDSP::readResponse(Command *out_cmd)
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

int ModalaiDSP::parseResponse(uint8_t *buf, uint8_t len)
{
	if (len < 4 || buf[0] != ESC_PACKET_HEADER) {
		return -1;
	}
	for (int i = 0; i <= len; i++){
    		PX4_ERR("BUFFER IN: %d", buf[i]);
	}
	switch (buf[2]) {
	case ESC_PACKET_TYPE_VERSION_RESPONSE:
		if (len != sizeof(QC_ESC_VERSION_INFO)) {
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

	case ESC_PACKET_TYPE_FB_RESPONSE:
    {
        // The extended feedback message and the feedback message use the same
        // message id so they need to be differentiated by message length.
        bool extended_fb = false;
		if (len == sizeof(QC_ESC_EXTENDED_FB_RESPONSE)) {
            extended_fb = true;
		} else if (len != (sizeof(QC_ESC_EXTENDED_FB_RESPONSE) - 4)) {
            // PX4_ERR("Got feedback response with invalid length %d", len);
			return -1;
		}

		QC_ESC_EXTENDED_FB_RESPONSE fb;
		memcpy(&fb, buf, len);
		uint8_t id = (fb.state & 0xF0) >> 4;

		if (id < MODALAI_ESC_OUTPUT_CHANNELS) {
			_esc_chans[id].rate_meas = fb.rpm;
			_esc_chans[id].state = fb.state & 0x0F;
			_esc_chans[id].cmd_counter = fb.cmd_counter;
			_esc_chans[id].power = fb.power;
			_esc_chans[id].voltage = fb.voltage * 0.001;

            if (extended_fb) {
				_esc_chans[id].current = fb.current * 0.008;
				_esc_chans[id].temperature = fb.temperature * 0.01;
                // PX4_INFO("EXT FB: id: %u, rpm: %u, state: %u, count: %u, pwr: %d, volts: %f, current %f, temperature %f",
                //           id, _esc_chans[id].rate_meas, _esc_chans[id].state,
                //           _esc_chans[id].cmd_counter, _esc_chans[id].power,
                //           _esc_chans[id].voltage, _esc_chans[id].current, _esc_chans[id].temperature);
            // } else {
            //     PX4_INFO("FB: id: %u, rpm: %u, state: %u, count: %u, pwr: %d, volts: %f",
            //               id, _esc_chans[id].rate_meas, _esc_chans[id].state,
            //               _esc_chans[id].cmd_counter, _esc_chans[id].power,
            //               _esc_chans[id].voltage);
            }

            // if (id == MODALAI_ESC_OUTPUT_CHANNELS - 1) {
            //     PX4_INFO("FB  %u %u %u %u", _esc_chans[0].rate_meas, _esc_chans[1].rate_meas, _esc_chans[2].rate_meas, _esc_chans[3].rate_meas);
            // }
		} else {
            PX4_ERR("Invalid ESC id %d in feedback packet", id);
        }
		break;
    }
	default:
		return -1;
	}

	return 0;
}

int ModalaiDSP::sendCommandThreadSafe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	/* wait until main thread processed it */
	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}



int ModalaiDSP::custom_command(int argc, char *argv[])
{
	//int myoptind = 0;
	//int ch;
	//const char *myoptarg = nullptr;

	Command cmd;
	//uint8_t esc_id = 255;
	//uint8_t period = 0;
	//uint8_t duration = 0;
	//uint8_t power = 0;
	//uint16_t led_mask = 0;
	//int16_t rate = 0;

	if (argc < 3) {
		return print_usage("unknown command");
	}

	const char *verb = argv[argc - 1];

	/* start the FMU if not running */
	if (!strcmp(verb, "start")) {
		if (!is_running()) {
			return ModalaiDSP::task_spawn(argc, argv);
		}
	}

	if (!is_running()) {
		PX4_INFO("Not running");
		return -1;
	}

	return print_usage("unknown command");
}

int ModalaiDSP::update_params()
{
	//int ret = PX4_ERROR;

	//updateParams();
	//ret = load_params(&_parameters, (ch_assign_t *)&_output_map);

	//if (ret == PX4_OK) {
	//	_mixing_output.setAllMinValues(_parameters.rpm_min);
	//	_mixing_output.setAllMaxValues(_parameters.rpm_max);
	//}

	//return ret;
	return 1;
}


int ModalaiDSP::ioctl(file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	//PX4_DEBUG("modalai_esc ioctl cmd: %d, arg: %ld", cmd, arg);

	//switch (cmd) {
	//case PWM_SERVO_ARM:
	//	PX4_INFO("PWM_SERVO_ARM");
	//	break;

	//case PWM_SERVO_DISARM:
	//	PX4_INFO("PWM_SERVO_DISARM");
	//	break;

	//case MIXERIOCRESET:
	//	_mixing_output.resetMixerThreadSafe();
	//	break;

	//case MIXERIOCLOADBUF: {
	//		const char *buf = (const char *)arg;
	//		unsigned buflen = strlen(buf);
	//		ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
	//	}
	//	break;

	//default:
	//	ret = -ENOTTY;
	//	break;
	//}

	/* if nobody wants it, let CDev have it */
	//if (ret == -ENOTTY) {
	//	ret = CDev::ioctl(filp, cmd, arg);
	//}

	return ret;
}

/* OutputModuleInterface */
void ModalaiDSP::mixerChanged()
{
	/*
	 * No need for mavlink understanding
	 */
}


void ModalaiDSP::updateLeds(vehicle_control_mode_s mode, led_control_s control)
{
	/*
	 * No need for mavlink understanding
	 */
}

/* OutputModuleInterface */
bool ModalaiDSP::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs, unsigned num_control_groups_updated)
{
	/*
	 * No need for mavlink understanding
	 */
	return 1;
}


void ModalaiDSP::Run()
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
		if (_uart_port->uart_open(_device_dsp, _parameters.baud_rate) == PX4_OK) {
			PX4_INFO("Opened UART ESC device");

		} else {
			PX4_ERR("Failed opening device");
			return;
		}
	}

	_mixing_output.update();

	/* update output status if armed */
	_outputs_on = _mixing_output.armed().armed;

	/* check for parameter updates */
	if (!_outputs_on && _parameter_update_sub.updated()) {
		/* clear update */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		/* update parameters from storage */
		update_params();
	}

	vehicle_control_mode_s vehicle_control_mode{};

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);
		updateLeds(vehicle_control_mode, _led_rsc.control);
	}

	led_control_s led_control{};

	if (_led_update_sub.updated()) {
		_led_update_sub.copy(&led_control);
		updateLeds(_led_rsc.mode, led_control);
	}

	/* breathing requires continuous updates */
	if (_led_rsc.breath_en) {
		updateLeds(_led_rsc.mode, _led_rsc.control);
	}

	/* Don't process commands if outputs on */
	if (!_outputs_on) {
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


int ModalaiDSP::print_usage(const char *reason)
{
	/*
	 * No need for mavlink understanding
	 */
	return 0;
}

int ModalaiDSP::print_status()
{
	/*
	 * No need for mavlink understanding
	 */

	return 0;
}

void
ModalaiDSP::handle_message_hil_sensor_dsp(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t hil_sensor;
	mavlink_msg_hil_sensor_decode(msg, &hil_sensor);

	const uint64_t timestamp = hrt_absolute_time();

	// temperature only updated with baro
	float temperature = NAN;

	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		temperature = hil_sensor.temperature;
	}

	// gyro
	if ((hil_sensor.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);
		}

		if (_px4_gyro != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_gyro->set_temperature(temperature);
			}

			_px4_gyro->update(timestamp, hil_sensor.xgyro, hil_sensor.ygyro, hil_sensor.zgyro);
		}
	}

	// accelerometer
	if ((hil_sensor.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);
		}

		if (_px4_accel != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_accel->set_temperature(temperature);
			}

			_px4_accel->update(timestamp, hil_sensor.xacc, hil_sensor.yacc, hil_sensor.zacc);
		}
	}

	// magnetometer
	if ((hil_sensor.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
		if (_px4_mag == nullptr) {
			// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
			_px4_mag = new PX4Magnetometer(197388);
		}

		if (_px4_mag != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_mag->set_temperature(temperature);
			}

			_px4_mag->update(timestamp, hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag);
		}
	}

	// baro
	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		if (_px4_baro == nullptr) {
			// 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
			_px4_baro = new PX4Barometer(6620172);
		}

		if (_px4_baro != nullptr) {
			_px4_baro->set_temperature(hil_sensor.temperature);
			_px4_baro->update(timestamp, hil_sensor.abs_pressure);
		}
	}

	// differential pressure
	if ((hil_sensor.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS) {
		differential_pressure_s report{};
		report.timestamp = timestamp;
		report.temperature = hil_sensor.temperature;
		report.differential_pressure_filtered_pa = hil_sensor.diff_pressure * 100.0f; // convert from millibar to bar;
		report.differential_pressure_raw_pa = hil_sensor.diff_pressure * 100.0f; // convert from millibar to bar;

		_differential_pressure_pub.publish(report);
	}

	// battery status
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.5f;
		hil_battery_status.voltage_filtered_v = 11.5f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		_battery_pub.publish(hil_battery_status);
	}
}

void
ModalaiDSP::handle_message_hil_gps_dsp(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	const uint64_t timestamp = hrt_absolute_time();

	sensor_gps_s hil_gps{};

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 0.1f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : wrap_2pi(math::radians(gps.cog * 1e-2f)));

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	hil_gps.heading = NAN;
	hil_gps.heading_offset = NAN;

	_gps_pub.publish(hil_gps);
}

extern "C" __EXPORT int modalai_dsp_main(int argc, char *argv[]);

int modalai_dsp_main(int argc, char *argv[])
{
	return ModalaiDSP::main(argc, argv);
}
