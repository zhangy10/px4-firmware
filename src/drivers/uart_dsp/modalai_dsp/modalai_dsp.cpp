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

#include <unistd.h>

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

	_uart_port = new ModalaiDSPSerial();

	ScheduleNow();

	return 0;
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
	PX4_ERR("VALUE OR RES: %d", res);
	PX4_ERR("SIZE OR CMD BUF: %d", sizeof(_current_cmd.buf));

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
	PX4_ERR("LENGTH OF BUFFER: %d", len);
	for (int i = 0; i <= len; i++){
    		PX4_ERR("BUFFER IN: %d", buf[i]);
	}
	return 0;
}

int ModalaiDSP::sendCommandThreadSafe(Command *cmd)
{
	cmd->id = _cmd_id++;
	_pending_cmd.store(cmd);

	while (_pending_cmd.load()) {
		px4_usleep(1000);
	}

	return 0;
}



int ModalaiDSP::custom_command(int argc, char *argv[])
{

	Command cmd;

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

void ModalaiDSP::Run()
{

	perf_begin(_cycle_perf);

	/* Open serial port in this thread */
	if (!_uart_port->is_open()) {
		if (_uart_port->uart_open(_device_dsp, 250000) == PX4_OK) {
			PX4_ERR("Opened UART ESC device");

		} else {
			PX4_ERR("Failed opening device");
			return;
		}
	}

	/* Don't process commands if outputs on */
	while (_uart_port->is_open()) {
		PX4_ERR("GOING INTO READ RESPONSE");
		readResponse(&_current_cmd);
		PX4_ERR("FINISHED READ RESPONSE");
		sleep(3);
	}

	perf_end(_cycle_perf);
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

/* OutputModuleInterface */
void ModalaiDSP::mixerChanged()
{
	/*
	 * This driver is only supporting 4 channel ESC
	 */
}

/* OutputModuleInterface */
int ModalaiDSP::print_status()
{
	return 1;
	/*
	 * This driver is only supporting 4 channel ESC
	 */
}

/* OutputModuleInterface */
int ModalaiDSP::print_usage(const char *reason)
{
	return 1;
	/*
	 * This driver is only supporting 4 channel ESC
	 */
}

/* OutputModuleInterface */
bool ModalaiDSP::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			       unsigned num_outputs, unsigned num_control_groups_updated)
{
	return true;
}

int ModalaiDSP::ioctl(file_t *filp, int cmd, unsigned long arg)
{
	return 1;
}



extern "C" __EXPORT int modalai_dsp_main(int argc, char *argv[]);

int modalai_dsp_main(int argc, char *argv[])
{
	return ModalaiDSP::main(argc, argv);
}
