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

#include <string.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#ifdef __PX4_QURT
#include <drivers/device/qurt/uart.h>
#endif

#include <commander/px4_custom_mode.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <v2.0/mavlink_types.h>
#include <v2.0/standard/mavlink.h>
#include <v2.0/standard/standard.h>
#include <v2.0/protocol.h>
#include <v2.0/mavlink_helpers.h>
#include <v2.0/minimal/mavlink_msg_heartbeat.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/telemetry_status.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>

#include <unistd.h>

#define MODALAI_ESC_DEVICE_PATH 	"/dev/uart_esc"
#define ASYNC_UART_READ_WAIT_US 2000

#ifdef __PX4_QURT
#define MODALAI_ESC_DEFAULT_PORT 	"2"
#else
#define MODALAI_ESC_DEFAULT_PORT 	"/dev/ttyS1"
#endif


extern "C" { __EXPORT int modalai_dsp_main(int argc, char *argv[]); }

namespace modalai_dsp
{

using matrix::wrap_2pi;

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
int _uart_fd = -1;

uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
uORB::Publication<sensor_gps_s>				_gps_pub{ORB_ID(sensor_gps)};
uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};

// hil_sensor and hil_state_quaternion
enum SensorSource {
	ACCEL		= 0b111,
	GYRO		= 0b111000,
	MAG		= 0b111000000,
	BARO		= 0b1101000000000,
	DIFF_PRESS	= 0b10000000000
};

PX4Accelerometer *_px4_accel{nullptr};
PX4Barometer *_px4_baro{nullptr};
PX4Gyroscope *_px4_gyro{nullptr};
PX4Magnetometer *_px4_mag{nullptr};

hrt_abstime _last_heartbeat_check{0};

hrt_abstime _heartbeat_type_antenna_tracker{0};
hrt_abstime _heartbeat_type_gcs{0};
hrt_abstime _heartbeat_type_onboard_controller{0};
hrt_abstime _heartbeat_type_gimbal{0};
hrt_abstime _heartbeat_type_adsb{0};
hrt_abstime _heartbeat_type_camera{0};

hrt_abstime _heartbeat_component_telemetry_radio{0};
hrt_abstime _heartbeat_component_log{0};
hrt_abstime _heartbeat_component_osd{0};
hrt_abstime _heartbeat_component_obstacle_avoidance{0};
hrt_abstime _heartbeat_component_visual_inertial_odometry{0};
hrt_abstime _heartbeat_component_pairing_manager{0};
hrt_abstime _heartbeat_component_udp_bridge{0};
hrt_abstime _heartbeat_component_uart_bridge{0};

vehicle_status_s _vehicle_status{};
actuator_outputs_s _actuator_outputs{};
actuator_controls_s _actuator_controls{};

const unsigned mode_flag_armed = 128;
const unsigned mode_flag_custom = 1;

int openPort(const char *dev, speed_t speed);
int closePort();

int readResponse(void *buf, size_t len);
int writeResponse(void *buf, size_t len);

int start(int argc, char *argv[]);
int stop();
int info();
bool isOpen() { return _uart_fd >= 0; };

void usage();
void task_main(int argc, char *argv[]);

void handle_message_hil_sensor_dsp(mavlink_message_t *msg);
void handle_message_hil_gps_dsp(mavlink_message_t *msg);
void handle_message_heartbeat_dsp(mavlink_message_t *msg);
void CheckHeartbeats(const hrt_abstime &t, bool force);
void handle_message_dsp(mavlink_message_t *msg);
void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg);

void
handle_message_dsp(mavlink_message_t *msg)
{
	PX4_DEBUG("msg ID: %d", msg->msgid);
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		handle_message_hil_sensor_dsp(msg);
		PX4_DEBUG("MAVLINK HIL SENSOR");
		break;
	case MAVLINK_MSG_ID_HIL_GPS:
		handle_message_hil_gps_dsp(msg);
		PX4_DEBUG("MAVLINK HIL GPS");
		break;
	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat_dsp(msg);
		PX4_DEBUG("MAVLINK HEART");

		mavlink_heartbeat_t hb = {};
		mavlink_message_t message = {};
		hb.autopilot = 12;
		hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
		mavlink_msg_heartbeat_encode(1, 1, &message, &hb);

		uint8_t  newBuf[MAVLINK_MAX_PACKET_LEN];
		uint16_t newBufLen = 0;
		newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
		int writeRetval = writeResponse(&newBuf, newBufLen);
		PX4_DEBUG("Succesful write of heartbeat back to jMAVSim: %d", writeRetval);
		break;
	}
}

void task_main(int argc, char *argv[])
{
	int openRetval = openPort(MODALAI_ESC_DEFAULT_PORT, 250000);
	int open = isOpen();
	if(open){
		PX4_ERR("Port is open: %d", openRetval);
	}

	//int _act_sub = orb_subscribe(ORB_ID(actuator_outputs));
	int _actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs), 0);
	PX4_DEBUG("Got %d from orb_subscribe", _actuator_outputs_sub);
	int _vehicle_control_mode_sub_ = orb_subscribe(ORB_ID(vehicle_control_mode));
	PX4_DEBUG("Got %d from orb_subscribe", _vehicle_control_mode_sub_);
	int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	PX4_DEBUG("Got %d from orb_subscribe", _vehicle_status_sub);
	int _actuator_controls_sub = orb_subscribe(ORB_ID(actuator_controls));
	PX4_DEBUG("Got %d from orb_subscribe", _actuator_controls_sub);

	while (!_task_should_exit){

		uint8_t rx_buf[256];
		rx_buf[255] = '\0';

		int readRetval = readResponse(&rx_buf[0], sizeof(rx_buf));
		if(readRetval){
			PX4_DEBUG("Value of rx_buff: %s", rx_buf);
		}

		//Take readRetval and convert it into mavlink msg
		mavlink_message_t msg;
		mavlink_status_t _status{};

		bool vehicle_updated = false;
       		(void) orb_check(_vehicle_status_sub, &vehicle_updated);
		if(vehicle_updated){
			PX4_DEBUG("Value of updated vehicle status: %d", vehicle_updated);
			orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		}

		for (int i = 0; i <= readRetval; i++){
			if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &_status)) {
				handle_message_dsp(&msg);
			}
		}

		bool actuator_updated = false;
       		(void) orb_check(_actuator_outputs_sub, &actuator_updated);
		if(actuator_updated){
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
			PX4_DEBUG("Value of updated actuator: %d", actuator_updated);
		}

		if (_actuator_outputs.timestamp > 0) {
			mavlink_hil_actuator_controls_t hil_act_control;
			actuator_controls_from_outputs_dsp(&hil_act_control);

			mavlink_message_t message{};
			mavlink_msg_hil_actuator_controls_encode(1, 1, &message, &hil_act_control);

			uint8_t  newBuf[MAVLINK_MAX_PACKET_LEN];
			uint16_t newBufLen = 0;
			newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
			int writeRetval = writeResponse(&newBuf, newBufLen);
			PX4_DEBUG("Succesful write of actuator back to jMAVSim: %d", writeRetval);
		}
	}
}

void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg)
{
	memset(msg, 0, sizeof(mavlink_hil_actuator_controls_t));

	msg->time_usec = hrt_absolute_time() + hrt_absolute_time_offset();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	int _system_type = MAV_TYPE_QUADROTOR;

	/* 'pos_thrust_motors_count' indicates number of motor channels which are configured with 0..1 range (positive thrust)
	all other motors are configured for -1..1 range */
	unsigned pos_thrust_motors_count;
	bool is_fixed_wing;

	switch (_system_type) {
	case MAV_TYPE_AIRSHIP:
	case MAV_TYPE_VTOL_DUOROTOR:
	case MAV_TYPE_COAXIAL:
		pos_thrust_motors_count = 2;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_TRICOPTER:
		pos_thrust_motors_count = 3;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_QUADROTOR:
	case MAV_TYPE_VTOL_QUADROTOR:
	case MAV_TYPE_VTOL_TILTROTOR:
		pos_thrust_motors_count = 4;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_VTOL_RESERVED2:
		pos_thrust_motors_count = 5;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_HEXAROTOR:
		pos_thrust_motors_count = 6;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_OCTOROTOR:
		pos_thrust_motors_count = 8;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_SUBMARINE:
		pos_thrust_motors_count = 0;
		is_fixed_wing = false;
		break;

	case MAV_TYPE_FIXED_WING:
		pos_thrust_motors_count = 0;
		is_fixed_wing = true;
		break;

	default:
		pos_thrust_motors_count = 0;
		is_fixed_wing = false;
		break;
	}

	for (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
		if (!armed) {
			/* send 0 when disarmed and for disabled channels */
			msg->controls[i] = 0.0f;

		} else if ((is_fixed_wing && i == 4) ||
			   (!is_fixed_wing && i < pos_thrust_motors_count)) {	//multirotor, rotor channel
			/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for rotors */
			msg->controls[i] = (_actuator_outputs.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
			msg->controls[i] = math::constrain(msg->controls[i], 0.f, 1.f);

		} else {
			const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;
			const float pwm_delta = (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2;

			/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for other channels */
			msg->controls[i] = (_actuator_outputs.output[i] - pwm_center) / pwm_delta;
			msg->controls[i] = math::constrain(msg->controls[i], -1.f, 1.f);
		}

	}
	msg->mode |= MAV_MODE_FLAG_HIL_ENABLED;
	msg->mode = mode_flag_custom;
	msg->mode |= (armed) ? mode_flag_armed : 0;
	msg->flags = 0;

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	msg->flags |= 1;
#endif
}

int openPort(const char *dev, speed_t speed)
{
	if (_uart_fd >= 0) {
		PX4_ERR("Port in use: %s (%i)", dev, errno);
		return -1;
	}

#ifdef __PX4_QURT
	_uart_fd = qurt_uart_open(dev, speed);
	PX4_ERR("qurt_uart_opened");
#else
	/* Open UART */
	_uart_fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
#endif

	if (_uart_fd < 0) {
		PX4_ERR("Error opening port: %s (%i)", dev, errno);
		return -1;
	}

#ifndef __PX4_QURT
	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(_uart_fd, &_orig_cfg)) < 0) {
		PX4_ERR("Error configuring port: tcgetattr %s: %d", dev, termios_state);
		uart_close();
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(_uart_fd, &_cfg);

	/* Disable output post-processing */
	_cfg.c_oflag &= ~OPOST;

	_cfg.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	_cfg.c_cflag &= ~CSIZE;
	_cfg.c_cflag |= CS8;                 /* 8-bit characters */
	_cfg.c_cflag &= ~PARENB;             /* no parity bit */
	_cfg.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
	_cfg.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	_cfg.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	_cfg.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	if (cfsetispeed(&_cfg, speed) < 0 || cfsetospeed(&_cfg, speed) < 0) {
		PX4_ERR("Error configuring port: %s: %d (cfsetispeed, cfsetospeed)", dev, termios_state);
		uart_close();
		return -1;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &_cfg)) < 0) {
		PX4_ERR("Error configuring port: %s (tcsetattr)", dev);
		uart_close();
		return -1;
	}
#endif

	return 0;
}

int closePort()
{
#ifndef __PX4_QURT
	if (_uart_fd < 0) {
		PX4_ERR("invalid state for closing");
		return -1;
	}

	if (tcsetattr(_uart_fd, TCSANOW, &_orig_cfg)) {
		PX4_ERR("failed restoring uart to original state");
	}

	if (close(_uart_fd)) {
		PX4_ERR("error closing uart");
	}
#endif

	_uart_fd = -1;

	return 0;
}

int readResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for reading or buffer");
		return -1;
	}

#ifdef __PX4_QURT
#define ASYNC_UART_READ_WAIT_US 2000
    // The UART read on SLPI is via an asynchronous service so specify a timeout
    // for the return. The driver will poll periodically until the read comes in
    // so this may block for a while. However, it will timeout if no read comes in.
    return qurt_uart_read(_uart_fd, (char*) buf, len, ASYNC_UART_READ_WAIT_US);
#else
	return read(_uart_fd, buf, len);
#endif
}

int writeResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for writing or buffer");
		return -1;
	}

#ifdef __PX4_QURT
    return qurt_uart_write(_uart_fd, (const char*) buf, len);
#else
	return write(_uart_fd, buf, len);
#endif
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("modalai_dsp__main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	if (!_is_running) {
		PX4_WARN("not running");
		return -1;
	}

	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

int info()
{
	PX4_INFO("running: %s", _is_running ? "yes" : "no");

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: modalai_dsp {start|info|stop}");
}

void
handle_message_hil_sensor_dsp(mavlink_message_t *msg)
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
handle_message_hil_gps_dsp(mavlink_message_t *msg)
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

void
handle_message_heartbeat_dsp(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	const hrt_abstime now = hrt_absolute_time();

	mavlink_heartbeat_t hb;
	mavlink_msg_heartbeat_decode(msg, &hb);

	const bool same_system = 1;

	if (same_system || hb.type == MAV_TYPE_GCS) {

		switch (hb.type) {
		case MAV_TYPE_ANTENNA_TRACKER:
			_heartbeat_type_antenna_tracker = now;
			break;

		case MAV_TYPE_GCS:
			_heartbeat_type_gcs = now;
			break;

		case MAV_TYPE_ONBOARD_CONTROLLER:
			_heartbeat_type_onboard_controller = now;
			break;

		case MAV_TYPE_GIMBAL:
			_heartbeat_type_gimbal = now;
			break;

		case MAV_TYPE_ADSB:
			_heartbeat_type_adsb = now;
			break;

		case MAV_TYPE_CAMERA:
			_heartbeat_type_camera = now;
			break;

		default:
			PX4_DEBUG("unhandled HEARTBEAT MAV_TYPE: %d from SYSID: %d, COMPID: %d", hb.type, msg->sysid, msg->compid);
		}


		switch (msg->compid) {
		case MAV_COMP_ID_TELEMETRY_RADIO:
			_heartbeat_component_telemetry_radio = now;
			break;

		case MAV_COMP_ID_LOG:
			_heartbeat_component_log = now;
			break;

		case MAV_COMP_ID_OSD:
			_heartbeat_component_osd = now;
			break;

		case MAV_COMP_ID_OBSTACLE_AVOIDANCE:
			_heartbeat_component_obstacle_avoidance = now;
			//_mavlink->telemetry_status().avoidance_system_healthy = (hb.system_status == MAV_STATE_ACTIVE);
			break;

		case MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY:
			_heartbeat_component_visual_inertial_odometry = now;
			break;

		case MAV_COMP_ID_PAIRING_MANAGER:
			_heartbeat_component_pairing_manager = now;
			break;

		case MAV_COMP_ID_UDP_BRIDGE:
			_heartbeat_component_udp_bridge = now;
			break;

		case MAV_COMP_ID_UART_BRIDGE:
			_heartbeat_component_uart_bridge = now;
			break;

		default:
			PX4_DEBUG("unhandled HEARTBEAT MAV_TYPE: %d from SYSID: %d, COMPID: %d", hb.type, msg->sysid, msg->compid);
		}

		CheckHeartbeats(now, true);
	}

}

void CheckHeartbeats(const hrt_abstime &t, bool force)
{
	// check HEARTBEATs for timeout
	static constexpr uint64_t TIMEOUT = telemetry_status_s::HEARTBEAT_TIMEOUT_US;

	if (t <= TIMEOUT) {
		return;
	}

	//int _telemetry_status_sub = orb_subscribe(ORB_ID(telemetry_status));
		// 	vehicle_control_mode_s control_mode;

		// 	if (_vehicle_control_mode_sub.copy(&control_mode)) {
	// if ((t >= _last_heartbeat_check + (TIMEOUT / 2)) || force) {
	// 	telemetry_status_s tstatus;

	// 	tstatus.heartbeat_type_antenna_tracker         = (t <= TIMEOUT + _heartbeat_type_antenna_tracker);
	// 	tstatus.heartbeat_type_gcs                     = (t <= TIMEOUT + _heartbeat_type_gcs);
	// 	tstatus.heartbeat_type_onboard_controller      = (t <= TIMEOUT + _heartbeat_type_onboard_controller);
	// 	tstatus.heartbeat_type_gimbal                  = (t <= TIMEOUT + _heartbeat_type_gimbal);
	// 	tstatus.heartbeat_type_adsb                    = (t <= TIMEOUT + _heartbeat_type_adsb);
	// 	tstatus.heartbeat_type_camera                  = (t <= TIMEOUT + _heartbeat_type_camera);

	// 	tstatus.heartbeat_component_telemetry_radio    = (t <= TIMEOUT + _heartbeat_component_telemetry_radio);
	// 	tstatus.heartbeat_component_log                = (t <= TIMEOUT + _heartbeat_component_log);
	// 	tstatus.heartbeat_component_osd                = (t <= TIMEOUT + _heartbeat_component_osd);
	// 	tstatus.heartbeat_component_obstacle_avoidance = (t <= TIMEOUT + _heartbeat_component_obstacle_avoidance);
	// 	tstatus.heartbeat_component_vio                = (t <= TIMEOUT + _heartbeat_component_visual_inertial_odometry);
	// 	tstatus.heartbeat_component_pairing_manager    = (t <= TIMEOUT + _heartbeat_component_pairing_manager);
	// 	tstatus.heartbeat_component_udp_bridge         = (t <= TIMEOUT + _heartbeat_component_udp_bridge);
	// 	tstatus.heartbeat_component_uart_bridge        = (t <= TIMEOUT + _heartbeat_component_uart_bridge);

	// 	//_mavlink->telemetry_status_updated();
	// 	_last_heartbeat_check = t;
	// }
}

}
int modalai_dsp_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		modalai_dsp::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return modalai_dsp::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return modalai_dsp::stop();
	}

	else if (!strcmp(verb, "info")) {
		return modalai_dsp::info();
	}

	else {
		modalai_dsp::usage();
		return 1;
	}
}

