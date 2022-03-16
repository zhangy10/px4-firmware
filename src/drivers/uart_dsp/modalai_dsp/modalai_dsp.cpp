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

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <uORB/Publication.hpp>

#include <lib/cdev/CDev.hpp>
#include <lib/cdev/CDev.hpp>


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

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
int _uart_fd = -1;

int openPort(const char *dev, speed_t speed);
int closePort();

int readResponse(FAR void *buf, size_t len);
int writeResponse(FAR void *buf, size_t len);

int start(int argc, char *argv[]);
int stop();
int info();
bool isOpen() { return _uart_fd >= 0; };

void usage();
void task_main(int argc, char *argv[]);

void handle_message_hil_sensor_dsp();
void handle_message_hil_gps_dsp();

void task_main(int argc, char *argv[])
{
	int openRetval = openPort(MODALAI_ESC_DEFAULT_PORT, 250000);
	int open = isOpen();
	if(open){
		PX4_ERR("Port is open: %d", openRetval);
	}

	uint8_t rx_buf[255];

	while (!_task_should_exit){
		int readRetval = readResponse(&rx_buf[0], sizeof(rx_buf));
		if(readRetval){
			PX4_ERR("Value of rx_buff: %s", rx_buf);
		}
		sleep(1);

		int writeRetval = writeResponse(&rx_buf[0], sizeof(rx_buf));
		if(writeRetval){
			PX4_ERR("Write published");
		}
		sleep(1);
	}
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

int readResponse(FAR void *buf, size_t len)
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

int writeResponse(FAR void *buf, size_t len)
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

