/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file uart_loopback.cpp
 *
 * This is a driver for a Spektrum satellite receiver connected to a Snapdragon
 * on the serial port. By default port J12 (next to J13, power module side) is used.
 */

#include <string.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#include <drivers/drv_hrt.h>

#ifdef __PX4_QURT
#include <drivers/device/qurt/uart.h>
#endif

#include <uORB/uORB.h>


// Snapdraogon: use J12 (next to J13, power module side)
#define SPEKTRUM_UART_DEVICE_PATH "/dev/tty-3"
#define ASYNC_UART_READ_WAIT_US 2000

#define UNUSED(x) (void)(x)

extern "C" { __EXPORT int uart_loopback_main(int argc, char *argv[]); }


namespace uart_loopback
{

volatile bool _task_should_exit = false;
static bool _is_running = false;
static px4_task_t _task_handle = -1;

static int esc_fd = -1;
static int gps_fd = -1;
static int rc_fd  = -1;

int start();
int stop();
int info();
void usage();
void task_main(int argc, char *argv[]);

void task_main(int argc, char *argv[])
{
	const char *device_path = SPEKTRUM_UART_DEVICE_PATH;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;
    bool verbose = true;

	while ((ch = px4_getopt(argc, argv, "vd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;
		case 'v':
			PX4_INFO("Spektrum RC: Enabling verbose mode");
			verbose = true;
			break;
		default:
			break;
		}
	}

	if (esc_fd < 0) {
		esc_fd = qurt_uart_open("2", 250000);
		if (esc_fd < 0) {
			PX4_ERR("uart open failed for esc port");
			return;
		} else if (verbose) {
			PX4_INFO("uart open successful for esc port");
		}
	} else {
		PX4_INFO("esc port already open");
	}

	px4_usleep(1000000);

	if (gps_fd < 0) {
		gps_fd = qurt_uart_open("6", 250000);
		if (gps_fd < 0) {
			PX4_ERR("uart open failed for gps port");
			return;
		} else if (verbose) {
			PX4_INFO("uart open successful for gps port");
		}
	} else {
		PX4_INFO("gps port already open");
	}

	px4_usleep(1000000);

	if (rc_fd < 0) {
		rc_fd  = qurt_uart_open("7", 250000);
		if (rc_fd < 0) {
			PX4_ERR("uart open failed for rc port");
			return;
		} else if (verbose) {
			PX4_INFO("uart open successful for rc port");
		}
	} else {
		PX4_INFO("rc port already open");
	}

	px4_usleep(1000000);

	uint8_t rx_buf[256];
	int bytes_written = 0;
	int bytes_read = 0;

	if (esc_fd > -1) {
		bytes_written = qurt_uart_write(esc_fd, (const char*) "Hello", 5);
		PX4_INFO("Wrote %d bytes to esc uart", bytes_written);

		bytes_read = qurt_uart_read(esc_fd, (char*) &rx_buf[0], sizeof(rx_buf), ASYNC_UART_READ_WAIT_US);
		if (bytes_read) {
			rx_buf[bytes_read] = 0;
			PX4_INFO("Read %d bytes from esc uart. Got %s", bytes_read, rx_buf);
		} else {
			PX4_ERR("Couldn't read any bytes from esc uart");
		}
	}

	px4_usleep(1000000);

	if (gps_fd > -1) {
		bytes_written = qurt_uart_write(gps_fd, (const char*) "Hello", 5);
		PX4_INFO("Wrote %d bytes to gps uart", bytes_written);

		bytes_read = qurt_uart_read(gps_fd, (char*) &rx_buf[0], sizeof(rx_buf), ASYNC_UART_READ_WAIT_US);
		if (bytes_read) {
			rx_buf[bytes_read] = 0;
			PX4_INFO("Read %d bytes from gps uart. Got %s", bytes_read, rx_buf);
		} else {
			PX4_ERR("Couldn't read any bytes from gps uart");
		}
	}

	px4_usleep(1000000);

	if (rc_fd > -1) {
		bytes_written = qurt_uart_write(rc_fd, (const char*) "Hello", 5);
		PX4_INFO("Wrote %d bytes to rc uart", bytes_written);

		bytes_read = qurt_uart_read(rc_fd, (char*) &rx_buf[0], sizeof(rx_buf), ASYNC_UART_READ_WAIT_US);
		if (bytes_read) {
			rx_buf[bytes_read] = 0;
			PX4_INFO("Read %d bytes from rc uart. Got %s", bytes_read, rx_buf);
		} else {
			PX4_ERR("Couldn't read any bytes from rc uart");
		}
	}
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("uart_loopback_main",
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
	PX4_INFO("Usage: uart_loopback {start|info|stop}");
}

} // namespace uart_loopback


int uart_loopback_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		uart_loopback::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		return uart_loopback::start(argc - 1, argv + 1);
	}

	else if (!strcmp(verb, "stop")) {
		return uart_loopback::stop();
	}

	else if (!strcmp(verb, "info")) {
		return uart_loopback::info();
	}

	else {
		uart_loopback::usage();
		return 1;
	}
}
