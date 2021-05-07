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

#pragma once

#include <px4_log.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <drivers/device/qurt/uart.h>

#ifdef __PX4_QURT
#define FAR
#endif

class ModalaiEscSerial
{
public:
	ModalaiEscSerial();
	virtual ~ModalaiEscSerial();

	int		uart_open(const char *dev, speed_t speed);
	int		uart_close();
	int		uart_write(FAR void *buf, size_t len);
	int		uart_read(FAR void *buf, size_t len);
	bool		is_open() { return _uart_fd >= 0; };

#ifdef __PX4_QURT
    static bool _callbacks_configured;

    static void configure_callbacks(open_uart_func_t open_func,
                                    write_uart_func_t write_func,
                                    read_uart_func_t read_func) {
        _open_uart  = open_func;
        _write_uart = write_func;
        _read_uart  = read_func;
        if (_open_uart && _write_uart && _read_uart) _callbacks_configured = true;
    }
#endif

private:
	int			       _uart_fd = -1;

#ifdef __PX4_QURT
    static open_uart_func_t  _open_uart;
    static write_uart_func_t _write_uart;
    static read_uart_func_t  _read_uart;
#else
	struct termios		_orig_cfg;
	struct termios		_cfg;
#endif
};
