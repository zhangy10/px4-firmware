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

#include <drivers/device/device.h>
#include <drivers/drv_mixer.h>
#include <lib/cdev/CDev.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_module.h>

#include "modalai_esc_serial.hpp"

class ModalaiEsc : public cdev::CDev, public ModuleBase<ModalaiEsc>, public OutputModuleInterface
{
public:
	ModalaiEsc();
	virtual ~ModalaiEsc();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

	/** @see OutputModuleInterface */
	void mixerChanged() override;

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	virtual int	init();

	typedef enum {
		UART_ESC_RESET,
		UART_ESC_VERSION,
		UART_ESC_TONE,
		UART_ESC_LED
	} uart_esc_cmd_t;

	struct Command {
		uint16_t	id = 0;
		uint8_t 	BUF_SIZE = 255;
		uint8_t 	buf[255] = {0x00};
		uint8_t 	len;
		uint16_t	repeats = 0;
		uint16_t	repeat_delay_us = 2000;
		uint8_t		retries = 0;
		bool		response = false;
		uint16_t	resp_delay_us = 1000;

		bool valid() const { return len > 0; }
		void clear() { len = 0; }
	};

	struct EscChans {
		uint16_t rate0;
		uint16_t rate1;
		uint16_t rate2;
		uint16_t rate3;
		uint8_t led0;
		uint8_t led1;
		uint8_t led2;
		uint8_t led3;
	};

	int sendCommandThreadSafe(Command *cmd);

private:
	static constexpr uint16_t DISARMED_VALUE = 0;
	static constexpr uint16_t MODALAI_ESC_OUTPUT_CHANNELS = 4;
	static constexpr uint16_t MODALAI_ESC_PWM_MIN = 0;
	static constexpr uint16_t MODALAI_ESC_PWM_MAX = 800;
	static constexpr uint16_t MODALAI_ESC_RPM_MIN = 5000;
	static constexpr uint16_t MODALAI_ESC_RPM_MAX = 20000;

	static constexpr uint16_t max_pwm(uint16_t pwm) { return math::min(pwm, MODALAI_ESC_PWM_MAX); }
	static constexpr uint16_t max_rpm(uint16_t rpm) { return math::min(rpm, MODALAI_ESC_RPM_MAX); }

	ModalaiEscSerial 	*_uart_port;

	unsigned		_output_count = MODALAI_ESC_OUTPUT_CHANNELS;
	MixingOutput 		_mixing_output{MODALAI_ESC_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	int			_class_instance{-1};

	perf_counter_t		_cycle_perf;
	perf_counter_t		_output_update_perf;

	bool			_outputs_on{false};

	unsigned		_current_update_rate{0};

	uORB::Subscription 	_parameter_update_sub{ORB_ID(parameter_update)};
	void			update_params();

	uint16_t		_cmd_id{0};
	Command 		_current_cmd;
	px4::atomic<Command *>	_pending_cmd{nullptr};

	EscChans		_esc_chans;
	Command			_esc_cmd;

	int			populateCommand(uart_esc_cmd_t cmd_type, uint8_t cmd_mask, Command *out_cmd);
	int 			readResponse(Command *out_cmd);
	int 			parseResponse(uint8_t *buf, uint8_t len);
};
