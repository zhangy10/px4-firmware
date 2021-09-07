/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ICP10100.hpp
 *
 * Driver for the ICP10100 barometric pressure sensor connected via I2C.
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/i2c_spi_buses.h>

class ICP10100 : public device::I2C, public I2CSPIDriver<ICP10100>
{
public:
	ICP10100(I2CSPIBusOption bus_option, const int bus, int bus_frequency);
	~ICP10100() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();


	int init() override;
	int probe() override;

	void print_status();

	void RunImpl();

private:
    static const uint8_t NUM_OTP_VALUES = 4;

    /* data structure to hold pressure sensor related parameters */
    struct inv_invpres {
        uint32_t min_delay_us;
        uint8_t pressure_en;
        uint8_t temperature_en;
        double sensor_constants[NUM_OTP_VALUES]; // OTP values
        double p_Pa_calib[3];
        double LUT_lower;
        double LUT_upper;
        double quadr_factor;
        double offst_factor;
    } sensor_params;

	void start();
	int  reset();

	int measure();
	int collect();

    int read_otp_from_i2c();

	int SendCommand(uint16_t cmd);
	int WriteData(const uint8_t *data, uint8_t len);
	int ReadData(uint8_t *data, uint8_t len);

    void CalculatePressure(int32_t raw_pressure, int32_t raw_temperature);

    void calculate_conversion_constants(double *p_Pa, double *p_LUT, double *out);

    double pressure;
    double temperature;

	PX4Barometer _px4_barometer;

	bool _collect_phase{false};

	perf_counter_t _sample_perf;
	perf_counter_t _measure_perf;
	perf_counter_t _comms_errors;
};
