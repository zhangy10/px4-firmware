/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "ICP10100.hpp"

#define ICP10100_ADDRESS        0x63

// ICP10100 command set
#define ICP10100_SOFT_RESET_CMD 0x805D
#define ICP10100_READ_ID_CMD    0xEFC8
#define ICP10100_READ_OTP_CMD   0xC7F7
#define ICP10100_START_LN_CMD   0x5059

// ICP10100 read values
#define ICP10100_ID_MASK        0x3F
#define ICP10100_ID_VAL         0x08

const uint8_t ICP10100::NUM_OTP_VALUES;

ICP10100::ICP10100(I2CSPIBusOption bus_option, const int bus, int bus_frequency) :
	I2C(DRV_BARO_DEVTYPE_ICP10100, MODULE_NAME, bus, ICP10100_ADDRESS, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_px4_barometer(get_device_id()),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_measure_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": measure")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err"))
{
}

ICP10100::~ICP10100()
{
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
}

int ICP10100::read_otp_from_i2c() {

    uint8_t data_write[10];
    uint8_t data_read[10] = {0};

    // OTP Read mode
    data_write[0] = 0xC5;
    data_write[1] = 0x95;
    data_write[2] = 0x00;
    data_write[3] = 0x66;
    data_write[4] = 0x9C;

    if (WriteData(data_write, 5) == PX4_OK) {
        for (int i = 0; i < NUM_OTP_VALUES; i++) {
            if (SendCommand(ICP10100_READ_OTP_CMD) == PX4_OK) {
                if (ReadData(data_read, 3) == PX4_OK) {
                    sensor_params.sensor_constants[i] = data_read[0] << 8 | data_read[1];
                    // TODO: Check CRC value in third byte
                } else {
                    PX4_ERR("%s ReadData failed", __FUNCTION__);
                    return PX4_ERROR;
                }
            } else {
                PX4_ERR("%s SendCommand failed", __FUNCTION__);
                return PX4_ERROR;
            }
        }
    } else {
        PX4_ERR("%s WriteData failed", __FUNCTION__);
        return PX4_ERROR;
    }

    return 0;
}

int ICP10100::init()
{
    PX4_INFO("In %s", __FUNCTION__);

	if (I2C::init() != PX4_OK) {
		PX4_ERR("I2C init failed");
		return PX4_ERROR;
	}

    if (read_otp_from_i2c() == PX4_OK) {
        sensor_params.p_Pa_calib[0] = 45000.0;
        sensor_params.p_Pa_calib[1] = 80000.0;
        sensor_params.p_Pa_calib[2] = 105000.0;
        sensor_params.LUT_lower = 3.5 * (1<<20);
        sensor_params.LUT_upper = 11.5 * (1<<20);
        sensor_params.quadr_factor = 1 / 16777216.0;
        sensor_params.offst_factor = 2048.0;
    } else {
		PX4_ERR("read_otp_from_i2c failed");
		return PX4_ERROR;
    }

	start();

	return PX4_OK;
}

int ICP10100::probe()
{
    reset();

    uint8_t buf[4];

    _retries = 10;

    if (SendCommand(ICP10100_READ_ID_CMD) == PX4_OK) {
        if (ReadData(buf, 4) == PX4_OK) {
            if ((buf[1] & ICP10100_ID_MASK) == ICP10100_ID_VAL) {
                PX4_INFO("%s probe succeeded. data = 0x%x 0x%x", __FUNCTION__, buf[0], buf[1]);
                return PX4_OK;
            } else {
                PX4_ERR("%s probe failed. data = 0x%x 0x%x", __FUNCTION__, buf[0], buf[1]);
            }
        } else {
            PX4_ERR("%s ReadData failed", __FUNCTION__);
        }
    } else {
        PX4_ERR("%s SendCommand failed", __FUNCTION__);
    }

	return PX4_ERROR;
	return PX4_OK;
}

int ICP10100::SendCommand(uint16_t cmd)
{
	uint8_t buf[2];
    buf[0] = (uint8_t) (cmd >> 8);
    buf[1] = (uint8_t) (cmd & 0xFF);
	return WriteData(buf, 2);
}

int ICP10100::WriteData(const uint8_t *data, uint8_t len)
{
	return transfer(data, len, NULL, 0);
}

int ICP10100::ReadData(uint8_t *data, uint8_t len)
{
	int ret = transfer(NULL, 0, data, len);

    if (ret == PX4_OK) {
        // if (len == 3) {
        //     PX4_INFO("0x%x 0x%x 0x%x",
        //               data[0], data[1], data[2]);
        // } else if (len == 4) {
        //     PX4_INFO("0x%x 0x%x 0x%x 0x%x",
        //               data[0], data[1], data[2], data[3]);
        // } else if (len == 9) {
        //     PX4_INFO("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
        //               data[0], data[1], data[2], data[3], data[4],
        //               data[5], data[6], data[7], data[8]);
        // }
    } else {
        PX4_ERR("%s transfer failed", __FUNCTION__);
    }

    return ret;
}

void ICP10100::start()
{
    PX4_INFO("In %s", __FUNCTION__);

	/* reset the report ring and state machine */
	_collect_phase = false;

	/* schedule a cycle to start things */
	ScheduleNow();
}

int ICP10100::reset()
{
    PX4_INFO("In %s", __FUNCTION__);

    SendCommand(ICP10100_SOFT_RESET_CMD);

	return PX4_OK;
}

void ICP10100::RunImpl()
{
	int ret = PX4_ERROR;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret == PX4_ERROR) {
			/* issue a reset command to the sensor */
			reset();

			/* reset the collection state machine and try again - we need
			 * to wait a few ms after issuing the sensor reset command.
			 */
			_collect_phase = false;
			ScheduleDelayed(5000);
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;
	}

	/* Look for a ready condition */
	ret = measure();

	if (ret == PX4_ERROR) {
		/* issue a reset command to the sensor */
		reset();

		/* reset the collection state machine and try again */
		start();
		return;
	}

	/* next phase is measurement */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(40000);
}

int ICP10100::measure()
{
    int ret = PX4_OK;

	perf_begin(_measure_perf);

    if (SendCommand(ICP10100_START_LN_CMD) == PX4_OK) {
        // PX4_INFO("%s Send start command succeeded", __FUNCTION__);
    } else {
        PX4_ERR("%s Send start command failed", __FUNCTION__);
		perf_count(_comms_errors);
        ret = PX4_ERROR;
    }

	perf_end(_measure_perf);

	return PX4_OK;
}

void ICP10100::CalculatePressure(int32_t raw_pressure, int32_t raw_temperature) {
    double t;
    double s1, s2, s3;
    double in[3];
    double out[3];
    double A,B,C;

    t = (double)(raw_temperature - 32768);

    s1 = sensor_params.LUT_lower + (double)(sensor_params.sensor_constants[0] * t * t) * sensor_params.quadr_factor;
    s2 = sensor_params.offst_factor * sensor_params.sensor_constants[3] + (double) (sensor_params.sensor_constants[1] * t * t) * sensor_params.quadr_factor;
    s3 = sensor_params.LUT_upper + (double)(sensor_params.sensor_constants[2] * t * t) * sensor_params.quadr_factor;

    in[0] = s1;
    in[1] = s2;
    in[2] = s3;

    calculate_conversion_constants(sensor_params.p_Pa_calib, in, out);

    A = out[0];
    B = out[1];
    C = out[2];

    pressure = A + B / (C + (double) raw_pressure);
    temperature = -45.0 + 175.0/65536.0 * (double) raw_temperature;
}

// p_Pa -- List of 3 values corresponding to applied pressure in Pa
// p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
void ICP10100::calculate_conversion_constants(double *p_Pa, double *p_LUT, double *out) {
    double A,B,C;
    C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
    p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
    p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
    (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
    p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
    p_LUT[1] * (p_Pa[2] - p_Pa[0]));

    A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
    B = (p_Pa[0] - A) * (p_LUT[0] + C);

    out[0] = A;
    out[1] = B;
    out[2] = C;

}

int ICP10100::collect()
{
	perf_begin(_sample_perf);

    uint8_t data_read[9] = {0};

    if (ReadData(data_read, 9) != PX4_OK) {
        PX4_ERR("%s ReadData failed", __FUNCTION__);

        perf_end(_sample_perf);

        return PX4_ERROR;
    }

    // TODO: Check CRC values

    int32_t temp_val = (int32_t)(data_read[6] << 8 | data_read[7]);
    temperature = ((175.0 / 65536.0) * (double) temp_val) - 45.0;
    PX4_INFO("Barometer temperature %.2f C", temperature);

    int32_t pressure_val = (int32_t)(data_read[0] << 16 | data_read[1] << 8 | data_read[3]);
    CalculatePressure(pressure_val, temp_val);
    // pressure /= 100.0;
    PX4_INFO("Raw pressure value %d", pressure_val);
    PX4_INFO("Calculated pressure value %f", pressure);

    _px4_barometer.set_error_count(perf_event_count(_comms_errors));
    _px4_barometer.set_temperature(temperature);
    // _px4_barometer.update(timestamp_sample, P / 100.0f);

	perf_end(_sample_perf);

	return PX4_OK;
}

void ICP10100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
