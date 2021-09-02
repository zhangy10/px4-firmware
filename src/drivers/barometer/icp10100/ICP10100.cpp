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

// // p_LSB -- Raw pressure data from sensor
// // T_LSB -- Raw temperature data from sensor
// int ICP10100::inv_invpres_process_data(struct inv_invpres * s, int p_LSB, int T_LSB,
// float * pressure, float * temperature)
// {
//     float t;
//     float s1,s2,s3;
//     float in[3];
//     float out[3];
//     float A,B,C;
//     t = (float)(T_LSB - 32768);
//     s1 = s->LUT_lower + (float)(s->sensor_constants[0] * t * t) * s->quadr_factor;
//     s2 = s->offst_factor * s->sensor_constants[3] + (float)(s->sensor_constants[1] * t * t) * s->quadr_factor;
//     s3 = s->LUT_upper + (float)(s->sensor_constants[2] * t * t) * s->quadr_factor;
//     in[0] = s1;
//     in[1] = s2;
//     in[2] = s3;
//     calculate_conversion_constants(s, s->p_Pa_calib, in, out);
//     A = out[0];
//     B = out[1];
//     C = out[2];
//     *pressure = A + B / (C + p_LSB);
//     *temperature = -45.f + 175.f/65536.f * T_LSB;
//     return 0;
// }
//
// // p_Pa -- List of 3 values corresponding to applied pressure in Pa
// // p_LUT -- List of 3 values corresponding to the measured p_LUT values at the applied pressures.
// void ICP10100::calculate_conversion_constants(struct inv_invpres * s, float *p_Pa,
//                                     float *p_LUT, float *out) {
//     float A,B,C;
//     C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
//     p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
//     p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
//     (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
//     p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
//     p_LUT[1] * (p_Pa[2] - p_Pa[0]));
//
//     A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] - (p_Pa[1] - p_Pa[0]) * C) / (p_LUT[0] - p_LUT[1]);
//     B = (p_Pa[0] - A) * (p_LUT[0] + C);
//
//     out[0] = A;
//     out[1] = B;
//     out[2] = C;
// }

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

    PX4_INFO("0x%x 0x%x 0x%x 0x%x", data[0], data[1], data[2], data[3]);

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
    PX4_INFO("Skipping ICP10100::RunImpl");

	// int ret = PX4_ERROR;
    //
	// /* collection phase? */
	// if (_collect_phase) {
    //
	// 	/* perform collection */
	// 	ret = collect();
    //
	// 	if (ret == -EIO) {
	// 		/* issue a reset command to the sensor */
	// 		reset();
    //
	// 		/* reset the collection state machine and try again - we need
	// 		 * to wait 2.8 ms after issuing the sensor reset command
	// 		 * according to the ICP10100 datasheet
	// 		 */
	// 		_collect_phase = false;
	// 		ScheduleDelayed(2800);
	// 		return;
	// 	}
    //
	// 	if (ret == -EAGAIN) {
	// 		/* Ready read it on next cycle */
	// 		ScheduleDelayed(ICP10100_CONVERSION_INTERVAL);
    //
	// 		return;
	// 	}
    //
	// 	/* next phase is measurement */
	// 	_collect_phase = false;
	// }
    //
	// /* Look for a ready condition */
	// ret = measure();
    //
	// if (ret == -EIO) {
	// 	/* issue a reset command to the sensor */
	// 	reset();
    //
	// 	/* reset the collection state machine and try again */
	// 	start();
	// 	return;
	// }
    //
	// /* next phase is measurement */
	// _collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(40000);
}

int ICP10100::measure()
{
	// perf_begin(_measure_perf);
    //
	// // Send the command to read the ADC for P and T.
	// unsigned addr = (ICP10100_CTRL_REG1 << 8) | ICP10100_CTRL_TRIGGER;
    //
	// /*
	//  * Disable retries on this command; we can't know whether failure
	//  * means the device did or did not see the command.
	//  */
	// _retries = 0;
	// int ret = RegisterWrite((addr >> 8) & 0xff, addr & 0xff);
    //
	// if (ret == -EIO) {
	// 	perf_count(_comms_errors);
	// }
    //
	// perf_end(_measure_perf);

	return PX4_OK;
}

int ICP10100::collect()
{
// 	perf_begin(_sample_perf);
//
// 	uint8_t ctrl{};
// 	int ret = RegisterRead(ICP10100_CTRL_REG1, (void *)&ctrl, 1);
//
// 	if (ret == -EIO) {
// 		perf_end(_sample_perf);
// 		return ret;
// 	}
//
// 	if (ctrl & CTRL_REG1_OST) {
// 		perf_end(_sample_perf);
// 		return -EAGAIN;
// 	}
//
//
// 	/* read the most recent measurement
// 	 * 3 Pressure and 2 temprtture
// 	 */
// 	uint8_t	b[3 + 2] {};
// 	uint8_t reg = OUT_P_MSB;
// 	const hrt_abstime timestamp_sample = hrt_absolute_time();
// 	ret = transfer(&reg, 1, &b[0], sizeof(b));
//
// 	if (ret == -EIO) {
// 		perf_count(_comms_errors);
// 		perf_end(_sample_perf);
// 		return ret;
// 	}
//
// #pragma pack(push, 1)
// 	struct ICP10100_data_t {
// 		union {
// 			uint32_t q;
// 			uint16_t w[sizeof(q) / sizeof(uint16_t)];
// 			uint8_t  b[sizeof(q) / sizeof(uint8_t)];
// 		} pressure;
//
// 		union {
// 			uint16_t w;
// 			uint8_t  b[sizeof(w)];
// 		} temperature;
// 	} reading;
// #pragma pack(pop)
//
// 	reading.pressure.q = ((uint32_t)b[0]) << 18 | ((uint32_t) b[1]) << 10 | (((uint32_t)b[2]) & 0xc0) << 2 | ((
// 				     b[2] & 0x30) >> 4);
// 	reading.temperature.w = ((uint16_t) b[3]) << 8 | (b[4] >> 4);
//
// 	float T = (float) reading.temperature.b[1] + ((float)(reading.temperature.b[0]) / 16.0f);
// 	float P = (float)(reading.pressure.q >> 8) + ((float)(reading.pressure.b[0]) / 4.0f);
//
// 	_px4_barometer.set_error_count(perf_event_count(_comms_errors));
// 	_px4_barometer.set_temperature(T);
// 	_px4_barometer.update(timestamp_sample, P / 100.0f);
//
// 	perf_end(_sample_perf);

	return PX4_OK;
}

void ICP10100::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
