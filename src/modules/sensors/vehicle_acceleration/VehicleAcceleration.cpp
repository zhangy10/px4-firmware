/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "VehicleAcceleration.hpp"

#include <px4_platform_common/log.h>

#include <uORB/topics/vehicle_imu_status.h>

using namespace matrix;

namespace sensors
{

VehicleAcceleration::VehicleAcceleration() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	CheckAndUpdateFilters();
}

VehicleAcceleration::~VehicleAcceleration()
{
	Stop();
}

bool VehicleAcceleration::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		_sensor_sub.registerCallback();
	}

	return true;
}

void VehicleAcceleration::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

void VehicleAcceleration::CheckAndUpdateFilters()
{
	bool sample_rate_changed = false;

	// get sample rate from vehicle_imu_status publication
	for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
		uORB::SubscriptionData<vehicle_imu_status_s> imu_status{ORB_ID(vehicle_imu_status), i};

		const float sample_rate_hz = imu_status.get().accel_rate_hz;

		if ((imu_status.get().accel_device_id != 0) && (imu_status.get().accel_device_id == _calibration.device_id())
		    && PX4_ISFINITE(sample_rate_hz) && (sample_rate_hz > 0)) {
			// check if sample rate error is greater than 1%
			if ((fabsf(sample_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
				PX4_DEBUG("sample rate changed: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)sample_rate_hz);
				_filter_sample_rate = sample_rate_hz;
				sample_rate_changed = true;

				// determine number of sensor samples that will get closest to the desired rate
				if (_param_imu_integ_rate.get() > 0) {
					const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
					const float sample_interval_avg = 1e6f / sample_rate_hz;
					const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
										(float)sensor_accel_s::ORB_QUEUE_LENGTH);

					_sensor_sub.set_required_updates(samples);

				} else {
					_sensor_sub.set_required_updates(1);
				}

				break;
			}
		}
	}

	// update software low pass filters
	if (sample_rate_changed || (fabsf(_lp_filter.get_cutoff_freq() - _param_imu_accel_cutoff.get()) > 0.1f)) {
		_lp_filter.set_cutoff_frequency(_filter_sample_rate, _param_imu_accel_cutoff.get());
		_lp_filter.reset(_acceleration_prev);
	}
}

void VehicleAcceleration::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.accel_device_id == _calibration.device_id()) {
				_bias = Vector3f{bias.accel_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

bool VehicleAcceleration::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_calibration.device_id() == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.accel_device_id != 0) && (_calibration.device_id() != sensor_selection.accel_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_accel_s> sensor_accel_sub{ORB_ID(sensor_accel), i};

				const uint32_t device_id = sensor_accel_sub.get().device_id;

				if ((device_id != 0) && (device_id == sensor_selection.accel_device_id)) {

					if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						PX4_DEBUG("selected sensor changed %d -> %d", _calibration.device_id(), device_id);

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(device_id);

						CheckAndUpdateFilters();

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.accel_device_id);
			_calibration.set_device_id(0);
		}
	}

	return false;
}

void VehicleAcceleration::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();

		CheckAndUpdateFilters();
	}
}

void VehicleAcceleration::Run()
{
	// backup schedule
#ifdef __PX4_QURT
	ScheduleDelayed(1_ms);
#else
	ScheduleDelayed(10_ms);
#endif

	// update corrections first to set _selected_sensor
	bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	// process all outstanding messages
	sensor_accel_s sensor_data;

	while (_sensor_sub.update(&sensor_data)) {

		// Apply calibration and filter
		//  - calibration offsets, scale factors, and thermal scale (if available)
		//  - estimated in run bias (if available)
		//  - biquad low-pass filter
		const Vector3f accel_corrected = _calibration.Correct(Vector3f{sensor_data.x, sensor_data.y, sensor_data.z}) - _bias;
		const Vector3f accel_filtered = _lp_filter.apply(accel_corrected);

		_acceleration_prev = accel_corrected;

        // uint8_t *raw_data = (uint8_t*) &sensor_data;
        // PX4_INFO("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
        //          raw_data[0], raw_data[1], raw_data[2], raw_data[3],
        //          raw_data[4], raw_data[5], raw_data[6], raw_data[7],
        //          raw_data[8], raw_data[9], raw_data[10], raw_data[11],
        //          raw_data[12], raw_data[13], raw_data[14], raw_data[15]);
        // PX4_INFO(" 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
        //          raw_data[16], raw_data[17], raw_data[18], raw_data[19],
        //          raw_data[20], raw_data[21], raw_data[22], raw_data[23],
        //          raw_data[24], raw_data[25], raw_data[26], raw_data[27],
        //          raw_data[28], raw_data[29], raw_data[30], raw_data[31]);
        // PX4_INFO(" 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
        //          raw_data[32], raw_data[33], raw_data[34], raw_data[35],
        //          raw_data[36], raw_data[37], raw_data[38], raw_data[39],
        //          raw_data[40], raw_data[41], raw_data[42], raw_data[43],
        //          raw_data[44], raw_data[45], raw_data[46], raw_data[47]);

#ifdef __PX4_QURT
        // PX4_INFO("%llu %llu %u %f %f %f %f", sensor_data.timestamp, sensor_data.timestamp_sample,
        // PX4_INFO("%llu %llu %u %f %f %f %f", hrt_absolute_time(), sensor_data.timestamp_sample,
        //          sensor_data.device_id, (double) sensor_data.x, (double) sensor_data.y, (double) sensor_data.z,
        //          (double) sensor_data.temperature);

        // Some code to measure either min and max of total delay from when the
        // sample was generated until now (total_delay) or just min and max times
        // between consecutive samples to make sure that we aren't seeing any drops.
        int measure_total_delay = 0;
        uint64_t current_time = hrt_absolute_time();
        static uint64_t last_sample_timestamp = 0;
        static int32_t min_time_difference_ms = 0xffffffff;
        static int32_t max_time_difference_ms = 0;
        static int32_t report_count = 0;
        if (last_sample_timestamp) {
            int32_t time_difference_ms = 0;
            if (measure_total_delay) {
                time_difference_ms = (current_time - sensor_data.timestamp_sample) / 1000;
            } else {
                time_difference_ms = (sensor_data.timestamp_sample - last_sample_timestamp) / 1000;
            }
            if (time_difference_ms < min_time_difference_ms) min_time_difference_ms = time_difference_ms;
            if (time_difference_ms > max_time_difference_ms) max_time_difference_ms = time_difference_ms;
            if ( ! (report_count++ % 1000)) {
                PX4_DEBUG("Min: %d, max: %d", min_time_difference_ms, max_time_difference_ms);
                min_time_difference_ms = 0xfffffff;
                max_time_difference_ms = 0;
            }
        }
        last_sample_timestamp = sensor_data.timestamp_sample;

        // PX4_INFO("0x%llx", sensor_data.timestamp);
        // PX4_INFO("0x%llx", sensor_data.timestamp_sample);
        // PX4_INFO("0x%x", sensor_data.device_id);
#endif

	// report.timestamp = 0x0001020304050607;
	// report.timestamp_sample = 0x08090A0B0C0D0E0F;
	// report.device_id = 0x10111213;
	// ((uint32_t*) &report.x)[0] = 0x14151617;
	// ((uint32_t*) &report.y)[0] = 0x18191a1b;
	// ((uint32_t*) &report.z)[0] = 0x1c1d1e1f;
	// ((uint32_t*) &report.temperature)[0] = 0x20212223;
	// report.error_count = 0x24252627;
	// report.clip_counter[0] = 0x28;
	// report.clip_counter[1] = 0x29;
	// report.clip_counter[2] = 0x2a;
	// report._padding0[0] = 0x2b;
	// report._padding0[1] = 0x2c;
	// report._padding0[2] = 0x2d;
	// report._padding0[3] = 0x2e;
	// report._padding0[4] = 0x2f;

		// publish once all new samples are processed
		if (!_sensor_sub.updated()) {
			// Publish vehicle_acceleration
			vehicle_acceleration_s v_acceleration;
			v_acceleration.timestamp_sample = sensor_data.timestamp_sample;
			accel_filtered.copyTo(v_acceleration.xyz);
			v_acceleration.timestamp = hrt_absolute_time();
			_vehicle_acceleration_pub.publish(v_acceleration);
			return;
		}
	}
}

void VehicleAcceleration::PrintStatus()
{
	PX4_INFO("selected sensor: %d, rate: %.1f Hz, estimated bias: [%.4f %.4f %.4f]",
		 _calibration.device_id(), (double)_filter_sample_rate,
		 (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
