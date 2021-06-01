#pragma once

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

class IMU_ThinClient
{
public:
	IMU_ThinClient();

    void start() { _started = true; }
    void stop() { _started = false; }

    void update_accel(float x, float y, float z) {
        if (_started) _px4_accel.update(hrt_absolute_time(), x, y, z);
    }

    void update_gyro(float x, float y, float z) {
        if (_started) _px4_gyro.update(hrt_absolute_time(), x, y, z);
    }

    void update_temperature(float temperature) {
        if (_started) {
            _px4_accel.set_temperature(temperature);
            _px4_gyro.set_temperature(temperature);
        }
    }

    bool is_running() { return _started; }

private:
    bool             _started;

    PX4Accelerometer _px4_accel;
    PX4Gyroscope     _px4_gyro;
};

extern "C" {

    int imu_thin_client_main(int argc, char *argv[]) __EXPORT;

	void imu_thin_client_accel_data(float x, float y, float z) __EXPORT;
	void imu_thin_client_gyro_data(float x, float y, float z) __EXPORT;
	void imu_thin_client_temperature_data(float temperature) __EXPORT;

}
