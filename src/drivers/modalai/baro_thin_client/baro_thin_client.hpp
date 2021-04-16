#pragma once

#include <lib/drivers/barometer/PX4Barometer.hpp>

class BaroThinClient
{
public:
	BaroThinClient();

    void start() { _started = true; }
    void stop() { _started = true; }

    void update_pressure(float pressure) {
        if (_started) _px4_baro.update(hrt_absolute_time(), pressure);
    }

    void update_temperature(float temperature) {
        if (_started) _px4_baro.set_temperature(temperature);
    }

    bool is_running() { return _started; }

private:
    bool             _started;

    PX4Barometer     _px4_baro;
};

extern "C" {

    int baro_thin_client_main(int argc, char *argv[]) __EXPORT;

	void baro_thin_client_pressure_data(float pressure) __EXPORT;
	void baro_thin_client_temperature_data(float temperature) __EXPORT;
}
