#include <string.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <lib/rc/dsm.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_hrt.h>

#ifdef __PX4_QURT
#include <drivers/device/qurt/uart.h>
#endif

#include <v2.0/standard/mavlink.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <uORB/Publication.hpp>
#include "modalai_dsp_serial.hpp"

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_log.h>
#include <px4_platform_common/module.h>

class Mavlink;

class ModalAI_DSP
{
public:
	ModalAI_DSP();
	virtual ~ModalAI_DSP();

	void Run(mavlink_message_t *msg);

private:

	static constexpr uint32_t MODALAI_DSP_UART_CONFIG = 2;
	static constexpr uint32_t MODALAI_DSP_DEFAULT_BAUD = 250000;
	static constexpr uint16_t MODALAI_DSP_OUTPUT_CHANNELS = 4;
	static constexpr uint16_t MODALAI_DSP_OUTPUT_DISABLED = 0;

	void handle_message_hil_gps_dsp(mavlink_message_t *msg);
	void handle_message_hil_optical_flow_dsp(mavlink_message_t *msg);
	void handle_message_hil_sensor_dsp(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion_dsp(mavlink_message_t *msg);

	uORB::Publication<battery_status_s>			_battery_pub{ORB_ID(battery_status)};
	uORB::Publication<sensor_gps_s>				_gps_pub{ORB_ID(sensor_gps)};
	uORB::Publication<differential_pressure_s>		_differential_pressure_pub{ORB_ID(differential_pressure)};

	// hil_sensor and hil_state_quaternion
	enum SensorSource {
		ACCEL		= 0b111,
		GYRO		= 0b111000,
		MAG		= 0b111000000,
		BARO		= 0b1101000000000,
		DIFF_PRESS	= 0b10000000000
	};
	PX4Accelerometer *_px4_accel{nullptr};
	PX4Barometer *_px4_baro{nullptr};
	PX4Gyroscope *_px4_gyro{nullptr};
	PX4Magnetometer *_px4_mag{nullptr};

	bool			_outputs_on{false};
	ModalaiDSPSerial 	*_uart_port;


};
