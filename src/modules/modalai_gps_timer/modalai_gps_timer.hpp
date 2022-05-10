#pragma once

#include <px4_log.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <chrono>
#include <ctime>
#include <time.h>

using namespace time_literals;

#define GPS_EPOCH_SECS ((time_t)1234567890ULL)
static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */

class ModalaiGPSTimer : public ModuleBase<ModalaiGPSTimer>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ModalaiGPSTimer();
	~ModalaiGPSTimer() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:

	/** @see ModuleBase::run() */
	void Run() override;

	void init();

	uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	bool _is_running = false;
	px4_task_t _task_handle = -1;
	perf_counter_t _perf_elapsed{};

};
