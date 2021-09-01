#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include "imu_server.hpp"

static px4_task_t  _thread_tid;
static const char *_thread_name = "imu_server_thread";

static IMU_Server  _server;

// Copied from another file. Needs to stay the same.
typedef struct icm4x6xx_imu_data_t {
	float accl_ms2[3];         ///< XYZ acceleration in m/s^2
	float gyro_rad[3];         ///< XYZ gyro rotation in rad/s
    float temp_c;         ///< temperature in degrees Celcius
	uint64_t timestamp_monotonic_ns; ///< Monotonic timestamp
	uint64_t dummy0;   ///< Not used for VIO
	uint64_t dummy1;   ///< Not used for VIO
} __attribute__((packed)) icm4x6xx_imu_data_t;

static int _imu_server_thread(int argc, char *argv[]) {

    PX4_INFO("imu_server thread starting");

    const char *imu_fifo = "/dev/imu-pipe0";
    struct stat pipe_stat;
    int stat_rc = lstat(imu_fifo, &pipe_stat);
    if (stat_rc == 0) {
        // The path already exists. Make sure it is a pipe.
        if ( ! S_ISFIFO(pipe_stat.st_mode)) {
            PX4_ERR("Error: %s exists but it is not a pipe", imu_fifo);
            return PX4_ERROR;
        } else {
            PX4_INFO("%s exists and it is a pipe", imu_fifo);
        }
    } else {
        // The pipe does not exist yet. Create it.
        int mkfifo_rc = mkfifo(imu_fifo, 0666);
        if (mkfifo_rc) {
            PX4_ERR("Error: Couldn't create pipe %s", imu_fifo);
            return PX4_ERROR;
        } else {
            PX4_INFO("Created pipe %s", imu_fifo);
        }
    }

    // The call to open will block until a reader attaches to the pipe
    int fifo_fd = open(imu_fifo, O_WRONLY);
    if (fifo_fd == -1) {
        PX4_ERR("Error: Couldn't open pipe %s", imu_fifo);
        return PX4_ERROR;
    } else {
        PX4_INFO("Opened pipe %s for writing", imu_fifo);
    }

    int vehicle_acceleration_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
    int vehicle_angular_velocity_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));

    struct vehicle_acceleration_s accel_data;
    struct vehicle_angular_velocity_s gyro_data;
    icm4x6xx_imu_data_t   imu_data;
    uint64_t              previous_timestamp = 0;

    memset(&imu_data, 0, sizeof(imu_data));

    // We send out the data when the accel timestamp is the same as the
    // gyro timestamp. That means they are a matched set and can be combined
    // into the imu data packet. Start with them unequal.
    accel_data.timestamp_sample = 0;
    gyro_data.timestamp_sample = 1;

    px4_pollfd_struct_t fds[2] = { { .fd = vehicle_acceleration_fd,  .events = POLLIN },
                                   { .fd = vehicle_angular_velocity_fd, .events = POLLIN } };
    while (true) {
    	px4_poll(fds, 2, 1000);
    	if (fds[0].revents & POLLIN) {
            orb_copy(ORB_ID(vehicle_acceleration), vehicle_acceleration_fd, &accel_data);
            imu_data.accl_ms2[0] = accel_data.xyz[0];
            imu_data.accl_ms2[1] = accel_data.xyz[1];
            imu_data.accl_ms2[2] = accel_data.xyz[2];
            // PX4_INFO("Got accel data %lu", accel_data.timestamp_sample);
    	} else if (fds[1].revents & POLLIN) {
            orb_copy(ORB_ID(vehicle_angular_velocity), vehicle_angular_velocity_fd, &gyro_data);
            imu_data.gyro_rad[0] = gyro_data.xyz[0];
            imu_data.gyro_rad[1] = gyro_data.xyz[1];
            imu_data.gyro_rad[2] = gyro_data.xyz[2];
            // PX4_INFO("Got gyro data  %lu", gyro_data.timestamp_sample);
        }

        if (accel_data.timestamp_sample == gyro_data.timestamp_sample) {

            imu_data.temp_c = 0;
            imu_data.timestamp_monotonic_ns = gyro_data.timestamp_sample * 1000;

            if (previous_timestamp < imu_data.timestamp_monotonic_ns) {
                // Write the data to the fifo
                size_t data_len = sizeof(icm4x6xx_imu_data_t);
                ssize_t bytes_written = write(fifo_fd, &imu_data, data_len);
                size_t unsigned_bytes_written = (bytes_written > 0) ? (size_t) bytes_written : 0;
                if ((bytes_written > 0) && (unsigned_bytes_written == data_len)) {
                    // PX4_INFO("Wrote %ld IMU data bytes to %s", unsigned_bytes_written, imu_fifo);
                } else {
                    PX4_ERR("Error: Couldn't write %lu bytes to the pipe", data_len);
                    PX4_ERR("       write returned %ld", bytes_written);
                    break;
                }
            } else {
                PX4_WARN("*** Dropping stale IMU data. Previous %lu, current %lu ***", previous_timestamp, imu_data.timestamp_monotonic_ns);
            }
            previous_timestamp = imu_data.timestamp_monotonic_ns;

            // PX4_INFO("**** %.2f %.2f %.2f %.2f %.2f %.2f %.2f %lu ****",
            //          (double) imu_data.accl_ms2[0], (double) imu_data.accl_ms2[1],
            //          (double) imu_data.accl_ms2[2], (double) imu_data.gyro_rad[0],
            //          (double) imu_data.gyro_rad[1], (double) imu_data.gyro_rad[2],
            //          (double) imu_data.temp_c, gyro_data.timestamp_sample);
        }
    }

    close(fifo_fd);

    PX4_INFO("imu_server thread ending");

    return PX4_OK;
}

int imu_server_main(int argc, char *argv[]) {

    PX4_INFO("imu_server_main");

    if ( ! _server.is_running()) {
        _server.start();
        _thread_tid = px4_task_spawn_cmd(_thread_name,
    			                         SCHED_DEFAULT,
    			                         SCHED_PRIORITY_PARAMS,
    			                         (1024 * 4),
    			                         _imu_server_thread,
    			                         NULL);
    }

    return 0;

}
