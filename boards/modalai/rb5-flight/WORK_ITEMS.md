# Work items :construction:

## Modules

### ORB
* qshell_retval sequence number mismatch and subsequent timeout
  - Is there an advertise loop if the topic is local and remote?
* Implement topic listener on Qurt - Maybe not really needed since apps side will subscribe to the message.
   * You can listen to slpi uorb topics from apps side. Need to use “-n 1”.
* Improve “uorb top” on Qurt
* Remove topic_unadvertised from the IChannel interface

### Parameters
* Full error handling

### Calibration
* set_tune in mag_calibration_worker in mag_calibration.cpp not working
* Try to get commander working on SLPI without all of the pthread hacks
    - commander/worker_thread. Switched to commander thread (inline) as a hack
    - SubscriptionBlocking causes crash due to some pthread call

### Logging
* Add option to use system time when there is no GPS time. This would be used in cases where there is Internet access and we can use NTP.
* sysctl parameters vm.dirty_writeback_centisecs and vm.dirty_expire_centisecs are set in qrb5165-px4-support
  - Should be modified in system image
* Logging to SD card option
* Log file management (Deleting old logs)
  - Move to a separate partition to prevent data overrun
* Tune log buffer size, topic frequency, etc.

### Mavlink
* Run with MAV_BROADCAST 0 and configure Mavlink to connect to a specific GCS
  * May need to be added to configuration script

## Drivers

### APM
* Add support for second INA231

### RC
* Implement on slpi side on M0053
* Move from spektrum_rc driver to input_rc driver
* Figure out how to implement binding

### GPS
* Implement on slpi side on M0053

### Magnetometer
* Why is it not running reliably every ~10ms???
   * pthread_kill doesn’t work on SLPI
   * Changed loop rate in hrt_thread to 1ms for QURT
* Activate temperature compensation in hmc5883 / ist8310 driver?

### IMU
* Clean up and mainline ICM42688p driver

### UART (SLPI)
* Add queue in SLPI for incoming messages so none are lost?
* Add support for multiple UART in SLPI
* Wait for feedback to come in. It can be messed up per Alex due to shared UART. Also, check update rate of ESC. If there is too long a wait then we may miss motor updates?
* Normally, write, then read is a “cycle”. If read data comes in after read timeout then we should drop it because we don’t want it to be picked up by next read in the write / read cycle. Otherwise it will be stale data!

### ModalAI UART ESC
* Enable feedback
* Enable test motor command from QGC (Need newer QGC)
* Allow leds to be set with led command
* Test tones
* Dual id = 0 feedback responses per feedback cycle (0, 0, 1, 2, 3)
* Add voltage / current reporting as a configurable item. That would replace reporting from the APM and free up an I2C port. But, APM also reports companion computer voltage / current which is not available at ESC. (Very low priority)
* Separate packet to request feedback without sending motor controls?
* Separate out the LED so that it doesn’t have to be sent with motor commands?

### Safety switch
* How would we wire this in? Need GPIO

## External components

### libfc_sensor_api
* Move it to a public repo
* Make it a git submodule for apps muorb
* Figure out a way to build stub library with aarch64 compiler in px4 tree
   * Try -zlazyload -lsomelib to get rid of the stub library
* Note: libfc_sensor.so is also required for px4-build-docker

### libfc_sensor
* Add test code
* coordinate suid with slpi_proc code
* Maybe rename this? fc_sensor doesn’t make any sense to the PX4 community
* Move build into off target docker

### sns_flight_controller protobuf
* Create Docker with correct version of protoc to compile sns_flight_controller.proto
   * GOOGLE_PROTOBUF_VERSION 3003000
* Generate so with sns_flight_controller.pb.o so that it doesn’t have to be integrated into libsnsapi.so in the system image
* sns_flight_controller.proto is in both apps_proc and slpi_proc. How to avoid duplication?
   * Perhaps make it a separate project with it’s own Docker makefile for the ARM side. Then use it as a submodule in qrb5165-slpi-build-docker?

## Build

### slpi_proc
* Clean up flight controller code
* Add proper copyright notices
* Make flight controller stuff a submodule?

### system image
* Investigate chipcode release 10.2
* Try to keep flight controller code out of system image build
* Need tcpdump on target

### PX4

* Update to latest PX4 master
   * Then start upstreaming the code
* Remove as much dspal stuff as possible
   * Also idl, fastrpc, shmem, stubs, etc.
* Clean up the build scripts
* Clean up the code
   * Run astyle to properly format code
   * Correct copyright notices
* Move to latest Hexagon SDK release
* Move to updated linaro64 release (e.g. http://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/aarch64-linux-gnu/)

## Issues

### tasks.cpp
* Finish implementing the missing task functions
* Make it thread safe

### Software
* telemetry_status GCS heartbeat timestamp in Commander.cpp is ahead of current time? line 3728
* ERROR [mavlink] vehicle_command lost, generation 0 -> 2
* Problems with wifi connection? Also happens with Ethernet!
   * Root cause: This is due to a heartbeat timestamp sync issue. Not a networking problem at all.
* “groups” error when launching bash shell
* Strange wlan ip address configuration:
   * wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
   *         inet 169.254.86.150  netmask 255.255.0.0  broadcast 169.254.255.255
   * Only happens on my home network?
* Error message on SLPI: “LED: open /dev/led0 failed (22)  0302  commander_helper.cpp”
* Calling shutdown from shell causes crash (It only stops apps side, not slpi)
* Can only run once. Then needs a power cycle. Can it be made to run multiple times?
* Cannot start mavlink shell from QGC. Get this error: ERROR [mavlink] Failed to start shell (-1)
  - Comments in the code says it only works for NuttX. All others return error.

### Hardware
* Sometimes TC SOM not going into fastboot mode with command (Needs switch)
* Always a different MAC ID on WiFi so always get different IP
* Sometimes the debug board USB hub doesn’t show up (Both M0062 and M0067)
  - Do they need rework? And / or special BSP support?

## Miscellaneous
* Sometimes QFIL generates a read only filesystem
  - But reflashing fixes it
* Version management of px4 and all components (eg libfc_sensor, slpi_proc, etc.)
  - How to specify particular dependencies (eg system image 8.1)
* Clean up header file includes in all source files
* Figure out how to better control log messages (DEBUG vs. INFO, etc.)
* Alternatives to mini-dm? Logcat?
* A better way to select high volume debug messages by category
* Tie fake function calls (stubs) (e.g. HAP_power_request) back into SLPI process
* Does adb reboot cause slpi reboot or not?
* SLPI message needed?: “Min: 1, max: 2  0273  VehicleAcceleration.cpp”
* Automatic file sync doesn't happen very frequently. Make it happen more often.
* Time doesn't start in 1970 anymore. Now it starts at last power down time?

### Preflight arm fails
* Set timeout to 5 seconds in msg/telemetry_status.msg
* ekf2Check.cpp 288 return true ekf2CheckSensorBias (accel bias)
   * Set EKF2_ABL_LIM to 0.8 to get around it for now...
* cpuResourceCheck.cpp return true
   * Can experiment with COM_CPU_MAX = -1 instead of hardcoding it.
   * Eventually need to figure out how to get the CPU percent (Combo of apps and slpi???)

## Testing

### HIL
* Bringup
