# Work items :construction:

## Modules

### ORB
* qshell_retval sequence number mismatch and subsequent timeout
* Implement topic listener on Qurt - Maybe not really needed since apps side will subscribe to the message.
   * You can listen to slpi uorb topics from apps side. Need to use “-n 1”.
* Improve “uorb top” on Qurt
* Remove topic_unadvertised from the IChannel interface
* Is there an advertise loop if the topic is local and remote?

### Parameters
* Add handshake for parameter_set from client to server (parameter_server_set_response)
* Full error handling
* Problems reported (ask Rich) pushing lots of parameters from QGC
* Having a param start command could help with the startup synchronization issues. But that means params cannot be used by anything starting before it.

### Calibration
* Remove the px4_usleeps when the parameter set handshake fixes are implemented
   * Accelerometer.cpp, Gyroscope.cpp, level_calibration.cpp
* Fix code problems
   * Fix pthread issues in commander/worker_thread.
   * Fix SubscriptionBlocking issues in the calibration routines

### Logging
* Bringup
* Determine best place for log file on server side (e.g. SD card)

### Mavlink
* Run with MAV_BROADCAST 0 and implement mavlink proxy
   * So we can direct the drone to a specific GCS

## Drivers

### RC
* Implement on apps side on M0052 (Use UART that FC used on M0051)

### GPS
* Implement on apps side
   * Need UART. Use USB / FTDI if not available

### Magnetometer
* Why is it not running reliably every ~10ms???
   * pthread_kill doesn’t work on SLPI
   * Changed loop rate in hrt_thread to 1ms for QURT
* Activate temperature compensation in hmc5883 driver?
* Move to the desired magnetometer, away from 3DR unit

### IMU
* Debug why the accel lookup is failing (Not important if we move to PX4 driver)
* Allow specification of the rotation at the driver level.
* Once there is SPI support in flight controller sensor, move to PX4 driver.

### Barometer
* Move from Invensense driver to PX4 driver

### UART (SLPI)
* Add queue in SLPI for incoming messages so none are lost?
* Add support for multiple UART in SLPI
* Wait for feedback to come in. It can be messed up per Alex due to shared UART. Also, check update rate of ESC. If there is too long a wait then we may miss motor updates?
* Normally, write, then read is a “cycle”. If read data comes in after read timeout then we should drop it because we don’t want it to be picked up by next read in the write / read cycle. Otherwise it will be stale data!

### Fake RC
* When moved to SLPI caused uorb issue and qshell timeouts

### ModalAI UART ESC
* Enable test motor command from QGC (Need newer QGC)
* Allow leds to be set with led command
* Test tones
* Timeout on feedback request, lots of read errors. Probably due to flaky cable.
* Dual id = 0 feedback responses per feedback cycle (0, 0, 1, 2, 3)
* Add voltage / current reporting as a configurable item. That would replace reporting from the APM and free up an I2C port. But, APM also reports companion computer voltage / current which is not available at ESC.
* Separate packet to request feedback without sending motor controls?
* Separate out the LED so that it doesn’t have to be sent with motor commands?

### APM
* May not be possible on M0051. Vinny provided a cable to hack it?
* Perhaps just wait until M0052 (Expected early July)

### System time (drv_hrt):
* Does the slpi clock offset need to be updated periodically?
* Move dsp offset from hrt_absolute time to px4_clock_gettime?

## External components

### voxl2-slpi-power-request
* Figure out how to call HAP_power_request from px4 and this won’t be needed anymore

### libfc_sensor_api
* Move it to a public repo
* Make it a git submodule for apps muorb
* Figure out a way to build stub library with aarch64 compiler in px4 tree
   * Try -zlazyload -lsomelib to get rid of the stub library

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
* Try to keep flight controller code out of system image build

## Issues

### Software
* SLPI flight controller stops responding
* telemetry_status GCS heartbeat timestamp in Commander.cpp is ahead of current time? line 3728
* ERROR [mavlink] vehicle_command lost, generation 0 -> 2
* PX4 application has to link against libsee_sensor.so and all of its dependencies!
   * Build fake so with real api and put into public project.
   * Try -zlazyload -lsomelib to get rid of the stub library
* Problems with wifi connection? Also happens with Ethernet!
   * Data link regained  3709  Commander.cpp (on dsp)
   * ERROR [mavlink] channel 0 has missed 9 mavlink log messages (apps)
   * ERROR [mavlink] channel 0 has missed 1 mavlink log messages
   * Connection to ground station lost (at QGC)
   * Set timeout to 5 seconds in msg/telemetry_status.msg
   * Root cause: This is due to a heartbeat timestamp sync issue. Not a networking problem at all.
* Cannot vi /data/misc/wifi/wpa_supplicant.conf???
* Cannot flash system image from adb sometimes on M0051 (Seems to work okay with perf build?)
* “groups” error when launching bash shell
* Strange wlan ip address configuration:
   * wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
   *         inet 169.254.86.150  netmask 255.255.0.0  broadcast 169.254.255.255
   * Only happens on my home network?
* Startup errors:
   * ERROR [mavlink] DM_KEY_MISSION_STATE lock failed
   * ERROR [mavlink] offboard mission init failed (-1)
* Error message on SLPI: “LED: open /dev/led0 failed (22)  0302  commander_helper.cpp”
* Why does apps side seg fault when too many PX4_INFO are sent out?
* Calling shutdown from shell causes crash (It only stops apps side, not slpi)
* Can only run once. Then needs a power cycle. Can it be made to run multiple times?
* Cannot start mavlink shell from QGC. Get this error: ERROR [mavlink] Failed to start shell (-1)

### Hardware
* ADB flaky on M0051 with old APM. Get a root cause.
* Sometimes the debug board USB hub doesn’t show up
* Always a different MAC ID on WiFi so always get different IP

## Miscellaneous

* Update to latest PX4 master
   * Then start upstreaming the code
* PX4 Autostart service
* Remove as much dspal stuff as possible
   * Also idl, fastrpc, shmem, stubs, etc.
* Clean up the build scripts
* Move to QRB5165 release 9.1
* Clean up header file includes in all source files
* Create a debian package to load everything onto target
* Figure out how to better control log messages (DEBUG vs. INFO, etc.)
* Need tcpdump on target
* Clean up the code
   * Run astyle to properly format code
   * Correct copyright notices
* Alternatives to mini-dm? Logcat?
* CPU profiling on DSP
* A better way to select high volume debug messages by category
* Tie fake function calls (stubs) (e.g. HAP_power_request) back into SLPI process
* Does adb reboot cause slpi reboot or not?
* SLPI message needed?: “Min: 1, max: 2  0273  VehicleAcceleration.cpp”

### Preflight arm fails
* Compass device id 73225 (CAL_MAG0_ID) mismatch
   * How is that device id determined and does it need to be in config file?
* ekf2Check.cpp 288 return true ekf2CheckSensorBias (accel bias)
   * Set EKF2_ABL_LIM to 0.8 to get around it for now...
* cpuResourceCheck.cpp return true
   * Can experiment with COM_CPU_MAX = -1 instead of hardcoding it.
   * Eventually need to figure out how to get the CPU percent
* RC_STICK reason: arm / disarm with sticks down / center
   * Normal range is 800 to 2200. 1500 is center.
* msg/telemetry_status.msg Changed heartbeat timeout to 5 seconds

## Testing

### HIL
* Bringup

### Stability over time
* Fly on a tether?
