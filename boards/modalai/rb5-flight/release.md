# Release procedure

- Add release notes to this document
- On master branch
- Bump package version number in debian/control
- Commit and push everything
- tag it and push tag
  - Tag format: vX.Y.Z-modalai-rb5-flight-<type>
    - Type v for dev
    - Type p for alpha
    - Type t for beta
    - Type rc for Release Candidate RC
    - Type r for Release
- make clean
- make default and qurt
- package
- deploy / validate
- post package to cloud bucket

# Releases

## 1.4.9alpha

- Some minor DSP test signature related changes

## 1.4.8alpha

- Added voxl-px4.service to autostart px4
- Added call to generate a SLPI DSP test signature in postinst
- Added a check in the voxl.config startup script to make sure test signature is available
- Consolidating m0052 / m0054 into single voxl structure with automatic platform configuration

## 1.4.7alpha

- Decreased the polling rate of UART drivers running on Qurt to avoid starving the OS services
- Improve comms error counting in icp10100 barometer driver
- Added a set command to the rgbled_ncp5623c driver
- Added a performance counter to rgbled_ncp5623c to count bad i2c transfers
- Changed qshell on apps side to return the value received from the DSP instead of 0

## 1.4.6alpha

- Fixed typo in qgc-ip.cfg

## 1.4.5alpha

- Fixed default value of MPC_THR_HOVER to 0.42
- Removed the -e logger option from m0054
- Added ability to configure QGC port number

## 1.4.4alpha

- Improved the bad packet detection in the Spektrum RC driver

## 1.4.3alpha

- Moved logger start to beginning of sequence to capture driver and module startup messages
- Fixed the QGC address script to allow blank lines

## 1.4.2alpha

- Fixed some Voxl2 startup script errors

## 1.4.1alpha

- Fixed packaging and installed scripts for Voxl2

## 1.4.0alpha

- Updated Spektrum RC and GPS drivers to work on QURT to support Voxl2

## 1.1.9alpha

- Updated barometer driver calculations to exactly match original TDK driver for icp10100

## 1.1.8alpha

- Added more default parameters. They are set up for indoor flights
- Removed hard-coded baud rate configuration of the gps. It will now auto-detect the rate.
- Enabled the LED on the GPS unit
- Added a new feature to allow choice of QGC IP address via configuration file
- Auto detect presence of M0065 (PX4IO). Start px4io if detected, spektrum rc driver otherwise

## 1.1.7alpha

- Completed the mechanism to allow sending debug messages from SLPI to Apps for logging

## 1.1.6alpha

- Added a mechanism to allow sending debug messages from SLPI to Apps for logging

## 1.1.5alpha

- Fixed casting error for icp10100 barometer otp data
- Create log file names without the time in the name unless we have GPS time
- Fixed BAT default parameter setting to use BAT instead of BAT1

## 1.1.4alpha

- Added default battery parameter settings to m0052-set-default-parameters.config
- Clean up I2C and SPI device id creation to zero out unused bits

## 1.1.3alpha

- Modified icm42688p driver such that flight control gets 500Hz IMU samples and
  VIO (via imu_server) gets 1kHz samples in a batch of 10 every 100ms.
- icm42688p driver is now configured for anti-alias and UI filtering

## 1.1.2alpha

- Moved from TDK icp10100 barometer driver to new px4 icp10100 barometer driver.

## 1.1.1alpha

- Changed IMU server from using raw sensor data to using calibrated / filtered sensor data

## 1.1.0alpha

- Moved from Invensense IMU driver to PX4 IMU driver
- Bumped minor revision number since the IMU change is quite significant

## 1.0.20alpha

- Added pwm_out_sim driver to qurt build to support HIL testing
- Add rc_update start back into startup configuration

## 1.0.19alpha

- Added fsync on log file close

## 1.0.18alpha

- Spektrum RC driver throws out first packet if it is too small. Prevents
  the loss of RC when the occasional first packet is a "runt" and corrupts DSM parser.

## 1.0.17alpha

- Removed the -e option from logger start
- Upped default MAX RPM to 10500
- Moved sensor driver start before the modules that rely on them to prevent timeouts

## 1.0.16alpha

- Add imu_server start to default configuration

## 1.0.15alpha

- Added method to update the time offset on SLPI
- Changed motor mappings to match default for seeker drone
- Changed RC parameters to no longer assume joystick

## 1.0.14alpha

- Added module starts for the flight_mode_manager and mc_hover_thrust_estimator

## 1.0.13alpha

- Added mc_hover_thrust_estimator module into qurt build

## 1.0.12alpha

- Set COM_ARM_WO_GPS to 0 to prevent QGC showing GPS as disabled
- Added flight_mode_manager module into apps (default) build

## 1.0.11alpha

- Touch SYS_AUTOCONFIG to make QGC missing parameter error message go away
- Remove annoying preflight fail debug messages
- Remove IMU server high frequency debug messages

## 1.0.10alpha

- Fixed orientation of magnetometer
- Added an IMU server for integration with VIO. It isn't started by default.
- Changed the way that parameters are loaded so that any changes won't get overwritten on the next reboot
- Moved some of the drone specific scripts and startup service files out of this project

## 1.0.9alpha

- Changed voxlpm to only support AN231 on Qurt PX4 to avoid the issues with slave address reconfiguration

## 1.0.8dev

- Do not hardcode Qurt I2C devices to be "internal". Experimental.

## 1.0.7alpha

- Added verbose mode to Spektrum RC driver
- Fixed serial port numbers for GPS and Spektrum RC

## 1.0.6alpha

- Added support for Spektrum RC

## 1.0.5alpha

- Support for new 9.1 based SLPI release

## 1.0.4alpha

- First functional release
