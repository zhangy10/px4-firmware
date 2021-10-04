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
