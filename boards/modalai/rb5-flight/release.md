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
