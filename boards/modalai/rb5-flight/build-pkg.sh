#!/bin/bash

mkdir -p pkg

rm -fR debian/usr
mkdir -p debian/usr/lib/rfsa/adsp
cp ../../../build/modalai_rb5-flight_qurt/platforms/qurt/libpx4.so debian/usr/lib/rfsa/adsp

# Install quadrotor mixer file for ESC.
# The mixer file comes from ROMFS/px4fmu_common/mixers
cp ../../../ROMFS/px4fmu_common/mixers/quad_x.main.mix debian/usr/lib/rfsa/adsp

mkdir -p debian/usr/bin
cp ../../../build/modalai_rb5-flight_default/bin/px4 debian/usr/bin
cp ../../../build/modalai_rb5-flight_default/bin/px4-alias.sh debian/usr/bin
chmod a+x debian/usr/bin/px4-alias.sh
cp m0052-px4 debian/usr/bin
chmod a+x debian/usr/bin/m0052-px4

rm -fR debian/etc
mkdir -p debian/etc/modalai
cp min-m0052.config debian/etc/modalai
cp full-m0052.config debian/etc/modalai
cp m0052-set-default-parameters.config debian/etc/modalai

# Create necessary directories for px4 operation
mkdir -p debian/home/linaro/eeprom

dpkg-deb --build debian pkg
