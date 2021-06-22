#!/bin/bash

mkdir -p pkg

rm -fR debian/usr
mkdir -p debian/usr/lib/rfsa/adsp
cp ../../../build/modalai_rb5-flight_qurt/platforms/qurt/libpx4.so debian/usr/lib/rfsa/adsp

mkdir -p debian/usr/bin
cp ../../../build/modalai_rb5-flight_default/bin/px4 debian/usr/bin
cp ../../../build/modalai_rb5-flight_default/bin/px4-alias.sh debian/usr/bin
cp net-check.sh debian/usr/bin
chmod a+x debian/usr/bin/px4-alias.sh

rm -fR debian/etc
mkdir -p debian/etc/modalai
cp min-m0051.config debian/etc/modalai
cp full-m0051.config debian/etc/modalai

mkdir -p debian/etc/systemd/system
cp net-check.service debian/etc/systemd/system
cp px4-start.service debian/etc/systemd/system

dpkg-deb --build debian pkg
