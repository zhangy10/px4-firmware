#!/bin/bash

set -e

COUNTER=1
while :
do
    echo
    echo "...Rebooting..."
    echo
    adb reboot
    adb wait-for-device
    echo
    echo "...Dumping px4 versions..."
    echo
    adb shell px4-versions
    echo
    echo "...Starting px4..."
    echo
    # adb shell px4 -d -s /etc/modalai/full-m0054.config &
    # sleep 15
    adb shell px4 -d -s /home/test/test.config &
    sleep 5
    echo
    echo "...Running test script..."
    echo
    adb shell /home/test/test.sh
    RETVAL=$?
    echo
    echo "...Return value is $RETVAL..."
    echo
    sleep 1
    echo
    echo "...Loop $COUNTER complete..."
    echo
    let "COUNTER=COUNTER+1"
done
