#!/bin/sh
# PX4 commands need the 'px4-' prefix in bash.
# (px4-alias.sh is expected to be in the PATH)
. px4-alias.sh

check_topic () {
    listener battery_status -n 1
    RETVAL=$?
    if [ $RETVAL -ne 0 ]; then
        echo ">>>>>>>>>>>>>>>>>>>> battery_status listener failed!!! <<<<<<<<<<<<<<<<<<<<<"
    fi
    sleep 1
}




# Magnetometer
qshell ist8310 start -R 10 -X -b 1

# LED driver for the Pixhawk 4 GPS module
qshell rgbled_ncp5623c start -X -b 1 -f 400 -a 56

# Barometer
qshell icp10100 start -I -b 5

# ESC driver
qshell modalai_esc start
qshell mixer load /dev/uart_esc quad_x.main.mix

# Pixhawk 4 GPS module
# qshell gps start -d 7 -b 115200

# Spektrum RC receiver
# qshell spektrum_rc start -d 8




# APM power monitor
qshell voxlpm start -X -b 2
RETVAL=$?
if [ $RETVAL -ne 0 ]; then
    echo ">>>>>>>>>>>>>>>>>>>> APM start failed!!! <<<<<<<<<<<<<<<<<<<<<"
    exit 1
fi

# COUNTER=1
# # Check battery_status topic
# while :
# do
#     check_topic
#     echo
#     echo "...Loop $COUNTER complete..."
#     echo
#     let "COUNTER=COUNTER+1"
# done

exit 0
