
# Priorities

## For RB5-Flight MVP

IMU widget to publish to pipe for VIO

Fix occasional log file corruption

Fix occasional parameters not being saved

CBRK_SUPPLY_CHK 894281 still required to arm, even with voxlpm

Remove "param load" in startup file. Add "param import" after all "param set"

Add SoftAP mode to WiFi (In px4 support for now)
- Do range testing on this

Feature request: Mavlink shell from QGC

## Other

Run with MAV_BROADCAST 0 and implement a mavlink proxy
   * So we can direct the drone to a specific GCS

M0053 / M0054 RC / GPS on DSP using UART 6 and 7

Implement CPU utilization monitor (DSP vs. Apps) (LoadMon.cpp)_

Move to PX4 barometer driver. Need to create one for ICP101xx

Move to PX4 IMU driver. Need to create SPI driver, interrupt driver

SDSP: Timeout waiting for parameter_client_set_value_response

Add queueing to make sure there are no lost IMU / Barometer samples in the thin clients

Investigate switch to altitude mode when manual mode specified
- Something to do with the RC switches

User and Developer Documentation

Search for all uses of pthread_create. Try to use px4_task_spawn_cmd instead

Search for all uses of clock_gettime. Try to use px4_clock_gettime instead

Log file management

Crash analysis tools
