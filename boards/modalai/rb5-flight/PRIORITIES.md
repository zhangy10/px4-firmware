
# Priorities

## For RB5-Flight MVP

Accel timeout seen at QGC
    - From sensors module voted_sensors_update.cpp
    - Set with _accel.voter.set_timeout(50000); in constructor (50ms!!!)
    - Move to PX4 IMU driver!!!

Fix occasional log file corruption

IMU widget to publish to pipe for VIO - See eric-m0052-imu-server branch

Fix occasional parameters not being saved

CBRK_SUPPLY_CHK 894281 still required to arm, even with voxlpm

Remove "param load" in startup file. Add "param import" after all "param set"

Add SoftAP mode to WiFi (In px4 support for now)
- Do range testing on this

Feature request: Mavlink shell from QGC
Feature request: Reboot from QGC

## Deployment considerations

Generate test signature before shipping. Preserve it across updates.
    - Better yet, find a way to not need signatures. (Filed a QC case...)
    - We need to also store these in case customer loses it

Customers cannot build our PX4. Can send out binary first.

Customers cannot QFIL. Stuck with whatever is there on shipment.

## Test team documentation

Document each drone and it's HW configuration. Log all changes.

Document each test flight. Include drone id, sw version, notes, etc.

File issues in (Asana?)

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
