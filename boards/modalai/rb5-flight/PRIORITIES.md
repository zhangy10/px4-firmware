
# Priorities

## For RB5-Flight MVP

Accel timeout seen at QGC
    - From sensors module voted_sensors_update.cpp
    - Set with _accel.voter.set_timeout(50000); in constructor (50ms!!!) (now 500ms)
    - Seems to be okay at driver level. Samples still flowing at 500Hz.

Fix occasional log file corruption.

Fix occasional parameters not being saved(?)

Implement CPU utilization monitor for DSP (load_mon)

CBRK_SUPPLY_CHK 894281 still required to arm, even with voxlpm
    - Requires system_power_s message for pre-flight power check
    - Checks voltage5v_v and brick_valid fields. Also COM_POWER_COUNT parameter.

## Feature requests

Frsky rc receivers

Add SoftAP mode to WiFi (In px4 support for now)
    - Do range testing on this

Mavlink shell from QGC

Reboot from QGC

## Deployment considerations

Generate test signature before shipping. Preserve it across updates.
    - Better yet, find a way to not need signatures. (Filed a QC case...)
    - We need to also store these in case customer loses it

Customers cannot build our PX4. Can send out binary first.

## Other

M0053 / M0054 RC / GPS on DSP using UART 6 and 7

Implement CPU utilization monitor for apps as a second instance of load_mon

SLPI: Timeout waiting for parameter_client_set_value_response

Investigate switch to altitude mode when manual mode specified
- Something to do with the RC switches

User and Developer Documentation

Search for all uses of pthread_create. Try to use px4_task_spawn_cmd instead

Search for all uses of clock_gettime. Try to use px4_clock_gettime instead

Log file management

Crash analysis tools
