
# Priorities

First flight video

Give PX4 overview in SW meeting 6/14

Debug SLPI "going silent" after some period of time
- Create a heartbeat task in slpi_proc to send out a periodic hello world message

Parameters not stored in local file, have to be in config file at startup

Parameter startup errors:
- ERROR [parameters] failed to open param file: /home/linaro/eeprom/parameters
- ERROR [parameters] param auto save failed (-1)

Logging bringup

HIL bringup

Documentation

Create a debian package to load everything onto target

Fix pthread issues in calibration procedures:
- commander/worker_thread.* switched to px4_task_spawn_cmd as a hack
- SubscriptionBlocking causes crash due to some pthread call
- Cannot create a second worker thread, px4_task_spawn_cmd pthread_create call fails

qshell_retval sequence number mismatch and subsequent timeout

Figure out how to call HAP_power_request from px4
- Filed a case with Qualcomm on 6/10
