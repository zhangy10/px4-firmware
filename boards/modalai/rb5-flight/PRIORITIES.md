
# Priorities

First flight video

Give PX4 overview in SW meeting 6/14

Debug SLPI "going silent" after some period of time
- Create a heartbeat task in slpi_proc to send out a periodic hello world message

PX4 Autostart service

Logging bringup

HIL bringup

Documentation

Create a Debian package to load everything onto target
- Verify with a fresh 8.1 installation

Fix pthread issues in calibration procedures:
- commander/worker_thread. Switched to commander thread (inline) as a hack
- SubscriptionBlocking causes crash due to some pthread call

qshell_retval sequence number mismatch and subsequent timeout

Figure out how to call HAP_power_request from px4
- Verify the method suggested by Qualcomm
