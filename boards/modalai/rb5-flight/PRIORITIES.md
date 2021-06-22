
# Priorities

## Primary

Debug SLPI "going silent" after some period of time
- Create a heartbeat task in slpi_proc to send out a periodic hello world message
- Need a full meta build to run crash analysis portal (QCAP)

PX4 Autostart service (Needs to wait for network connection first)

Create an ipk to load everything onto target
- Verify with a fresh 8.1 installation

## Secondary

GPS driver via USB-to-UART dongle

Logging bringup

Run with MAV_BROADCAST 0 and implement mavlink proxy
   * So we can direct the drone to a specific GCS

HIL bringup

User and Developer Documentation

Fix pthread issues in calibration procedures:
- commander/worker_thread. Switched to commander thread (inline) as a hack
- SubscriptionBlocking causes crash due to some pthread call
- Try placing commander on apps side

qshell_retval sequence number mismatch and subsequent timeout
Is there an advertise loop if the topic is local and remote?
