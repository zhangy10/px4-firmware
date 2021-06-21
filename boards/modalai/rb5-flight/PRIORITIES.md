
# Priorities

## Primary

Debug SLPI "going silent" after some period of time
- Create a heartbeat task in slpi_proc to send out a periodic hello world message

PX4 Autostart service (Needs to wait for network connection first)

Create an ipk to load everything onto target
- Verify with a fresh 8.1 installation

## Secondary

GPS driver via USB-to-UART dongle

Logging bringup

HIL bringup

User and Developer Documentation

Fix pthread issues in calibration procedures:
- commander/worker_thread. Switched to commander thread (inline) as a hack
- SubscriptionBlocking causes crash due to some pthread call

qshell_retval sequence number mismatch and subsequent timeout
