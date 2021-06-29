
# Priorities

## Primary

Debug SLPI "going silent" after some period of time
- Need a full meta build to run crash analysis portal (QCAP)
- Experiment with sysmon

## Secondary

Logging bringup

Navigator bringup

Run with MAV_BROADCAST 0 and implement mavlink proxy
   * So we can direct the drone to a specific GCS

HIL bringup

User and Developer Documentation

Fix pthread issues in calibration procedures:
- commander/worker_thread. Switched to commander thread (inline) as a hack
- SubscriptionBlocking causes crash due to some pthread call
- Or just run commander on apps side?

qshell_retval sequence number mismatch and subsequent timeout
Is there an advertise loop if the topic is local and remote?
