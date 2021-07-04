
# Priorities

## Primary

Testing
- Issue fix (stability)
- Validate update rate of all drivers in SLPI
- Use battery, calibrate, arm and "fly"

Add queueing to make sure there are no lost IMU / Barometer samples in the thin clients

Add mutex into tasks.cpp to protect the main data structure integrity.

Change pthread_create in WorkQueueManager.cpp to use a new version of px4_task_spawn_cmd

Merge branch and produce v0.0.2 for testing. Load it all on to the new drone and test

## Secondary

Investigate switch to altitude mode when manual mode specified
- Something to do with the RC switches

Run with MAV_BROADCAST 0 and implement mavlink proxy
   * So we can direct the drone to a specific GCS

User and Developer Documentation

Fix pthread issues in calibration procedures:
- commander/worker_thread. Switched to commander thread (inline) as a hack
- SubscriptionBlocking causes crash due to some pthread call
- Or just run commander on apps side?

qshell_retval sequence number mismatch and subsequent timeout
Is there an advertise loop if the topic is local and remote?

Search for all uses of pthread_create. Try to use px4_task_spawn_cmd instead

Search for all uses of clock_gettime. Try to use px4_clock_gettime instead
