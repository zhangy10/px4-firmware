# File: auto-test.py
# Company: ModalAI, Inc.

# tcpdump command to look for RC_CHANNELS messages
# sudo tcpdump -X host 192.168.0.46 and udp port 14556 and ip[28]==0xfd and ip[35]==0x41

import argparse

from pyulog import *
from pyulog.px4 import *
from pymavlink import mavutil

import subprocess
import math
import time
import sys
import os

adb_cmd = 'adb shell '

def reboot_VOXL():
    print("Rebooting VOXL...")
    os.system("adb reboot")
    os.system("adb wait-for-device")

def wait_for_message(client, msg):
    print("Waiting for " + msg + "...")
    timer = 0
    got_message = False
    while timer < 10:
        m = client.recv_match(type=msg)
        if m is not None:
            print("Got " + msg)
            got_message = True
            break
        else:
            time.sleep(2)
        timer += 1
    return got_message

def dump_messages():
    #(2) Get latest log
    log_dir = "/home/linaro/log/"

    dirs = os.popen(adb_cmd + ' ls -Art ' + log_dir).read()

    dirs = dirs.split()
    dirs.sort()
    latest_dir = dirs[-1]
    print (latest_dir)

    logs = os.popen(adb_cmd + ' ls -Art ' + log_dir + latest_dir).read()
    print(logs)
    logs = logs.split()
    logs.sort()
    latest_log = logs[-1]

    fullpath_log = log_dir + latest_dir + "/" + latest_log #full path
    subprocess.call(["adb", "pull", fullpath_log, "."])


    #ulog_file_name = args.filename
    ulog_file_name = latest_log

    #disable_str_exceptions = args.ignore

    msg_filter = [] # we don't need the data messages
    ulog = ULog(ulog_file_name, msg_filter)

    ulog.logged_messages

    for m in ulog.logged_messages:
        m1, s1 = divmod(int(m.timestamp/1e6), 60)
        h1, m1 = divmod(m1, 60)

        #if m.log_level_str() == "ERROR":
        print("{:d}:{:02d}:{:02d} {:}: {:}".format(
            h1, m1, s1, m.log_level_str(), m.message))

##################
#
# Start of main
#
##################

loop_count = 0

while True:
    loop_count += 1
    print("Running test loop " + str(loop_count))

    os.system(adb_cmd + ' rm -fR /home/linaro/log/*')
    os.system(adb_cmd + ' sync')

    reboot_VOXL()

    # Create the connection
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14547')

    # Wait a heartbeat before sending commands
    print("Waiting for heartbeat...")
    master.wait_heartbeat()

    print("Got heartbeat")

    # Wait for other drivers and modules to startup
    time.sleep(10)

    # desired_msg = "ATTITUDE"
    desired_msg = "RC_CHANNELS"

    if not wait_for_message(master, desired_msg):
        print("Timeout on " + desired_msg + " message")

        time.sleep(10)

        dump_messages()
        sys.exit(-1)
