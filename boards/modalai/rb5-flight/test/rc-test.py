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

##################
#
# Start of main
#
##################

loop_count = 0

while True:
    loop_count += 1
    print("Running test loop " + str(loop_count))

    reboot_VOXL()

    result = subprocess.check_output(['adb', 'shell', 'rc-test']);

    print result

    if 'Error:' in result:
        sys.exit(-1)

    print "Pause before reboot"
    time.sleep(2)
