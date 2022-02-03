
import os
import time
import subprocess

# First data value is the timestamp. Second one is the count of consecutive matching
# timestamp values
topic_data = {'gps': [0, 0], 'barometer': [0, 0], 'battery': [0, 0], 'mag': [0, 0]}

def print_topic_timestamp(topic_name, topic_info):
    TIMESTAMP_POSITION = 3
    len_topic_info = len(topic_info)
    if len_topic_info > TIMESTAMP_POSITION:
        timestamp_value = topic_info[TIMESTAMP_POSITION].split()[1]
        output = topic_name + ' ' + timestamp_value
        if topic_data[topic_name][0] != int(timestamp_value):
            topic_data[topic_name][0] = int(timestamp_value)
            topic_data[topic_name][1] = 0
        else:
            output += " timestamp didn't change!!!"
            topic_data[topic_name][1] += 1
            if topic_data[topic_name][1] > 2:
                raise RuntimeError

        print output
    else:
        print topic_name + 'Incorrect list length ' + str(len_topic_info)
        print topic_info

#######################
#
# Start of main program
#
#######################

start_time = int(time.time())

TOPIC_NAME_POSITION = 3

listener_command = ['px4-listener', '--instance', '0', '-n', '1']

get_gps = listener_command[:]
get_barometer = listener_command[:]
get_battery_status = listener_command[:]
get_magnetometer = listener_command[:]

get_gps.insert(TOPIC_NAME_POSITION, 'sensor_gps')
get_barometer.insert(TOPIC_NAME_POSITION, 'sensor_baro')
get_battery_status.insert(TOPIC_NAME_POSITION, 'battery_status')
get_magnetometer.insert(TOPIC_NAME_POSITION, 'sensor_mag')

try:
    print
    while True:
        print_topic_timestamp('gps', subprocess.check_output(get_gps).splitlines())
        print_topic_timestamp('barometer', subprocess.check_output(get_barometer).splitlines())
        print_topic_timestamp('battery', subprocess.check_output(get_battery_status).splitlines())
        print_topic_timestamp('mag', subprocess.check_output(get_magnetometer).splitlines())
        print
        time.sleep(6)
except KeyboardInterrupt:
    print 'Got ctrl-c'
except RuntimeError:
    print 'RuntimeError'

total_time_seconds =  int(time.time()) - start_time
print
print 'Exited after ' + str(total_time_seconds) + ' seconds'
print '(' + str(total_time_seconds / 60) + ' minutes)'
print
