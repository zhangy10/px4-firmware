
import subprocess

#######################
#
# Start of main program
#
#######################

listener_command = ['px4-listener', '--instance', '0', '-n', '1']

get_global_position = listener_command[:]
get_local_position = listener_command[:]
get_barometer = listener_command[:]

TOPIC_NAME_POSITION = 3

get_global_position.insert(TOPIC_NAME_POSITION, 'vehicle_global_position')
get_local_position.insert(TOPIC_NAME_POSITION, 'vehicle_local_position')
get_barometer.insert(TOPIC_NAME_POSITION, 'sensor_baro')

print
print subprocess.check_output(get_global_position)
print subprocess.check_output(get_local_position)
print subprocess.check_output(get_barometer)
print
