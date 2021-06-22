#!/bin/bash

set -e

if [ $# -lt 1 ]; then
	echo "Error, need to specify a network interface"
	exit -1
fi

# The interface to check is the required first argument.
# This is usually something like "eth0" for ethernet or
# "wlan0" for wifi.
INTERFACE=$1

# An address prefix is the optional second argument. If
# you know that the address will always start with, for example,
# "192.168.0" then you can specify that as the prefix to
# make sure that any assigned address is valid.
PREFIX=
if [ $# -gt 1 ]; then
	PREFIX=$2
fi

# This function uses the ifconfig command to query the desired interface.
# awk is then used to get the IP address if it has been assigned.
get_address () {
	ADDRESS="$(ifconfig $INTERFACE | awk '/inet / {print $2}')"
}

# Get the address. If it exists and if the optional prefix matches
# then exit the loop. Otherwise, sleep for a second and then check
# again.
# TODO: Add an optional timeout
get_address
while [[ -z $ADDRESS || ("$ADDRESS" != "$PREFIX"*) ]]; do
	get_address
	if [[ $ADDRESS ]]; then
		echo $ADDRESS
	else
		echo "0.0.0.0"
	fi
	sleep 1
done

echo $ADDRESS
