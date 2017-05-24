#!/bin/bash

SECONDS=0
while true
do
	if ! ( echo `fping www.google.com` | grep -q "alive" ) ; then
		duration=$((SECONDS / 60))
		if ! [[ $duration  -gt 10 ]] ; then
			wlan=$(ip link show | grep wlan | cut -d: -f2 | awk '{print $1}')
			ifdown $wlan
			SECONDS=0
			ifup $wlan
		else
			reboot;
		fi
	fi
done
