#!/bin/bash
val=`iwlist scan`
if echo "$val" | grep -q "wlan6"; then
	rm /etc/udev/rules.d/70-persistent-net.rules ;
	reboot;
else 
	echo "no"
	sh /home/gtdollar/startup/internet_check.sh

fi
