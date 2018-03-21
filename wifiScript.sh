#!bin/bash

> wifi.txt
nmcli -t -f ssid dev wifi >> wifi.txt
