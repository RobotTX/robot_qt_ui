#!/usr/bin/env python

import time
import serial
import commands
import time 

file_battery = "/home/gtdollar/computer_software/Robot_Infos/battery.txt"

# configure the serial connections (the parameters differs on the device you are connecting to)
deviceNode = "1-3.2:1.0"
(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
devPort = "/dev"+output[output.index(deviceNode)+9:output.index(deviceNode)+17]
print devPort

sonar = serial.Serial(
	port = devPort,
        baudrate = 38400,
        stopbits = 1,
        bytesize = serial.EIGHTBITS,
        timeout = 0.5
)

if sonar.isOpen():
	sonar.write("\xBA\x02\x00\x00\x00\x00\x00\x00\x00\x00\x1B")
	res = sonar.read(5)
	time.sleep(1)

	sonar.write("\xBA\x01\x00\x00\x00\x00\x00\x00\x00\x00\x1B")
	res = sonar.read(15)

	if len(res) == 15:
		print 'Writing state of charge :', ord(res[3]), '%'
		with open(file_battery, 'w') as file_name:
			file_name.truncate()
			file_name.write(str(ord(res[3])))
			file_name.close()
	else:
		print "Wrong length read :", len(res)

else:
	print "Could not connect on port", sonar.port 


