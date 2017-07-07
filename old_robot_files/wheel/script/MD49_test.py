#!/usr/bin/env python

import sys
import time
import serial
import commands
import struct


if __name__ == "__main__":
    deviceNode="2-3.3:1.1"
    (status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
    devPort = "/dev"+output[output.index(deviceNode)+9:output.index(deviceNode)+17]
    print devPort
    wheel_serial = serial.Serial(port = devPort, baudrate=9600)
    wheel_serial.close()
    wheel_serial.open()

    #wheel mode 0
    wheel_serial.write(serial.to_bytes([0x00,0x34,0x00]))
    #disable 2sec timeout
    wheel_serial.write(serial.to_bytes([0x00,0x38]))
    #acceleration step 5
    wheel_serial.write(serial.to_bytes([0x00,0x33,0x01]))

    print "Start : %s" % time.ctime()

    wheel_serial.write(serial.to_bytes([0x00,0x31]))
    wheel_serial.write(chr(int(sys.argv[1])))
    wheel_serial.write(serial.to_bytes([0x00,0x32]))
    wheel_serial.write(chr(int(sys.argv[1])))

    time.sleep(20)

    wheel_serial.write(serial.to_bytes([0x00,0x31]))
    wheel_serial.write(chr(128))
    wheel_serial.write(serial.to_bytes([0x00,0x32]))
    wheel_serial.write(chr(128))

    print "End : %s" % time.ctime()
