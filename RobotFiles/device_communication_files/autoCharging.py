import time
import serial
import commands


# configure the serial connections (the parameters differs on the device you are connecting to)
#deviceNode = "1-8:1.1"
#(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyACM*")
#devPort = "/dev"+output[output.index(deviceNode)+7:output.index(deviceNode)+15]
devPort="/dev/ttyACM1"
print devPort

sonar = serial.Serial(
	port = devPort,
        baudrate = 115200,
        stopbits = 1,
        bytesize = serial.EIGHTBITS,
        timeout = 4
)

if sonar.isOpen():
    print "Connected on port", sonar.port 

    while True:
        print "Ready to read value"

        sonar.write("\x90\x02\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
	res = sonar.read()
	print len(res)
        if len(res) == 18:
	    if ord(res[0]) == 238:
            	print 'sonar 0', ord(res[3])
        else:
           print len(res)

        time.sleep(0.5)
