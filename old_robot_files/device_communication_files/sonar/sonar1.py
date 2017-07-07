import time
import serial
import commands


# configure the serial connections (the parameters differs on the device you are connecting to)
deviceNode = "1-2:1.1"
(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
devPort = "/dev"+output[output.index(deviceNode)+7:output.index(deviceNode)+15]
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

        sonar.write("\xE8\x51\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
        res = sonar.read(6)
        if len(res) == 6:
            print 'sonar 0', ord(res[3])*256 + ord(res[4])
        else:
           print len(res)

        time.sleep(1)

    
