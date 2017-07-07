import time
import serial
import commands


# configure the serial connections (the parameters differs on the device you are connecting to)
deviceNode = "1-3.3:1.0"
(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
devPort = "/dev"+output[output.index(deviceNode)+9:output.index(deviceNode)+17]
print devPort

sonar = serial.Serial(
	port = devPort,
        baudrate = 115200,
        stopbits = 1,
        bytesize = serial.EIGHTBITS,
        timeout = 0.5
)

if sonar.isOpen():
    print "Connected on port", sonar.port 

    while True:
        print "Ready to read value"

        #sonar.write("\xEE\x51\x04\xE0\xE2\xE4\xE6\x00\x00\x00\xB1")
        #res = sonar.read(12)

        #if len(res) == 12:
        #    print 'sonar0:', ord(res[3])*256 + ord(res[4]), ',sonar2:', ord(res[5])*256 + ord(res[6]), ',sonar4:', ord(res[7])*256 + ord(res[8]), ',sonar6:', ord(res[9])*256 + ord(res[10])
        #else:
        #    print len(res)

        #time.sleep(0.5)

        sonar.write("\xE0\x51\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
        res = sonar.read(6)
        if len(res) == 6:
            print 'sonar 0', ord(res[3])*256 + ord(res[4])
        else:
           print len(res)

        #time.sleep(4)

    
