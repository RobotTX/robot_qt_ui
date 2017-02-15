import time
import serial
import commands


# configure the serial connections (the parameters differs on the device you are connecting to)
deviceNode = "1-2:1.0"
(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
devPort = "/dev"+output[output.index(deviceNode)+7:output.index(deviceNode)+15]
print devPort

sonar = serial.Serial(
    port = devPort,
        baudrate = 38400,
        stopbits = 1,
        bytesize = serial.EIGHTBITS,
        timeout = 0.5
)

if sonar.isOpen():
    print "Connected on port", sonar.port 


    while True:

        sonar.write("\x02")
        res = sonar.read(1)
        if len(res) == 1:
            print '\nReset :', ord(res[0])
        else:
            print "\nLength read :", len(res)

        sonar.write("\x03")
        res = sonar.read(1)
        if len(res) == 1:
            print '\nReset :', ord(res[0])
        else:
            print "\nLength read :", len(res)



    
