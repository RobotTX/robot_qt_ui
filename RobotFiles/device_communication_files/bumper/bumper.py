import time
import serial
import commands

# configure the serial connections (the parameters differs on the device you are connecting to)
deviceNode = "1-2:1.1"
(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
devPort = "/dev"+output[output.index(deviceNode)+7:output.index(deviceNode)+15]
print devPort

bumper = serial.Serial(
	port = devPort,
        baudrate = 115200,
        stopbits = 1,
        bytesize = serial.EIGHTBITS,
        timeout = 1
)
bumper.close()
bumper.open()

if bumper.isOpen():
    while True:
        bumper.write("\xC0\x01\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
	res = bumper.read(12)
	print "bumper read len:",len(res)
	if len(res) ==12:
		print ord(res[3]),ord(res[4]),ord(res[5]),ord(res[6]),ord(res[7]),ord(res[8]),ord(res[9]),ord(res[10])
	time.sleep(1)    
