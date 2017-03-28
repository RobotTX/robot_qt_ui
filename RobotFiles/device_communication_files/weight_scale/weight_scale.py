import time
import serial
import commands

# configure the serial connections (the parameters differs on the device you are connecting to)
deviceNode = "1-3.2:1.0"
(status, output) = commands.getstatusoutput("ls -l /sys/class/tty/ttyUSB*")
devPort = "/dev"+output[output.index(deviceNode)+9:output.index(deviceNode)+17]
print devPort

weight_scale = serial.Serial(
    port = devPort,
    baudrate = 115200,
    stopbits = 1,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

weight_scale.close()
weight_scale.open()

if weight_scale.isOpen():
    
    weight_scale.write("\xA0\x02\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
    time.sleep(1)
    res = weight_scale.read(5)
    print "weight_scale read len calibration : ",len(res)
    if len(res) == 5:
        print "weight_scale calibration : ", ord(res[0]), ord(res[1]), ord(res[2]), ord(res[3]), ord(res[4])
    time.sleep(1)   

    while True:
        weight_scale.write("\xA0\x01\x00\x00\x00\x00\x00\x00\x00\x00\xB1")
        res = weight_scale.read(7)
        print "weight_scale read len : ",len(res)
        if len(res) == 7:
            print ord(res[0]), ord(res[1]), ord(res[2]), ord(res[3]), ord(res[4]), ord(res[5]), ord(res[6])
            print "weight_scale : ", (ord(res[3]) * 256 * 256 + ord(res[4]) * 256 + ord(res[5]))
        time.sleep(1)    
