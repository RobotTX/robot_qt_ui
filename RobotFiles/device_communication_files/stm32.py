import time
import serial
import commands


# configure the serial connections (the parameters differs on the device you are connecting to)
devPort = "/dev/ttyACM1"
print devPort

stm32 = serial.Serial(
    port = devPort,
        baudrate = 115200,
        stopbits = 1,
        bytesize = serial.EIGHTBITS,
        timeout = 0.5
)

if stm32.isOpen():
    print "Connected on port", stm32.port 

    while True:
        print "Ready to read value"


        stm32.write("\x90\x01\x00\x00\x00\x00\x00\x00\x00\x00\x1B")
        res = stm32.read(20)
        if len(res) == 5:
            print 'stm32 ', ord(res[0]), ' ', ord(res[1]), ' ', ord(res[2]), ' ', ord(res[3]), ' ', ord(res[4])
        else:
           print 'got', len(res), 'byte'

        time.sleep(1)

    
