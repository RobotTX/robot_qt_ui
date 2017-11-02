import imp
pin = imp.load_source('Pin','/home/ubuntu/movement/pwmControl/Pin.py')
gpio = imp.load_source('Gpio','/home/ubuntu/movement/pwmControl/Gpio.py')

pin = gpio.Gpio("9.41","in")

pin = gpio.Gpio("9.23",'out')
pin.setValue(1)
