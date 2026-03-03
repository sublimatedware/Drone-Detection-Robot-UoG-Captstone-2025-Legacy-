import serial 
import time 

arduino = serial.Serial(port='COM4',  baudrate=115200, timeout=1)

while True:
    #num = 100
    #print(num)
    #print(num.to_bytes())
    serial_send = bytes('~','utf-8')
    arduino.write(serial_send)
    time.sleep(0.05)
    inthing = arduino.read()
    print(inthing)
