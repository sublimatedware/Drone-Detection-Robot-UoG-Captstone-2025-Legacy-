import sys
import time

from telemetrix import telemetrix

DISABLE_Y = 6
DIR_Y = 5
PULSE_Y = 3
LIMIT_Y = 2

DISABLE_Z = 11
DIR_Z = 10
PULSE_Z = 9
LIMIT_Z = 8

exit_flag = 0

def Limit_Y_Callback(data):
    print("Y limit: ", end = "")
    print(data[2])

def Limit_Z_Callback(data):
    print("Z limit: ", end = "")
    print(data[2])

board = telemetrix.Telemetrix()

board.set_pin_mode_digital_input(LIMIT_Y, Limit_Y_Callback)
time.sleep(.01)

board.set_pin_mode_digital_input(LIMIT_Z, Limit_Z_Callback)
time.sleep(.01)

motor_Z = board.set_pin_mode_stepper(interface = 1, pin1 = PULSE_Z, pin2 = DIR_Z)
time.sleep(.01)

motor_Y = board.set_pin_mode_stepper(interface = 1, pin1 = PULSE_Y, pin2 = DIR_Y)
time.sleep(.01)

board.set_pin_mode_digital_output(DISABLE_Z)
time.sleep(.01)

board.set_pin_mode_digital_output(DISABLE_Y)
time.sleep(.01)


try:
    while True:
        #pass
        board.digital_write(DISABLE_Y, 1)
        board.digital_write(DISABLE_Z, 1)
        time.sleep(.01)
except KeyboardInterrupt:
    board.shutdown()
    sys.exit(0)
