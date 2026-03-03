# Shitty drone detection robot code we're gonna strap a glizzy to the side of this thing and I'm not talking about a fuckking hotdog
# Jacob Lamoureux Winter 2025
# University of Guelph

#Requires that max speed, max accelearation, set speed, all that has its max increased from 1000 to 10000 in the telemtrix library file

import sys
import time

from telemetrix import telemetrix

DISABLE_Y = 6
DIR_Y = 5
PULSE_Y = 3
LIMIT_Y = 8
limit_Y_state = 0
homing_Y_initiated = 0
homed_Y = 0

DISABLE_Z = 6
DIR_Z = 5
PULSE_Z = 3
LIMIT_Z = 8
limit_Z_state = 0
homing_Z_initiated = 0
homed_Z = 0

#board_Y = telemetrix.Telemetrix(com_port=COM1, arduino_instance_id=1)
#board_Z = telemetrix.Telemetrix(com_port=COM3, arduino_instance_id=2)

board_Y = telemetrix.Telemetrix(arduino_instance_id=1)
board_Z = telemetrix.Telemetrix(arduino_instance_id=2)

def Limit_Y_Callback(data):
    global limit_Y_state
    global false_start_Y
    global homed_Y
    limit_Y_state = (data[2])
    if limit_Y_state == 0:
        board_Y.stepper_stop(motor_Y)
    if limit_Y_state == 0 and homing_Y_initiated == 1:
        homed_Y = 1
    #pass

def Limit_Z_Callback(data):
    global limit_Z_state
    global false_start_Z
    global homed_Z
    limit_Z_state = data[2]
    if limit_Z_state == 1:
        board_Z.stepper_stop(motor_Z)
    if limit_Z_state == 1 and homing_Z_initiated == 1:
        homed_Z = 1
       
def motor_Z_complete_callback(data):
    pass
    #global limit_Z_state
    #limit_Z_state = 0

def false_start_Z_callback(data):
    #run opposite direction from whatever
    board_Z.stepper_move(motor_Z,0)
    time.sleep(.01)
    board_Z.stepper_set_speed(motor_Z,100)
    time.sleep(.01)
    board_Z.stepper_run_speed(motor_Z)
    
def false_start_Y_callback(data):
    #run opposite direction from whatever
    board_Y.stepper_move(motor_Y,0)
    time.sleep(.01)
    board_Y.stepper_set_speed(motor_Y,-100)
    time.sleep(.01)
    board_Y.stepper_run_speed(motor_Y)
    
    


board_Y.set_pin_mode_digital_input(LIMIT_Y, Limit_Y_Callback)
time.sleep(.01)

board_Z.set_pin_mode_digital_input(LIMIT_Z, Limit_Z_Callback)
time.sleep(.01)

motor_Y = board_Y.set_pin_mode_stepper(interface = 1, pin1 = PULSE_Y, pin2 = DIR_Y)
time.sleep(.01)

motor_Z = board_Z.set_pin_mode_stepper(interface = 1, pin1 = PULSE_Z, pin2 = DIR_Z)
time.sleep(.01)

board_Y.set_pin_mode_digital_output(DISABLE_Y)
time.sleep(.01)

board_Z.set_pin_mode_digital_output(DISABLE_Z)
time.sleep(.01)

board_Y.stepper_set_max_speed(motor_Y,4000)
time.sleep(.01)
board_Z.stepper_set_max_speed(motor_Z,4000)
time.sleep(.01)

board_Y.stepper_set_acceleration(motor_Y, 1000)
time.sleep(.01)
board_Z.stepper_set_acceleration(motor_Z, 1000)
time.sleep(.01)

#board.digital_write(DISABLE_Z, 1)

if limit_Y_state == 0:
    print("false start Y")
    #all the shit to make it move
    board_Y.stepper_set_acceleration(motor_Y, 10000)
    board_Y.stepper_move(motor_Y,150)
    board_Y.stepper_run_speed_to_position(motor_Y, false_start_Y_callback)
    homing_Y_initiated = 1
else:
    board_Y.stepper_move(motor_Y,0)
    time.sleep(.01)
    board_Y.stepper_set_speed(motor_Y,-100)
    time.sleep(.01)
    board_Y.stepper_run_speed(motor_Y)
    homing_Y_initiated = 1

if limit_Z_state == 1:
    print("false start Z")
    board_Z.stepper_set_acceleration(motor_Z, 10000)
    board_Z.stepper_move(motor_Z,-150)
    board_Z.stepper_run_speed_to_position(motor_Z, false_start_Z_callback)
    homing_Z_initiated = 1
else:
    board_Z.stepper_move(motor_Z,0)
    time.sleep(.01)
    board_Z.stepper_set_speed(motor_Z,100)
    time.sleep(.01)
    board_Z.stepper_run_speed(motor_Z)
    homing_Z_initiated = 1

try:
    while homed_Z == 0 or homed_Y == 0:
        if homed_Z == 0 and homed_Y == 1:
            print("Homing Z")
        elif homed_Y == 0 and homed_Z ==1:
            print("Homing Y")
        else:
            print("Homing X and Y")
        time.sleep(0.01)
        #pass
    while True:
        print("Homed :D")
        #pass
except KeyboardInterrupt:
    board_Z.stepper_stop(motor_Z)
    board_Y.stepper_stop(motor_Y)
    board_Z.digital_write(DISABLE_Z, 1)
    board_Y.digital_write(DISABLE_Y, 1)
    board_Z.shutdown()
    board_Y.shutdown()
    sys.exit(0)
