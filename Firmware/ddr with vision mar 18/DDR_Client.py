# Shitty drone detection robot code we're gonna strap a glizzy to the side of this thing and I'm not talking about a fuckking hotdog
# Jacob Lamoureux Winter 2025
# University of Guelph

# Requires high speed telemetrix library
# OR Requires that max speed, max accelearation, set speed, all that has its max increased from 1000 to 10000 in the telemtrix library file
# Located in AppData\Local\Programs\Python\Python313\Lib\site-packages\telemetrix\telemetrix.py

import sys
import time
import cv2
import numpy as np

#from telemetrix import telemetrix
from telemetrix import telemetrix_high_speed
from pynput.mouse import Controller


MAX_MOTOR_Z = -5350 #DO NOT CHANGE
MAX_MOTOR_Y = 1490 #DO NOT CHANGE EITHER

MAX_MOUSE_Z = 1919
MAX_MOUSE_Y = 1079 #Your monitors vertical resosleaust

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


board_Y = telemetrix_high_speed.Telemetrix(arduino_instance_id=1)
board_Z = telemetrix_high_speed.Telemetrix(arduino_instance_id=2)

mouse = Controller()

cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)

def Limit_Y_Callback(data):
    global limit_Y_state
    global false_start_Y
    global homed_Y
    limit_Y_state = (data[2])
    if limit_Y_state == 0:
        board_Y.stepper_stop(motor_Y)
    if limit_Y_state == 0 and homing_Y_initiated == 1:
        homed_Y = 1
        board_Y.stepper_set_current_position(motor_Y, 0)
        board_Y.stepper_set_max_speed(motor_Y,5000)
        time.sleep(.01)
        board_Y.stepper_set_acceleration(motor_Y, 10000)
        time.sleep(.01)
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
        board_Z.stepper_set_current_position(motor_Z, 0)
        board_Z.stepper_set_max_speed(motor_Z,4000)
        time.sleep(.01)
        board_Z.stepper_set_acceleration(motor_Z, 8000)
        time.sleep(.01)
        
       
def motor_Y_complete_callback(data):
    pass

def motor_Z_complete_callback(data):
    pass
    
def motor_Z_current_position_callback(data):
    print(f'motor_Z pos: {data[2]}\n')
    
def motor_Y_current_position_callback(data):
    print(f'motor_Y pos: {data[2]}\n')

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
    
def find_bluest_spot(frame):
    # Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define lower and upper bounds for blue color
    lower_blue = np.array([100, 120, 100])
    upper_blue = np.array([130, 255, 255])
    
    # Create a mask to detect blue color
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(mask, (15, 15), 0)
    
    # Find the bluest pixel location
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(blurred)
    return max_loc  # (x, y) coordinates of the bluest spot
    
    
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


if limit_Y_state == 0:
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
            print("\rHoming Z...      ", end='')
            sys.stdout.flush()
        elif homed_Y == 0 and homed_Z ==1:
            print("\rHoming Y...      ", end='')
            sys.stdout.flush()
        else:
            print("\rHoming Z and Y...", end='')
            sys.stdout.flush()
    print("shit happeng")
    
    time.sleep(1)
    
    print("\nProgram Begin")
    
    while True:
        ret, frame = cap.read()   
        
        
        
        x, y = find_bluest_spot(frame)                 #chatGPTs finest in computer vision research
        cv2.circle(frame, (x, y), 2, (255, 0, 0), 2)  # Blue circle
        cv2.imshow("THINGY",frame)
        
        Z_destination = int(np.interp(mouse.position[0], [0,MAX_MOUSE_Z], [MAX_MOTOR_Z,0]))             #maps the position of the mouse on screen to the range of the motor
        Y_destination = int(np.interp(mouse.position[1], [0,MAX_MOUSE_Y], [0,MAX_MOTOR_Y]))             #the z motor has range (0, -5350) and y has range (0, +1490). going past this range causes the motors to grind (actually harmless but it sure sounds bad)
            

        board_Z.stepper_move_to(motor_Z, Z_destination) #sets the motor to move to a pos relative to 0 (its homing position). move_to(0) should theroetically always go home if the motor doesnt skip along the way
        board_Y.stepper_move_to(motor_Y, Y_destination) #see also "stepper_move" which will move relative to its current position, also adjust set_acceleration and set_max_speed
        
        board_Z.stepper_run(motor_Z, motor_Z_complete_callback) # this command actually causes the motor to move after all parameters have been set
        board_Y.stepper_run(motor_Y, motor_Y_complete_callback)
        
        cv2.waitKey(1) #this is crucial for some fucking reason, the camera feed wont work without this
        
        time.sleep(0.008) #approx 120 fps.every time a command is sent to the arduino (board_"x"."y"()) there should be a short sleep(0.001) to give time for the ardunio to read and process before sending another command. important when using set_max_speed and set_acceleration
        
except KeyboardInterrupt:
    board_Z.stepper_stop(motor_Z)
    board_Y.stepper_stop(motor_Y)
    board_Z.digital_write(DISABLE_Z, 1)
    board_Y.digital_write(DISABLE_Y, 1)
    board_Z.shutdown()
    board_Y.shutdown()
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)
