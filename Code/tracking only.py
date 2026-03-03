import sys
import time
import cv2
import numpy as np
from ultralytics import YOLO
from telemetrix import telemetrix_high_speed

MAX_MOTOR_Z = -5350  # DO NOT CHANGE
MAX_MOTOR_Y = 1490   # DO NOT CHANGE EITHER

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

# Initialize both motor boards
board_Y = telemetrix_high_speed.Telemetrix(arduino_instance_id=1)
board_Z = telemetrix_high_speed.Telemetrix(arduino_instance_id=2)

# Load YOLOv8 model
model = YOLO('C:/VSCode/DroneDetection/Advanced-Aerial-Drone-Detection-System/best_test.pt')

# Define the classes you want to detect
classes = ['Drone']

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

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

def get_drone_center(x1, y1, x2, y2):
    center_x = int((x1 + x2) / 2)
    center_y = int((y1 + y2) / 2)
    return center_x, center_y
    
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

# Define dead zone as a percentage of the screen's width and height
DEAD_ZONE_PERCENT = 0.10  # 10% dead zone

try:
    while homed_Y == 0 or homed_Z == 0:
        if homed_Y == 0:
            print("\rHoming Y...      ", end='')
        if homed_Z == 0:
            print("\rHoming Z...      ", end='')
        sys.stdout.flush()
    print("Homing complete")

    time.sleep(1)

    print("\nProgram Begin")

    # Create a window for display
    cv2.namedWindow('frame')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, -1)


        # Get the center of the screen
        screen_center_x = frame.shape[1] // 2
        screen_center_y = frame.shape[0] // 2

        # Calculate dead zone size
        dead_zone_x = int(screen_center_x * DEAD_ZONE_PERCENT)
        dead_zone_y = int(screen_center_y * DEAD_ZONE_PERCENT)

        # Run inference
        results = model(frame)

        # Process the results and draw bounding boxes on the frame
        for result in results[0].boxes.data:  # Access detected objects
            x1, y1, x2, y2, conf, cls = result.tolist()
            if conf > 0.5 and classes[int(cls)] in classes:
                # Draw bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

                # Get the center of the drone's bounding box
                drone_center_x, drone_center_y = get_drone_center(x1, y1, x2, y2)

                # Calculate the difference between the drone's center and the screen's center
                delta_x = drone_center_x - screen_center_x
                delta_y = drone_center_y - screen_center_y

                # Check if the drone is outside the dead zone
                if abs(delta_x) > dead_zone_x or abs(delta_y) > dead_zone_y:
                    # Move the motors based on the difference
                    # The speed is proportional to the distance from the center
                    speed_Z = int(np.interp(abs(delta_x), [0, screen_center_x], [0, 4000]))
                    speed_Y = int(np.interp(abs(delta_y), [0, screen_center_y], [0, 4000]))

                    if delta_x < 0:
                        board_Z.stepper_set_speed(motor_Z, speed_Z)
                        board_Z.stepper_move(motor_Z, -10)  # Move left
                    else:
                        board_Z.stepper_set_speed(motor_Z, speed_Z)
                        board_Z.stepper_move(motor_Z, 10)  # Move right

                    if delta_y < 0:
                        board_Y.stepper_set_speed(motor_Y, speed_Y)
                        board_Y.stepper_move(motor_Y, 5)  # Move down
                    else:
                        board_Y.stepper_set_speed(motor_Y, speed_Y)
                        board_Y.stepper_move(motor_Y, -5)  # Move up

                    # Execute motor movements
                    board_Z.stepper_run(motor_Z, motor_Z_complete_callback)
                    board_Y.stepper_run(motor_Y, motor_Y_complete_callback)
                else:
                    # Drone is within the dead zone, no movement needed
                    board_Z.stepper_stop(motor_Z)
                    board_Y.stepper_stop(motor_Y)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # Wait for a key press and check if it's 'q'
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # If 'q' is pressed, exit the loop
            break

        time.sleep(0.008)  # Approx 120 FPS.

except KeyboardInterrupt:
    pass
finally:
    # Clean up and shut down everything properly
    board_Z.stepper_stop(motor_Z)
    board_Y.stepper_stop(motor_Y)
    board_Z.digital_write(DISABLE_Z, 1)
    board_Y.digital_write(DISABLE_Y, 1)
    board_Z.shutdown()
    board_Y.shutdown()
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)