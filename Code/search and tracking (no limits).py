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

# Variables for search pattern
search_mode = True
search_direction_z = 1  # 1 for right, -1 for left
search_direction_y = 1  # 1 for up, -1 for down
last_detection_time = 0
SEARCH_TIMEOUT = 2  # seconds before returning to search mode after losing detection

# Movement parameters
MAX_Z_STEPS = 160      # Maximum steps for Z axis per movement
MAX_Y_STEPS = 80      # Maximum steps for Y axis per movement
MIN_Z_STEPS = 1       # Minimum steps for Z axis
MIN_Y_STEPS = 1       # Minimum steps for Y axis
DEAD_ZONE_PERCENT = 0.10  # 10% dead zone

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
    board_Z.stepper_move(motor_Z,0)
    time.sleep(.01)
    board_Z.stepper_set_speed(motor_Z,100)
    time.sleep(.01)
    board_Z.stepper_run_speed(motor_Z)
    
def false_start_Y_callback(data):
    board_Y.stepper_move(motor_Y,0)
    time.sleep(.01)
    board_Y.stepper_set_speed(motor_Y,-100)
    time.sleep(.01)
    board_Y.stepper_run_speed(motor_Y)

def get_drone_center(x1, y1, x2, y2):
    center_x = int((x1 + x2) / 2)
    center_y = int((y1 + y2) / 2)
    return center_x, center_y

def calculate_movement(delta, frame_size, max_steps, min_steps):
    """
    Calculate movement steps based on how far the object is from center.
    Uses a non-linear response for smoother tracking near center and faster response at edges.
    
    Args:
        delta: Distance from center (pixels)
        frame_size: Size of frame in the relevant dimension (width or height)
        max_steps: Maximum steps to move
        min_steps: Minimum steps to move (even when just outside dead zone)
        
    Returns:
        int: Number of steps to move
    """
    # Calculate normalized distance from center (0 to 1)
    normalized_dist = abs(delta) / (frame_size / 2)
    
    # Apply non-linear scaling (quadratic curve for smooth acceleration)
    # This gives finer control near center and faster movement at edges
    scaled_value = normalized_dist ** 2
    
    # Calculate steps with minimum and maximum bounds
    steps = int(min_steps + (max_steps - min_steps) * scaled_value)
    
    # Ensure we're within bounds
    return min(steps, max_steps)

def execute_search_pattern(current_z, current_y):
    """Execute a smooth but responsive search pattern within motor limits"""
    global search_direction_z, search_direction_y
    
    # Check Z motor limits and reverse direction if needed
    if current_z <= MAX_MOTOR_Z:
        search_direction_z = 1  # reverse to right
        current_z = MAX_MOTOR_Z  # prevent going beyond limit
    elif current_z >= 0:
        search_direction_z = -1  # reverse to left
        current_z = 0  # prevent going beyond limit
        
    # Check Y motor limits and reverse direction if needed
    if current_y >= 400:
        search_direction_y = -1  # reverse to down
        current_y = 400  # prevent going beyond limit
    elif current_y <= 0:
        search_direction_y = 1  # reverse to up
        current_y = 0  # prevent going beyond limit
    
    # Configure movement parameters - faster than tracking but still smooth
    search_speed_z = 1000  # steps/sec for Z axis
    search_speed_y = 800   # steps/sec for Y axis
    search_distance_z = search_direction_z * 5  # steps for Z axis
    search_distance_y = search_direction_y * 2  # steps for Y axis
    
    # Set movement parameters
    board_Z.stepper_set_max_speed(motor_Z, search_speed_z)
    board_Y.stepper_set_max_speed(motor_Y, search_speed_y)
    
    # Set acceleration for smooth but responsive movement
    board_Z.stepper_set_acceleration(motor_Z, 5000)
    board_Y.stepper_set_acceleration(motor_Y, 4000)
    
    # Move motors
    board_Z.stepper_move(motor_Z, search_distance_z)
    board_Y.stepper_move(motor_Y, search_distance_y)
    
    # Start the movement
    board_Z.stepper_run(motor_Z, motor_Z_complete_callback)
    board_Y.stepper_run(motor_Y, motor_Y_complete_callback)
    
    # Return updated positions
    return (current_z + search_distance_z, 
            current_y + search_distance_y)

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

    # Variables to track motor positions (approximate)
    current_z_pos = 0
    current_y_pos = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Wait for a key press and check if it's 'q'
        key = cv2.waitKey(1)
        if key == ord('q'):  # If 'q' is pressed, exit the loop
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
        
        drone_detected = False
        drone_center_x, drone_center_y = None, None

        # Process the results and draw bounding boxes on the frame
        for result in results[0].boxes.data:  # Access detected objects
            x1, y1, x2, y2, conf, cls = result.tolist()
            if conf > 0.5 and classes[int(cls)] in classes:
                drone_detected = True
                last_detection_time = time.time()
                search_mode = False
                
                # Draw bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                
                # Get drone center coordinates
                drone_center_x, drone_center_y = get_drone_center(x1, y1, x2, y2)

        # If drone is detected and we're not in search mode, track it
        if drone_detected and not search_mode and drone_center_x is not None and drone_center_y is not None:
            # Calculate distance from center
            delta_x = drone_center_x - screen_center_x
            delta_y = drone_center_y - screen_center_y
            
            # Only move if drone is outside dead zone
            if abs(delta_x) > dead_zone_x or abs(delta_y) > dead_zone_y:
                # Calculate movement steps using your existing function
                z_steps = calculate_movement(delta_x, frame.shape[1], MAX_Z_STEPS, MIN_Z_STEPS)
                y_steps = calculate_movement(delta_y, frame.shape[0], MAX_Y_STEPS, MIN_Y_STEPS)
                
                # Determine direction (negative steps for left/down)
                z_steps = -z_steps if delta_x < 0 else z_steps
                y_steps = y_steps if delta_y < 0 else -y_steps
                
                tracking_speed_z = 800  # steps/sec for Z axis
                tracking_speed_y = 600  # steps/sec for Y axis
                
                board_Z.stepper_set_max_speed(motor_Z, tracking_speed_z)
                board_Y.stepper_set_max_speed(motor_Y, tracking_speed_y)
                
                # Set acceleration for smooth movement
                board_Z.stepper_set_acceleration(motor_Z, 4000)
                board_Y.stepper_set_acceleration(motor_Y, 3000)
                
                # Move motors
                board_Z.stepper_move(motor_Z, z_steps)
                board_Y.stepper_move(motor_Y, y_steps)
                
                # Start the movement
                board_Z.stepper_run(motor_Z, motor_Z_complete_callback)
                board_Y.stepper_run(motor_Y, motor_Y_complete_callback)
                
                # Update approximate positions
                current_z_pos += z_steps
                current_y_pos += y_steps

        # If no drone detected, enter search mode after timeout
        elif not drone_detected:
            if time.time() - last_detection_time > SEARCH_TIMEOUT:
                search_mode = True
                
            if search_mode:
                current_z_pos, current_y_pos = execute_search_pattern(current_z_pos, current_y_pos)
                pass
            

        # Display the resulting frame
        cv2.imshow('frame', frame)

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