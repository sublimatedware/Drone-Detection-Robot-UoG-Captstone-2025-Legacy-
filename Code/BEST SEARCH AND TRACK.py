import sys
import time
import cv2
import numpy as np
from ultralytics import YOLO
from telemetrix import telemetrix_high_speed

# Motor physical limits (DO NOT CHANGE)
MAX_MOTOR_Z = -5350  # Maximum steps for Z axis (left limit)
MAX_MOTOR_Y = 1490   # Maximum steps for Y axis (up limit)

# Motor pins
DISABLE_Y = 6
DIR_Y = 5
PULSE_Y = 3
LIMIT_Y = 8

DISABLE_Z = 6
DIR_Z = 5
PULSE_Z = 3
LIMIT_Z = 8

# Motor states
limit_Y_state = 0
homing_Y_initiated = 0
homed_Y = 0

limit_Z_state = 0
homing_Z_initiated = 0
homed_Z = 0

# Initialize motor boards
board_Y = telemetrix_high_speed.Telemetrix(arduino_instance_id=1)
board_Z = telemetrix_high_speed.Telemetrix(arduino_instance_id=2)

# Load YOLOv8 model
model = YOLO('C:/VSCode/DroneDetection/Advanced-Aerial-Drone-Detection-System/best_test.pt')
classes = ['Drone']

# Video capture
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Tracking and search parameters
search_mode = True
search_direction_z = 1  # 1 for right, -1 for left
search_direction_y = 1  # 1 for up, -1 for down
last_detection_time = 0
SEARCH_TIMEOUT = 2  # seconds before returning to search mode

# Movement parameters
MAX_Z_STEPS = 160      # Maximum steps for Z axis per movement
MAX_Y_STEPS = 80       # Maximum steps for Y axis per movement
MIN_Z_STEPS = 1        # Minimum steps for Z axis
MIN_Y_STEPS = 1        # Minimum steps for Y axis
DEAD_ZONE_PERCENT = 0.10  # 10% dead zone

# Boundary tracking variables
current_z_pos = 0      # Tracks approximate Z position (steps from home)
current_y_pos = 0      # Tracks approximate Y position (steps from home)
SAFETY_MARGIN_Z = 100  # Safety margin in steps for Z axis
SAFETY_MARGIN_Y = 50   # Safety margin in steps for Y axis

def Limit_Y_Callback(data):
    global limit_Y_state, homed_Y, homing_Y_initiated
    limit_Y_state = data[2]
    if limit_Y_state == 0:
        board_Y.stepper_stop(motor_Y)
    if limit_Y_state == 0 and homing_Y_initiated == 1:
        homed_Y = 1
        board_Y.stepper_set_current_position(motor_Y, 0)
        board_Y.stepper_set_max_speed(motor_Y, 5000)
        time.sleep(0.01)
        board_Y.stepper_set_acceleration(motor_Y, 10000)
        time.sleep(0.01)

def Limit_Z_Callback(data):
    global limit_Z_state, homed_Z, homing_Z_initiated
    limit_Z_state = data[2]
    if limit_Z_state == 1:
        board_Z.stepper_stop(motor_Z)
    if limit_Z_state == 1 and homing_Z_initiated == 1:
        homed_Z = 1
        board_Z.stepper_set_current_position(motor_Z, 0)
        board_Z.stepper_set_max_speed(motor_Z, 4000)
        time.sleep(0.01)
        board_Z.stepper_set_acceleration(motor_Z, 8000)
        time.sleep(0.01)
        
def motor_Y_complete_callback(data):
    pass

def motor_Z_complete_callback(data):
    pass
    
def motor_Z_current_position_callback(data):
    print(f'motor_Z pos: {data[2]}\n')
    
def motor_Y_current_position_callback(data):
    print(f'motor_Y pos: {data[2]}\n')

def false_start_Z_callback(data):
    board_Z.stepper_move(motor_Z, 0)
    time.sleep(0.01)
    board_Z.stepper_set_speed(motor_Z, 100)
    time.sleep(0.01)
    board_Z.stepper_run_speed(motor_Z)
    
def false_start_Y_callback(data):
    board_Y.stepper_move(motor_Y, 0)
    time.sleep(0.01)
    board_Y.stepper_set_speed(motor_Y, -100)
    time.sleep(0.01)
    board_Y.stepper_run_speed(motor_Y)

def get_drone_center(x1, y1, x2, y2):
    return int((x1 + x2) / 2), int((y1 + y2) / 2)

def calculate_movement(delta, frame_size, max_steps, min_steps):
    """Calculate movement steps with non-linear response"""
    normalized_dist = abs(delta) / (frame_size / 2)
    scaled_value = normalized_dist ** 2
    steps = int(min_steps + (max_steps - min_steps) * scaled_value)
    return min(steps, max_steps)

def is_within_z_bounds(proposed_steps):
    """Check if proposed movement would stay within Z bounds"""
    global current_z_pos
    new_pos = current_z_pos + proposed_steps
    return (MAX_MOTOR_Z + SAFETY_MARGIN_Z <= new_pos <= 0 - SAFETY_MARGIN_Z)

def is_within_y_bounds(proposed_steps):
    """Check if proposed movement would stay within Y bounds"""
    global current_y_pos
    new_pos = current_y_pos + proposed_steps
    return (0 + SAFETY_MARGIN_Y <= new_pos <= MAX_MOTOR_Y/2 - SAFETY_MARGIN_Y)

def recover_from_z_limit():
    """Recover when approaching Z limit"""
    global current_z_pos
    recovery_steps = 500  # Fixed recovery amount
    if current_z_pos <= MAX_MOTOR_Z + SAFETY_MARGIN_Z:
        move_z(recovery_steps)  # Too far left, move right
    elif current_z_pos >= 0 - SAFETY_MARGIN_Z:
        move_z(-recovery_steps)  # Too far right, move left

def recover_from_y_limit():
    """Recover when approaching Y limit"""
    global current_y_pos
    recovery_steps = 250  # Fixed recovery amount
    if current_y_pos <= 0 + SAFETY_MARGIN_Y:
        move_y(recovery_steps)  # Too far down, move up
    elif current_y_pos >= MAX_MOTOR_Y/2 - SAFETY_MARGIN_Y:
        move_y(-recovery_steps)  # Too far up, move down

def move_z(steps):
    """Safe Z axis movement with boundary checking"""
    global current_z_pos
    
    if not is_within_z_bounds(steps):
        print("Warning: Z movement would exceed limits!")
        recover_from_z_limit()
        return False
    
    current_z_pos += steps
    board_Z.stepper_move(motor_Z, steps)
    board_Z.stepper_run(motor_Z, motor_Z_complete_callback)
    return True

def move_y(steps):
    """Safe Y axis movement with boundary checking"""
    global current_y_pos
    
    if not is_within_y_bounds(steps):
        print("Warning: Y movement would exceed limits!")
        recover_from_y_limit()
        return False
    
    current_y_pos += steps
    board_Y.stepper_move(motor_Y, steps)
    board_Y.stepper_run(motor_Y, motor_Y_complete_callback)
    return True

def execute_search_pattern():
    """Execute safe search pattern within motor limits"""
    global current_z_pos, current_y_pos, search_direction_z, search_direction_y
    
    # Check if we need to reverse direction due to approaching limits
    if not is_within_z_bounds(search_direction_z * 5):
        search_direction_z *= -1
    if not is_within_y_bounds(search_direction_y * 2):
        search_direction_y *= -1
    
    # Configure movement parameters
    search_speed_z = 1000  # steps/sec for Z axis
    search_speed_y = 800   # steps/sec for Y axis
    
    board_Z.stepper_set_max_speed(motor_Z, search_speed_z)
    board_Y.stepper_set_max_speed(motor_Y, search_speed_y)
    
    board_Z.stepper_set_acceleration(motor_Z, 5000)
    board_Y.stepper_set_acceleration(motor_Y, 4000)
    
    # Use safe movement functions
    move_z(search_direction_z * 5)
    move_y(search_direction_y * 2)

# Initialize motor pins and settings
board_Y.set_pin_mode_digital_input(LIMIT_Y, Limit_Y_Callback)
board_Z.set_pin_mode_digital_input(LIMIT_Z, Limit_Z_Callback)
time.sleep(0.01)

motor_Y = board_Y.set_pin_mode_stepper(interface=1, pin1=PULSE_Y, pin2=DIR_Y)
motor_Z = board_Z.set_pin_mode_stepper(interface=1, pin1=PULSE_Z, pin2=DIR_Z)
time.sleep(0.01)

board_Y.set_pin_mode_digital_output(DISABLE_Y)
board_Z.set_pin_mode_digital_output(DISABLE_Z)
time.sleep(0.01)

board_Y.stepper_set_max_speed(motor_Y, 4000)
board_Z.stepper_set_max_speed(motor_Z, 4000)
time.sleep(0.01)

board_Y.stepper_set_acceleration(motor_Y, 1000)
board_Z.stepper_set_acceleration(motor_Z, 1000)
time.sleep(0.01)

# Homing procedure
if limit_Y_state == 0:
    board_Y.stepper_set_acceleration(motor_Y, 10000)
    board_Y.stepper_move(motor_Y, 150)
    board_Y.stepper_run_speed_to_position(motor_Y, false_start_Y_callback)
    homing_Y_initiated = 1
else:
    board_Y.stepper_move(motor_Y, 0)
    time.sleep(0.01)
    board_Y.stepper_set_speed(motor_Y, -100)
    time.sleep(0.01)
    board_Y.stepper_run_speed(motor_Y)
    homing_Y_initiated = 1

if limit_Z_state == 1:
    board_Z.stepper_set_acceleration(motor_Z, 10000)
    board_Z.stepper_move(motor_Z, -150)
    board_Z.stepper_run_speed_to_position(motor_Z, false_start_Z_callback)
    homing_Z_initiated = 1
else:
    board_Z.stepper_move(motor_Z, 0)
    time.sleep(0.01)
    board_Z.stepper_set_speed(motor_Z, 100)
    time.sleep(0.01)
    board_Z.stepper_run_speed(motor_Z)
    homing_Z_initiated = 1

# Wait for homing to complete
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
    cv2.namedWindow('frame')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        frame = cv2.flip(frame, -1)
        screen_center_x = frame.shape[1] // 2
        screen_center_y = frame.shape[0] // 2
        dead_zone_x = int(screen_center_x * DEAD_ZONE_PERCENT)
        dead_zone_y = int(screen_center_y * DEAD_ZONE_PERCENT)

        # Run inference
        results = model(frame)
        drone_detected = False
        closest_drone = None
        min_distance = float('inf')

        # Process detection results to find the closest drone to center
        for result in results[0].boxes.data:
            x1, y1, x2, y2, conf, cls = result.tolist()
            if conf > 0.5 and classes[int(cls)] in classes:
                drone_center_x, drone_center_y = get_drone_center(x1, y1, x2, y2)
                distance_to_center = ((drone_center_x - screen_center_x)**2 + 
                                    (drone_center_y - screen_center_y)**2)**0.5
                
                if distance_to_center < min_distance:
                    min_distance = distance_to_center
                    closest_drone = {
                        'x1': x1,
                        'y1': y1,
                        'x2': x2,
                        'y2': y2,
                        'center_x': drone_center_x,
                        'center_y': drone_center_y
                    }

        # Tracking or search logic
        if closest_drone is not None:
            drone_detected = True
            last_detection_time = time.time()
            search_mode = False
            
            # Draw rectangle around the closest drone
            cv2.rectangle(frame, 
                         (int(closest_drone['x1']), int(closest_drone['y1'])), 
                         (int(closest_drone['x2']), int(closest_drone['y2'])), 
                         (0, 0, 255), 2)
            
            delta_x = closest_drone['center_x'] - screen_center_x
            delta_y = closest_drone['center_y'] - screen_center_y
            
            if abs(delta_x) > dead_zone_x or abs(delta_y) > dead_zone_y:
                z_steps = calculate_movement(delta_x, frame.shape[1], MAX_Z_STEPS, MIN_Z_STEPS)
                y_steps = calculate_movement(delta_y, frame.shape[0], MAX_Y_STEPS, MIN_Y_STEPS)
                
                z_steps = -z_steps if delta_x < 0 else z_steps
                y_steps = y_steps if delta_y < 0 else -y_steps
                
                tracking_speed_z = 800
                tracking_speed_y = 600
                
                board_Z.stepper_set_max_speed(motor_Z, tracking_speed_z)
                board_Y.stepper_set_max_speed(motor_Y, tracking_speed_y)
                
                board_Z.stepper_set_acceleration(motor_Z, 4000)
                board_Y.stepper_set_acceleration(motor_Y, 3000)
                
                move_z(z_steps)
                move_y(y_steps)

        elif not drone_detected:
            if time.time() - last_detection_time > SEARCH_TIMEOUT:
                search_mode = True
                
            if search_mode:
                #execute_search_pattern() // Delete comment to enable search mode
                pass

        cv2.imshow('frame', frame)
        time.sleep(0.008)

except KeyboardInterrupt:
    pass
finally:
    # Cleanup
    board_Z.stepper_stop(motor_Z)
    board_Y.stepper_stop(motor_Y)
    board_Z.digital_write(DISABLE_Z, 1)
    board_Y.digital_write(DISABLE_Y, 1)
    board_Z.shutdown()
    board_Y.shutdown()
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)