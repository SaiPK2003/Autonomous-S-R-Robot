# High-level state machine finalized for S&R mission
STATE_RESCUE = "going_to_rescue"
STATE_RETURN = "returning_to_start"
STATE_IDLE = "shutting_off"

def run_mission():
    current_state = STATE_RESCUE
    
    while current_state != STATE_IDLE:
        frame = get_camera_frame() # Capture from Logitech cam
        
        if current_state == STATE_RESCUE:
            # Look for blue target
            cx, cy = find_path_center(frame, BLUE_LOWER, BLUE_UPPER)
            if cx != -1:
                perform_pickup_routine() # V-plow approach
                perform_turnaround()     # Gyro-guided 180 turn
                current_state = STATE_RETURN
            else:
                # Follow red path 
                cx, cy = find_path_center(frame, RED_LOWER, RED_UPPER)
                error = (cx - FRAME_WIDTH/2) / (FRAME_WIDTH/2)
                drive_robot(error)

        elif current_state == STATE_RETURN:
            # Logic to follow path back to green drop-off or start
            pass
