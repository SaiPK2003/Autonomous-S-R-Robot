# Constants finalized through iterative testing 
BASE_SPEED = 0.72 
K_P = 1.5

def drive_robot(error):
    """
    Adjusts motor speeds based on deviations from the path center
    """
    # Calculate control signal (Proportional value only)
    control = K_P * error
    
    # Calculate differential motor values 
    # Positive error (path is right) speeds up left motor to steer right 
    left_val = max(min(BASE_SPEED + control, 1), 0)
    right_val = max(min(BASE_SPEED - control, 1), 0)
    
    # Command motors 
    # left_motor.forward(left_val)
    # right_motor.forward(right_val)
    return left_val, right_val
