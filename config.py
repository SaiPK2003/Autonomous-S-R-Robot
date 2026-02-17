"""
Hardware Configuration and Constants for SDW-25 Mk. II
Autonomous Search & Rescue Robot
"""

# ============== GPIO Pin Assignments ==============
# L298N Motor Driver
MOTOR_LEFT_PIN1 = 17    # GPIO pin for left motor direction 1
MOTOR_LEFT_PIN2 = 18    # GPIO pin for left motor direction 2
MOTOR_RIGHT_PIN1 = 22   # GPIO pin for right motor direction 1
MOTOR_RIGHT_PIN2 = 23   # GPIO pin for right motor direction 2
MOTOR_LEFT_PWM = 12     # PWM pin for left motor speed
MOTOR_RIGHT_PWM = 13    # PWM pin for right motor speed

# SG90 Servo Motor for plow/scooper
SERVO_PIN = 25          # GPIO pin for servo control

# MPU9250 IMU (I2C interface)
IMU_I2C_BUS = 1         # I2C bus number
IMU_ADDRESS = 0x68      # MPU9250 I2C address

# ============== Camera Settings ==============
CAMERA_INDEX = 0        # USB camera device index
FRAME_WIDTH = 640       # Camera frame width
FRAME_HEIGHT = 480      # Camera frame height
FPS = 30                # Frames per second

# ============== HSV Color Thresholds ==============
# Red path detection (HSV range)
RED_LOWER_1 = (0, 100, 100)     # Lower red hue range
RED_UPPER_1 = (10, 255, 255)
RED_LOWER_2 = (160, 100, 100)   # Upper red hue range (wraps around)
RED_UPPER_2 = (180, 255, 255)

# Blue target/bullseye detection
BLUE_LOWER = (90, 100, 100)
BLUE_UPPER = (130, 255, 255)

# White detection (for bullseye center)
WHITE_LOWER = (0, 0, 200)
WHITE_UPPER = (180, 30, 255)

# Green detection (for start zone - optional)
GREEN_LOWER = (40, 50, 50)
GREEN_UPPER = (80, 255, 255)

# ============== PID Control Parameters ==============
# Path following PID gains
KP = 1.5                # Proportional gain
KI = 0.05               # Integral gain
KD = 0.3                # Derivative gain

# Motor speed settings
BASE_SPEED = 0.72       # Base forward speed (0.0 to 1.0)
MIN_SPEED = 0.3         # Minimum motor speed to overcome stiction
MAX_SPEED = 1.0         # Maximum motor speed
TURN_SPEED = 0.5        # Speed during turning maneuvers

# Stiction compensation
STICTION_OFFSET = 0.15  # Voltage offset to overcome static friction

# ============== Vision Processing ==============
# Contour filtering
MIN_CONTOUR_AREA = 500          # Minimum contour area (pixels)
MAX_CONTOUR_AREA = 50000        # Maximum contour area (pixels)
FRAME_CENTER_X = FRAME_WIDTH // 2
FRAME_CENTER_Y = FRAME_HEIGHT // 2

# Target detection thresholds
BULLSEYE_MIN_AREA = 1000        # Minimum area for bullseye detection
BULLSEYE_PROXIMITY = 150        # Stop distance from bullseye (pixels from bottom)

# ============== IMU/Gyroscope Settings ==============
GYRO_CALIBRATION_SAMPLES = 1000 # Number of samples for calibration
GYRO_SAMPLE_DELAY = 0.01        # Delay between samples (seconds)
TURN_AROUND_ANGLE = 180         # Target angle for turnaround (degrees)
ANGLE_TOLERANCE = 5             # Acceptable error in degrees

# ============== Servo Positions ==============
SERVO_UP_POSITION = 30          # Servo angle when plow is up (degrees)
SERVO_DOWN_POSITION = 90        # Servo angle when plow is down (degrees)
SERVO_SCOOP_POSITION = 120      # Servo angle when scooping (degrees)

# ============== State Machine Timings ==============
PICKUP_DELAY = 2.0              # Time to wait during pickup (seconds)
SCOOP_DELAY = 1.5               # Time to raise scoop (seconds)
TURN_TIMEOUT = 10.0             # Maximum time for 180° turn (seconds)
MISSION_TIMEOUT = 180.0         # Maximum mission time (seconds)

# ============== Debug Settings ==============
DEBUG_MODE = True               # Enable debug output
SHOW_CAMERA_FEED = True         # Display processed camera feed
SAVE_DEBUG_LOGS = True          # Save debug logs to file
