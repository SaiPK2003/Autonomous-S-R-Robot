# SDW-25 Mk. II - Complete Implementation Guide

## Overview
This is the complete, production-ready code for the autonomous search & rescue robot. The robot uses computer vision to follow a red path, detect a blue/white bullseye target, scoop a LEGO figure using a V-shaped plow mechanism, turn around 180° using gyroscope feedback, and return to start.

## File Structure

```
Autonomous-S-R-Robot/
├── config.py                  # Hardware configuration and constants
├── main_robot_node.py         # Main FSM and mission control
├── vision_navigation.py       # Computer vision and path detection
├── motor_control.py          # PID controller and motor drivers
├── imu_calibration.py        # Gyroscope calibration for turnaround
└── README.md                 # Project documentation
```

## Hardware Requirements

### Components
- **Raspberry Pi 5** - Main compute unit
- **Logitech USB Webcam** - Vision system
- **L298N H-Bridge** - Dual DC motor driver
- **2x Geared DC Motors** - Differential drive
- **SG90 Servo Motor** - Scooper/plow actuation
- **MPU9250 IMU** - 9-axis gyroscope/accelerometer
- **6V/5V Power System** - Battery pack

### GPIO Pin Configuration (in config.py)
- Motors: GPIO 17, 18, 22, 23, 12, 13
- Servo: GPIO 25
- IMU: I2C bus 1, address 0x68

## Software Dependencies

### Install Required Libraries
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python packages
pip install opencv-python numpy RPi.GPIO gpiozero

# Install MPU9250 library
pip install mpu9250-jmdev

# Install system dependencies for OpenCV
sudo apt install python3-opencv libatlas-base-dev
```

## Configuration

### HSV Color Tuning
The HSV color thresholds in `config.py` may need adjustment based on lighting conditions:

```python
# Red path detection (adjust if needed)
RED_LOWER_1 = (0, 100, 100)
RED_UPPER_1 = (10, 255, 255)
RED_LOWER_2 = (160, 100, 100)
RED_UPPER_2 = (180, 255, 255)

# Blue bullseye detection
BLUE_LOWER = (90, 100, 100)
BLUE_UPPER = (130, 255, 255)
```

**Tip:** Use a separate HSV calibration script to find optimal values for your specific lighting.

### PID Tuning
The PID gains are pre-tuned but can be adjusted in `config.py`:

```python
KP = 1.5    # Proportional gain (responsiveness)
KI = 0.05   # Integral gain (steady-state error)
KD = 0.3    # Derivative gain (damping)
```

**Tuning Guide:**
- Increase `KP` for faster response (may cause oscillation)
- Increase `KI` to eliminate steady-state drift
- Increase `KD` to reduce overshoot and oscillation

### Motor Speed Calibration
Adjust motor speeds based on your robot's weight and terrain:

```python
BASE_SPEED = 0.72       # Forward speed (0.0 to 1.0)
MIN_SPEED = 0.3         # Minimum to overcome friction
STICTION_OFFSET = 0.15  # Compensation for static friction
```

### Servo Positions
Adjust servo angles for your scooper mechanism:

```python
SERVO_UP_POSITION = 30      # Plow raised
SERVO_DOWN_POSITION = 90    # Plow lowered
SERVO_SCOOP_POSITION = 120  # Scooping position
```

## Running the Robot

### 1. Setup and Calibration
```bash
# Navigate to project directory
cd ~/Autonomous-S-R-Robot

# Run the main program
python3 main_robot_node.py
```

### 2. Initialization Sequence
The robot will:
1. Initialize camera and motors
2. Calibrate gyroscope (**keep robot stationary!**)
3. Wait 3 seconds before starting mission

### 3. Mission States
The FSM progresses through these states:
1. **RESCUE** - Follow red path, detect bullseye
2. **APPROACH** - Center on bullseye and approach
3. **PICKUP** - Execute scooper routine
4. **TURNAROUND** - 180° turn using gyroscope
5. **RETURN** - Follow red path back to start

### 4. Stopping the Robot
- Press `Ctrl+C` for graceful shutdown
- Robot will stop motors and release resources

## Debugging

### Enable Debug Mode
In `config.py`, set:
```python
DEBUG_MODE = True
SHOW_CAMERA_FEED = True
```

This will:
- Display processed camera feed with overlays
- Print detailed sensor readings
- Show PID control values

### Common Issues

**Camera not detected:**
- Check USB connection
- Run `ls /dev/video*` to verify device
- Update `CAMERA_INDEX` in config.py

**Motors not responding:**
- Verify GPIO pin connections
- Check L298N power supply (separate from Pi)
- Test motors individually with gpiozero

**Gyroscope drift:**
- Ensure robot is stationary during calibration
- Re-run calibration if drift is excessive
- Check I2C connection: `i2cdetect -y 1`

**Path following issues:**
- Adjust HSV thresholds for lighting
- Tune PID gains
- Check camera focus and angle

## Performance Optimization

### Speed Tuning
For optimal Performance Index:
- Balance speed vs. accuracy
- Current `BASE_SPEED = 0.72` achieved 15+ cm/s
- Reduce speed in tight turns for stability

### Vision Processing
- Camera runs at 30 FPS for smooth control
- Contour filtering removes noise
- Morphological operations improve mask quality

### Control Loop
- PID updates every frame (~30 Hz)
- Anti-windup prevents integral saturation
- Stiction compensation ensures low-speed precision

## Testing Checklist

- [ ] Camera captures clear images
- [ ] Red path is correctly detected
- [ ] Bullseye detection works at range
- [ ] Motors respond to PID commands
- [ ] Gyroscope calibration completes
- [ ] 180° turn is accurate (±5°)
- [ ] Scooper mechanism operates smoothly
- [ ] Full mission completes end-to-end

## Safety Notes

⚠️ **Important Safety Guidelines:**
- Always have emergency stop ready (Ctrl+C)
- Test on soft surface initially
- Ensure adequate battery capacity
- Monitor motor temperatures
- Keep fingers clear of moving parts

## Advanced Features

### Adding Green Zone Detection
To detect the green start zone for automatic completion:

```python
# In vision_navigation.py, add method to VisionNavigator class
def detect_green_zone(self, frame):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    green_mask = cv.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    # Add contour detection logic similar to bullseye
```

### Logging Mission Data
Enable logging in `config.py`:
```python
SAVE_DEBUG_LOGS = True
```

This creates timestamped logs for post-mission analysis.

## Credits
**Project:** SDW-25 Mk. II Autonomous Search & Rescue Robot  
**Technologies:** OpenCV, PID Control, Sensor Fusion, FSM  
**Achievement:** Top-2 Performance Index

## License
Educational/Research Use
