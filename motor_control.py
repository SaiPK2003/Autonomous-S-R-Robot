"""
Motor Control Module for Autonomous Search & Rescue Robot
Implements PID controller and hardware interfaces for L298N and servo
"""

import time
import numpy as np
from config import *

try:
    import RPi.GPIO as GPIO
    from gpiozero import Motor, Servo
    HARDWARE_AVAILABLE = True
except ImportError:
    print("Warning: RPi.GPIO or gpiozero not available. Running in simulation mode.")
    HARDWARE_AVAILABLE = False


class PIDController:
    """Discrete-time PID controller for path following"""
    
    def __init__(self, kp=KP, ki=KI, kd=KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # State variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Anti-windup limits
        self.integral_max = 1.0
        self.integral_min = -1.0
        
    def reset(self):
        """Reset PID state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, error):
        """
        Compute PID control signal.
        
        Args:
            error: Normalized error in range [-1.0, 1.0]
            
        Returns:
            control: Control signal in range [-1.0, 1.0]
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Avoid division by zero
        if dt <= 0:
            dt = 0.01
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, self.integral_min, self.integral_max)
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Total control signal
        control = p_term + i_term + d_term
        
        # Update state
        self.prev_error = error
        self.last_time = current_time
        
        return np.clip(control, -1.0, 1.0)


class MotorController:
    """Controls L298N motor driver and SG90 servo"""
    
    def __init__(self):
        self.pid = PIDController(KP, KI, KD)
        self.left_motor = None
        self.right_motor = None
        self.servo = None
        self.is_initialized = False
        
    def initialize(self):
        """Initialize motor hardware"""
        if not HARDWARE_AVAILABLE:
            print("Running in simulation mode - no hardware control")
            self.is_initialized = True
            return
        
        try:
            # Set up GPIO mode
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Initialize motors using gpiozero (handles PWM automatically)
            self.left_motor = Motor(
                forward=MOTOR_LEFT_PIN1,
                backward=MOTOR_LEFT_PIN2,
                enable=MOTOR_LEFT_PWM
            )
            
            self.right_motor = Motor(
                forward=MOTOR_RIGHT_PIN1,
                backward=MOTOR_RIGHT_PIN2,
                enable=MOTOR_RIGHT_PWM
            )
            
            # Initialize servo
            self.servo = Servo(SERVO_PIN)
            self.servo_up()  # Start with plow/scooper up
            
            self.is_initialized = True
            print("Motor hardware initialized successfully")
            
        except Exception as e:
            print(f"Error initializing motors: {e}")
            self.is_initialized = False
            
    def cleanup(self):
        """Clean up GPIO resources"""
        if HARDWARE_AVAILABLE and self.is_initialized:
            self.stop()
            if self.servo:
                self.servo.close()
            if self.left_motor:
                self.left_motor.close()
            if self.right_motor:
                self.right_motor.close()
            GPIO.cleanup()
            print("Motor cleanup complete")
    
    def _apply_stiction_compensation(self, speed):
        """
        Add offset to overcome static friction at low speeds.
        Returns compensated speed.
        """
        if speed > 0:
            return max(speed + STICTION_OFFSET, MIN_SPEED)
        elif speed < 0:
            return min(speed - STICTION_OFFSET, -MIN_SPEED)
        return 0
    
    def set_motor_speeds(self, left_speed, right_speed):
        """
        Set individual motor speeds with stiction compensation.
        
        Args:
            left_speed: Speed for left motor (-1.0 to 1.0)
            right_speed: Speed for right motor (-1.0 to 1.0)
        """
        # Apply compensation
        left_compensated = self._apply_stiction_compensation(left_speed)
        right_compensated = self._apply_stiction_compensation(right_speed)
        
        # Clamp values
        left_final = np.clip(left_compensated, -MAX_SPEED, MAX_SPEED)
        right_final = np.clip(right_compensated, -MAX_SPEED, MAX_SPEED)
        
        if DEBUG_MODE:
            print(f"Motors: L={left_final:.2f}, R={right_final:.2f}")
        
        if not HARDWARE_AVAILABLE or not self.is_initialized:
            return
        
        try:
            # Set left motor
            if left_final > 0:
                self.left_motor.forward(abs(left_final))
            elif left_final < 0:
                self.left_motor.backward(abs(left_final))
            else:
                self.left_motor.stop()
            
            # Set right motor
            if right_final > 0:
                self.right_motor.forward(abs(right_final))
            elif right_final < 0:
                self.right_motor.backward(abs(right_final))
            else:
                self.right_motor.stop()
                
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
    
    def drive_with_pid(self, error):
        """
        Drive robot using PID control based on path error.
        
        Args:
            error: Normalized error from path center (-1.0 to 1.0)
                  Positive error means path is to the right
        """
        # Compute PID control signal
        control = self.pid.compute(error)
        
        # Differential drive: adjust speeds to steer
        # Positive control means turn right (slow down right motor)
        left_speed = BASE_SPEED + control
        right_speed = BASE_SPEED - control
        
        # Set motor speeds
        self.set_motor_speeds(left_speed, right_speed)
    
    def drive_straight(self, speed=BASE_SPEED):
        """Drive straight at specified speed"""
        self.set_motor_speeds(speed, speed)
    
    def turn_in_place(self, speed=TURN_SPEED, direction='right'):
        """
        Turn in place.
        
        Args:
            speed: Turning speed (0.0 to 1.0)
            direction: 'right' or 'left'
        """
        if direction == 'right':
            self.set_motor_speeds(speed, -speed)
        else:
            self.set_motor_speeds(-speed, speed)
    
    def stop(self):
        """Stop both motors"""
        self.set_motor_speeds(0, 0)
        if HARDWARE_AVAILABLE and self.is_initialized:
            if self.left_motor:
                self.left_motor.stop()
            if self.right_motor:
                self.right_motor.stop()
    
    def servo_up(self):
        """Raise the plow/scooper"""
        if not HARDWARE_AVAILABLE or not self.servo:
            if DEBUG_MODE:
                print("Servo: UP position")
            return
        
        try:
            # Servo values range from -1 to 1 in gpiozero
            # Convert angle to servo value
            servo_value = (SERVO_UP_POSITION - 90) / 90
            self.servo.value = servo_value
            if DEBUG_MODE:
                print(f"Servo UP: {SERVO_UP_POSITION}°")
        except Exception as e:
            print(f"Error moving servo up: {e}")
    
    def servo_down(self):
        """Lower the plow/scooper"""
        if not HARDWARE_AVAILABLE or not self.servo:
            if DEBUG_MODE:
                print("Servo: DOWN position")
            return
        
        try:
            servo_value = (SERVO_DOWN_POSITION - 90) / 90
            self.servo.value = servo_value
            if DEBUG_MODE:
                print(f"Servo DOWN: {SERVO_DOWN_POSITION}°")
        except Exception as e:
            print(f"Error moving servo down: {e}")
    
    def servo_scoop(self):
        """Scoop position for pickup"""
        if not HARDWARE_AVAILABLE or not self.servo:
            if DEBUG_MODE:
                print("Servo: SCOOP position")
            return
        
        try:
            servo_value = (SERVO_SCOOP_POSITION - 90) / 90
            self.servo.value = servo_value
            if DEBUG_MODE:
                print(f"Servo SCOOP: {SERVO_SCOOP_POSITION}°")
        except Exception as e:
            print(f"Error moving servo to scoop: {e}")


# Legacy function for backward compatibility
def drive_robot(error):
    """
    Legacy function - use MotorController class instead.
    Calculates motor speeds based on error.
    """
    control = KP * error
    left_val = max(min(BASE_SPEED + control, 1), 0)
    right_val = max(min(BASE_SPEED - control, 1), 0)
    return left_val, right_val
