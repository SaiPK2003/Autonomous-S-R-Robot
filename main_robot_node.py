"""
Main Robot Node - Autonomous Search & Rescue Robot
Implements Finite State Machine for complete rescue mission
Author: SDW-25 Mk. II Project
"""

import time
import sys
import signal
from config import *
from vision_navigation import VisionNavigator
from motor_control import MotorController
from imu_calibration import IMUController


# ============== State Machine States ==============
STATE_INIT = "initializing"
STATE_RESCUE = "going_to_rescue"
STATE_APPROACH = "approaching_target"
STATE_PICKUP = "picking_up_target"
STATE_TURNAROUND = "turning_around"
STATE_RETURN = "returning_to_start"
STATE_COMPLETE = "mission_complete"
STATE_IDLE = "shutting_off"


class SearchRescueRobot:
    """Main robot controller with FSM logic"""
    
    def __init__(self):
        # Initialize subsystems
        self.vision = VisionNavigator()
        self.motor = MotorController()
        self.imu = IMUController()
        
        # Mission state
        self.current_state = STATE_INIT
        self.mission_start_time = 0
        self.target_detected = False
        
        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C for graceful shutdown"""
        print("\n\nShutdown signal received...")
        self.shutdown()
        sys.exit(0)
    
    def initialize(self):
        """Initialize all subsystems"""
        print("=" * 60)
        print("SDW-25 Mk. II - Autonomous Search & Rescue Robot")
        print("=" * 60)
        print("\nInitializing subsystems...")
        
        # Initialize camera
        print("\n[1/3] Camera initialization...")
        try:
            self.vision.initialize_camera()
            print("  ✓ Camera ready")
        except Exception as e:
            print(f"  ✗ Camera error: {e}")
            return False
        
        # Initialize motors
        print("\n[2/3] Motor initialization...")
        try:
            self.motor.initialize()
            print("  ✓ Motors ready")
        except Exception as e:
            print(f"  ✗ Motor error: {e}")
            return False
        
        # Initialize and calibrate IMU
        print("\n[3/3] IMU initialization...")
        if self.imu.initialize():
            print("  Starting gyroscope calibration...")
            print("  >> KEEP ROBOT STATIONARY <<")
            time.sleep(2)  # Give user time to read
            
            offset, rate = self.imu.calibrate()
            if offset is not None:
                print("  ✓ IMU ready")
            else:
                print("  ⚠ IMU calibration failed - will use time-based turns")
        else:
            print("  ⚠ IMU not available - will use time-based turns")
        
        print("\n" + "=" * 60)
        print("ALL SYSTEMS READY - Starting mission in 3 seconds...")
        print("=" * 60)
        time.sleep(3)
        
        return True
    
    def shutdown(self):
        """Clean shutdown of all subsystems"""
        print("\n\nShutting down robot...")
        self.current_state = STATE_IDLE
        
        # Stop motors
        if self.motor:
            self.motor.stop()
            self.motor.cleanup()
        
        # Release camera
        if self.vision:
            self.vision.release_camera()
        
        print("Shutdown complete.")
    
    def state_rescue(self, frame):
        """
        STATE: Going to rescue - Follow red path and detect bullseye
        """
        # Check for bullseye target
        detected, bx, by, dist = self.vision.detect_bullseye(frame)
        
        if detected:
            print(f"BULLSEYE DETECTED! Distance from bottom: {dist}px")
            
            # Check if close enough to approach
            if dist < BULLSEYE_PROXIMITY:
                print("Target reached! Transitioning to PICKUP")
                self.motor.stop()
                return STATE_PICKUP
            else:
                # Approaching target - switch to approach mode
                return STATE_APPROACH
        
        # No target detected - follow red path
        cx, cy, mask = self.vision.find_red_path_center(frame)
        
        if cx != -1:
            # Calculate error and drive with PID
            error = self.vision.calculate_path_error(cx)
            self.motor.drive_with_pid(error)
            
            if DEBUG_MODE:
                print(f"Following path | cx={cx}, error={error:.3f}")
        else:
            # Path lost - continue straight slowly
            print("Warning: Red path lost, continuing straight")
            self.motor.drive_straight(BASE_SPEED * 0.5)
        
        return STATE_RESCUE
    
    def state_approach(self, frame):
        """
        STATE: Approaching target - Center on bullseye
        """
        detected, bx, by, dist = self.vision.detect_bullseye(frame)
        
        if not detected:
            # Lost target, return to path following
            print("Target lost, returning to path following")
            return STATE_RESCUE
        
        # Check if close enough
        if dist < BULLSEYE_PROXIMITY:
            print("Close enough to target!")
            self.motor.stop()
            return STATE_PICKUP
        
        # Center on bullseye and approach
        error = self.vision.calculate_path_error(bx)
        
        # Reduce speed as we approach
        approach_speed = BASE_SPEED * 0.6
        control = self.motor.pid.compute(error)
        
        left_speed = approach_speed + control
        right_speed = approach_speed - control
        
        self.motor.set_motor_speeds(left_speed, right_speed)
        
        if DEBUG_MODE:
            print(f"Approaching target | dist={dist}px, error={error:.3f}")
        
        return STATE_APPROACH
    
    def state_pickup(self, frame):
        """
        STATE: Picking up target - Execute scooper routine
        """
        print("\n" + "=" * 50)
        print("EXECUTING PICKUP SEQUENCE")
        print("=" * 50)
        
        # Stop motors
        self.motor.stop()
        time.sleep(0.5)
        
        # Lower scooper
        print("Step 1: Lowering scooper...")
        self.motor.servo_down()
        time.sleep(PICKUP_DELAY)
        
        # Drive forward slowly to push LEGO onto scooper
        print("Step 2: Scooping target...")
        self.motor.drive_straight(BASE_SPEED * 0.4)
        time.sleep(1.5)
        
        # Raise scooper to secure target
        print("Step 3: Securing target...")
        self.motor.servo_scoop()
        time.sleep(SCOOP_DELAY)
        
        # Stop
        self.motor.stop()
        time.sleep(0.5)
        
        print("=" * 50)
        print("PICKUP COMPLETE - Starting turnaround")
        print("=" * 50)
        
        return STATE_TURNAROUND
    
    def state_turnaround(self, frame):
        """
        STATE: 180° turnaround using gyroscope
        """
        print("\n" + "=" * 50)
        print("EXECUTING 180° TURNAROUND")
        print("=" * 50)
        
        # Reset IMU angle
        self.imu.reset_angle()
        start_time = time.time()
        
        # Determine if IMU is available
        use_imu = self.imu.is_calibrated
        
        if use_imu:
            print("Using IMU for precise turn...")
            
            # Turn using gyroscope feedback
            while True:
                # Update angle from gyroscope
                current_angle = self.imu.update_angle()
                angle_remaining = TURN_AROUND_ANGLE - abs(current_angle)
                
                if DEBUG_MODE:
                    print(f"  Angle: {abs(current_angle):.1f}° / {TURN_AROUND_ANGLE}°")
                
                # Check if turn is complete
                if abs(current_angle) >= TURN_AROUND_ANGLE - ANGLE_TOLERANCE:
                    print(f"Turn complete! Final angle: {abs(current_angle):.1f}°")
                    break
                
                # Check for timeout
                if time.time() - start_time > TURN_TIMEOUT:
                    print("Turn timeout - continuing anyway")
                    break
                
                # Variable speed turning (slow down near target)
                if angle_remaining > 30:
                    turn_speed = TURN_SPEED
                else:
                    turn_speed = TURN_SPEED * 0.6
                
                # Turn in place
                self.motor.turn_in_place(turn_speed, direction='right')
                time.sleep(0.05)
        
        else:
            # Fallback: Time-based turning
            print("Using time-based turn (IMU not available)...")
            
            # Empirically determined turn time (adjust based on robot)
            turn_time = 2.5  # seconds for ~180° turn
            
            self.motor.turn_in_place(TURN_SPEED, direction='right')
            time.sleep(turn_time)
        
        # Stop turning
        self.motor.stop()
        time.sleep(0.5)
        
        print("=" * 50)
        print("TURNAROUND COMPLETE - Returning to start")
        print("=" * 50)
        
        return STATE_RETURN
    
    def state_return(self, frame):
        """
        STATE: Return to start - Follow red path back
        """
        # Follow red path back to start
        cx, cy, mask = self.vision.find_red_path_center(frame)
        
        if cx != -1:
            # Follow the path with PID control
            error = self.vision.calculate_path_error(cx)
            self.motor.drive_with_pid(error)
            
            if DEBUG_MODE:
                print(f"Returning | cx={cx}, error={error:.3f}")
        else:
            # Path lost - continue straight
            print("Warning: Path lost during return")
            self.motor.drive_straight(BASE_SPEED * 0.5)
        
        # Optional: Detect green start zone or time-based completion
        # For now, the operator can stop the robot manually
        # You could add green zone detection here similar to bullseye
        
        return STATE_RETURN
    
    def run_mission(self):
        """Main mission loop with FSM"""
        
        # Initialize systems
        if not self.initialize():
            print("Initialization failed. Exiting.")
            return
        
        self.current_state = STATE_RESCUE
        self.mission_start_time = time.time()
        
        print("\n" + "█" * 60)
        print("MISSION START")
        print("█" * 60 + "\n")
        
        try:
            # Main state machine loop
            while self.current_state not in [STATE_IDLE, STATE_COMPLETE]:
                
                # Check mission timeout
                elapsed = time.time() - self.mission_start_time
                if elapsed > MISSION_TIMEOUT:
                    print("\nMission timeout reached!")
                    break
                
                # Capture frame
                frame = self.vision.get_frame()
                if frame is None:
                    print("Warning: No frame captured")
                    time.sleep(0.1)
                    continue
                
                # Execute current state
                if self.current_state == STATE_RESCUE:
                    self.current_state = self.state_rescue(frame)
                    
                elif self.current_state == STATE_APPROACH:
                    self.current_state = self.state_approach(frame)
                    
                elif self.current_state == STATE_PICKUP:
                    self.current_state = self.state_pickup(frame)
                    
                elif self.current_state == STATE_TURNAROUND:
                    self.current_state = self.state_turnaround(frame)
                    
                elif self.current_state == STATE_RETURN:
                    self.current_state = self.state_return(frame)
                
                # Show debug feed
                if SHOW_CAMERA_FEED:
                    self.vision.show_debug_feed(frame)
                
                # Small delay to prevent CPU overload
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            print("\n\nMission interrupted by user")
        
        except Exception as e:
            print(f"\n\nERROR during mission: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Mission complete
            mission_time = time.time() - self.mission_start_time
            
            print("\n" + "█" * 60)
            print(f"MISSION COMPLETE - Time: {mission_time:.1f}s")
            print("█" * 60 + "\n")
            
            # Clean shutdown
            self.shutdown()


# ============== Legacy Function (for backward compatibility) ==============
def run_mission():
    """
    Legacy function from skeletal code.
    Use SearchRescueRobot class for full functionality.
    """
    print("Legacy run_mission() called - creating new robot instance")
    robot = SearchRescueRobot()
    robot.run_mission()


# ============== Main Entry Point ==============
if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("  SDW-25 Mk. II - Autonomous Search & Rescue Robot")
    print("  Final Implementation with PID Control and Computer Vision")
    print("=" * 60 + "\n")
    
    # Create and run robot
    robot = SearchRescueRobot()
    robot.run_mission()
