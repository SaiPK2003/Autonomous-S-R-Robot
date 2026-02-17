"""
Quick Test Script for Individual Robot Components
Run this to test subsystems independently before full mission
"""

import cv2 as cv
import time
from config import *
from vision_navigation import VisionNavigator
from motor_control import MotorController
from imu_calibration import IMUController


def test_camera():
    """Test camera initialization and red path detection"""
    print("\n" + "="*50)
    print("CAMERA TEST")
    print("="*50)
    
    vision = VisionNavigator()
    vision.initialize_camera()
    
    print("Press 'q' to quit, 's' to save frame")
    
    try:
        while True:
            frame = vision.get_frame()
            if frame is None:
                continue
            
            # Test red path detection
            cx, cy, mask = vision.find_red_path_center(frame)
            
            # Test bullseye detection
            detected, bx, by, dist = vision.detect_bullseye(frame)
            
            # Display info
            if cx != -1:
                cv.putText(frame, f"Path Center: ({cx}, {cy})", (10, 30),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if detected:
                cv.putText(frame, f"Bullseye: ({bx}, {by}) Dist:{dist}", (10, 60),
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
            
            cv.imshow("Camera Test", frame)
            cv.imshow("Red Mask", mask if mask is not None else frame)
            
            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv.imwrite(f"test_frame_{int(time.time())}.jpg", frame)
                print("Frame saved")
    
    finally:
        vision.release_camera()
        print("Camera test complete")


def test_motors():
    """Test motor control and servo"""
    print("\n" + "="*50)
    print("MOTOR TEST")
    print("="*50)
    
    motor = MotorController()
    motor.initialize()
    
    try:
        # Test forward
        print("Test 1: Forward (2s)")
        motor.drive_straight(0.5)
        time.sleep(2)
        motor.stop()
        time.sleep(1)
        
        # Test turn
        print("Test 2: Turn right (2s)")
        motor.turn_in_place(0.5, 'right')
        time.sleep(2)
        motor.stop()
        time.sleep(1)
        
        # Test servo
        print("Test 3: Servo positions")
        print("  - UP")
        motor.servo_up()
        time.sleep(1)
        print("  - DOWN")
        motor.servo_down()
        time.sleep(1)
        print("  - SCOOP")
        motor.servo_scoop()
        time.sleep(1)
        print("  - UP")
        motor.servo_up()
        
        print("\nMotor test complete!")
    
    finally:
        motor.cleanup()


def test_imu():
    """Test IMU calibration and angle tracking"""
    print("\n" + "="*50)
    print("IMU TEST")
    print("="*50)
    
    imu = IMUController()
    
    if not imu.initialize():
        print("IMU not available")
        return
    
    print("Starting calibration - keep robot stationary!")
    time.sleep(2)
    
    offset, rate = imu.calibrate()
    
    if offset is None:
        print("Calibration failed")
        return
    
    print("\nTest: Manually rotate robot and observe angle")
    print("Press Ctrl+C to stop\n")
    
    imu.reset_angle()
    
    try:
        while True:
            angle = imu.update_angle()
            print(f"Current angle: {angle:.2f}°", end='\r')
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nIMU test complete")


def test_pid():
    """Test PID controller response"""
    print("\n" + "="*50)
    print("PID CONTROLLER TEST")
    print("="*50)
    
    motor = MotorController()
    motor.initialize()
    
    print("Simulating path following with varying errors")
    print("Motors will adjust to correct simulated deviations\n")
    
    errors = [0.5, 0.3, 0.0, -0.3, -0.5, -0.3, 0.0]
    
    try:
        for i, error in enumerate(errors):
            print(f"Step {i+1}: Error = {error:+.1f}")
            motor.drive_with_pid(error)
            time.sleep(1)
        
        motor.stop()
        print("\nPID test complete!")
    
    finally:
        motor.cleanup()


def interactive_menu():
    """Interactive test menu"""
    print("\n" + "="*60)
    print("  ROBOT COMPONENT TEST SUITE")
    print("="*60)
    print("\nSelect a test:")
    print("  1. Camera & Vision")
    print("  2. Motors & Servo")
    print("  3. IMU & Gyroscope")
    print("  4. PID Controller")
    print("  5. Run All Tests")
    print("  q. Quit")
    
    choice = input("\nEnter choice: ").strip().lower()
    
    if choice == '1':
        test_camera()
    elif choice == '2':
        test_motors()
    elif choice == '3':
        test_imu()
    elif choice == '4':
        test_pid()
    elif choice == '5':
        print("\nRunning all tests...")
        test_camera()
        test_motors()
        test_imu()
        test_pid()
    elif choice == 'q':
        print("Exiting...")
        return
    else:
        print("Invalid choice")


if __name__ == "__main__":
    try:
        interactive_menu()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nError during test: {e}")
        import traceback
        traceback.print_exc()
