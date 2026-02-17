"""
IMU Calibration Module for MPU9250 Gyroscope
Handles sensor calibration and 180° turnaround routine
"""

import time
import numpy as np
from config import *

try:
    from mpu9250_jmdev.registers import *
    from mpu9250_jmdev.mpu_9250 import MPU9250
    IMU_AVAILABLE = True
except ImportError:
    print("Warning: mpu9250 library not available. Running without IMU.")
    IMU_AVAILABLE = False


class IMUController:
    """Manages MPU9250 IMU for orientation tracking"""
    
    def __init__(self):
        self.mpu = None
        self.gyro_offset = None
        self.sample_rate = 0
        self.current_angle = 0.0
        self.is_calibrated = False
        
    def initialize(self):
        """Initialize the MPU9250 sensor"""
        if not IMU_AVAILABLE:
            print("IMU not available - turnaround will use time-based estimation")
            return False
        
        try:
            # Initialize MPU9250 on I2C bus
            self.mpu = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=IMU_ADDRESS,
                address_mpu_slave=None,
                bus=IMU_I2C_BUS,
                gfs=GFS_250,  # Gyroscope full scale range
                afs=AFS_2G,   # Accelerometer full scale range
                mfs=AK8963_BIT_16,  # Magnetometer resolution
                mode=AK8963_MODE_C100HZ  # Magnetometer mode
            )
            
            # Configure the sensor
            self.mpu.configure()
            
            print("MPU9250 initialized successfully")
            return True
            
        except Exception as e:
            print(f"Error initializing IMU: {e}")
            return False
    
    def calibrate(self, num_samples=GYRO_CALIBRATION_SAMPLES):
        """
        Calibrate gyroscope to eliminate zero-offset drift.
        Robot must be stationary during calibration.
        
        Args:
            num_samples: Number of samples to average for calibration
            
        Returns:
            (gyro_offset, sample_rate): Calibration parameters
        """
        if not IMU_AVAILABLE or self.mpu is None:
            print("IMU not available for calibration")
            return None, 0
        
        print("=" * 50)
        print("GYROSCOPE CALIBRATION")
        print("=" * 50)
        print(f"Ensure robot is stationary!")
        print(f"Collecting {num_samples} samples...")
        
        gyro_readings = []
        start_time = time.time()
        
        try:
            for i in range(num_samples):
                # Read gyroscope data (returns [gx, gy, gz])
                gyro = self.mpu.readGyroscopeMaster()
                gyro_readings.append(gyro[2])  # Use Z-axis for 2D rotation
                
                # Progress indicator
                if i % 100 == 0:
                    print(f"  Progress: {i}/{num_samples}")
                
                time.sleep(GYRO_SAMPLE_DELAY)
            
            # Calculate calibration parameters
            elapsed_time = time.time() - start_time
            self.gyro_offset = np.mean(gyro_readings)
            self.sample_rate = num_samples / elapsed_time
            self.is_calibrated = True
            
            print("=" * 50)
            print(f"Calibration complete!")
            print(f"  Gyro offset: {self.gyro_offset:.4f} deg/s")
            print(f"  Sample rate: {self.sample_rate:.2f} Hz")
            print("=" * 50)
            
            return self.gyro_offset, self.sample_rate
            
        except Exception as e:
            print(f"Error during calibration: {e}")
            self.is_calibrated = False
            return None, 0
    
    def reset_angle(self):
        """Reset the current angle to zero"""
        self.current_angle = 0.0
    
    def update_angle(self):
        """
        Update current orientation angle using calibrated gyro data.
        
        Returns:
            current_angle: Updated angle in degrees
        """
        if not IMU_AVAILABLE or self.mpu is None or not self.is_calibrated:
            return self.current_angle
        
        try:
            # Read gyroscope Z-axis (rotation in 2D plane)
            gyro = self.mpu.readGyroscopeMaster()
            gyro_z = gyro[2]
            
            # Subtract calibration offset
            gyro_calibrated = gyro_z - self.gyro_offset
            
            # Integrate to get angle (gyro returns deg/s)
            self.current_angle += gyro_calibrated / self.sample_rate
            
            return self.current_angle
            
        except Exception as e:
            print(f"Error reading gyro: {e}")
            return self.current_angle
    
    def get_angle(self):
        """Get current angle without updating"""
        return self.current_angle


# Standalone calibration function for backward compatibility
def calibrate_gyro(mpu, num_samples=1000):
    """
    Legacy function - use IMUController class instead.
    Calculates the mean offset for the gyroscope while robot is stationary.
    """
    gyro_data_list = []
    start_time = time.time()
    
    print("Calibrating... Ensure robot is stationary.")
    for _ in range(num_samples):
        # Read from sensor interface
        data = mpu.readGyroscopeMaster()
        gyro_data_list.append(data)
        time.sleep(0.01)  # Sensor accuracy delay
        
    # Calculate mean offset
    gx_offset = np.mean(gyro_data_list, axis=0)
    sample_rate = num_samples / (time.time() - start_time)
    
    return gx_offset, sample_rate


def get_calibrated_angle(current_angle, mpu, offset, sample_rate):
    """
    Legacy function - use IMUController class instead.
    Updates the orientation angle using calibrated sensor data.
    """
    raw_data = mpu.readGyroscopeMaster()
    # Subtract offset for precise turning
    current_angle += (raw_data[0] - offset) / sample_rate
    return current_angle
