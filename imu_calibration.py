import time
import numpy as np

def calibrate_gyro(mpu, num_samples=1000):
    """
    [cite_start]Calculates the mean offset for the gyroscope while the robot is stationary[cite: 320, 339, 341].
    """
    gyro_data_list = []
    start_time = time.time()
    
    print("Calibrating... Ensure robot is stationary.")
    for _ in range(num_samples):
        # Read from sensor interface
        data = mpu.readGyroscopeMaster() 
        gyro_data_list.append(data)
        [cite_start]time.sleep(0.01) # Sensor accuracy delay
        
    # Calculate mean offset 
    gx_offset = np.mean(gyro_data_list, axis=0)
    sample_rate = num_samples / (time.time() - start_time)
    
    return gx_offset, sample_rate

def get_calibrated_angle(current_angle, mpu, offset, sample_rate):
    """
    Updates the orientation angle using calibrated sensor data.
    """
    raw_data = mpu.readGyroscopeMaster()
    # Subtract offset for precise turning 
    current_angle += (raw_data[0] - offset) / sample_rate
    return current_angle
