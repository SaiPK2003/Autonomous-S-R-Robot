# Autonomous Search & Rescue (S&R) Robot: SDW-25 Mk. II

## Project Overview
This project involved the end-to-end design, development, and testing of an autonomous Search & Rescue (S&R) vehicle. The robot was engineered to navigate a dynamic obstacle course, identify a target (LEGO figure), and retrieve it to a safe zone without human intervention. The system achieved a top-two performance ranking during final evaluation based on a calculated Performance Index (PI).



## Core Technologies
* **Compute:** Raspberry Pi 5 for high-level logic and vision processing.
* **Vision:** Logitech webcam utilizing **OpenCV**.
* **Actuation:** Dual geared DC motors with an L298N H-bridge driver and an SG90 servo motor.
* **Control:** Discrete-time P-control algorithm for path-following.
* **Power:** Distributed 6V/5V system to isolate motor noise from logic circuits.

## Software Architecture & Concurrency
The software was designed as a modular **Finite State Machine (FSM)** to handle real-time perception and actuation. 
* **Multithreaded Execution:** Leveraged the Raspberry Pi's concurrent processing capabilities to handle low-latency computer vision while simultaneously managing motor control interrupts.
* **Vision Pipeline:**
    * Initially implemented **Canny Edge Detection** for line tracking.
    * Transitioned to a more robust **Contour Detection** approach to improve reliability under variable lighting conditions.
    * Utilized Gaussian blurring and HSV color masking to isolate mission-critical cues (Red/Blue/Green).

## Systems Integration & Control Theory
* **Feedback Control:** Designed and tuned a **PID controller** (finalized as P-control) to regulate motor speed and orientation.
* **Linearization:** Developed software-level sensor scaling to invert non-linear potentiometer readings into radians.
* **Stiction Compensation:** Implemented voltage offsets in the motor drive logic to overcome static friction, ensuring precise low-speed maneuvers.
* **Sensor Fusion:** Integrated an **MPU9250 Gyroscope** with a custom calibration function to eliminate zero-offset drift, facilitating an accurate 180° turnaround routine.



## Engineering Challenges & Iterations
* **Gripper vs. Plow:** Replaced a complex servo-actuated linkage gripper with a specialized **V-shaped plow** after testing revealed a 100% success rate in object transport versus 40% with the linkage design.
* **Weight Optimization:** Refactored the chassis from acrylic to a wooden frame, reducing the total mass to **764 grams** to improve acceleration and responsiveness.
* **Power Management:** Resolved motor driver overheating issues by recalculating power distribution and upgrading to higher-torque motors.

## Results
* **Autonomy:** Successfully traversed a complex path with sharp turns and uneven terrain autonomously.
* **Speed:** Maintained a forward velocity exceeding **15 cm/s** while maintaining path accuracy.
* **Performance:** Placed within the top two performance brackets among 13 competing teams.
