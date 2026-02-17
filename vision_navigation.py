"""
Computer Vision Module for Autonomous Search & Rescue Robot
Implements HSV-based color detection and contour tracking
"""

import cv2 as cv
import numpy as np
from config import *


class VisionNavigator:
    """Handles all computer vision and path detection logic"""
    
    def __init__(self):
        self.camera = None
        self.last_valid_cx = FRAME_CENTER_X
        self.bullseye_detected = False
        
    def initialize_camera(self):
        """Initialize the Logitech webcam"""
        self.camera = cv.VideoCapture(CAMERA_INDEX)
        self.camera.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.camera.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.camera.set(cv.CAP_PROP_FPS, FPS)
        
        if not self.camera.isOpened():
            raise RuntimeError("Failed to open camera")
        
        # Allow camera to warm up
        for _ in range(10):
            self.camera.read()
            
        print("Camera initialized successfully")
        
    def get_frame(self):
        """Capture a frame from the camera"""
        if self.camera is None:
            raise RuntimeError("Camera not initialized")
        
        ret, frame = self.camera.read()
        if not ret:
            print("Warning: Failed to capture frame")
            return None
        return frame
    
    def release_camera(self):
        """Release camera resources"""
        if self.camera:
            self.camera.release()
            cv.destroyAllWindows()
    
    def find_red_path_center(self, frame):
        """
        Detect red path using HSV color space and return center coordinates.
        Red color wraps around in HSV, so we use two ranges.
        """
        if frame is None:
            return -1, -1, None
        
        # Convert to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Apply Gaussian blur to reduce noise
        blur = cv.GaussianBlur(hsv, (9, 9), 0)
        
        # Create masks for red color (two ranges due to hue wraparound)
        mask1 = cv.inRange(blur, RED_LOWER_1, RED_UPPER_1)
        mask2 = cv.inRange(blur, RED_LOWER_2, RED_UPPER_2)
        red_mask = cv.bitwise_or(mask1, mask2)
        
        # Morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv.morphologyEx(red_mask, cv.MORPH_CLOSE, kernel)
        red_mask = cv.morphologyEx(red_mask, cv.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # Filter and find the largest valid contour
        valid_contours = [c for c in contours 
                         if MIN_CONTOUR_AREA < cv.contourArea(c) < MAX_CONTOUR_AREA]
        
        if not valid_contours:
            # Return last known good position to maintain stability
            return self.last_valid_cx, -1, red_mask
        
        # Get the largest contour
        largest_contour = max(valid_contours, key=cv.contourArea)
        
        # Calculate centroid
        M = cv.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            self.last_valid_cx = cx  # Update last valid position
            
            # Draw contour and center for debugging
            if DEBUG_MODE and SHOW_CAMERA_FEED:
                cv.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                cv.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv.line(frame, (FRAME_CENTER_X, 0), (FRAME_CENTER_X, FRAME_HEIGHT), 
                       (255, 0, 0), 1)
            
            return cx, cy, red_mask
        
        return self.last_valid_cx, -1, red_mask
    
    def detect_bullseye(self, frame):
        """
        Detect blue and white bullseye target.
        Returns (detected, center_x, center_y, distance_from_bottom)
        """
        if frame is None:
            return False, -1, -1, -1
        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        blur = cv.GaussianBlur(hsv, (9, 9), 0)
        
        # Detect blue outer ring
        blue_mask = cv.inRange(blur, BLUE_LOWER, BLUE_UPPER)
        
        # Detect white/red center
        white_mask = cv.inRange(blur, WHITE_LOWER, WHITE_UPPER)
        
        # Combine masks
        target_mask = cv.bitwise_or(blue_mask, white_mask)
        
        # Clean up
        kernel = np.ones((7, 7), np.uint8)
        target_mask = cv.morphologyEx(target_mask, cv.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv.findContours(target_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # Look for circular bullseye
        for contour in contours:
            area = cv.contourArea(contour)
            if area > BULLSEYE_MIN_AREA:
                # Calculate circularity
                perimeter = cv.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # Check if it's roughly circular (0.7 to 1.0)
                    if 0.5 < circularity < 1.2:
                        M = cv.moments(contour)
                        if M['m00'] != 0:
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])
                            
                            # Distance from bottom of frame
                            dist_from_bottom = FRAME_HEIGHT - cy
                            
                            if DEBUG_MODE and SHOW_CAMERA_FEED:
                                cv.drawContours(frame, [contour], -1, (255, 0, 255), 3)
                                cv.circle(frame, (cx, cy), 10, (255, 255, 0), -1)
                                cv.putText(frame, "BULLSEYE DETECTED", (10, 30),
                                         cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                            
                            # Check if we're close enough to stop
                            if dist_from_bottom < BULLSEYE_PROXIMITY:
                                self.bullseye_detected = True
                                return True, cx, cy, dist_from_bottom
                            
                            return True, cx, cy, dist_from_bottom
        
        return False, -1, -1, -1
    
    def calculate_path_error(self, cx):
        """
        Calculate normalized error from path center.
        Returns error in range [-1.0, 1.0]
        """
        if cx == -1:
            return 0.0  # No path detected, go straight
        
        # Normalize error to [-1, 1]
        error = (cx - FRAME_CENTER_X) / FRAME_CENTER_X
        return np.clip(error, -1.0, 1.0)
    
    def show_debug_feed(self, frame, window_name="Robot Vision"):
        """Display the camera feed with overlays"""
        if frame is not None and SHOW_CAMERA_FEED:
            cv.imshow(window_name, frame)
            cv.waitKey(1)


# Standalone helper functions for backward compatibility
def get_camera_frame():
    """Legacy function - use VisionNavigator class instead"""
    # This is for backward compatibility with the skeletal code
    pass


def find_path_center(frame, lower_threshold, upper_threshold):
    """
    Legacy function for simple path detection.
    Recommended to use VisionNavigator class for full functionality.
    """
    if frame is None:
        return -1, -1
    
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    blur = cv.GaussianBlur(hsv, (5, 5), 0)
    
    mask = cv.inRange(blur, lower_threshold, upper_threshold)
    
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return -1, -1
    
    largest_contour = max(contours, key=cv.contourArea)
    M = cv.moments(largest_contour)
    
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return cx, cy
    
    return -1, -1
