import cv2 as cv
import numpy as np

def find_path_center(frame, lower_threshold, upper_threshold):
    """
    Implements the vision logic to detect path contours and return 
    [cite_start]the center coordinates for the control algorithm.
    """
    Apply color mask and blur to isolate red path
    color_mask = cv.inRange(frame, lower_threshold, upper_threshold)
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (5, 5), 0)
    
    # Thresholding and contour detection
    _, color_thresh = cv.threshold(blur, 60, 255, cv.THRESH_BINARY_INV)
    contours, _ = cv.findContours(color_thresh.copy(), 1, cv.CHAIN_APPROX_NONE)
    
    # Sort contours by area to find the most significant path
    sorted_contours = sorted(contours, key=cv.contourArea)
    
    if len(sorted_contours) > 0:
        c = sorted_contours[-1]  # Get largest contour
        M = cv.moments(c)
        if M['m00'] != 0:
            # [cite_start]Calculate center (cx, cy) 
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return cx, cy
            
    return -1, -1  # Signal no path detected
