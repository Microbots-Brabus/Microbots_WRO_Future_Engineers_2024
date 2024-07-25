import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2, Preview

# Function to detect red color in the HSV image
def detect_red(hsv):
    # Define HSV range for detecting red color
    redlower1 = np.array([0, 150, 0])
    redupper1 = np.array([2, 255, 255])
    redlower2 = np.array([176, 120, 0])
    redupper2 = np.array([179, 255, 255])
    # Create masks for the defined ranges
    lower_mask = cv2.inRange(hsv, redlower1, redupper1)
    upper_mask = cv2.inRange(hsv, redlower2, redupper2)
    # Combine the masks
    red_mask = lower_mask + upper_mask
    return red_mask 

# Function to detect green color in the HSV image
def detect_green(hsv):
    # Define HSV range for detecting green color
    green_lower = np.array([60, 132, 53])
    green_upper = np.array([70, 232, 203])
    # Create mask for the defined range
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    return green_mask 
    
# Function to find contours in a given mask
def find_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return contours

# Function to filter contours by a minimum area
def filter_contours_by_area(contours, min_area):
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_area]
    return filtered_contours
    
# Function to get the largest contour from a list of contours
def get_largest_contour(contours):
    if len(contours) == 0:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    return largest_contour

# Main function
def main():
    # Initialize the camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1300, 480)})
    picam2.configure(config)
    picam2.start()
    
    # Initialize the serial communication
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()

    try:
        while True:
            # Capture a frame from the camera
            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate the frame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV
            
            # Detect red and green colors
            red = detect_red(hsv)
            green = detect_green(hsv)
            
            # Apply morphological transformations to clean up the masks
            kernel = np.ones((5, 5), np.uint8)
            red_mask = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            green_mask = cv2.morphologyEx(green, cv2.MORPH_CLOSE, kernel)
            green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours in the masks
            red_contours = find_contours(red_mask)
            green_contours = find_contours(green_mask)
            
            # Filter contours by area
            filtered_green = filter_contours_by_area(green_contours, 1000)
            filtered_red = filter_contours_by_area(red_contours, 1000)
            
            # Create an empty map for drawing contours
            resulted_map = np.full((480, 1300), 255, dtype=np.uint8)
            cv2.drawContours(resulted_map, filtered_green, -1, (0, 0, 255), 2)
            cv2.drawContours(resulted_map, filtered_red, -1, (0, 255, 0), 2)
            
            # Get the largest contour for each color
            largest_green = get_largest_contour(filtered_green)
            largest_red = get_largest_contour(filtered_red)
            
            # Determine which color has the largest contour
            if largest_green is None and largest_red is None:
                color = "N"
            else:
                if largest_green is not None and (largest_red is None or cv2.contourArea(largest_green) > cv2.contourArea(largest_red)):
                    color = "G"
                else:
                    color = "R"
            
            # Print the detected color and send it over serial communication
            print(color)
            ser.write(color.encode('utf-8'))
            time.sleep(1)
            
            # Display the original frame, green mask, and contours
            cv2.imshow('Original', frame)
            cv2.imshow('Green Mask', green_mask)
            cv2.imshow('Contours', resulted_map)
            
            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Release resources
        picam2.close()
        cv2.destroyAllWindows()
        ser.close()

# Entry point of the program
if __name__ == "__main__":
    main()
