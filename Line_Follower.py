# Import necessary libraries
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import RPi.GPIO as GPIO  # Import GPIO library for Raspberry Pi
from PiMotor import Motor  # Import Motor class from PiMotor module

# Set GPIO mode to use board pin numbering
GPIO.setmode(GPIO.BOARD)

# Define individual motors (MOTOR1 and MOTOR2)
m1 = Motor("MOTOR1", 1)
m2 = Motor("MOTOR2", 1)

# Define a function for motor control based on steering input
def Motor_Steer(m1, m2, speed, steering):
    if steering == 0:
        # If no steering, both motors run at the same speed
        m1_speed = speed
        m2_speed = speed
    elif steering > 0:
        # If steering to the right, adjust motor speeds accordingly
        m1_speed = speed * (100 - steering) / 100
        m2_speed = speed
    elif steering < 0:
        # If steering to the left, adjust motor speeds accordingly
        steering = abs(steering)
        m1_speed = speed
        m2_speed = speed * (100 - steering) / 100

    # Ensure motor speeds are within the valid range (0-100)
    m1_speed = max(min(m1_speed, 100), 0)
    m2_speed = max(min(m2_speed, 100), 0)

    # Apply motor speeds using PWM (Pulse Width Modulation)
    m1.ChangeDutyCycle(m1_speed)
    m2.ChangeDutyCycle(m2_speed)

# Set up the PiCamera
camera = PiCamera()
camera.resolution = (200, 125)
camera.framerate = 60
rawCapture = PiRGBArray(camera, size=(200, 120))
time.sleep(0.1)

# Initialize variables for image processing and motor control
kernel = np.ones((3,3), np.uint8)
x_last = 100
y_last = 80
kp = 0.75
ap = 1


# Main loop for capturing frames, processing images, and controlling motors
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    counter += 1
    image = frame.array

    # Preprocess the image for line detection
    Blackline = cv2.inRange(image, (0,0,0), (75,75,75))
    Blackline = cv2.erode(Blackline, kernel, iterations=1)
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    
    # Check if any black lines are detected
    if len(contours_blk) > 0:
        # Process each detected line
        for con_num in range(len(contours_blk)):
            # Get the parameters of the detected line
            blackbox = cv2.minAreaRect(contours_blk[con_num])
            (x_min, y_min), (w_min, h_min), ang = blackbox

            # Adjust the angle of the line if necessary
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90-ang)*-1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            
            # Calculate the error and angle for steering control
            setpoint = 160
            error = int(x_min - setpoint)
            ang = int(ang)
        
            # Perform steering control based on the error and angle
            error_correction = (error * kp) + (ang * ap)
            Motor_Steer(m1, m2, 30, error_correction)

            # Draw the detected line on the image
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(image, [box], 0, (0,0,255), 3)
            cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(image, str(error), (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.line(image, (int(x_min), 90), (int(x_min), 110), (255, 0, 0), 3)
            
    # Display the processed image with overlays
    cv2.imshow("Original with Line", image)
    rawCapture.truncate(0)
    
    # Check for user input to exit the loop
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    
# Clean up GPIO
GPIO.cleanup()
