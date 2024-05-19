import math
import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from datetime import datetime, timedelta
import threading

import serial
import RPi.GPIO as gpio

# Global variables
# Purpose: Continuosly locate and identify green light from video feed
# Pull in code from Assignment 2 and adjust
# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))

frame_center = (320,240)

# create object to read camera
video = cv2.VideoCapture(0)

if (video.isOpened() == False):
    print("Error reading video")

# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('videonameNew.avi', fourcc, 3, (640, 480))

# Identify serial connection
ser = serial.Serial('/dev/ttyUSB0', 19200)

# Experimentally found duty cycle values for left and right motor
# in movements of the four cardinal directions, converted to  dictionary
dutyset = [('l', (80,80)),('r',(80,80))]
duty = dict(dutyset)

# Control Gains
Kp = 0.1
Ki = 0.1
Kd = 0.1

# Open .txt file to save data
f = open('hw9data_0.txt','a')

# Initialize FL and BR button count
counterBR = np.uint64(0)
counterFL = np.uint64(0)
buttonBR = int(0)
buttonFL = int(0)

def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31, gpio.OUT) # IN1
	gpio.setup(33, gpio.OUT) # IN2
	gpio.setup(35, gpio.OUT) # IN3
	gpio.setup(37, gpio.OUT) # IN4

	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
	
def pwmsInit(pwms):
    
    pwms.clear()
    
    # initialize pwm signal to control motor
    pwm01 = gpio.PWM(31, 50)  # BackLeft motor
    pwm11 = gpio.PWM(33, 50) # FrontLeft motor
    pwm22 = gpio.PWM(35, 50) # FrontRight motor
    pwm02 = gpio.PWM(37, 50)  # BackRight motor
    
    pwms = [pwm01,pwm11,pwm22,pwm02]
    
    return pwms
    
def pwmZero(pwms):
    
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(0)
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(0)
    
    return pwms

def gameover(pwms):

    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(0)
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(0)

def pivotleft(pwms,vals):
#     if (yaw_diff < -1):
    # Left wheels
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(vals[0])
    # Right wheels
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(vals[1])

    
def pivotright(pwms,vals):
#     if (yaw_diff > 1):
    # Left wheels
    pwms[0].ChangeDutyCycle(vals[0])
    pwms[1].ChangeDutyCycle(0)
    # Right wheels
    pwms[2].ChangeDutyCycle(vals[1])
    pwms[3].ChangeDutyCycle(0)
    
def rot2encoder(deg):
    
    # Approximate radius of rotation computed from Baron
    radius = 0.111#0.146 # meters
    # Angle needed for robot to rotate
    arc = ((deg * math.pi) / 180) * radius
    encoder = round(float(arc / (2*math.pi*0.0325))*960)
    
    return encoder

def encoder2deg(encd):
    
    # Approximate radius of rotation computed from Baron
    radius = 0.111
    
    arc = (encd / 960) * 2*math.pi*0.0325
    deg = round((arc / radius) * (180 / math.pi),1)
    
    return deg

def encoderControl(direction, error_encoder, duty):
    
    # Initialize left and right motor duty cycles
    valL = 0
    valR = 0

    thresh = 5
    
    if error_encoder > thresh: # when left motor advances more than the right
        
        # Give power to corresponding motors
        valL = duty[direction]['motion']['lMotor'][0]
        valR = duty[direction]['motion']['lMotor'][1]
        

    elif error_encoder < -thresh: # when right motor advances more than the left
        
        # Give power to corresponding motors
        valL = duty[direction]['motion']['rMotor'][0]
        valR = duty[direction]['motion']['rMotor'][1]
        
    else:
        
        # Give power to corresponding motors
        valL = duty[direction]['start'][0]
        valR = duty[direction]['start'][1]
    
    return (valL, valR)
    
def drive2goal(orientation, duty, pwms,K=1):
    
    # Convert yaw angles to encoder and add to encoder counts from
    # both motors
    turn = ''
    if orientation > 1:
        turn = 'right'
        [valL,valR] = duty['r']
        valL = valL * K
        valR = valR * K
        pivotright(pwms,(valL,valR))

    elif orientation < -1:
        turn = 'left'
        valL,valR = duty['l']
        valL = valL * K
        valR = valR * K
        pivotleft(pwms,(valL,valR))
        
    #print(f"Turning {turn} with {orientation} deg off")
    return pwms

def img_show(name, img):
	cv2.imshow(name,img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def mask_color(image, imageHSV):
#     # Trail Green Bock - LAB
    minHSV = np.array([47,56,172])
    maxHSV = np.array([255,255,255])
    
#     # Trail 6 - On top of wooden table gym mat
#     minHSV = np.array([42,84,124])
#     maxHSV = np.array([255,255,255])

    # Trail 1 Blue Block- LAB
#     minHSV = np.array([64,44,89])
#     maxHSV = np.array([255,255,255])
#     
#     # Trail 1 Red Block- LAB
#     minHSV = np.array([29,81,78])
#     maxHSV = np.array([255,255,255])
    
    # mask HSV
    maskHSV = cv2.inRange(imageHSV, minHSV, maxHSV)
    return maskHSV

def blur_img(maskHSV):
    # Mask HSV masked image of arrow

    blurred = cv2.GaussianBlur(maskHSV,(11,11), 0)
    
    return blurred

def corner_detect(img,orig_img, corners):

    # Create a list to store the x,y location of points
    pts_loc = []
    
    # Detect corners from image
    corners = cv2.goodFeaturesToTrack(img,5,0.01,10)
    
    if corners is not None:
    
        corners = np.int0(corners)
        
        # identify location of corners in image
        for i in corners:
            # Extract x,y coordinate of points
            x,y = i.ravel()

            pts_loc.append([x,y])

        # Create a column vector from pts list
        pts_loc = np.array(pts_loc)
        
        return img, pts_loc, orig_img
    else:
        return img, pts_loc, orig_img

def center_det(pt_list,rad_ave):
    
    # Extract x,y points from pt_list
    x = pt_list[:,0]
    y = pt_list[:,1]
    
    # Determine the min and max width & height values
    # of the points, as if to drow rectangle around arrow
    x_min = x.min()
    y_min = y.min()
    
    x_max = x.max()
    y_max = y.max()
    
    # Store height of bounding box
    vert_dst = y_max - y_min
    
    # Store width of bounding box
    horz_dst = x_max - x_min
    
    # Compute and store half dimensions of
    # box, will come later when determining
    # arrow direction
    y_half = vert_dst/2 + y_min
    x_half = horz_dst/2 + x_min
    
    # Store center of the block
    center = [int(round(x_half)), int(round(y_half))]
	
    # Estimate radius
    radius = int(round(math.sqrt((x_max - x_half)**2 + (y_max - y_half)**2)))
    
    if len(rad_ave)>2 and radius > (max(rad_ave)):
        radius = int(vert_dst)
        
    return tuple(center), radius

def imu_serial():
    
    ser.reset_input_buffer()
    
    while True:
        # Read for imu from serial
        if(ser.in_waiting > 0):

            # Strip serial stream of extra characters
            line = ser.readline()

            line = line.rstrip().lstrip()

            line = str(line)
            line = line.strip("'")
            line = line.strip("b'")

            # Return float
            line = float(line)
            
            return line
            
def pivot(frame_center, ave_center, pixel2deg,yaw,pwms):
    
    # Align the robot so that it faces the block directly
    #alignRobot(frame_center, ave_center, pixel2deg, yaw, pwms)
    # Compute the diffrerence between block and center of frame
    diff = ave_center[0]-frame_center[0]
    # Convert the pixel difference to degrees
    deg_diff = diff * pixel2deg
    # Convert degrees to encoder as a safety check
    encoder_rot = rot2encoder(deg_diff)
    
    success = False

    yaw = imu_serial()
    
    if (yaw > 180 and yaw <=360):
        yaw = yaw - 360
    
    # track using imu
    yaw_final = yaw + deg_diff
    
    yaw_diff = yaw_final - yaw
    
    print(f"Current yaw {yaw}; Expected yaw {yaw_final}; Error yaw {yaw_diff}")     
    if yaw_diff > 1:
        turning = 'r'
        valL,valR = duty['r']
        valL = valL * Kp * 10
        valR = valR * Kp * 10
        pivotright(pwms,(valL,valR)) 
    elif yaw_diff < -1:
        turning = 'l'
        valL,valR = duty['l']
        valL = valL * Kp * 10
        valR = valR * Kp * 10
        pivotleft(pwms,(valL,valR))
            
    # Check if task is completed
    while True:
            
        if (abs(yaw_diff) <= 1):#(yaw_diff >= -359 and yaw_diff <= 1):

            #if (yaw_diff - imu_serial() > -359 and (yaw_diff - imu_serial())  <=1.5):
            print(f"Initial orientation of roboot {yaw}")
            print("Angle rotated: ", deg_diff)
            print(f"Current Robot orientation {imu_serial()}")

            pwms = pwmZero(pwms)
            time.sleep(1)

            #time.sleep(1)
            print(f"Turn action completed! \n")
            
            success = True
            break
        
        yaw = imu_serial()
        if (yaw > 180 and yaw <=360):
            yaw = yaw - 360

        yaw_diff = yaw_final - yaw
                
    return success, yaw_diff

def main():

    # Initialize board
    init()
    
    # Initialize pwms
    pwms= []
    pwms = pwmsInit(pwms)
    
    for pwm in pwms:
        pwm.start(0)
        
    # Pull in code from steps 1 and 2
    # allow the camera to warmup
    time.sleep(0.1)
    start = time.time()

    frm_cnt = 0
    duration = 0
    start = time.time()
    
    # Inirtialize circle variables
    center = 0
    radius = 0.0
    
    x_ave = []
    y_ave = []
    
    rad_ave = []
    
    x = 0.0
    y = 0.0
    
    counter = 0
    
    corners = [""]
    
    yaw = imu_serial()
    
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    
    pixel2deg = 0.061 #deg
    
    result = False
    success = False
    yaw_diff = 3
    
    yaw_diff_old = 0
    turning = ''
    
    while True:
        try:
        # keep looping
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):    
                # Record iteration start time
                startR = datetime.now() #.microsecond / 1000000
                # grab the current frame
                image = frame.array
                
                # to accomodate for pi camera mount
                image = cv2.flip(image, -1)

                # Convert image from BGR to HSV space
                imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

                #img_show("Hsv img", imageHSV)
                # mask the green light from HSV and convert to grayscale
                mask = mask_color(image, imageHSV)
                
                # Apply Gaussian bluring on image
                img_blurred = blur_img(mask)
                
                # Detect corners from image
                img_crnr, pts_loc, org_img = corner_detect(img_blurred,image, corners)
                
                ## Draw contours over an image, if available
                center, radius = center_det(pts_loc,rad_ave)

                # Check if corners are detected
                if len(pts_loc) > 3:
                    
                    # Draw a cross at center of frame
                    cv2.line(image,(frame_center[0]-100, frame_center[1]),(frame_center[0]+100, frame_center[1]), (0,0,0))
                    cv2.line(image,(frame_center[0], frame_center[1]-100),(frame_center[0], frame_center[1]+100), (0,0,0))
                    
                    (cx,cy) = (image.shape[1]/4.5, image.shape[0]/8)
                    
                    x_ave.append(center[0])
                    y_ave.append(center[1])
                    rad_ave.append(radius)
                    
                    if ((max(x_ave) - min(x_ave)) < 5) and ((max(y_ave) - min(y_ave)) < 5) and ((max(rad_ave) - min(rad_ave)) < 50):
                            
                            #col = np.transpose(center_ave)
                            x = int(round(np.mean(x_ave)))
                            y = int(round(np.mean(y_ave)))
                            ave_center = (x,y)
                            ave_rad = int(round(np.mean(radius)))
                            # center of block
                            block_coordinate = "(" + str(ave_center[0]) + "," + str(ave_center[1]) + ")"
                            # Draw circle ontop of original image
                            cv2.circle(image, ave_center, ave_rad, (0,255,255),2)
                            cv2.circle(image, ave_center, 0,(0,0,255),5)
                            cv2.putText(image,block_coordinate,(0,int(cy/2)),font,2,(0,0,0),2)
                    else:
                        x_ave.clear()
                        y_ave.clear()
                        rad_ave.clear()
                        x_ave.append(center[0])
                        y_ave.append(center[1])
                        rad_ave.append(radius)
                        
                        x = int(round(np.mean(x_ave)))
                        y = int(round(np.mean(y_ave)))
                        ave_center = (x,y)
                        ave_rad = int(round(np.mean(radius)))
                        
                        # center of block
                        block_coordinate = "(" + str(ave_center[0]) + "," + str(ave_center[1]) + ")"
                        # Draw circle ontop of original image
                        cv2.circle(image, ave_center, ave_rad, (0,255,255),2)
                        cv2.circle(image, ave_center,0,(0,0,255),5)
                        cv2.putText(image,block_coordinate,(0,int(cy/2)),font,2,(0,0,0),2)
                    
                    # show the frame to our screen
                    cv2.imshow("Frame", image)
                    key = cv2.waitKey(1) & 0xFF
                    # write frame into file
                    out.write(image)

                    if len(pts_loc) >= 4:
                        result = pivot(frame_center, ave_center, pixel2deg, yaw, pwms)
                    elif len(pts_loc)<4:
                        pwms = pwmZero(pwms)
                        time.sleep(1)                        
                    elif result != False:
                        pwms = pwmZero(pwms)
                        time.sleep(1)


                    frm_cnt += 1

                    # clear the stream in preparation for the next frame
                    rawCapture.truncate(0)
                        
                    # Read new orientation
                    yaw = imu_serial()

                    outstring = str(ave_center) + '\n'
                    f.write(outstring)
                
                duration = time.time() - start
                # press the 'q' key to stop the video stream
                if (key == ord("q")) or (duration >= 40) or (frm_cnt > 110):
                # Release video capture and video object
                    video.release()
                    out.release()
                    
                    gameover(pwms)
                    
                    for pwm in pwms:
                        pwm.stop()
                    gpio.cleanup()
                    f.close()
                    print("Video Stream stopped")
                    return

        except KeyboardInterrupt:
            print("Keyboard Interrupted")
            gameover(pwms)
            for pwm in pwms:
                pwm.stop()
            gpio.cleanup()
            f.close()
            print("Tasks Interrupted!")
            break
            
        except picamera.exc.PiCameraValueError:
            print("Caught buffer error")
            continue
        
    # Close all windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Begin Program
    print("*" * 30, "PROGRAM STARTED","*"*30, "\n")
    main()
