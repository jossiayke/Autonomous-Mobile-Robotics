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
import picamera
# import trackblock01_ver01 as tb
# import qrcode01 as qr

import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.image import MIMEImage
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from datetime import datetime
import imaplib
import threading

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
dutyset = [('f', dict([('start',(35,40)), #35,40
                ('motion',dict([('lMotor',(35,50)), #40,50
                               ('rMotor',(35,40))])
                )]
            )),
            ('rev', dict([('start',(35,40)),
                ('motion',dict([('lMotor',(35,50)),
                               ('rMotor',(35,40))]) # 45,35
                )]
            )),
            ('l', (80,80)),
            ('r',(80,80))]

duty = dict(dutyset)

# Initialize pwms
pwms = []

# Control Gains
Kp = 0.1
Ki = 0.1
Kd = 0.1

# Open .txt file to save data
f = open('hw9data_0.txt','a')

corners = [""]

x_ave = []
y_ave = []

rad_ave = []

x = 0.0
y = 0.0

yaw = 0

font = cv2.FONT_HERSHEY_COMPLEX_SMALL

pixel2deg = 0.061 #deg

result = False
success = False
scanning = False
placing = False
picking = False
ave_center = [0,0]
yaw_diff = 3

yaw_diff_old = 0
turning = ''
direction = ['f','rev','l','r']
grip_state = 0

counter = 0

# Initialize FL and BR button count
counterBR = np.uint64(0)
counterFL = np.uint64(0)
buttonBR = int(0)
buttonFL = int(0)

# Initialize gripper states
closed = 2.5
half = 5
open_full = 7.5

# Define pin allocations
trig = 16
echo = 18

# Server gripper variables
frm_cnt = 0
duty_cycle = 0

def init():
    gpio.setmode(gpio.BOARD)
    
    # Setup GPIO pin(s)
    gpio.setup(36, gpio.OUT) # Servo
    
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
    pwmS = gpio.PWM(36, 50) # Servo
    
    pwms = [pwm01,pwm11,pwm22,pwm02,pwmS]
    
    return pwms
    
def pwmZero(pwms):
    
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(0)
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(0)
    
    return pwms

def gameover(pwms):
    
    print("Gameover")
    pwmZero(pwms)
    pwms[-1].ChangeDutyCycle(closed)
    time.sleep(1)
    for pwm in pwms:
        pwm.stop()
    gpio.cleanup()
    f.close()

def forward(pwms,vals):
    # Left wheels
    pwms[0].ChangeDutyCycle(vals[0])
    pwms[1].ChangeDutyCycle(0)
    # Right wheels
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(vals[1])
    
def reverse(pwms,vals):
    # Left wheels
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(vals[0])
    # Right wheels
    pwms[2].ChangeDutyCycle(vals[1])
    pwms[3].ChangeDutyCycle(0)

def pivotleft(pwms,vals):
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
    
def distance():
    
    pulse_start = 0
    pulse_end = 0
    
    # Define pin allocations
    trig = 16
    echo = 18
    # Setup GPIO board & pins
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig, gpio.OUT)
    gpio.setup(echo, gpio.IN)
    
    # Ensure output has no value
    gpio.output(trig, False)
    time.sleep(0.01)
    
    # Generate trigger pulse
    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)
    
    # Generate echo time signal
    while gpio.input(echo) == 0:
        pulse_start = time.time()
        
    while gpio.input(echo) == 1:
        pulse_end = time.time()
        
    pulse_duration = pulse_end - pulse_start
    
    # Convert time to distance
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    return distance

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

# Distance to encoding conversion
# x_meters * (1 rev / (2pi*0.0325m)) = # wheel rev = 960 counter
def meter2encoder(x_dist):
    encod = round(float(x_dist / (2*math.pi*0.0325))*960)
    
    return encod

def encoderControl(direction, error_encoder):
    
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

def mask_color(image, imageHSV):
    
    # Trial Green Bock - Apartment Kitchen  
    minHSV = np.array([48,61,127])
    maxHSV = np.array([80,255,255])

#     # Trail Green Bock - LAB
#     minHSV = np.array([47,56,172])
#     maxHSV = np.array([255,255,255])
    
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
        
        return img, pts_loc, orig_img, 1
    else:
        return img, pts_loc, orig_img, None

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

def take_img(data, grip_state, dist): #camera, rawCapture, out, data, grip_state, dist
    
    start = time.time()
    while True:
        try:
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
                
                # grab the current frame
                img = frame.array
                
                img = cv2.flip(img,-1)
                #for i in range(len(bbox)):
        #                 cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
                
                dist = str(dist) + "cm"
                data = "Duty Cycle: " + data + "%"
                
                cv2.putText(img, data, (20,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                cv2.putText(img, dist, (500,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
                
                if grip_state == half:
                    cv2.putText(img, "Half-Opened", (40,80),cv2.FONT_HERSHEY_SIMPLEX,1,(0,125,125),2)
                elif grip_state == open_full:
                    cv2.putText(img, "Fully-Opened", (40,80),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
                elif grip_state == closed:
                    cv2.putText(img, "Fully-Closed", (40,80),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                    
                # write frame into file
                out.write(img)
                
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)
                    
                return img

        except picamera.exc.PiCameraValueError:
            print("Caught buffer error")
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            continue

def cmd_motors(direction,pwms,vals):
    # Drive robot towards direction
    
    if direction == 'f':
        forward(pwms,vals)
    elif direction == 'rev':
        reverse(pwms,vals)
    elif direction == 'l':
        pivotleft(pwms,vals)
    elif direction == 'r':
        pivotright(pwms,vals)
        
    return distance()

def rotate(turning, Kp):
    global pwms
    # Keep rotating robot towards given angle
    if turning == 'r':
        valL,valR = duty['r']
        valL = valL * Kp
        valR = valR * Kp
        pivotright(pwms,(valL,valR)) 
    elif turning == 'l':
        valL,valR = duty['l']
        valL = valL * Kp
        valR = valR * Kp
        pivotleft(pwms,(valL,valR))

def servo_cntrl(duty_cycle, pwm):
    
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)

    img = take_img(str(duty_cycle), duty_cycle, distance())

    # Show resutls to the screen
    cv2.imshow("Servo motor status", img)
    key = cv2.waitKey(1) & 0xFF
    out.write(img)
    
    # Break out of loop by pressing the q key
    if(key == ord("q")):
        print("Program Terminated")
        video.release()
        cv2.destroyAllWindows()

def imu_serial():
    
    ser.reset_input_buffer()
    
    while True:
        
        try:
            # Read for imu from serial
            if(ser.in_waiting > 0):
                ser.flush()
                # Strip serial stream of extra characters
                line = ser.readline()

                line = line.rstrip().lstrip()

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")

                # Return float
                line = float(line)
                
                if (line > 180 and line <=360):
                    line = line - 360
                
                return line
        except:
            print("imu unflushed string read warning")

def drive2goal(direction, error_encoder):
    global pwms
    # Convert yaw angles to encoder and add to encoder counts from
    # both motors
    
    valL,valR= encoderControl(direction, error_encoder)
    
    # Based on the direction drive the motors accordingly
    if direction == 'f':
        
        # Drive forward with the above duty cycles
        forward(pwms,(valL,valR))
        
    elif direction == 'rev':
        
        # Drive in reverse with the above duty cycles
        reverse(pwms,(valL,valR))
        
        
    elif direction == 'l':
        
        # Pivot left with the above duty cycles
        pivotleft(pwms,(valL,valR))
        
    else:
        
        # Pivot right with the above duty cycles
        pivotright(pwms,(valL,valR))
        

def pivot(block_center=[0,0],deg_diff=0, just_turn=False):
    
    global yaw_diff
    global pwms
    global ave_center
    
    if not just_turn:
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
    
    # track using imu
    yaw_final = yaw + deg_diff
    
    yaw_diff = yaw_final - yaw
    
    # Check if task is completed
    try:
        while True:

            print(f"Current yaw {yaw}; Expected yaw {yaw_final}; Error yaw {yaw_diff}")     
            if (yaw_diff >= 0.1):
                turning = 'r'
                valL,valR = duty['r']
                valL = valL * Kp * 10
                valR = valR * Kp * 10
                pivotright(pwms,(valL,valR)) 
            elif (yaw_diff <= -0.1):
                turning = 'l'
                valL,valR = duty['l']
                valL = valL * Kp * 10
                valR = valR * Kp * 10
                pivotleft(pwms,(valL,valR))
            
            if (abs(yaw_diff) < 0.4):#(yaw_diff >= -359 and yaw_diff <= 1):

                #if (yaw_diff - imu_serial() > -359 and (yaw_diff - imu_serial())  <=1.5):
                print(f"Initial orientation of roboot {yaw}")
                print("Angle rotated: ", deg_diff)
                print(f"Current Robot orientation {imu_serial()}")

                pwms = pwmZero(pwms)
                time.sleep(1)

                #time.sleep(1)
                print(f"Turn action completed! \n")
        
                        
                return yaw_diff
            
            yaw = imu_serial()
            yaw_diff = yaw_final - yaw
            
    except KeyboardInterrupt:
        print("Pivot Interrupted")
        return None
        
def alignRobot(yaw):

    global corners
    align = False
    global yaw_diff
    global pwms
    global ave_center
    
    while not align:
    
        try:
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
                img_crnr, pts_loc, org_img, var = corner_detect(img_blurred,image, corners)

                if var is not None:

                    # Check if corners are detected
                    if len(pts_loc) > 3:
                        
                        ## Draw contours over an image, if available
                        center, radius = center_det(pts_loc,rad_ave)

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
                            ave_center = [x,y]
                            ave_rad = int(round(np.mean(radius)))
                            # center of block
                            block_coordinate = "(" + str(ave_center[0]) + "," + str(ave_center[1]) + ")"
                            # Draw circle ontop of original image
                            cv2.circle(image, tuple(ave_center), ave_rad, (0,255,255),2)
                            cv2.circle(image, tuple(ave_center), 0,(0,0,255),5)
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
                            ave_center = [x,y]
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
                    
                    # clear the stream in preparation for the next frame
                    rawCapture.truncate(0)
                    
                    yaw = imu_serial()
                    if len(pts_loc) >= 3 and abs(yaw_diff) >=0.3:
                        yaw_diff = pivot()
                        
                    if abs(yaw_diff) < 0.3:
                        align = True
                        return align, yaw_diff
                    
                    if len(pts_loc)<3 and not align:
                        pwms = pwmZero(pwms)
                        time.sleep(1)
                        print("No toy in frame")
                    
                    if len(pts_loc) >= 3 and align:
                        pwms = pwmZero(pwms)
                        time.sleep(1)
                        return align, yaw_diff
                        
                    if abs(yaw_diff) > 0.5:
                        print("Not properly aligned")
                        yaw = imu_serial()
                        continue

                    if yaw_diff is None:
                        return False, None

                else:
                    # show the frame to our screen
                    cv2.imshow("Frame", image)
                    key = cv2.waitKey(1) & 0xFF
                    # write frame into file
                    out.write(image)

                    # clear the stream in preparation for the next frame
                    rawCapture.truncate(0)

                # Read new orientation
                yaw = imu_serial()

                # press the 'q' key to stop the video stream
                if (key == ord("q")):
                    # Release video capture and video object
                    video.release()
                    out.release()

                    pwmZero(pwms)

                    pwms = pwmZero(pwms)
                        
                    print("Waiting for QR code stopped")
                    return None, yaw_diff

        except KeyboardInterrupt:
            print("Keyboard Interrupted")
            pwmZero(pwms)
            print("Tasks Interrupted!")
            return None, yaw_diff

        except picamera.exc.PiCameraValueError:
            print("Caught buffer error")
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            continue

def detectObject():
    
    global ave_center
    
    while True:
        try:
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
                        img_crnr, pts_loc, org_img, var = corner_detect(img_blurred,image, corners)

                        if var is not None:

                            # Check if corners are detected
                            if len(pts_loc) > 3:
                                
                                ## Draw contours over an image, if available
                                center, radius = center_det(pts_loc,rad_ave)

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
                                    ave_center = [x,y]
                                    ave_rad = int(round(np.mean(radius)))
                                    # center of block
                                    block_coordinate = "(" + str(ave_center[0]) + "," + str(ave_center[1]) + ")"
                                    # Draw circle ontop of original image
                                    cv2.circle(image, tuple(ave_center), ave_rad, (0,255,255),2)
                                    cv2.circle(image, tuple(ave_center), 0,(0,0,255),5)
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
                                    ave_center = [x,y]
                                    ave_rad = int(round(np.mean(radius)))

                                    # center of block
                                    block_coordinate = "(" + str(ave_center[0]) + "," + str(ave_center[1]) + ")"
                                    # Draw circle ontop of original image
                                    cv2.circle(image, tuple(ave_center), ave_rad, (0,255,255),2)
                                    cv2.circle(image, tuple(ave_center),0,(0,0,255),5)
                                    cv2.putText(image,block_coordinate,(0,int(cy/2)),font,2,(0,0,0),2)

                            # show the frame to our screen
                            cv2.imshow("Frame", image)
                            key = cv2.waitKey(1) & 0xFF
                            # write frame into file
                            out.write(image)
                            
                            # clear the stream in preparation for the next frame
                            rawCapture.truncate(0)
                            
                            diff = ave_center[0]-frame_center[0]
                            # Convert the pixel difference to degrees
                            deg_diff = diff * pixel2deg
                            
                            return deg_diff

                        else:
                            return None
        
        except picamera.exc.PiCameraValueError:
                print("Caught buffer error")
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)
                continue 

def wait2start():
    
    # Define detector
    detector = cv2.QRCodeDetector()
    
    # Check if program start initiated
    start = False
    
    cnt = 0
    
    print("Waiting for start cue . . . ")
    
    try:
    
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):

            # grab the current frame
            img = frame.array

            img = cv2.flip(img,-1)
            
            data, bbox, _ = detector.detectAndDecode(img)

            if(bbox is not None):
                for i in range(len(bbox)):
                    cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
                    #            cv2.putText(img, data, (int(bbox[0][0][0]),int(bbox[0][0][1]-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

            if data:
                print("Data: ", data)
                    
            # Show resutls to the screen
            cv2.imshow("QR Code detector",img)
            key = cv2.waitKey(1) & 0xFF
            #     cv2.imshow("QR Code detector", img)
        
            # write frame into file
            out.write(img)

            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            
            # Break out of loop by pressing the q key
            if(key == ord("q")):
                print("Program Terminated")
                video.release()
                cv2.destroyAllWindows()
                break

            if data == "ENPM701":
                print("Cue Received \n")
                print('@' * 45)
                print("Starting Grand Challenge! \n")
                print('#' * 45)
                start = True
                video.release()
                cv2.destroyAllWindows()
                break
        
    except:
        cv2.destroyAllWindows()
    
    return start

def drive_robot(dist, dirtn):
    
    global counterBR
    global counterFL
    global buttonBR
    global buttonFL
    global pwms
    global yaw
    global yaw_diff
    global ave_center
    global placing
    global picking
    
    check = False
    
    # direction is a list
    dist_now = 11
    
    encoder_dist = meter2encoder(dist)
    yaw = imu_serial()
    try:
        
        while True:
            # Compute the difference between the left and right encoders
            error_encoder = counterFL - counterBR

            # Count encoders for left and right motors
            if (int(gpio.input(12)) != int(buttonBR)):
                buttonBR = int(gpio.input(12))
                counterBR += 1

            if (int(gpio.input(7)) != int(buttonFL)):
                buttonFL  = int(gpio.input(7))
                counterFL += 1
                
            # Command the robot to drive to user commanded postion
            # in sequence
    #         drive2goal(direction[0], error_encoder)
            valL,valR= encoderControl(dirtn, error_encoder)
            
            # Compute the difference in distance between start state  and goal state
            # by considering the minimum of the two encoders
            pos_encoder = int(round(min(counterFL,counterBR)))

            # Global error between start and goal state will be
            error = encoder_dist - pos_encoder
            
            if dirtn == 'f':
                forward(pwms,(valL,valR))
            elif dirtn == 'rev':
                reverse(pwms,(valL,valR))
                check = True

            if (error >= -10 and error <= 10):

                print("counterBR: ", counterBR, "counterFL: ", counterFL)
                angle_diff = encoder2deg(pos_encoder) - yaw
                print("Angle rotated: ", angle_diff)
                print("Expected turn: ", encoder_dist)
                
                counterBR = 0
                counterFL = 0
                
                
                yaw_diff = imu_serial() - yaw
                
    #             while (abs(yaw_diff) > 1):
    #                 if (imu_serial() - yaw > 0.5):
    #                     rotate('r', Kp)
    #                 elif(imu_serial() - yaw < -0.5):
    #                     rotate('l', Kp)
    #                 yaw_diff = imu_serial() - yaw
                    
                pwmZero(pwms)
                break
                
    #         if (pos_encoder % 480) == 0:
    #             print("adjust robot if needed")
            
    #         if picking:
            deg_off = detectObject()
            dist = distance()
            if deg_off is not None and (not check):
                if (abs(deg_off) > 0.7):
                    print("adjust robot if needed")
                    yaw = imu_serial()
                    yaw_diff = pivot()
                    dist = distance()
                    encoder_dist = meter2encoder(dist)
                    counterBR = 0
                    counterFL = 0
                    
                    check = True
                    dist_now = dist
            
            if (dist_now < 10):
                pwmZero(pwms)
                break
                
        return True

    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        return

def main():
    
    # Initialize board
    init()
    
    global pwms
    global yaw
    global yaw_diff
    global counterBR
    global counterFL
    global direction
    global grip_state
    global placing
    global picking
    
    detected = False
    dist = 0
    encoder_dist = 0
    result = True
    arrived = False
    aligned = False

    yaw = imu_serial()
    pwms = pwmsInit(pwms)

    for pwm in pwms:
        pwm.start(0)

    # Start program by reading QR code
    counter = 0
    
    try:

        #start = wait2start()
        start = True
        # Give few seconds of delay
        print("LET'S GET READY TO RUMBLEEEEE \n")
        print("*" * 15, "o" * 15, "+"*15)
        print("-" * 45)
        time.sleep(5)
        
        while start:
            
            # Check if toy in frame
            #detected, scanning, yaw = detectBlock()
            aligned, yaw_diff = alignRobot(yaw)
            
            deg_off = detectObject()
            
            if aligned is None:
                break
            else:
                print(f"Robot is aligned [{aligned}] and angle error {yaw_diff}") 
            # Start panning robot left and right until object is found
#             result = tb.main()
            if not aligned:
                # Let turn direction be left
                
                yaw = imu_serial()
                yaw_final = yaw - 45
                turning = 'l'
                
                while not detected:

                    # Pan 360deg until toy detected
                    rotate(turning, Kp)
                    
                    yaw= imu_serial()
                    yaw_diff = yaw_final - yaw
                    # Check if toy in frame
                    #detected, scanning, yaw, key = detectBlock()
                    if (abs(yaw_diff) <= 1):
                        time.sleep(2)
                        # Try to search and aligh robot again
                        aligned, yaw_diff = alignRobot(yaw)
                        
                        if aligned:
                            # write frame into file
                            out.write(image)
                            result = True
                            break
                        else:
                            time.sleep(1)
                            yaw_final = yaw - 45
                            counter += 1
                            result = False
                            print("Can't find block, search in other attitudes of robot")
                            continue
                    
                    if counter > 2:    
                        break
     
            else:
            
                if result == True:
                    print("GREEN TOY BLOCK FOUND AND ROBOT ALIGNED \n")
                    print("*" * 40)
                    
                    print("Toy Block Found and Robot Aligned")
                    print("Open Gripper")
                    
                    if not arrived:
                        if grip_state != open_full:
                            grip_state = open_full
                        # Command gripper
                        servo_cntrl(grip_state, pwms[-1])
                        dist = distance()
                        print(dist)
                        
                        picking = True
                        # Go towards block
                        if (dist > 12):
                            if (abs(yaw_diff) > 0.5): #0.5, deg_diff
                                yaw = imu_serial()
                                aligned, yaw_diff = alignRobot(yaw)
                            if yaw_diff is None:
                                print("Program Terminated")
                                return
                            arrived = drive_robot(distance(),direction[0])
                            yaw = imu_serial()
                        else: #if (distance() <= 12):
                            # Drive forward for the measured distance   
                            arrived = drive_robot(distance(),direction[0])
                            yaw = imu_serial()
#                             aligned, yaw_diff = alignRobot(yaw)
#                             arrived = drive_robot(distance())
                            pwms = pwmZero(pwms)
                            
                        grip_state = closed
                        servo_cntrl(grip_state, pwms[-1])
                        print("Toy Block gripped")
                        break
                    else:
                        print("ARRIVED AT TOY BLOCK")
                        # Make final corrections
                        aligned, yaw_diff = alignRobot(yaw)
                        # Drive forward for the measured distance
                        dist = distance()
                        arrived = drive_robot(dist, direction[0])
                        # close gripper 
                        if (distance() < 10):
                            servo_cntrl(closed, pwms[-1])
                            break
                        
                else:
                    print("Can't find object. Reorient or drive to another location.") 
                    video.release()
                    cv2.destroyAllWindows()
                    gameover(pwms)
                    break
        
        # Rotate 90 degrees and move 1 meter with toy block
        yaw= imu_serial()
        yaw_final = yaw - 90
        yaw_diff = yaw_final - yaw
        turning = 'l'
        
        print("Rotate an angle")
        yaw_diff = pivot(deg_diff=-90,just_turn=True)
        
        if yaw_diff is None:
            print("Keyboard Interrupted")
            return
            

        yaw = imu_serial()
            
        # Drive forward for 1 meter and release the toy block
        # Drive forward for the measured distance   
        arrived = drive_robot(20,direction[0])
        grip_state = open_full
        servo_cntrl(grip_state, pwms[-1])
        print("Toy Block dropped")
        
        # Retreat for 0.3 meters in reverse
        arrived = drive_robot(15, direction[1])
        
        
    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        return
    
    
if __name__ == "__main__":
    # Begin Program
    print("*" * 30, "PROGRAM STARTED","*"*30, "\n")
#     main()
    
#     t1 = threading.Thread(target=detectObject, args=(ave_center,))
#     
#     t1.start()
#     
#     t1.join()
    init()
    pwms = pwmsInit(pwms)
    gameover(pwms)
    video.release()
    cv2.destroyAllWindows()
