import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot
import math
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
#from Locomotion.drive_robot import pivot, imu_serial,pwmZero,pwms

import smtplib
from smtplib import SMTP
from smtplib import SMTPException
import email
from email.mime.image import MIMEImage
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from datetime import datetime
import imaplib

# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.vflip = True
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(2)

font = cv2.FONT_HERSHEY_COMPLEX_SMALL

# # Read original RGB Image from library
# block = cv2.imread("greenBlock.png")
pics = []

ave_center = [0,0]
bbox_radius = 0

frame_center = (320,240)

(cx,cy) = (frame_center[0]/4.5, frame_center[1]/8)

# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('blockRetrieval.avi', fourcc, 3, (640, 480))

# Open .txt file to save data
f = open('pxlradius_distance-02.txt','w')

image = np.empty((480*640*3,),dtype=np.uint8)

detected = False

obj = ''

pixel2deg = 0.061

# Initialize servo gripper states
closed = 2.5
half = 5
open_full = 7.5
 
# # create object to read camera
# video = cv2.VideoCapture(0)
# 
# if (video.isOpened() == False):
#     print("Error reading video")
    
# def cv2_cam_frame():
#     # Pick image from video stream
#     success, image = video.read()
#     
#     return image

def picam_frame():
    global image
    
    # Take picture with camera
    camera.capture (image, format="bgr")
    image = image.reshape((480,640,3))
    
    image = cv2.flip(image,1)
        
    
    return image

def img_show(name, img):
    cv2.imshow(name,img)
#     key = cv2.waitKey(1) & 0xFF
    cv2.waitKey(0)
    # Write frame to video
    # out.write
    cv2.destroyAllWindows()
    
def dist_img(img,dist):
    
    dist = str(dist) + "cm"
    data = f"{obj} Sodar: " + dist
    
    cv2.putText(img, data, (355,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
    
    return img
    
def servo_img(grip_state):
    
    img = picam_frame()
    data = str(grip_state)
    data = "Duty Cycle: " + data + "%"
    
    cv2.putText(img, data, (20,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
    
    if grip_state == half:
        cv2.putText(img, "Half-Opened", (20,55),cv2.FONT_HERSHEY_SIMPLEX,1,(0,125,125),2)
    elif grip_state == open_full:
        cv2.putText(img, "Fully-Opened", (20,55),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
    elif grip_state == closed:
        cv2.putText(img, "Fully-Closed", (20,55),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
        
    return img

def mask_color(image, imageHSV):
    global obj
    # HSV bounds
    # minHSV = np.array([41,74,163])
    # maxHSV = np.array([63,159,255])
#     # Trail Green Bock - LAB
#     minHSV = np.array([47,56,172])
#     maxHSV = np.array([255,255,255])

    if obj == 'green':
        # Trail Green Bock - LAB
        minHSV = np.array([46,88,121])
        maxHSV = np.array([65,141,255])
    elif obj == 'red':
        # Trail Red block - LAB
        minHSV = np.array([151,107,147])
        maxHSV = np.array([255,255,255])
    else:
        # Trail Blue block - LAB
        minHSV = np.array([72,94,97])
        maxHSV = np.array([126,188,212])
    
#     # Mask the gripper
#     mask_gripper = np.zeros(image.shape[:2], np.uint8)
#     mask_gripper[0:0, 380:0] = 255
#     
#     # Cut out gripper
#     mask_img = cv2.bitwise_and(image, image, mask = mask_gripper)
    maskHSV = cv2.inRange(imageHSV, minHSV, maxHSV)

    return maskHSV

def prep_image(image):
    
     # Convert image from BGR to HSV space
    imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    # mask the green light from HSV and convert to grayscale
    mask = mask_color(image, imageHSV)

    # Mask HSV masked image of arrow
    blurred = cv2.GaussianBlur(mask,(11,11), 0)
    
    return blurred    

def corner_detect(img,origImg):

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
            
            # Draw circle ontop of original image
            cv2.circle(origImg, (x,y), 3, (0,0,255),-1)

        # Create a column vector from pts list
        pts_loc = np.array(pts_loc)
        
        return img, pts_loc, origImg, True
    else:
        return img, pts_loc, origImg, None

def center_det(pt_list, center):
    global bbox_radius
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
    #center = [int(round(x_half)), int(round(y_half))]
    center.append(int(round(x_half)))
    center.append(int(round(y_half)))
    # Estimate radius
    bbox_radius = int(round(math.sqrt((x_max - x_half)**2 + (y_max - y_half)**2)))
        
    return center, bbox_radius


def bbox_detect(image, image_blurred):
    global bbox_radius
    global obj
    
    center = []
    radius = 0
    success = None
    
#     # Extract frame from camera
#     image = picam_frame()
#     
#     # Prep image for object detection
#     img_blurred = prep_image(image)
    
    # Detect corners from image
    img_crnr, pts_loc, image, var = corner_detect(image_blurred, image)

    if var is not None:

        # Check if corners are detected
        if len(pts_loc) > 3:
            ## Draw contours over an image, if available
            center, bbox_radius = center_det(pts_loc, center)

            block_coordinate = "(" + str(center[0]) + "," + str(center[1]) + ")"
            
            # Draw circle ontop of original image
            cv2.circle(image, tuple(center), bbox_radius, (0,255,255),2)
            cv2.circle(image, tuple(center), 0,(0,0,255),5)
            cv2.putText(image,block_coordinate,(0,int(cy)),font,2,(0,0,0),2)
            
            # Draw a cross at center of frame
            cv2.line(image,(frame_center[0]-100, frame_center[1]),(frame_center[0]+100, frame_center[1]), (0,0,0))
            cv2.line(image,(frame_center[0], frame_center[1]-100),(frame_center[0], frame_center[1]+100), (0,0,0))
            
            success = True
            
            print(f"{obj} block is detected")
            time.sleep(1)
                        
            return image, center, bbox_radius, success
        
        else:
            print("Not clearly detected")
            success = False
            
            return image, center, bbox_radius, success
    
    else:
        print("No object of interest in scene")
        return image, 0, 0, success
    
def scanObject(color):
    global ave_center
    global obj
    global bbox_radius
    
    detected = False
    obj = color
    # Rotate every small angle, take a picture to detect existence of toy block
    stat = 0
    turn_angle = 15
    deg_diff = 0
    yaw = imu_serial()
    yaw_orig = yaw
    
    while not detected:

        image, ave_center, bbox_radius, success = bbox_detect()
           
        # write frame into file
#         out.write(image)
        if success:
            diff = ave_center[0]-frame_center[0]
            # Convert the pixel difference to degrees
            deg_diff = diff * pixel2deg
            
            detected = True                
            
            return detected, deg_diff, image
        
        else:
            # Turn the robot by 15 degrees
            #stat += turn_angle
            
            return detected, turn_angle, image
            
            yaw = pivot(deg_diff= turn_angle)
            time.sleep(1)
            
            
        if stat >=360:
            print("Could not detect block from area")
            print("Drive to a different location and try again")
            break
        
    else:
        print("Desired object has been located")
        
    return detected, deg_diff, image

def dist_estimate():
    # Read in pixel radius from image
    global bbox_radius
    
    img, __, bbox_radius, __ = bbox_detect()
    
    # Plug in pixel size into depth equation
    distance = (0.0099 * bbox_radius**2) - (1.8846*bbox_radius) + 103.47
    
    # Append image of estimated distance on image frame
    image = dist_img(img,round(distance,2))
    
    return distance, image

def email_media(obj):
    
    image = picam_frame()
    
    # send email to user with images
    smtpUser = 'ykebede2@terpmail.umd.edu'
    smtpPass = 'QwE@$d1219'

    toAdd = ['ENPM809TS19@gmail.com','yosephcollege@gmail.com']
    fromAdd = smtpUser

    f_time = datetime.now().strftime('%a %d %b @ %H:%M')
    subject = f'{obj} block picked up for transportation to construction area: ' + f_time

    msg = MIMEMultipart()
    msg['Subject'] = subject
    msg['From'] = fromAdd
    msg['To'] = ",".join(toAdd)

    msg.preamble = "Image @ " + f_time

    body = email.mime.text.MIMEText("Baron Robot image: " + f_time)
    msg.attach(body)

    fp = open(sodar_img, 'rb')
    img = MIMEImage(fp.read())
    fp.close()
    msg.attach(img)
    
    fp = open(grip_img, 'rb')
    img = MIMEImage(fp.read())
    fp.close()
    msg.attach(img)

    s = smtplib.SMTP('smtp.gmail.com', 587)
    s.ehlo()
    s.starttls()
    s.ehlo()
    s.login(smtpUser, smtpPass)
    s.sendmail(fromAdd, toAdd, msg.as_string())
    s.quit()

    print("Email delivered!")

# if __name__ == "__main__":
#     
#     global obj
#     i = 20
#     block = ['g','b','r']
# 
#     while True:
#         
# #         obj = block.pop(0)
#         obj = 'g'
#         
#         detected, deg_diff = detectObject()
#         
# #         # Read image from picamera
# #         image = picam_frame()
# #         
# #         # to accomodate for pi camera mount
# #         image = cv2.flip(image, 1)
# #         
# #         # Draw bbox
# #         radius = bbox_detect(image)
#         
#         outstring = str(i) + ',' + str(radius) + '\n'
#         f.write(outstring)
#         
#         i += 10
#             
#     f.close()
#     
#     for i in range(40,200,30):
#         
#         # Read an image from library
#         image = cv2.imread(f"block_pics/blocks_{i}.png")
#         
#         # Convert image from BGR to HSV space
#         imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
#         
#     #     tmp = cv2.cvtColor(block, cv2.COLOR_BGR2GRAY)
#     #     
#     #     _, alpha = cv2.threshold(tmp, 0, 200, cv2.THRESH_BINARY)
#     #     
#     #     b, g, r = cv2.split(block)
#     #     
#     #     rgba = [b, g, r, alpha]
#     #     
#     #     dst = cv2.merge(rgba,4)
# 
#         # mask the green light from HSV and convert to grayscale
#         mask = mask_color(image, imageHSV)
# 
#         blurred = cv2.GaussianBlur(mask,(11,11), 0)
#         
#         # Detect corners from image
#         img_crnr, pts_loc, image = corner_detect(mask, image)
# 
#         ## Draw contours over an image, if available
#         center, radius = center_det(pts_loc)
# 
#         block_coordinate = "(" + str(center[0]) + "," + str(center[1]) + ")"
#         
#         # Draw circle ontop of original image
#         cv2.circle(image, center, radius, (0,255,255),2)
#         cv2.circle(image, center, 0,(0,0,255),5)
#         cv2.putText(image,block_coordinate,(0,int(cy/2)),font,2,(0,0,0),2)
#         
#         # Draw a cross at center of frame
#         cv2.line(image,(frame_center[0]-100, frame_center[1]),(frame_center[0]+100, frame_center[1]), (0,0,0))
#         cv2.line(image,(frame_center[0], frame_center[1]-100),(frame_center[0], frame_center[1]+100), (0,0,0))
#         
#         cv2.imshow('pixel map of block',image)
#         cv2.imshow('black mask',mask)
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()
#         pxl_radius.append(radius)
#         
#         outstring = str(i) + ' ' + str(radius) + '\n'
#         f.write(outstring)
