import math
import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera

# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.vflip = True
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(0.1)

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
    # Take picture with camera
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    
    return image

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

def main():
    
    # Read an image from library
    block = cv2.imread("block_pics/blocks_40.png")
    
    # mask the green light from HSV and convert to grayscale
    mask = mask_color(image, imageHSV)

    # Apply Gaussian bluring on image
    img_blurred = blur_img(mask)

    # Detect corners from image
    img_crnr, pts_loc, org_img = corner_detect(img_blurred,image, corners)

    ## Draw contours over an image, if available
    center, radius = center_det(pts_loc,rad_ave)
    
    # Draw a cross at center of frame
    cv2.line(image,(frame_center[0]-100, frame_center[1]),(frame_center[0]+100, frame_center[1]), (0,0,0))
    cv2.line(image,(frame_center[0], frame_center[1]-100),(frame_center[0], frame_center[1]+100), (0,0,0))

    (cx,cy) = (image.shape[1]/4.5, image.shape[0]/8)

    x_ave.append(center[0])
    y_ave.append(center[1])
    rad_ave.append(radius)

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
    
    
if __name__ == "__main__":
    
    main()