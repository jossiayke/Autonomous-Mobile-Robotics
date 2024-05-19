import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plt

def img_masking(gnArrow):
    
    # Save the shape of the image array
    (height, width, c)  = gnArrow.shape
    # Convert image from BGR to HSV space
    gnArrowHSV = cv2.cvtColor(gnArrow, cv2.COLOR_BGR2HSV)

    # Display HSV converted image
    cv2.imwrite("grnArrowHSV.png", gnArrowHSV)

    # HSV bounds
    minHSV = np.array([49, 103, 61]) 
    maxHSV = np.array([93, 255, 248]) 

    # Create a function that will search through every
    # pixel and mask
    
    # Need cv2.inRange() or custom function for HSV masking
    maskHSV = cv2.inRange(gnArrowHSV, minHSV, maxHSV)
    
    cv2.imwrite("gnArrow_MaskOnlyHSV.png",maskHSV)

    # Read all three differnet pictures for ease of display
    grn_HSV = cv2.imread("grnArrowHSV.png")
    grn_Mask = cv2.imread("gnArrow_MaskOnlyHSV.png")
    
    # Now stack images horizontally for ease of display
    grn_all = np.hstack([gnArrow,grn_HSV,grn_Mask])
    
    return maskHSV, grn_all

def blur_img(maskHSV):

    blurred = cv2.GaussianBlur(maskHSV,(11,11), 0)
    
    return blurred

def corner_detect(img,orig_img):

    # Detect corners from image
    corners = cv2.goodFeaturesToTrack(img,5,0.01,10)
    corners = np.int0(corners)

    # Create a list to store the x,y location of points
    pts_loc = []
    
    # identify location of corners in image
    for i in corners:
        # Extract x,y coordinate of points
        x,y = i.ravel()
        # Draw circle of corners on image
        cv2.circle(img,(x,y),3,255,-1)
        cv2.circle(orig_img,(x,y),3,(255,0,0),-1)
        # Store image coordinate of corners in a list
        pts_loc.append([x,y])

    # Create a column vector from pts list
    pts_loc = np.array(pts_loc)
    
    return img, pts_loc, orig_img

def def_det(pt_list):   
    
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
    
    # Initializing the direction of the arrow
    direction = ""
    
    # Initialize counter to track the number of
    # points below the half vertical distance.
    count = 0
        
    # When the arrow vertical (up/down) or (left/right)
    if vert_dst > horz_dst:
        
        # Since the head has 3 corners identified,
        # if the arrow is pointing up (noting origin of image
        # is at the left top corner), there will be three
        # points below the middle of the arrow vertically
        
        # loop through all vertical points of corner ordered
        # points
        for i in y:
            if (i < y_half):
                count+=1
        # Determine direction
        if count >= 3:
            direction = "Up"
        else:
            direction = "Down"
            
    else:
        # Similar to above if logic

        # Since the head has 3 corners identified,
        # if the arrow is pointing left (noting origin of image
        # is at the left top corner), there will be three
        # points to the left of the middle of the arrow horizontally
        
        # loop through all vertical points of corner ordered
        # points
        for i in x:
            if (i < x_half):
                count+=1
        # Determine direction
        if count >= 3:
            direction = "Left"
        else:
            direction = "Right"
        
    return direction

def img_dir_txt(img,img_crn,direction):
    
    (cx,cy) = (img.shape[1]/4.5, img.shape[0]/8)
    # Drawing white background
    cv2.rectangle(img,(0,0),(int(cx),int(cy)),(255,255,255),-1)
    # Placing text over white background
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    green = (0,255,0)
    cv2.putText(img,direction,(0,int(cy/2)),font,2,green,2)
    
    return img    

def main():

    print("Arrow detection program started!")
    
    # Read an image from library
    gnArrow = cv2.imread("greenArrow01.png")
    
    # Apply HSV masking to image read
    img_msk, grn_all = img_masking(gnArrow)
    
    # Apply Gaussian bluring on image
    img_blurred = blur_img(img_msk)
    cv2.imwrite("arw_blur.png",img_blurred)
    # Detect corners from image
    img_crnr, pts_loc, org_img = corner_detect(img_blurred,gnArrow)
    cv2.imwrite("arw_crnr_det.png",img_crnr)
    
    # Identify direction of arrow
    direction = def_det(pts_loc)
    print("The arrow is pointing " + direction)
    
    # Display original image, corner, and text together
    fin_img = img_dir_txt(org_img,img_crnr,direction)

    # Read masked images for display purposes
    blur = cv2.imread("arw_blur.png")
    img_crnr = cv2.imread("arw_crnr_det.png")
    
    # Stack the arrow detection steps
    arr_det = np.hstack([blur,img_crnr,fin_img])
    # Stack the whole process from original to final
    fin_disp = np.vstack([grn_all,arr_det])
    
    # Display full output of process
    cv2.imshow("Green Arrow Masked",fin_disp)
    cv2.imwrite("gnArrow_det_full.png",fin_disp)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
