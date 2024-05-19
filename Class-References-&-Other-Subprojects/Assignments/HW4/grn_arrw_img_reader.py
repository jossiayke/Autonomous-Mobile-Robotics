import cv2

# Initialize camera
cam = cv2.VideoCapture(0)

# Read input via camera
res, img = cam.read()

# For image detected without any error
if res:
    
    # Flig image vertically and horizontally
    # to accomodate for pi camera mount
    flipped = cv2.flip(img, -1)

    # Show image detected by camera
    cv2.imshow("Green Arrow Image", flipped)
    
    # Saving image in local storage
    cv2.imwrite("greenArrow01.png", flipped)
    
    # Destroy window with keyboard interrupt
    cv2.waitKey(0)
    cv2.destroyWindow("Green Arrow Image")
    
# For corrupted image    
else:
    print("No Image detected. Try running again")
