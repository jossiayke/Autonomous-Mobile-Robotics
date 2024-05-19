import cv2

# Initialize camera
cam = cv2.VideoCapture(0)

# Read input via camera
res, img = cam.read()

# For image detected without any error
greenBlock = "greenBlockHome.png"
redBlock = "redBlock.png"
blueBlock = "blueBlock.png"

greenKitchen = 'greenKitchen.png'

greenGrip = 'greenGripped.png'
blueGrip = 'blueGripped.png'
redGrip = 'redGripped.png'
if res:
    
    # Flig image vertically and horizontally
    # to accomodate for pi camera mount
    flipped = cv2.flip(img, -1)

    # Show image detected by camera
    cv2.imshow("Image Block", flipped)
    
    # Saving image in local storage
    cv2.imwrite("blocks_40.png", flipped)
    
    # Destroy window with keyboard interrupt
    cv2.waitKey(0)
    # Close all windows
    cv2.destroyAllWindows()
    
# For corrupted image    
else:
    print("No Image detected. Try running again")
