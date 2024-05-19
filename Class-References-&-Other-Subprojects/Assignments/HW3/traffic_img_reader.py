import cv2

# Initialize camera
cam = cv2.VideoCapture(0)

# Read input via camera
res, img = cam.read()

# For image detected without any error
if res:
    
    # Show image detected by camera
    cv2.imshow("Traffic Light Image", img)
    
    # Saving image in local storage
    cv2.imwrite("trafficLight.png", img)
    
    # Destroy window with keyboard interrupt
    cv2.waitKey(0)
    cv2.destroyWindow("Traffic Light Image")
    
# For corrupted image    
else:
    print("No Image detected. Try running again")