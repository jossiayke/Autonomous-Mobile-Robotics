import numpy as np
import cv2
import imutils
import RPi.GPIO as gpio
import time
import os

# Define pin allocations
trig = 16
echo = 18

def distance():
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
    
    # Cleanup gpio pins & return distance estimate
    gpio.cleanup()
    return distance

if __name__ == "__main__":
    
    # Record image using Raspistill
    name = "lecture4inclass.jpg"
    os.system('raspistill -w 640 -h 480 -o /home/pi/ENPM701-class-files/' + name)
    
    print("Hi!")
    print("Ultrasonic sensor will print 10 successive range estimates per 1Hz")
    
    img = cv2.imread('lecture4inclass.jpg')
    img = imutils.resize(img, width=400)
    
    # show image
#     cv2.imshow("Ultrasonic on image", img)
#     cv2.waitKey(0)
    
    # Start printing range values 
    start = time.time()
    idx = 1
    measurements = []
    
    while idx <= 10:
        
        if (round((time.time() - start), 2) == 1):
            dist = distance()
            print ("Counter: ", idx, "| Distance: ", dist, " cm")
            idx += 1
            start = time.time()
            measurements.append(dist)
    
    # Take the average of the measurements
    ave = round(np.average(measurements), 2)
    
    # Choose font style and color
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    red = (0, 0, 255)
    
    # Append measurement on image
    cv2.putText(img, str(ave)+" cm", (100, 200), font, 1, red, 2)
    
    # show image
    cv2.imshow("Ultrasonic on image", img)
   
    
    # Save appended image
    cv2.imwrite('lecture4inclass_US_txt.jpg', img)
    
    print("Done")
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()
