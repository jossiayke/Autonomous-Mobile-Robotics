import numpy as np
import cv2
import imutils
import RPi.GPIO as gpio
import time
import os
from picamera.array import PiRGBArray
from picamera import PiCamera

# Initialize gripper states
closed = 2.5
half = 5
open_full = 7.5

def distance():
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
    
    # Cleanup gpio pins & return distance estimate
    gpio.cleanup()
    return distance

def init():
    
    gpio.cleanup()
    gpio.setmode(gpio.BOARD)
    
    # Setup GPIO pin(s)
    gpio.setup(36, gpio.OUT) # Servo
    
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4
    
    # Initialize pwm signal & move gripper to center position
    #pwm = gpio.PWM(36, 50)
    
    #return pwm
    
def take_img(camera, rawCapture, out, data, grip_state, dist):
    
    start = time.time()
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
        
        # grab the current frame
        img = frame.array
        
        img = cv2.flip(img,-1)
        #for i in range(len(bbox)):
#                 cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
        
        dist = str(dist) + "cm"
        
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

def gameover():
    # Set all pins low
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    
def forward(tf):
    init()
    # Left wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right wheels
    gpio.output(35, False)
    gpio.output(37, True)
    # Wait
    time.sleep(tf)
    # 
    # Send all pins low & cleanup
    gameover()
    #gpio.cleanup()
    
def reverse(tf):
    init()
    # Left wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right wheels
    gpio.output(35, True)
    gpio.output(37, False)
    # Wait
    time.sleep(tf)
    # Send all pins low & cleanup
    gameover()
    #gpio.cleanup()
    
def pivotleft(tf):
    init()
    # Left wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right wheels
    gpio.output(35, False)
    gpio.output(37, True)
    # Wait
    time.sleep(tf)
        
    # Send all pins low & cleanup
    gameover()
    #gpio.cleanup()
    
def pivotright(tf):
    init()
    # Left wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right wheels
    gpio.output(35, True)
    gpio.output(37, False)
    # Wait
    time.sleep(tf)
    # Send all pins low & cleanup
    gameover()
    #gpio.cleanup()
    
def servo_cntrl(duty_cycle, pwm):
    
    pwm.stop()
    init()
    
    # Initialize pwm signal & move gripper to center position
    pwm = gpio.PWM(36, 50)
    
    pwm.start(duty_cycle)
    
    time.sleep(1)
    
    pwm.ChangeDutyCycle(duty_cycle)
    #time.sleep(2)

def key_input(event):
    # Initialize board
    #init()
    
    print("Key: ", event)
    key_press = event
    tf = 1
    
    if key_press.lower() == 'w':
        forward(tf)
    elif key_press.lower() == 's':
        reverse(tf)
    elif key_press.lower() == 'a':
        pivotleft(tf)
    elif key_press.lower() == 'd':
        pivotright(tf)
    elif key_press.lower() == 'p':
        gamestop()
    else:
        print("Invalid key pressed!!")
        
    return distance()

def main():
    
    # initialize the Raspberry Pi camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 25
    rawCapture = PiRGBArray(camera, size=(640,480))

    # allow the camera to warmup
    time.sleep(0.1)

    gpio.cleanup()
    gpio.setmode(gpio.BOARD)

    # Setup GPIO pin(s)
    gpio.setup(36, gpio.OUT) # Servo

    # Initialize pwm signal & move gripper to center position
    pwm = gpio.PWM(36, 50)

    pwm.start(5)
    
    # Initialize variables
    start = time.time()
    
    # Initialize pins and pwm
    init()

    # Open video capture
    cap = cv2.VideoCapture(0)

    if (cap.isOpened() == False):
        print("Error reading video")
        
    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('servo_action3.avi', fourcc, 3, (640, 480))

    frm_cnt = 0
    duty_cycle = 0
    data = ""
    grip_state = ""

    try:

        while True:
            time.sleep(1)
            dist = distance()
            print("Distance: ", dist, " cm")
            key_press = input("Select Driving Mode [w-forward, s-revert, a-pivotLeft, d-pivotRight]: ")
            print("Drive Mode: " + key_press)    
            
            if key_press == 'p':
                break
            key_input(key_press)
            dist = distance()
            img = take_img(camera, rawCapture, out, data, grip_state, dist)

            # Show resutls to the screen
            cv2.imshow("Distance", img)
            
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            #key = cv2.waitKey(1) & 0xFF
            #time.sleep(3)
            check = input("Keep Driving? [y/n]: ")
            
            if check == 'n':
                while True:
                    grip_state = input("Servo duty cycle desired [2.5,7.5]: ")
                    data = "Duty Cycle: " + grip_state + "%"
                    print(data)
                    #duty_cycle = float(grip_state)
                    
                    
                    grip_state = float(grip_state)
                    
                    servo_cntrl(grip_state, pwm)
                    time.sleep(1)
                    img = take_img(camera, rawCapture, out, data, grip_state, dist)
                        
                    # Show resutls to the screen
                    cv2.imshow("Servo motor status", img)
                    #time.sleep(3)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    #cv2.waitKey(1) & 0xFF   
                    
                    result = input("Satisfied with grip [y/n]: ")
                    
                    if result == 'y':
                        print("Satisified with grip")
                        break
                    
                    print("Reselect duty cycle")

            # Break out of loop by pressing the q key
            # press the 'q' key to stop the video stream
            #if (key == ord("q")) or ( frm_cnt > 60):
            if (frm_cnt > 60):           
                print("Terminating run as frm_cnt reached: " + str(frm_cnt))
                pwm.stop()
                gpio.cleanup()
                break
            frm_cnt +=1
            
    except KeyboardInterrupt:
        
        pwm.stop()
        gpio.cleanup()
        
        # Release video capture and video object
        cap.release()
        out.release()

        cv2.destroyAllWindows()

if __name__ == "__main__":
    
    main()
