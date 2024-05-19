import numpy as np
import cv2
import imutils
import RPi.GPIO as gpio
import time
import os
from picamera.array import PiRGBArray
from picamera import PiCamera

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
    
    # initialize the Raspberry Pi camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 25
    rawCapture = PiRGBArray(camera, size=(640,480))

    # allow the camera to warmup
    time.sleep(0.1)
    
    # Setup GPIO pin(s)
    gpio.setup(36, gpio.OUT) # Servo
    
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4
    
    # Initialize pwm signal & move gripper to center position
    pwm = gpio.PWM(36, 50)
    pwm.start(5.5)

    # Initial video feed
#     command = 'sudo modprobe bcm2835-v4l2'
#     os.system(command)
    
    return pwm, camera, rawCapture
    
def gameover():
    # Set all pins low
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    
def forward(tf):
    #init()
    # Left wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right wheels
    gpio.output(35, False)
    gpio.output(37, True)
    # Wait
    time.sleep(tf)
    # Send all pins low & cleanup
#     gameover()
#     gpio.cleanup()
    
def reverse(tf):
    #init()
    # Left wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right wheels
    gpio.output(35, True)
    gpio.output(37, False)
    # Wait
    time.sleep(tf)
    # Send all pins low & cleanup
#     gameover()
#     gpio.cleanup()
    
def pivotleft(tf):
    #init()
    # Left wheels
    gpio.output(31, False)
    gpio.output(33, True)
    # Right wheels
    gpio.output(35, False)
    gpio.output(37, True)
    # Wait
    time.sleep(tf)
        
    # Send all pins low & cleanup
#     gameover()
#     gpio.cleanup()
    
def pivotright(tf):
    #init()
    # Left wheels
    gpio.output(31, True)
    gpio.output(33, False)
    # Right wheels
    gpio.output(35, True)
    gpio.output(37, False)
    # Wait
    time.sleep(tf)
    # Send all pins low & cleanup
#     gameover()
#     gpio.cleanup()


def key_input(event):
    # Initialize board
    gpio.cleanup()
    gpio.setmode(gpio.BOARD)
    
    # Setup GPIO pin(s)
    gpio.setup(36, gpio.OUT) # Servo
    
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4
    
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

def check_input(inp, typ):
    
    if typ == 1: 
    
        # Check string input is numeric float
        while (inp.replace(".","").isnumeric() == False):
            print(duty_cycle + " is not a numeric string that we can work with")
            inp = input("Reenter Correcet format for Servo duty cycle desired [5.5,8.0]: ")
        
        while (float(inp) < 5.5) or (float(inp) > 8.0):
            print("Invalid number given outside the desired range")
            inp = input("Reenter Servo duty cycle desired [5.5,8.0]: ")
            
    elif typ == 2:
        while inp not in ['w','s','a','d']:
            print("Wrong input. Provide characters w, s, a, or d")
            inp = input("Select Driving Mode [w-forward, s-revert, a-pivotLeft, d-pivotRight: ")
    elif typ == 3:
        while inp not in ['a','b','s','c']:
            print("Wrong input. Provide characters a, b, s, or c")
            inp = input("PivotLeft (a), PivotRight (b), Reverse (s), Close gripper (c): ")
            
    return inp

if __name__ == "__main__":
    
    # Initialize pins to motors
    pwm, camera, rawCapture = init()
    
    # create object to read camera
    video = cv2.VideoCapture(0)
    
    if (video.isOpened() == False):
        print("Error reading video")
    
    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('servo_action2.avi', fourcc, 3, (640, 480))
    
    go = True
    
    key_press = ""
    duty_cycle = ""    
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
#     while True:
        # grab the current frame
        img = frame.array
        
        time.sleep(1)
        dist = distance()
        print("Distance ", dist, "cm")
        duty_cycle = input("Servo duty cycle desired [5.5,8.0]: ")

        # Check input 
        duty_cycle = check_input(duty_cycle, 1)

        # Initiate gripper in first with given duty cycle command from user
        pwm.ChangeDutyCycle(float(duty_cycle))

        key_press = input("Select Driving Mode [w-forward, s-revert, a-pivotLeft, d-pivotRight]: ")
        
        # Check input
        key_press = check_input(key_press, 2)
        # Send signal to motors
        key_input(key_press)
        
        # Show resutls to the screen
        cv2.imshow("Robot driving to block", img)
        key = cv2.waitKey(1) & 0xFF
        
        # write frame into file
        out.write(img)
        
        if key_press == 'k':
            print("It was a good run while it lasted. Bye Bye")
            key_input(key_press)
            break
        elif key_press == 'p':
            print("What's wrong? Run paused")
            key_input(key_press)
        
        if go: 
            time.sleep(1)          
            
            if dist > 10:
                print("Distance ", dist, "cm")
            else:
                print("Distance ", dist, "cm")
                print("Robot too close to obstacle")
                print("Hitting brakes.")
                gameover()
                
                go = False
            
                print("Rover has encountered an Obstacle. What do you want to do now?")
                key_press = input("PivotLeft (a), PivotRight (b), Reverse (s), Close gripper (c): ")
                # Check input
                #key_press = check_input(key_press, 3)
        
        elif (dist<40) and (go == False):
            if key_press in ['a','b','s']:
                key_input(key_press)
                print("Distance ", dist, "cm")
            elif key_press == 'c':
                # Initiate closing gripper to hold block
                pwm.ChangeDutyCycle(4.5)
                go = True
        else:
            print("Obstacle cleared")
            gameover()
            go = True    
    
    # Release video capture and video object
    video.release()
    out.release()

    # Close all windows
    cv2.destroyAllWindows()
    
    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()
