import cv2
import os
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

# Initialize gripper states
closed = 2.5
half = 5
open_full = 7.5

def init():
    # initialize the Raspberry Pi camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 25
    rawCapture = PiRGBArray(camera, size=(640,480))

    # allow the camera to warmup
    time.sleep(0.1)
    
    # Setup GPIO pin(s)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(36, GPIO.OUT)

    GPIO.setup(31, GPIO.OUT) # IN1
    GPIO.setup(33, GPIO.OUT) # IN2
    GPIO.setup(35, GPIO.OUT) # IN3
    GPIO.setup(37, GPIO.OUT) # IN4

    # Set all pins low
    GPIO.output(31, False)
    GPIO.output(33, False)
    GPIO.output(35, False)
    GPIO.output(37, False)
        
    # Initialize pwm signal & move gripper to center position
    pwm = GPIO.PWM(36, 50)
    pwm.start(5.5)
    
    return pwm, camera, rawCapture

def take_img(camera, rawCapture, out, data, grip_state):
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
        
        # grab the current frame
        img = frame.array
        
        img = cv2.flip(img,-1)
        
        cv2.putText(img, data, (20,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
        
        if grip_state == half:
            cv2.putText(img, "Half-Opened", (40,80),cv2.FONT_HERSHEY_SIMPLEX,1,(0,125,125),2)
        elif grip_state == open_full:
            cv2.putText(img, "Fully-Opened", (40,80),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        elif grip_state == closed:
            cv2.putText(img, "Fully-Closed", (40,80),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
            
        # write frame into file
        out.write(img)
        
        return img

def slowly(cap,pwm,camera,rawCapture):
    
    # Initiate gripper in a closed state first
    pwm.ChangeDutyCycle(2.5)
    grip_state = 2.5
    cycle = 0
    increment = 0.25
    # Start incrementing gripper every 0.5 until full open
    # then revert direction
    start = time.time()
    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('servo_action3.avi', fourcc, 3, (640, 480))
    
    frm_cnt = 0
    duration = 0
    
    while True:
        
        if(grip_state <= open_full) and (grip_state >=2.5):
            
            if frm_cnt != 0:
                pwm.ChangeDutyCycle(grip_state)
                time.sleep(5)
            
            data = "Duty: " + str(grip_state) + "%"
            
            img = take_img(camera, rawCapture, out, data, grip_state)
            
            # Show resutls to the screen
            cv2.imshow("Servo motor slowly open/close", img)
            key = cv2.waitKey(1) & 0xFF   
            
            print(data)
            
            # Break out of loop by pressing the q key
            # press the 'q' key to stop the video stream
            if (key == ord("q")):
                pwm.stop()
                GPIO.cleanup()
                break
            
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
                          
            if grip_state == open_full:
                increment = -increment
                cycle += 1
            elif (grip_state == closed) and cycle>0:
                increment = -increment
                cycle += 1
                
            if cycle > 1:
                pwm.ChangeDutyCycle(2.5)
                pwm.stop()
                GPIO.cleanup()
                break
            
            grip_state += increment            
            
        frm_cnt += 1
        
    # Release video capture and video object
    cap.release()
    out.release()

def main():
    
    # Initialize pins and pwm
    pwm, camera, rawCapture = init()
    
    # Open video capture
    cap = cv2.VideoCapture(0)
    
    if (cap.isOpened() == False):
        print("Error reading video")
    
    # Slowly open and close gripper
    slowly(cap,pwm,camera,rawCapture)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    
    main()
    