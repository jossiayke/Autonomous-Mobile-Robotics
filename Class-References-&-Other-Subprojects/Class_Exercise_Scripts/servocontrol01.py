import cv2
import os
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

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

    # Initial video feed
#     command = 'sudo modprobe bcm2835-v4l2'
#     os.system(command)
    
    return pwm, camera, rawCapture

def slowly(cap,pwm,camera,rawCapture):
    # Initialize gripper states
    closed = 3.5
    half = 5.5
    open_full = 7.5
    # Initiate gripper in a closed state first
    pwm.ChangeDutyCycle(3.5)
    grip_state = 3.5
    cycle = 0
    increment = 0.5
    # Start incrementing gripper every 0.5 until full open
    # then revert direction
    start = time.time()
    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('servo_action2.avi', fourcc, 3, (640, 480))
        
    #while True:
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
#         if (time.time()> start+5):
        start = time.time()
        
        # grab the current frame
        img = frame.array
        
#         check, img = cap.read()
        
        if(grip_state <= open_full) and (grip_state >=3.5):
            
            data = "Duty: " + str(grip_state)
            img = cv2.flip(img,-1)
            #for i in range(len(bbox)):
#                 cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
            
            print(data)
            
            cv2.putText(img, data, (20,20),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2)
            
            if grip_state == half:
                cv2.putText(img, "Half-Opened", (20,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,125,125),2)
            elif grip_state == open_full:
                cv2.putText(img, "Fully-Opened", (20,40),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
            elif grip_state == closed:
                cv2.putText(img, "Fully-Closed", (20,40),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                
            # Show resutls to the screen
            cv2.imshow("Servo motor slowly open/close", img)
            key = cv2.waitKey(1) & 0xFF
            
            # write frame into file
            out.write(img)
        # 	out.write(frame1)
             
            pwm.ChangeDutyCycle(grip_state)
            time.sleep(5)
            grip_state += increment
            
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            
            # Break out of loop by pressing the q key
            if(key == ord("q")):
               pwm.stop()
               GPIO.cleanup()
               break
            if grip_state == open_full:
#             grip_state = open_full
                increment = -increment
                cycle +=1
            elif (grip_state == closed) and cycle>0:
                increment = -increment
#             grip_state += increment
#             cycle +=1
#             if cycle > 2:
#                 pwm.ChangeDutyCycle(3.5)
#                 pwm.stop()
#                 GPIO.cleanup()
#                 break
#         else:
#             continue
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
    slowly(cap,pwm,camera, rawCapture)
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    
    main()
    
    
