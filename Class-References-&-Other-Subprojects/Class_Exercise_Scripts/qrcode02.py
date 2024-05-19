import cv2
import os
import RPi.GPIO as GPIO

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
command = 'sudo modprobe bcm2835-v4l2'
os.system(command)

# Open video capture
cap = cv2.VideoCapture(0)

# Define detector
detector = cv2.QRCodeDetector()

while True:
    check, img = cap.read()
    data, bbox, _ = detector.detectAndDecode(img)
    if(bbox is not None):
        for i in range(len(bbox)):
            cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
#            cv2.putText(img, data, (int(bbox[0][0][0]),int(bbox[0][0][1]-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

    if data:
        print("Data: ", data)
        
        if data == "HALF":
            pwm.ChangeDutyCycle(5.5)
        if data == "OPEN":
            pwm.ChangeDutyCycle(7.5)
        if data == "CLOSE":
            pwm.ChangeDutyCycle(3.5)
            
    # Show resutls to the screen
    cv2.imshow("QR Code detector", cv2.flip(img,-1))
#     cv2.imshow("QR Code detector", img)
    
    # Break out of loop by pressing the q key
    if(cv2.waitKey(1) == ord("q")):
       pwm.stop()
       GPIO.cleanup()
       break
    
cap.release()
cv2.destroyAllWindows()