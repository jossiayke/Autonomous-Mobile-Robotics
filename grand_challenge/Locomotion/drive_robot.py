import math
import cv2
import os
import serial
import numpy as np
import matplotlib.pyplot as plot
import RPi.GPIO as gpio
import time

from Perception import cntr_bbox

# Identify serial connection
ser = serial.Serial('/dev/ttyUSB0', 19200)

# Experimentally found duty cycle values for left and right motor
# in movements of the four cardinal directions, converted to  dictionary
dutyset = [('f', dict([('start',(50,50)), #35,40
                ('motion',dict([('lMotor',(35,50)), #40,50
                               ('rMotor',(50,40))])
                )]
            )),
            ('rev', dict([('start',(35,40)),
                ('motion',dict([('lMotor',(35,50)),
                               ('rMotor',(35,40))]) # 45,35
                )]
            )),
            ('l', (100,100)),
            ('r',(100,100))]

duty = dict(dutyset)

# Store left right motor duty cycle values
vals = [0,0]

# Initialize pwms
pwms = []

# Initialize servo gripper states
closed = 2.5
half = 5
opened = 7.5

# Initialize FL and BR button count
counterBR = np.uint64(0)
counterFL = np.uint64(0)
buttonBR = int(0)
buttonFL = int(0)

# Control Gains
Kp = 0.1
Ki = 0.1
Kd = 0.1

yaw = 0
yaw_diff = 0
angle_diff = 0
search_drive = 5 #feet
sodar = 0
yaw_final = 0

def init():

    gpio.setmode(gpio.BOARD)
    
    # Setup GPIO pin(s)
    gpio.setup(36, gpio.OUT) # Servo
    
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4
    
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
    
def pwmsInit(pwms):
    
    pwms.clear()
    
    # initialize pwm signal to control motor
    pwm01 = gpio.PWM(31, 50)  # BackLeft motor
    pwm11 = gpio.PWM(33, 50) # FrontLeft motor
    pwm22 = gpio.PWM(35, 50) # FrontRight motor
    pwm02 = gpio.PWM(37, 50)  # BackRight motor
    pwmS = gpio.PWM(36, 50) # Servo
    
    pwms = [pwm01,pwm11,pwm22,pwm02,pwmS]
    
    return pwms

# init()
# pwms = pwmsInit(pwms)

for pwm in pwms:
    pwm.start(0)
    
def pwmZero(pwms):
    
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(0)
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(0)
    
    return pwms

def gameover(pwms=pwms):
    
    print("Gameover")
    pwmZero(pwms)
    pwms[-1].ChangeDutyCycle(closed)
    time.sleep(1)
    for pwm in pwms:
        pwm.stop()
    gpio.cleanup()

def forward(pwms,vals):
    # Left wheels
    pwms[0].ChangeDutyCycle(vals[0])
    pwms[1].ChangeDutyCycle(0)
    # Right wheels
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(vals[1])
    
def reverse(pwms,vals):
    # Left wheels
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(vals[0])
    # Right wheels
    pwms[2].ChangeDutyCycle(vals[1])
    pwms[3].ChangeDutyCycle(0)

def pivotleft(pwms,vals):
    # Left wheels
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(vals[0])
    # Right wheels
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(vals[1])
    
def pivotright(pwms,vals):
#     if (yaw_diff > 1):
    # Left wheels
    pwms[0].ChangeDutyCycle(vals[0])
    pwms[1].ChangeDutyCycle(0)
    # Right wheels
    pwms[2].ChangeDutyCycle(vals[1])
    pwms[3].ChangeDutyCycle(0)
  
def distance():
    
    pulse_start = 0
    pulse_end = 0
    
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
    
    return distance

def servo_cntrl(duty_cycle, pwm):
    
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)

    img = cntr_bbox.servo_img(str(duty_cycle), duty_cycle, distance())

#     # Show resutls to the screen
#     cv2.imshow("Servo motor status", img)
#     key = cv2.waitKey(1) & 0xFF
    #out.write(img)

def imu_serial():
    
    ser.reset_input_buffer()
    ser.flush()
    while True:
        
        try:
            # Read for imu from serial
            if(ser.in_waiting > 0):
#                 if cnt >5:
                # Strip serial stream of extra characters
                line = ser.readline()

                line = line.rstrip().lstrip()

                line = str(line)
                line = line.strip("'")
                line = line.strip("b'")

                # Return float
                line = float(line)
                
                if (line > 180 and line <=360):
                    line = line - 360
                
                return line
#                 cnt += 1
        except Exception as error:
            print(f"Imu Error: {error}")
            
# Distance to encoding conversion
# x_meters * (1 rev / (2pi*0.0325m)) = # wheel rev = 960 counter


def meter2encoder(x_dist):
    encod = round(float(x_dist / (2*math.pi*0.0325))*960)
    
    return encod

def cm2encoder(x_dist):
    dist = x_dist/100
    encod = meter2encoder(dist)
    return encod

def encoder2cm(encd):
    dist = round(float((encd / 960) * 2*math.pi*0.0325),1)
    
    return round(dist/100,2)

def feet2encoder(x_dist):
    dist = round(float(x_dist * 12 * 0.0254))
    encod = meter2encoder(dist)
    return encod

def encoder2meter(encd):
    dist = round(float((encd / 960) * 2*math.pi*0.0325),1)
    
    return dist

def encoderControl(direction, error_encoder, num):
    
    # Initialize left and right motor duty cycles
    valL = 0
    valR = 0

    thresh = 5

    if (error_encoder > thresh): # when left motor advances more than the right

        # Give power to corresponding motors
        valL = duty[direction]['motion']['lMotor'][0] * Kp * num
        valR = duty[direction]['motion']['lMotor'][1] * Kp * num


    elif (error_encoder < -thresh): # when right motor advances more than the left

        # Give power to corresponding motors
        valL = duty[direction]['motion']['rMotor'][0] * Kp * num
        valR = duty[direction]['motion']['rMotor'][1] * Kp * num

    else:

        # Give power to corresponding motors
        valL = duty[direction]['start'][0] * Kp * num
        valR = duty[direction]['start'][1] * Kp * num

    return (valL, valR)

def rotate(direction,pwms, num):

    # Keep rotating robot towards given angle
    if direction == 'r':
        valL,valR = duty[direction]
        valL = valL * Kp*num
        valR = valR * Kp*num
        pivotright(pwms,(valL,valR)) 
    else:
        valL,valR = duty[direction]
        valL = valL * Kp*num
        valR = valR * Kp*num
        pivotleft(pwms,(valL,valR))
        
def pivot(block_center=[0,0],deg_diff=0, pixel=False):
    
    global yaw_diff
    global pwms
    global yaw_final
    global yaw
    
    print("Inside pivot function")
#     pwmZero(pwms)

    if pixel:
        # Align the robot so that it faces the block directly
        #alignRobot(frame_center, ave_center, pixel2deg, yaw, pwms)
        # Compute the diffrerence between block and center of frame
        diff = block_center[0]-cntr_bbox.frame_center[0]
        # Convert the pixel difference to degrees
        deg_diff = diff * cntr_bbox.pixel2deg
    
    success = False

    yaw = imu_serial()
    yawOld = yaw 
    # track using imu
    yaw_final = yaw + deg_diff
    yaw_diff = yaw_final - yaw
    if yaw_diff >= 0:
        direction = 'r'
    else:
        direction = 'l'
    
    yaw_fraction = 10
    yaw_diff_old = yaw_diff

    try:
        while True:
            
            if (abs(yaw_diff) >= 0.4 and (abs(yaw_diff-yaw_diff_old) < 5)):
                print(f"Current yaw {yaw}; Expected yaw {yaw_final}; Error yaw {yaw_diff}")
                rotate(direction, pwms, yaw_fraction)
                yaw = imu_serial()
                yaw_diff = yaw_final - yaw
                yaw = yawOld
                yaw_diff_old = yaw_diff
            else:
                yaw = imu_serial()
                yaw_diff = yaw_final - yaw
                print("Robot turn done")
                break
#                 if (abs(yaw_diff) <10):
#                     yaw_fraction = 8
    #             
        pwms = pwmZero(pwms)
        yaw = imu_serial()
        print("Pivot Finished")
        return yaw
        
    except KeyboardInterrupt:
        print("Pivot Interrupted")
        return None
    except Exception as error:
        print(f"Error {error}")
        return None

def alignRobot(yaw_diff):

    global pwms
    align = False
    
    # Read new orientation
    yaw = 0
    yaw_final = imu_serial() + yaw_diff
    
    try:
    
        while not align:
            
            image, center, radius, success = cntr_bbox.bbox_detect()
            
            if success is not None or success:
                
                if abs(yaw_diff) >=0.4:
                    yaw_diff = pivot(block_center=center,deg_diff=yaw_diff, pixel=True)
                    yaw = imu_serial()
                    yaw_diff = yaw_final - yaw
                elif abs(yaw_diff) < 0.4:
                    align = True
                    pwms = pwmZero(pwms)
                    time.sleep(1)
                    return align, yaw_diff
                
            else:
                yaw = imu_serial()
                return False, None
    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        pwmZero(pwms)
        print("Tasks Interrupted!")
        return None, yaw_diff
    except Exception as error:
        print(f"Pivot Error {error}")
        return None, yaw_diff
#                 if not align:
#                     pwms = pwmZero(pwms)
#                     time.sleep(1)
#                     print("No toy in frame")
            # show the frame to our screen
#             cv2.imshow("Frame", image)
#             key = cv2.waitKey(1) & 0xFF
#             # write frame into file
#             out.write(image)
#             # press the 'q' key to stop the video stream
#             if (key == ord("q")):
#                 # video object
#                 out.release()
# 
#                 pwms = pwmZero(pwms)
#                     
#                 print("Waiting for QR code stopped")
#                 return None, yaw_diff

def quick_drive(direction,pwms, error_encoder, Kprop):
    global vals
    
    # Compute the difference in distance between start state  and goal state        
    L,R = encoderControl(direction, error_encoder, Kprop)

    # Drive robot towards direction

    if direction == 'f':
        forward(pwms,(L,R))
    elif direction == 'rev':
        reverse(pwms,(L,R))
        
    vals[0]= L
    vals[1] = R

def cmd_motors(direction,pwms, encoder_dist, yawIn, steady=False):
    global vals
    global counterBR
    global counterFL
    global buttonBR
    global buttonFL
    global yaw
    global angle_diff
    global sodar

    verify = False

    print("Inside command motors function")
    counterBR = 0
    counterFL = 0
    
    #detected, angle_diff, image = cntr_bbox.scanObject(cntr_bbox.obj)
    # Access object distance from frame
    drive_dist, image = cntr_bbox.dist_estimate()
    #yaw0 = imu_serial()
    sodar = distance()
    Kprop = 10
    cnt = 0
    approach = 0
    thresh = 30
    yaw0 = imu_serial() #abs(angle_diff)
    print(f"yaw start: {yaw0}")
    angle_diff = 0
    angle_diff0 = 0
    est_dist = encoder2cm(encoder_dist)
    #yaw_fin = yaw0 + imu_serial()
    
    try:
        print("Marching forward...")
        while True:
            
            # Count encoders for left and right motors
            if int(gpio.input(12)) != int(buttonBR):
                buttonBR = int(gpio.input(12))
                counterBR += 1

            if int(gpio.input(7)) != int(buttonFL):
                buttonFL  = int(gpio.input(7))
                counterFL += 1
            
            error_encoder = counterFL - counterBR
            
            if steady:
                Kprop = 5
                thresh = 20
            
            quick_drive(direction,pwms, error_encoder, Kprop)
            
#             # Compute the difference in distance between start state  and goal state        
#             vals[0],vals[1] = encoderControl(direction, error_encoder, Kprop)
#             
#             # Drive robot towards direction
#             
#             if direction == 'f':
#                 forward(pwms,vals)
#             elif direction == 'rev':
#                 reverse(pwms,vals)
#             elif direction == 'l':
#                 pivotleft(pwms,vals)
#             elif direction == 'r':
#                 pivotright(pwms,vals)
                
            # by considering the minimum of the two encoders
            pos_encoder = int(min(counterFL,counterBR))

            # Global error between start and goal state will be
            error = encoder_dist - pos_encoder
            #error_prcnt = error / encoder_dist
            
            # Apply proporitonal controller
            #Kprop = Kp * (5+10 *(1 - error_prcnt))
            
#             if (error % 100 == 0):
#                 angle_diff0 = angle_diff
            drive_dist, image = cntr_bbox.dist_estimate()
            sodar = distance()
                
            yaw = imu_serial()
            angle_diff = yaw - yaw0
                
            #angle_diff = yaw_fin - yaw
#             angle_diff = yaw0 - yaw
                
            if steady:
                sodar = distance()
                drive_dist, __ = cntr_bbox.dist_estimate()
                Kprop = 7
                
                quick_drive(direction,pwms, error_encoder, Kprop)
#                 # Compute the difference in distance between start state  and goal state        
#                 vals[0],vals[1] = encoderControl(direction, error_encoder, Kprop)
# 
#                 # Drive robot towards direction
# 
#                 if direction == 'f':
#                     forward(pwms,vals)
#                 elif direction == 'rev':
#                     reverse(pwms,vals)
                
                if (sodar <= 17) or (drive_dist <= 17) or (abs(error) <= 1000):
                    print("Ready to close grip")
#                     pwms = pwmZero(pwms)
#                     return True
                
                #if (abs(error) <= 1000) and (direction in ['f','rev']):
                    
                    print("counterBR: ", counterBR, "counterFL: ", counterFL)
                    print(f"Angle shifted: {angle_diff} deg ")
                    print(f"Estimated distance traveled: {est_dist} cms")
                    counterBR = 0
                    counterFL = 0
                    print("Arrived in front of block")
                    sodar = distance()
                    pwms = pwmZero(pwms)
                    return True
                
            else:
                    
#                     # Compute the difference in distance between start state  and goal state        
#                     vals[0],vals[1] = encoderControl(direction, error_encoder, Kprop)
# 
#                     # Drive robot towards direction
# 
#                     if direction == 'f':
#                         forward(pwms,vals)
#                     elif direction == 'rev':
#                         reverse(pwms,vals)
                
                if not verify:
                    if (abs(error) <1200):
                        Kprop = 8
                    if (abs(angle_diff) > 3 and (abs(angle_diff0-angle_diff) <5)):
                        print(f"Angle diff is {angle_diff}")
                        print("Adjusting to face block front")
                        pwms = pwmZero(pwms)
                        
                        sodar = distance()
        #                 sodar_diff = sodar0 - sodar
                        if sodar > 35:
                            detected, deg_diff, image = cntr_bbox.scanObject(obj)
                            yaw = pivot(deg_diff=deg_diff)
                        time.sleep(1)
                        # Access object distance from frame
                        drive_dist, image = cntr_bbox.dist_estimate()
                        cntr_bbox.img_show('Block',image)
                        encoder_dist = cm2encoder(drive_dist)
                        #align, yaw_diff = alignRobot(angle_diff)
                        counterBR = 0
                        counterFL = 0
                        yaw0 = imu_serial()
                        angle_diff = 0
                        angle_diff_old = 0
                        approach = encoder_dist
                        
                        quick_drive(direction,pwms, error_encoder, Kprop)
                        
                    
                    if (drive_dist>thresh-10 and drive_dist<thresh+10) or (sodar>thresh-10 and sodar<thresh+10):
                        pwms = pwmZero(pwms)
                        sodar = distance()
                        drive_dist, __ = cntr_bbox.dist_estimate()
                        Kprop = 6
                        verify = True
                        encoder_dist = cm2encoder(drive_dist)
                        est_dist = drive_dist
                        time.sleep(1)
                        print("Driving slowly to block via bbox fn and sodar")
                        quick_drive(direction,pwms, error_encoder, Kprop)
                        
                    elif (drive_dist <= thresh-10) or (sodar <= thresh-10):
                        print("Arrived infront of block")
                        pwms = pwmZero(pwms)
                        return True
                    
                    if (abs(error) <= cm2encoder(10)) and (direction in ['f','rev']):
                    
                        print("counterBR: ", counterBR, "counterFL: ", counterFL)
                        print(f"Angle shifted: {angle_diff} deg ")
                        print(f"Estimated distance traveled: {est_dist} cms")
                        counterBR = 0
                        counterFL = 0
                        print("Arrived in front of block")
                        sodar = distance()
                        pwms = pwmZero(pwms)
                        return True
                    
                else:
                    sodar = distance()
                    drive_dist, __ = cntr_bbox.dist_estimate()
                    if (sodar < 22) or (drive_dist < 22):
                        print("Arrived")
                        pwms = pwmZero(pwms)
                        return True
                    if (abs(error) <= cm2encoder(10)) and (direction in ['f','rev']):
                    
                        print("counterBR: ", counterBR, "counterFL: ", counterFL)
                        print(f"Angle shifted: {angle_diff} deg ")
                        print(f"Estimated distance traveled: {est_dist} cms")
                        counterBR = 0
                        counterFL = 0
                        print("Arrived in front of block")
                        sodar = distance()
                        pwms = pwmZero(pwms)
                        return True
                
#             drive_dist, image = cntr_bbox.dist_estimate()
#             
#             sodar = distance()
            
#             if cnt >2:
#                 print("Struggling to align to block correctly")
#                 print("I recommend dring in reverse a bit")
#                 return False
#                 if (sodar >= 20):
#                     encoder_dist = cm2encoder(sodar)
#                     print("Failed the ultrasonic check (robot still 60% far)")
    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        pwms = pwmZero(pwms)
        return False
    except Exception as error:
            print(f"Cmd Motor Error: {error}")

