import serial
import RPi.GPIO as gpio
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import pprint
import cv2

# Identify serial connection
ser = serial.Serial('/dev/ttyUSB0', 19200)

# Initialize pwms
pwms = []

vals = [0,0,0,0]

# dutyset = [('f', dict([('start',(50,60)), #35,40
#             ('motion',dict([('lMotor',(55,60)), #40,50
#                            ('rMotor',(45,60))])
#             )]
#         )),
#         ('rev', dict([('start',(35,40)),
#             ('motion',dict([('lMotor',(35,50)),
#                            ('rMotor',(35,40))]) # 45,35
#             )]
#         )),
#         ('l', (80,80)),
#         ('r',(80,80))]

dutyset = [('f', dict([('start',(80,80)), #35,40
            ('motion',dict([('lMotor',(80,82.5)), #40,50
                           ('rMotor',(82.5,80))])
            )]
        )),
        ('rev', dict([('start',(35,40)),
            ('motion',dict([('lMotor',(35,50)),
                           ('rMotor',(35,40))]) # 45,35
            )]
        )),
        ('l', (80,80)),
        ('r',(80,80))]


duty = dict(dutyset)

yaw = 0

font = cv2.FONT_HERSHEY_COMPLEX_SMALL

pixel2deg = 0.061 #deg

result = False
success = False
scanning = False
placing = False
picking = False
ave_center = [0,0]
yaw_diff = 3

yaw_diff_old = 0
turning = ''
#### Initialize GPIO pins ####

# def init():
# 	gpio.setmode(gpio.BOARD)
# 	gpio.setup(31, gpio.OUT) # IN1
# 	gpio.setup(33, gpio.OUT) # IN2
# 	gpio.setup(35, gpio.OUT) # IN3
# 	gpio.setup(37, gpio.OUT) # IN4
# 
# 	gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
# 	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
# 	
# 	
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
    
def pwmInit(pwms):
    
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(0)
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(0)
    
    return pwms

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

def pwmZero(pwms):
    
    pwms[0].ChangeDutyCycle(0)
    pwms[1].ChangeDutyCycle(0)
    pwms[2].ChangeDutyCycle(0)
    pwms[3].ChangeDutyCycle(0)
    
    return pwms
# def gameover(pwms):
# # 	gpio.output(31, False)
# # 	gpio.output(33, False)
# # 	gpio.output(35, False)
# # 	gpio.output(37, False)
#     pwms[0].ChangeDutyCycle(0)
#     pwms[1].ChangeDutyCycle(0)
#     pwms[2].ChangeDutyCycle(0)
#     pwms[3].ChangeDutyCycle(0)
# 
# def forward(pwms,vals):
#     #init()
#     # Left wheels
#     #gpio.output(31, True)
#     pwms[0].ChangeDutyCycle(vals[0])
#     #gpio.output(33, False)
#     pwms[1].ChangeDutyCycle(vals[1])
#     
#     # Right wheels
#     #gpio.output(35, False)
#     pwms[2].ChangeDutyCycle(vals[2])
#     #gpio.output(37, True)
#     pwms[3].ChangeDutyCycle(vals[3])
# 
#     
# def reverse(tf,pwms,vals):
#     #init()
#     # Left wheels
# #     gpio.output(31, False)
#     pwms[0].ChangeDutyCycle(vals[0])
# #     gpio.output(33, True)
#     pwms[1].ChangeDutyCycle(vals[1])
#     # Right wheels
# #     gpio.output(35, True)
#     pwms[2].ChangeDutyCycle(vals[2])
# #     gpio.output(37, False)
#     pwms[3].ChangeDutyCycle(vals[3])
# 
#     
# def pivotleft(pwms,val):
#     # Left wheels
#     pwms[1].ChangeDutyCycle(vals[0])
#     # Right wheels
#     pwms[3].ChangeDutyCycle(vals[1])
# 
#     
# def pivotright(pwms,val):
#     # Left wheels
#     pwms[0].ChangeDutyCycle(vals[0])
#     # Right wheels
#     pwms[2].ChangeDutyCycle(vals[1])
def gameover(pwms):
    
    print("Gameover")
    pwmZero(pwms)
    #pwms[-1].ChangeDutyCycle(closed)
    time.sleep(1)
    for pwm in pwms:
        pwm.stop()
    #gpio.cleanup()
    #f.close()

def forward(pwms,vals):
#     print("vals: ", vals)
#     print("pwms: ", pwms)
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

# Distance to encoding conversion
# x_meters * (1 rev / (2pi*0.0325m)) = # wheel rev = 960 counter
def meter2encoder(x_dist):
    encod = round(float(x_dist / (2*math.pi*0.0325))*960)
    
    return encod

def encoder2meter(encd):
    dist = round(float((encd / 960) * 2*math.pi*0.0325),1)
    
    return dist

def rot2encoder(deg):
    
    # Approximate radius of rotation computed from Baron
    radius = 0.111#0.146 # meters
    # Angle needed for robot to rotate
    arc = ((deg * math.pi) / 180) * radius
    distance = round(float(arc / (2*math.pi*0.0325))*960)
    
    return distance

def encoder2deg(encd):
    
    # Approximate radius of rotation computed from Baron
    radius = 0.111
    
    arc = (encd / 960) * 2*math.pi*0.0325
    deg = round((arc / radius) * (180 / math.pi),1)
    
    return deg
    

def userInput(duty):
    print("*" * 70,'\n')
    print("Welcome to the Grand Challenge \n")
    print("*" * 70,'\n')
   
    print("Duty cycle for each motor in cardinal directions obtained from multiple trial run experimentation \n")
    pprint.pprint(duty)
    print("*" * 70,'\n')
    
    print("Provide sequence of commands the robot should move in as [direction_value direction_value direction_value]")
    print("Example: f_2 l_90 f_2")
    
    print("This would drive the robot 2 meters forward, rotate 90 left(ccw) and 2 meters forward again")
    print("+" * 70, '\n')
   
    sequence = list(input("Provide drive sequence for robot: ").split())
    sequence = [tuple(element.split("_")) for element in sequence]
    print("Your chosen sequence is: \n")
    print(sequence)
    
    return sequence

def imu_serial():
    
    ser.reset_input_buffer()
    
    while True:
        
        try:
            # Read for imu from serial
            if(ser.in_waiting > 0):
                ser.flush()
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
        except:
            print("imu unflushed string read warning")

def encoderControl(direction, error_encoder):
    global duty
    global pwms
    global yaw
    # Initialize left and right motor duty cycles
    valL = 0
    valR = 0
    thresh = 0
    
#     yaw_diff = imu_serial() - yaw
    
    if direction in ['f','rev']:
        thresh = 30
        
        if error_encoder > thresh: #and (yaw_diff > 1.5) : # when left motor advances more than the right
        
            # Give power to corresponding motors
            valL = duty[direction]['motion']['lMotor'][0]
            valR = duty[direction]['motion']['lMotor'][1]
        

        elif error_encoder < -thresh:# and (yaw_diff < -1.5) : # when right motor advances more than the left
            
            # Give power to corresponding motors
            valL = duty[direction]['motion']['rMotor'][0]
            valR = duty[direction]['motion']['rMotor'][1]
            
        else:
            # Give power to corresponding motors
            valL = duty[direction]['start'][0]
            valR = duty[direction]['start'][1]
    else:
        thresh = 5
        # Give power to corresponding motors
        valL = duty[direction][0]
        valR = duty[direction][1]
    
#         # error_encoder is FL - BR
#         
#         if error_encoder > thresh: #and (yaw_diff > 1.5) : # when left motor advances more than the right
#             
#             # Give power to corresponding motors
#             valL = duty[direction]['motion']['lMotor'][0]
#             valR = duty[direction]['motion']['lMotor'][1]
#             
# 
#         elif error_encoder < -thresh:# and (yaw_diff < -1.5) : # when right motor advances more than the left
#             
#             # Give power to corresponding motors
#             valL = duty[direction]['motion']['rMotor'][0]
#             valR = duty[direction]['motion']['rMotor'][1]
    
    return valL, valR
    
# def drive2goal(direction, error_encoder, duty, pwms):
#     
#     # Convert yaw angles to encoder and add to encoder counts from
#     # both motors
#     #encod_yaw = rot2encoder(yaw)
#     #new_error = error_encoder - encod_yaw
#     
#     valL,valR= encoderControl(direction, error_encoder, duty)
#     
#     # Based on the direction drive the motors accordingly
#     if direction == 'f':
#         
#         # Drive forward with the above duty cycles
#         pwms[0].ChangeDutyCycle(valL)
#         pwms[3].ChangeDutyCycle(valR)
#         
#        # forward((pwm1,pwm2),vals)
#         
#     elif direction == 'rev':
#         
#         # Drive in reverse with the above duty cycles
#         pwms[1].ChangeDutyCycle(valL)
#         pwms[2].ChangeDutyCycle(valR)
#         
#         #reverse((pwm1,pwm2),vals)
#         
#     elif direction == 'l':
#         
#         # Pivot left with the above duty cycles
#         pwms[1].ChangeDutyCycle(valL)
#         pwms[3].ChangeDutyCycle(valR)
#         
#         #pivotleft((pwm1,pwm2),vals)
#         
#     else:
#         
#         # Pivot right with the above duty cycles
#         pwms[0].ChangeDutyCycle(valL)
#         pwms[2].ChangeDutyCycle(valR)
#         
#         #pivotright((pwm1,pwm2),vals)
#         
#     return pwms

def drive2goal(direction, error_encoder,pwms):
    # Convert yaw angles to encoder and add to encoder counts from
    # both motors
    
    valL,valR= encoderControl(direction, error_encoder)
    
    # Based on the direction drive the motors accordingly
    if direction == 'f':
        
        # Drive forward with the above duty cycles
        forward(pwms,(valL,valR))
        
    elif direction == 'rev':
        
        # Drive in reverse with the above duty cycles
        reverse(pwms,(valL,valR))
        
        
    elif direction == 'l':
        
        # Pivot left with the above duty cycles
        pivotleft(pwms,(valL,valR))
        
    else:
        
        # Pivot right with the above duty cycles
        pivotright(pwms,(valL,valR))
        
    return pwms

def imu_serial():
    
    ser.reset_input_buffer()
    
    while True:
        
        try:
            # Read for imu from serial
            if(ser.in_waiting > 0):
                ser.flush()
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
        except:
            print("imu unflushed string read warning")

def imuQuick(ser):
    
    # Read serial stream
    line = ser.readline()

    # Strip serial stream of extra characters

    line = line.rstrip().lstrip()

    line = str(line)
    line = line.strip("'")
    line = line.strip("b'")

    # Return float
    line = float(line)
    
    return line
    
def dispPlot():
    
    states = np.genfromtxt("FLBR_in_motion_encoder_states.txt", dtype=str)
    
    size = states.shape
    
    print(size,'\n')
    
    x = np.int64(states[:,0].astype(np.float64))
    y = np.int64(states[:,1].astype(np.float64))
    
    # fig plot
    fig, ax = plt.subplots(1,1)
    
    # title
    fig.suptitle('Robot Path')
    
    ax.plot(x,y,ls='solid', color='blue',linewidth=2, label='Robo-path')
    
    ax.set(title="Robot Trajectory",
           ylabel="Yi",
           xlabel="Xi")
    
    plt.savefig('Robot-path-encoder-imu.png')
    plt.show()
    plt.close()
    
#     # Save the array to a text file
#     states_1 = np.loadtxt('IMU_and_encoder_states.txt', dtype=str)
#     
#     size_1 = states_1.shape
#     
#     print('\n',size_1)
#     
#     x_1 = np.int64(states_1[:,0].astype(np.float64))
#     y_1 = np.int64(states_1[:,1].astype(np.float64))
#     
#     fig2, ax2 = plt.subplots(1,1)
#     
#     # title
#     fig2.suptitle('Robot Path')
#     
#     ax2.plot(x_1,y_1,ls='solid', color='red',linewidth=2, label='Robo-path')
#     
#     ax2.set(title="Robot Trajectory",
#            ylabel="Yi",
#            xlabel="Xi")
#     
#     plt.savefig('Robot-path-encoder-imu-ver2.png')
#     plt.show()
#     plt.close()

def pivot(block_center=[0,0],deg_diff=0, just_turn=True):
    
    yaw_diff = 0
    global pwms
    global ave_center
    global yaw
    
    if not just_turn:
        # Align the robot so that it faces the block directly
        #alignRobot(frame_center, ave_center, pixel2deg, yaw, pwms)
        # Compute the diffrerence between block and center of frame
        diff = ave_center[0]-frame_center[0]
        # Convert the pixel difference to degrees
        deg_diff = diff * pixel2deg
        # Convert degrees to encoder as a safety check
        encoder_rot = rot2encoder(deg_diff)
    
    success = False

    yaw = imu_serial()
    
    # track using imu
    yaw_final = yaw + deg_diff
    
    yaw_diff = yaw_final - yaw
    
    # Check if task is completed
    try:
        while True:

            print(f"Current yaw {yaw}; Expected yaw {yaw_final}; Error yaw {yaw_diff}")     
            if (yaw_diff >= 0.1):
                turning = 'r'
                valL,valR = duty['r']
                valL = valL * Kp * 10
                valR = valR * Kp * 10
                pivotright(pwms,(valL,valR)) 
            elif (yaw_diff <= -0.1):
                turning = 'l'
                valL,valR = duty['l']
                valL = valL * Kp * 10
                valR = valR * Kp * 10
                pivotleft(pwms,(valL,valR))
            
            if (abs(yaw_diff) < 0.4):#(yaw_diff >= -359 and yaw_diff <= 1):

                #if (yaw_diff - imu_serial() > -359 and (yaw_diff - imu_serial())  <=1.5):
                print(f"Initial orientation of roboot {yaw}")
                print("Angle rotated: ", deg_diff)
                print(f"Current Robot orientation {imu_serial()}")

                pwms = pwmZero(pwms)
                time.sleep(1)

                #time.sleep(1)
                print(f"Turn action completed! \n")
        
                        
                return yaw_diff
            
            yaw = imu_serial()
            yaw_diff = yaw_final - yaw
            
    except KeyboardInterrupt:
        print("Pivot Interrupted")
        return None

def main():
    global pwms
    global yaw
    #### Main Code ####
    init()

    # Initialize pwm and duty cycle
            #pwms, duty, vals = pwmInit()
#     # initialize pwm signal to control motor
#     pwm01 = gpio.PWM(31, 50)  # BackLeft motor
#     pwm11 = gpio.PWM(33, 50) # FrontLeft motor
#     pwm22 = gpio.PWM(35, 50) # FrontRight motor
#     pwm02 = gpio.PWM(37, 50)  # BackRight motor
# 
#     pwms = [pwm01,pwm11,pwm22,pwm02]

    pwms = pwmsInit(pwms)
    
    for pwm in pwms:
        pwm.start(0)
        
    # Initialize FL and BR button count
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)

    
    # Experimentally found duty cycle values for left and right motor
    # in movements of the four cardinal directions, converted to  dictionary
#     dutyset = [('f', dict([('start',(30,40)), #35,40
#                            ('motion',dict([('lMotor',(35,50)), #40,50
#                                            ('rMotor',(50,45))])
#                         )]
#                     )),
#                ('rev', dict([('start',(35,40)),
#                            ('motion',dict([('lMotor',(22,30)),
#                                            ('rMotor',(45,35))])
#                         )]
#                     )),
#                ('l', dict([('start',(80,80)),
#                            ('motion',dict([('lMotor',(80,84)),
#                                            ('rMotor',(90,80))])
#                             )])
#                 ),
#                ('r',dict([('start',(100,100)),#90,94
#                           ('motion',dict([('lMotor',(90,100)), #80,84#90,94
#                                           ('rMotor',(100,95))])) #90,80 99,90
#                           ])
#                 )]
#     
#     duty = dict(dutyset)

    

    # Initialize pwms with 0 duty cycle so that we can pass voltage signals
    # later
#     #map(lambda x: x.start(0), pwms)
#     pwms[0].start(0)
#     pwms[1].start(0)
#     pwms[2].start(0)
#     pwms[3].start(0)
        
    # Open .txt file to save data
    f = open('LAB_test_encoder_states.txt','a')
    outstringF = ''
    
    f_1 = open('LAB_IMU_and_encoder_states.txt','a')
    outstring = ''
    # Initialize variables to record encoder count
    encoder_dist = 0
    
    # Initialize variables to store commanded direction and value
    direction = "placeholder"
    drive = 0
    
    # PID constants
    Kp = 0.1
    Ki = 0.1
    Kd = 0.1
    
    # Initialize variable to store the error between currrent location
    # and destination
    error = 0
    
    # Initialize variable to save difference between left and
    # right motor encoder count
    error_encoder = 0
    
    # Coordinates for horizontal (x) and vertical(y)
    Xr = [0]
    Yr = [0]
    
    pose = (0,0)
    
    # Initialize yaw in degrees from imu
    yaw = 0
    
    # Initialize a boolean to keep track of robot completing task at hand
    completed = False
    
    print("Initializing IMU, and clipping first few data reads ... \n")
    
    # Initialize minimum pose in encoder count robot moved
    pos_encoder = 0
    
    # Total distance
    s = 0
    # Angle
    ang = []
    angLast = 0
    delta = 0
    deltaX = 0
    deltaY = 0
    # Save cmmd anchor
    start = [0,0]
    
    yaw = imu_serial()
    yaw_diff = 2
    
    try:
        # Take user input commands
        sequence = userInput(duty)
        
        order = [cmd[0] for cmd in sequence]
        # To keep count of iteration
        count = 0
        # Initialize line with none
        line = None
        
        cnt = 0
        
        yaw1 = imu_serial()
        
        xT = 0.0
        yT = 0.0
        
#         # Loop through code until all the commands in the sequence are processed
#         while True:            
#             
#             if(ser.in_waiting > 0):
# 
#                 count +=1
#                 
#                 # Read serial stream
#                 line = ser.readline()
#             
#                 # Avoid first n-lines of serial information
#                 if (count > 10):# and (len(sequence) > 0)):
# 
#                     # Read serial stream
#                     line = ser.readline()
#                 
#                     # Strip serial stream of extra characters
# 
#                     line = line.rstrip().lstrip()
# 
#                     line = str(line)
#                     line = line.strip("'")
#                     line = line.strip("b'")
# 
#                     # Return float
#                     line = float(line)
#                     
#                     completed = True
#                     
#                     delX = 0
#                     delY = 0
#                     
#                     rot = ['','','','']
#                     
#                     cnt += 0
#                     
#                     
             
        while len(sequence) > 0:
            # Initialize boolean from start or after completion of action
            completed = False
            
            # Current yaw at the start of action
            yaw1 = imu_serial()
            
            # Pop out the first command from the sequence
            cmd = sequence.pop(0)
            print("Current task -> ", cmd)
            
#             if yaw1 <= 360 and yaw1 > 180:
#                     yaw1 = -(360 - yaw1)
            
            # Assign direction and value from ordered pair
            direction = cmd[0]
            drive = float(cmd[1])
            goal = 0
            
            xT = pose[0]
            yT = pose[1]
            
            start_bool = True
            
            # Convert drive (linear/angular) to encoders
            if direction in ['f','rev']:
                # convert linear distance to encoder count
                encoder_dist = meter2encoder(drive)
                print(f"Driving {direction}")
                ang.append(0)
                goal = encoder_dist
                
#                 # Give power to corresponding motors
#                 valL = duty[direction]['start'][0]
#                 valR = duty[direction]['start'][1]
#                 
#                 # Drive forward with the above duty cycles
#                 pwms[0].ChangeDutyCycle(valL)
#                 pwms[3].ChangeDutyCycle(valR)
                
#                 deltaX = x - drive 
            else:
                
                yaw_diff = pivot(drive)
                
                # Convert angle to encoder
#                 encoder_dist = rot2encoder(drive)
#                 print(f"Pivoting {direction}")
#                 if direction == 'l':
#                     goal = encoder_dist
#                 elif direction == 'r':
#                     goal = -encoder_dist
#                     
#                 ang.append(goal)
#                     
                print("Turn to this angle: ", drive)
#                     if goal <= 360 and goal > 180:
#                             goal = -(360 - goal)
                    
            # Check Angle difference
            angle_diff = 0.0
            flag = False
            cnt = 0
            
            tick = 0
            s = 0
            # Check if robot completed prior task
            while not completed:
                cnt += 1
                
                #if direction in ['r','l']:
#                             # Read serial stream
#                             line = ser.readline()
#                         
#                             # Strip serial stream of extra characters
# 
#                             line = line.rstrip().lstrip()
# 
#                             line = str(line)
#                             line = line.strip("'")
#                             line = line.strip("b'")
# 
#                             # Return float
#                             angle_diff = float(line)
#                             
#                             error = yaw1 - angle_diff
                    
                #else:
                
                # Compute the difference between the left and right encoders
                error_encoder = counterFL - counterBR
                
                # Count encoders for left and right motors
                if int(gpio.input(12)) != int(buttonBR):
                    buttonBR = int(gpio.input(12))
                    counterBR += 1

                if int(gpio.input(7)) != int(buttonFL):
                    buttonFL  = int(gpio.input(7))
                    counterFL += 1
                
                # Command the robot to drive to user commanded postion
                # in sequence
                pwms = drive2goal(direction, error_encoder, pwms) 
                    
                # Compute the difference in distance between start state  and goal state
                # by considering the minimum of the two encoders
#                 pos_encoder = int(round((counterFL+counterBR)/2))
                pos_encoder = int(min(counterFL,counterBR))
                
                # Global error between start and goal state will be
                error = encoder_dist - pos_encoder
                    
                # Save robot path
                if (cnt % 50) == 0:
                
                    if direction in ['f','rev']:
                        s = encoder2meter(pos_encoder)
                        
                        delta = ang[0] - angLast
                        
#                                 Xr.append(x + s*math.cos(math.radians(ang[-1]))) # -
#                                 Yr.append(y - s*math.sin(math.radians(ang[-1])))
                        
#                                 Xr = x + s*math.cos(math.radians(ang[-1]))
#                                 Yr = y - s*math.sin(math.radians(ang[-1]))
                        
                        #move = order[cnt]
#                                 delta += (360 + ang[-1])
#                                 delta = delta % 360
#
#if delta <= 360 and delta > 180:
#                                     delta = -(360 - delta)

#                                 if (delta > 270 and delta <= 360):
#                                 if (rot.count('l') == 2):
                        if tick == 0:
                            #xT = s 
#                             Xr.append(xT)#*math.cos(math.radians(delta))) # -
#                             Yr.append(yT- 0)#s*math.sin(math.radians(delta)))
                            xT = s*math.cos(math.radians(delta))
                            yT = s*math.sin(math.radians(delta))
#                             Xr.append(s*math.cos(math.radians(delta)))
#                             Yr.append(s*math.sin(math.radians(delta)))
                            
                            #Record encoder states to txt file
                            outstringF = str(xT) + ' ' + str(yT) + '\n'
                            #print(outstringF)
                            f.write(outstringF)
#                                 elif (rot.count('l') == 3):
#                                     Xr.append(x - s*math.cos(math.radians(ang[-1]))) # -
#                                     Yr.append(y + s*math.sin(math.radians(ang[-1])))
#                                 
#                                 else:
#                                     Xr.append(x - s*math.cos(math.radians(ang[-1]))) # -
#                                     Yr.append(y - s*math.sin(math.radians(ang[-1])))
                            
#                                 elif move == 'f' and (rot[3] == 'l'):
                        if tick == 1:
                            #yT += s
#                             Xr.append(xT-0) # -
#                             Yr.append(yT)#*math.cos(math.radians(90+delta)))
                            xT = 0
                            yT = s#*math.sin(math.radians(delta))
                            #Xr.append(Xr[-1] + 0)#s*math.cos(math.radians(delta)))
                            #Yr.append(-s*math.sin(math.radians(delta)))
                            
                            #Record encoder states to txt file
                            outstringF = str(Xr[-1]) + ' ' + str(Yr[-1]+yT) + '\n'
                            #print(outstringF)
                            f.write(outstringF)
                            
#
# #                                 elif move == 'f' and (rot.count('l') == 2):
                        if tick == 2:
                            #xT = -s
#                             Xr.append(xT)#*math.cos(math.radians(90+delta))) # -
#                             Yr.append(yT-0)#s*math.sin(math.radians(delta)))#s*math.sin(math.radians(90 + angLast)))
                            xT = -s#*math.cos(math.radians(0))
                            yT = 0
#                             Xr.append()
#                             Yr.append(Yr[-1])#-s*math.sin(math.radians(0)))
                            
                            #pose[0] += xT
                            
                            #Record encoder states to txt file
                            outstringF = str(Xr[-1]+xT) + ' ' + str(Yr[-1]) + '\n'
                            #print(outstringF)
                            f.write(outstringF)
#                                     
#                                     rot = ['','']
#                                     
# #                                 elif move == 'f' and (rot.count('l') == 3) and (rot[1] == 'l'):
                        if tick == 3:
                            #xT = -s
#                             Xr.append(xT - 0) # -
#                             Yr.append(yT)#*math.cos(math.radians(90+delta)))
                            xT = 0#- s*math.cos(math.radians(delta))
                            yT = -s#*math.sin(math.radians(delta))
                            
#                             Xr.append(Xr[-1] - s*math.cos(math.radians(delta)))
#                             Yr.append(Yr[-1] + s*math.sin(math.radians(delta)))
                            
                            #pose[1] += xT
                            
                            #Record encoder states to txt file
                            outstringF = str(Xr[-1]+xT) + ' ' + str(Yr[-1]+yT) + '\n'
                            #print(outstringF)
                            f.write(outstringF)
#
                            #yT = pose[1]
#                                     #Xtot - Xr[-1]
#                                     
# #                                 elif move == 'f' and (rot.count('l') == 4) and (rot[3] == 'l'):
                        if tick == 4:
                            #xT = s
#                             Xr.append(xT)#*math.cos(math.radians(90+delta))) # -
#                             Yr.append(yT - 0)
                            
                            xT = s#*math.cos(math.radians(delta))
                            yT = 0#-s*math.sin(math.radians(0))
                            
                            #pose[0] += xT
#                             Xr.append(Xr[-1] + s*math.cos(math.radians(delta)))
#                             Yr.append(Yr[-1] - s*math.sin(math.radians(0)))
                            
                            #Record encoder states to txt file
                            outstringF = str(Xr[-1]+xT) + ' ' + str(Yr[-1]+yT) + '\n'
                            #print(outstringF)
                            f.write(outstringF)
                            
                            #xT = pose[0]
#
#elif (delta >= 90 and delta <= 270)
#                                 
#                                     Xr.append(x + s*math.cos(math.radians(delta))) #    
                            
#                                 if (delta < 360 and delta > 180):
                        #Yr.append(y - s*math.sin(math.radians(ang[-1]))) # -
#                                 elif (delta > 0 and delta <= 180):
                        
#                                 #if ((delX == 0) and (delta < 0)) and (abs(delY) > 0):
#                                 if (deltaX > 0 and ((delta > -90) or (delta < 90))):
#                                 
#                                     Xr.append(x + s*math.cos(math.radians(delta))) # -
#                                     Yr.append(y - s*math.sin(math.radians(delta))) # -
#                                     
#                                 elif ((deltaX == 0) and (delta < 0)) or (deltaY > 0 and delta < 0):
#                                     
#                                     Xr.append(x - s*math.cos(math.radians(delta))) # -
#                                     Yr.append(y - s*math.sin(math.radians(delta))) # -
#                                     
#                                 elif (deltaY ==0) and (delta <0):
#                                     
#                                     Xr.append(x + s*math.cos(math.radians(delta)))
#                                     Yr.append(y + s*math.sin(math.radians(delta)))
#                                     
#                                 elif (deltaY < 0 and delta > 0) or (deltaX < 0 and ((delta > -90) or (delta < 90))):
#                                     
#                                     Xr.append(x - s*math.cos(math.radians(delta)))
#                                     Yr.append(y + s*math.sin(math.radians(delta)))
                            
                        
#                                 x = Xr[-1]
#                                 y = Yr[-1]
                        
#                                 Xtot += x
#                                 Ytot += y
                        
                    else:
                        #rot.pop(0)
                        #rot.append(order[cnt])
#                         xT = Xr[-1]
#                         yT = Yr[-1]
#                         Xr.append(Xr[-1])
#                         Yr.append(Yr[-1])
                     
                        #Record encoder states to txt file
                        outstringF = str(Xr[-1]) + ' ' + str(Yr[-1]) + '\n'
                        #print(outstringF)
                        f.write(outstringF)
                
                #angle_diff = imuQuick(ser)
#                 print(distance(), "cm")       
#                 if (distance() < 15):
#                     for pwm in pwms:
#                         pwm.start(0)
#                     
#                     time.sleep(2)
#                     completed = True
                
#                 if yaw_diff < 0.4:
#                     completed = True
#                     angLast = ang[-1]
                    
                
                if (error >= -10 and error <= 10) and (direction in ['f','rev']):
                    
                    print("counterBR: ", counterBR, "counterFL: ", counterFL)
                    angle_diff = encoder2deg(pos_encoder) - yaw1
                    print("Angle rotated: ", angle_diff)
                    print("Expected turn: ", encoder_dist)
                    print(angle_diff)
            
                    Xr.append(xT)
                    Yr.append(yT)
            
                    pose = (Xr[-2]+Xr[-1],Yr[-2]+Yr[-1])
                    
                    print("current ordered pair: ", pose)
            
                    completed = True
                    
                    counterBR = 0
                    counterFL = 0
                    
                    angLast = ang[-1]
                    pwms = pwmInit(pwms)
                    
                    
                    for pwm in pwms:
                        pwm.start(0)
                    
                    time.sleep(2)
                    print(f"Drive {direction} action completed! \n")
                    #start = [xT,yT]
                    
                    tick += 2
                
                # Check if task is completed
                if (error >= 0 and error <= 4) and (direction in ['r','l']):
                    
                    print("counterBR: ", counterBR, "counterFL: ", counterFL)
                    #print(angle_diff)
                    angle_diff = encoder2deg(pos_encoder) - yaw1
                    print("Angle rotated: ", angle_diff)
                    print("Expected turn: ", encoder_dist)
                    completed = True
                    
                    counterBR = 0
                    counterFL = 0
                    s = 0
                    #angLast = ang[-1]
                    pwms = pwmInit(pwms)
                    
                    deltaX = xT - start[0]
                    deltaY = yT - start[0]
                    
                    for pwm in pwms:
                        pwm.start(0)
                    
#                     Xr.append(xT)
#                     Yr.append(yT)
            
                    pose = (Xr[-1],Yr[-1])
                    
                    time.sleep(2)
                    print(f"Drive {direction} action completed! \n")
                    #start = [xT,yT]
                    
                    tick -= 1
#                     # IMU Correction: Check if robot is in correct orientation
#                     while True:
#                         
#                         if direction in ['f','rev']:
#                             # Current angle
#                             yaw_diff = encoder2deg(error)
#                         else:
#                             yaw_diff = error
#                         
#                         #delta=  yaw_now - yaw1
#                         
#                         print("Initial angle: ", yaw1)
#                         print("The desired change yaw is: ", angLast)
#                         print("Needed yaw diff is: ", yaw_diff)
#                         print("Recorded encoder delta: ", delta)
#                         if yaw_now <= 360 and yaw_now > 180:
#                             yaw_now = -(360 - yaw_now)
#                             
#                         # diff between current yaw and original yaw
#                         diff1 = 0
#                         
#                         #mod = (diff1 % 360)
#                         
#                         if diff1 <= 360 and diff1 > 180:
#                             diff1 = -(360 - diff1)
#                         
#                         # diff between expected goal and current yaw
#                         #diff2 = goal - diff1
#                                     
#                         print("The angular differnce in yaw between current and goal state is: ", yaw_diff)
#                         
#                         if (yaw_diff >= 0 and yaw_diff <180):
#                             direction = 'l'
#                             diff1 = yaw_diff
#                         else:
#                             direction = 'r'
#                             diff1 = 360 - yaw_diff
#                     
#                         # Adjust robot
#                         if (diff1 >= 1.5):
#                             
#                             encoder_dist = rot2encoder(diff1)
#                             
#                             yaw = imuQuick(ser)
#                             
#                             # Global error between start and goal state will
#                             error = encoder_dist - 0
#                             
#                             error = yaw
#                             
#                             while (error >= 1.5):
#                                 # Command the robot to drive to user commanded postion
#                                 # in sequence
#                                 
#                                 # Compute the difference between the left and right encoders
#                                 #error_encoder = counterFL - counterBR
#                                 
#                                 pwms = drive2goal(direction, rot2encoder(error), duty, pwms)
#                                 
#                                 error = abs(yaw - imuQuick(ser))
#                                 
#                                 # Count encoders for left and right motors
#                                 if int(gpio.input(12)) != int(buttonBR):
#                                     buttonBR = int(gpio.input(12))
#                                     counterBR += 1
# 
#                                 if int(gpio.input(7)) != int(buttonFL):
#                                     buttonFL  = int(gpio.input(7))
#                                     counterFL += 1
#                                 
#                                 # Compute the difference in distance between start state  and goal state
#                                 # by considering the minimum of the two encoders
#                                 pos_encoder = min(counterFL,counterBR)
#                                 
#                                 # Global error between start and goal state will be
#                                 error = encoder_dist - pos_encoder
#                                 
#                             # Check if task is completed
#                             else:
#                                         
#                                 print("counterBR: ", counterBR, "counterFL: ", counterFL)                      
#                             
#                                 encoder_dist = 0
#                                 pos_encoder = 0
#                                 counterBR = 0
#                                 counterFL = 0
#                                 
#                                 pwms = pwmInit(pwms)
#                                 
#                                 for pwm in pwms:
#                                     pwm.start(0)
#                                 time.sleep(1)
#                                 print(f"Adjustment towards the {direction} direction completed!")
#                                 
#                             break
#                         else:
#                             break

        else:
            print("Destination Reached")
            gameover(pwms)
            #f.close()
            for pwm in pwms:
                pwm.stop()
            
            gpio.cleanup()
            f.close()
            
            # Create a 2D NumPy array
            arr = np.hstack((np.array(Xr).T, np.array(Yr).T))

            f_1.write(str(arr))
            f_1.close()

            # Save the array to a text file
            #np.savetxt('IMU_and_encoder_states.txt', arr)
            
        
    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        gameover(pwms)
        for pwm in pwms:
            pwm.stop()
        gpio.cleanup()
        f.close()
        print("Tasks Interrupted!")
        
        # Create a 2D NumPy array
        arr = np.hstack((np.array(Xr).T, np.array(Yr).T))
        
        f_1.write(str(arr))
        f_1.close()

        # Save the array to a text file
        #np.savetxt('IMU_and_encoder_states.txt', arr)
    
    
if __name__ == "__main__":
    
    # Begin Program
    print("*" * 30, "PROGRAM STARTERD","*"*30, "\n")
    main()
    
    # Plot result
    #dispPlot()
