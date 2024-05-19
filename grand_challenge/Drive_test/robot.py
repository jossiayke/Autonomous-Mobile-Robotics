# General Python packages
import math
import cv2
import os
import numpy as np
import matplotlib.pyplot as plot
import time
import RPi.GPIO as gpio
import serial
import logging
import threading
import queue

from typing import List
from random import randint

# This is the main interface script that will control the robot
# as it traverses in the challenge arena.

# Below are the necessary imports for its different features

# For visual perception tasks
from Perception import cntr_bbox


class Robot(object):
    """
    A class defining basic attributes of Baron robot that
    will be autonomously achieving objective
    
    Attributes:
        name (str): Name of robot
        _x (float): horizontal position of robot from start postion
        _y (float): vertical position of robot from start position
        _yaw (float): yaw orientation from +x as horizontal [0,180] or (-180,0)
        _travel(float): estimating the distance to drive and reach block
        _turn (float): keeps the next turn needed by block
        _depth (float): measures distance of obstacle in front of robot from image
        _path (list): stores the path traveled by robot
        _pwms (list): stores a list of pwms for the robot motors
        _dutycycle (dict): holds the baseline pwm dutycycle for forward,reverse,
                           pivotleft and pivotright
        _success (bool): keeps track whether a given task is successful or not
        _gripper_state (str): stores gripper duty cycle state as opened or closed
        _order(list): List of blocks to pickup
        _end_game (bool): tells competition is over by victory or failure
        _order_no (int): keeps track of number of orders completed
        _block_now (str): the block being pursued now
        _task_now (str): keeps track of the task being completed at the moment
        _actions (dict): holding list of all service actions robot performs with bool
                         on which process is being done at a given time
        _status (dict): a dictionary keeping track of completed actions along with
        _img_frame (img): saves current image frame
        _out (cv2 video obj): gather all frames into a video
        _logger (log): log statements for debuging and status over the terminal
        _kp (float): proportional gain
        _ki (float): integral gain
        _kd (float): derivative gain
        _direction (str): tells weather robot is moving forward or reverse
        _event (event): event object to end the thread call
        _pause_event (event): event object to pause and resume thread
        imu_reader (thread): threading object to read from imu during forward run
        _ser (serial): object reading serial input streams from imu sensor
        
    """
    
    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # Gripper States
    grip_state = dict([('closed',2.5),('opened',7.5)])
    # Matrix for Image frame
    image = np.empty((480*640*3,),dtype=np.uint8)
    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    
    # Experimentally found duty cycle values for left and right motor
    # in movements of the four cardinal directions, converted to  dictionary
#     dutyset = [('f', dict([('start',(50,50)), #35,40
#                     ('motion',dict([('lMotor',(35,50)), #40,50
#                                    ('rMotor',(50,40))])
#                     )]
#                 )),
#                 ('rev', dict([('start',(35,40)),
#                     ('motion',dict([('lMotor',(35,50)),
#                                    ('rMotor',(35,40))]) # 45,35
#                     )]
#                 )),
#                 ('l', (100,100)),
#                 ('r',(100,100))]
    
    dutyset = [('f', [80,80]), ('rev', [30,30]),('l', 90),('r',90)]

    duty = dict(dutyset)
    
    # turn angel
    turn = 15
    
    # Initialize FL and BR button count
    counterBR = np.uint64(0)
    counterFL = np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)
    
    # Proportional Control Gain
    Kp = 1
    Ki = 0.05
    Kd = 0.08
    
    # Important locations in map in units of feet
    start_pt = [1,1]
    clutter_env = [5,5]
    cnst_zone = [2,8]
    
    # create object to read camera
    video = cv2.VideoCapture(0)

    frame_center = (320,240)
    
    queue = queue.Queue()
    
    def __init__(self, name: str):
        """
        Initialize Robot attributes
        
        Args:
            name: str - takes name of robot
        
        Returns:
            None
        """

        self.name = name
        self._x = 0
        self._y = 0
        self._yaw = 0.0
        self._travel = 0
        self._turn = 0
        self._depth = 0
        self._path = []
        self._pwms = []
        self._dutycycle = self.duty
        self._success = False
        self._gripper_state = self.grip_state['closed']
        self._order = []
        self._end_game = False
        self._order_no = 0
        self._block_now = ''
        self._task_now = ''
        self._actions = dict([('driving',False),('rotating',False),('picking',False),('placing',False),('localizing',False),('scanning',False)])
        self._status = dict([('start',False),('picked_up',False),('found',False),('placed',False),('dropped',False)])
        self._img_frame = self.image
        self._out = cv2.VideoWriter('grand_chall_vid.avi', self.fourcc, 3, (640, 480))
        self._logger = logging.getLogger(name)
        self._kp = self.Kp
        self._ki = self.Ki
        self._kd = self.Kd
        self._direction = 'f'
        self._event = threading.Event()
        self._pause_event = threading.Event()
        self.imu_reader = threading.Thread(target = self._imu_serial_straight, args=(self.queue,))
        self._ser = serial.Serial('/dev/ttyUSB0', 19200)
        
        # Set up logger with Debug initially
        self._logger.setLevel(logging.DEBUG)
        # Create console handler and set level to debug
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        # Create formatter
        formatter = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        # Add formatter to handler
        console_handler.setFormatter(formatter)
        # Add console handler to logger
        self._logger.addHandler(console_handler)
        
        
        # Initialize gpios and pwms
        self._init()
        
        
        
        
    @property
    def name(self):
        return self._name
    
    @name.setter
    def name(self, name):
        self._name = name
        
    def _init(self):
        """
        Initialize gpio pins and set pwms to command DC motors, encoders and
        servo
        
        Args:
            None
        
        Returns:
            None
        """
        
        gpio.cleanup()
        gpio.setmode(gpio.BOARD)
        
        # Setup GPIO pin(s)
        gpio.setup(36, gpio.OUT) # Servo
        
        gpio.setup(31, gpio.OUT) # IN1
        gpio.setup(33, gpio.OUT) # IN2
        gpio.setup(35, gpio.OUT) # IN3
        gpio.setup(37, gpio.OUT) # IN4
        
        gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
        gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)
        
        self._pwms.clear()

        # initialize pwm signal to control motor
        pwm01 = gpio.PWM(31, 50)  # BackLeft motor
        pwm11 = gpio.PWM(33, 50) # FrontLeft motor
        pwm22 = gpio.PWM(35, 50) # FrontRight motor
        pwm02 = gpio.PWM(37, 50)  # BackRight motor
        pwmS = gpio.PWM(36, 50) # Servo

        self._pwms = [pwm01,pwm11,pwm22,pwm02,pwmS]
        
        for pwm in self._pwms:
            pwm.start(0)

    def _pwmZero(self):
        """
        Stops motors by zeroing pwm values in all pins
        controlling DC motors
        
        Args:
            None
        
        Returns:
            None
        """


        self._pwms[0].ChangeDutyCycle(0)
        self._pwms[1].ChangeDutyCycle(0)
        self._pwms[2].ChangeDutyCycle(0)
        self._pwms[3].ChangeDutyCycle(0)
        
    def _gameover(self):
        """
        Terminates run by stopping pwms and cleaning up
        gpio pins
        
        Args:
            None
        
        Returns:
            None
        """
    
        self._logger.info("Gameover")
        self._event.set()
        self._pwmZero()
        self._pwms[-1].ChangeDutyCycle(self.grip_state['closed'])
        time.sleep(1)
        for pwm in self._pwms:
            pwm.stop()
        gpio.cleanup()
        
    def _forward(self,vals):
        """
        Commands DC motors to drive robot forward by sending
        Dutycycles to respective pwm pins
        
        Args:
            None
        
        Returns:
            None
        """
        # Left wheels
        self._pwms[0].ChangeDutyCycle(vals[0])
        self._pwms[1].ChangeDutyCycle(0)
        # Right wheels
        self._pwms[2].ChangeDutyCycle(0)
        self._pwms[3].ChangeDutyCycle(vals[1])
        
    def _reverse(self,vals):
        """
        Commands DC motors to drive robot in reverses by sending
        Dutycycles to respective pwm pins
        
        Args:
            None
        
        Returns:
            None
        """
        # Left wheels
        self._pwms[0].ChangeDutyCycle(0)
        self._pwms[1].ChangeDutyCycle(vals[0])
        # Right wheels
        self._pwms[2].ChangeDutyCycle(vals[1])
        self._pwms[3].ChangeDutyCycle(0)

    def _pivotleft(self,vals):
        """
        Commands DC motors to pivot robot left by sending
        Dutycycles to respective pwm pins
        
        Args:
            None
        
        Returns:
            None
        """
        # Left wheels
        self._pwms[0].ChangeDutyCycle(0)
        self._pwms[1].ChangeDutyCycle(vals)
        # Right wheels
        self._pwms[2].ChangeDutyCycle(0)
        self._pwms[3].ChangeDutyCycle(vals)
        
    def _pivotright(self,vals):
        """
        Commands DC motors to pivot robot right by sending
        Dutycycles to respective pwm pins
        
        Args:
            None
        
        Returns:
            None
        """

        # Left wheels
        self._pwms[0].ChangeDutyCycle(vals)
        self._pwms[1].ChangeDutyCycle(0)
        # Right wheels
        self._pwms[2].ChangeDutyCycle(vals)
        self._pwms[3].ChangeDutyCycle(0)
        
    def _servo_cntrl(self,duty_cycle):
        """
        Commands servo motors to open and close gripper by sending
        Dutycycles to respective pwm pins
        
        Args:
            None
        
        Returns:
            None
        """
        
        self._pwms[-1].ChangeDutyCycle(duty_cycle)
        time.sleep(1)

        img = cntr_bbox.servo_img(str(duty_cycle), duty_cycle, self._distance())
        
        self._out.write(img)
        
    def _picam_frame(self):
        """
        Calls picamera object to take image frame from the pi camera
        and applies corrections by flipping to reflect reality
        
        Args:
            None
        
        Returns:
            None
        """
        # Take picture with camera
        cntr_bbox.camera.capture(self.image, format="bgr")
        image = self.image.reshape((480,640,3))

        self._img_frame = cv2.flip(image,1)
    
    def _mask_color(self, imageHSV):

        if self._block_now == 'green':
            # Trail Green Bock - LAB
            minHSV = np.array([46,88,121])
            maxHSV = np.array([65,141,255])
        elif self._block_now == 'red':
            # Trail Red block - LAB
            minHSV = np.array([151,107,147])
            maxHSV = np.array([255,255,255])
        elif self._block_now == 'blue':
            # Trail Blue block - LAB
            minHSV = np.array([72,94,97])
            maxHSV = np.array([126,188,212])

        maskHSV = cv2.inRange(imageHSV, minHSV, maxHSV)

        return maskHSV
    
    def _prep_image(self):
        """
        Processes images by applying HSV masking and gaussian blur
        for use by latter object detection functions
        
        Args:
            None
        
        Returns:
            None
        """
    
         # Convert image from BGR to HSV space
        imageHSV = cv2.cvtColor(self._img_frame,cv2.COLOR_BGR2HSV)

        # mask the green light from HSV and convert to grayscale
        mask = self._mask_color(imageHSV)

        # Mask HSV masked image of arrow
        blurred = cv2.GaussianBlur(mask,(11,11), 0)

        return blurred 
        
    def _grip_checker(self):
        """
        Processes images by applying a rectangular mask over desired
        region of interest to verify whether gripper has held toy block 
        
        Args:
            None
        
        Returns:
            self._image_frame (img): Masked image
        """
        self._logger.info("Inside Grip Checker Function")
        # Create a mask of gripped block in image
        # Save the shape of the image array
        (height, width, c)  = self._img_frame.shape

        # Create mask of image just like picture 
        mask = np.zeros_like(self._img_frame[:,:,0])

        # Create a white rectangle
        x,y,w,h = round(width/2)-130, round(height/2)-30, 250, 300
        cv2.rectangle(mask,  (x,y), (x+w, y+h), 255, -1)
        
        # Extract the image that aligns in the rectangle
        self._img_frame = cv2.bitwise_and(self._img_frame, self._img_frame, mask=mask)
        
        return self._img_frame
        
    def _wait2start(self):
        """
        Waits for the competition start QR code to be visible,
        and reads QR code in, which if corresponds to "ENPM701"
        returns a True boolean to begin competition
        
        Args:
            None
        
        Returns:
            start (bool): True/False based on detection of expected QR
        """
        # Define detector
        detector = cv2.QRCodeDetector()
        
        # Check if program start initiated
        start = False
        
        cnt = 0
        
        self._logger.info("Waiting for start cue . . . ")
        
        try:
        
            for frame in cntr_bbox.camera.capture_continuous(cntr_bbox.rawCapture, format="bgr", use_video_port=False):

                # grab the current frame
                img = frame.array

                img = cv2.flip(img,1)
                
                data, bbox, _ = detector.detectAndDecode(img)

                if(bbox is not None):
                    for i in range(len(bbox)):
                        cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
                        #            cv2.putText(img, data, (int(bbox[0][0][0]),int(bbox[0][0][1]-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

                if data:
                    self._logger.info(f"Data: {data}")
                        
                # Show resutls to the screen
                cv2.imshow("QR Code detector",img)
                key = cv2.waitKey(1) & 0xFF
            
                # write frame into file
                self._out.write(img)

                # clear the stream in preparation for the next frame
                cntr_bbox.rawCapture.truncate(0)
                
                # Break out of loop by pressing the q key
                if(key == ord("q")):
                    self._logger.warn("Program Terminated")
                    self.video.release()
                    cv2.destroyAllWindows()
                    break

                if data == "ENPM701":
                    self._logger.info("Cue Received \n")
                    self._logger.info('@' * 45)
                    self._logger.info("Starting Grand Challenge! \n")
                    self._logger.info('#' * 45)
                    start = True
                    self.video.release()
                    cv2.destroyAllWindows()
                    time.sleep(5)
                    break
            
        except Exception as error:
            self._logger.error(error)
            self.video.release()
            cv2.destroyAllWindows()
        
        return start
    
    def _meter2encoder(self,x_dist):
        """
        Reads in distance values in meters and converts to
        encoder counts
        
        Args:
            None
        
        Returns:
            encod (int): converted encoder counts from distance in m 
        """
        encod = round(float(x_dist / (2*math.pi*0.0325))*960)
        
        return encod

    def _cm2encoder(self,x_dist):
        """
        Reads in distance values in centimeters and converts to
        encoder counts
        
        Args:
            None
        
        Returns:
            encod (int): converted encoder counts from distance in cms 
        """
        dist = x_dist/100
        encod = self._meter2encoder(dist)
        return encod

    def _encoder2cm(self,encd):
        """
        Reads in encoder values and converts them to centimeters
        
        Args:
            None
        
        Returns:
            cms (float): converted encoder counts to distance in cms 
        """
        dist = round(float((encd / 960) * 2*math.pi*0.0325),1)
        cms = round(dist/100,2)

        return cms

    def _feet2encoder(self,x_dist):
        """
        Reads in distance values in feet and converts to
        encoder counts
        
        Args:
            None
        
        Returns:
            encod (int): converted encoder counts from distance in feet 
        """
        dist = round(float(x_dist * 12 * 0.0254))
        encod = meter2encoder(dist)
        return encod

    def _imu_serial(self):
        """
        Reads in serial robot's yaw angle value from imu sensor, cleans buffer,
        and converts it into a float data type. Convers reading from 0-360 to
        -180 < yaw < 180.
        
        Args:
            None
        
        Returns:
            self._yaw (float): robots yaw in degrees
        """
        self._ser.reset_input_buffer()
        self._ser.flush()
        
        while True:
            
            try:
                # Read for imu from serial
                if(self._ser.in_waiting > 0):
        #                 if cnt >5:
                    # Strip serial stream of extra characters
                    line = self._ser.readline()

                    line = line.rstrip().lstrip()

                    line = str(line)
                    line = line.strip("'")
                    line = line.strip("b'")

                    # Return float
                    line = float(line)
                    
                    if (line > 180 and line <=360):
                        line = line - 360
                        
                    self._yaw = line
                    return self._yaw
                
            except Exception as error:
                self._logger.error(f"Imu Error: {error}")
                
    def _imu_serial_straight(self,queue):
        """
        Reads in serial robot's yaw angle value from imu sensor, cleans buffer,
        and converts it into a float data type. Convers reading from 0-360 to
        -180 < yaw < 180.
        
        Only difference from self._imu_serial() is that this is run in a separate
        thread which is controlled by events to start, pause, and stop function call 
        
        Args:
            None
        
        Returns:
            self._yaw (float): robots yaw in degrees
        """
        
        while not self._event.is_set():
            
            self._ser.reset_input_buffer()
            self._ser.flush()
            
            try:
                while self._pause_event.is_set():
                    # Read for imu from serial
                    if(self._ser.in_waiting > 0):
                            line = self._ser.readline()

                            line = line.rstrip().lstrip()

                            line = str(line)
                            line = line.strip("'")
                            line = line.strip("b'")

                            # Return float
                            line = float(line)
                            
                            if (line > 180 and line <=360):
                                line = line - 360
                                
                            queue.put(line)

            except Exception as error:
                self._logger.warning(f"Imu Error inside thread: {error}")
    
    def _distance(self):
        """
        Reads in serial robot's distance from an obstacle using ultrasonic sensor by
        sending a trigger pulse and listening the returning echo to compuse the separation
        distance from nearby object using triangulation technique. Then it saves computed
        distance in self._depth
        
        Args:
            None
        
        Returns:
            None
        """
        pulse_start = 0
        pulse_end = 0
        
        # Define pin allocations
        trig = 16
        echo = 18
        # Setup GPIO board & pins
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
        self._depth = round(distance, 2)
    
    def _rotate(self,deg_diff):
        """
        Reads in direction of rotation requested, and starts
        rotating the motors accordingly to achieve desired orientation.
        
        
        Args:
            None
        
        Returns:
            None
        """

        
        if self._direction == 'r':
            self._pivotright(60) 
        elif self._direction == 'l':
            self._pivotleft(60)
            
    def _dist_estimate(self):
        """
        Reads in radius of bounding box from bbox detector function, and 
        plugs radius into precomputed correlation function to estimate the
        depth of toy block from robot.
        
        Assigns results to self._travel and saves image showing value of distance
        appended to it in self._img_frame and writes frame to video object self._out
        
        Args:
            None
        
        Returns:
            None
        """
        # Take image with picamera
        self._picam_frame()
        self._img_frame, __, bbox_radius, self._success = cntr_bbox.bbox_detect(self._img_frame, self._prep_image())

        if self._success not in [None, False]:
            # Plug in pixel size into depth equation
            distance = (0.0099 * bbox_radius**2) - (1.8846*bbox_radius) + 103.47
            self._travel = round(distance,2)
            # Append image of estimated distance on image frame
            self._img_frame = cntr_bbox.dist_img(self._img_frame,self._travel)
            # save image into video object
            self._out.write(self._img_frame)
            
            
    def _bbox_detect(self):
        """
        Reads in an image, and then applies preprocessing functions that mask and
        blur image. Then, attempts to detect corners of an object in frame, and if 
        successful it will run further computation to determine the center and radius
        of detected object. Finally adds the draws the bounding circle and center point
        as well as write computed values on original image
        
        Assigns results to self._travel and saves image showing value of distance
        appended to it in self._img_frame and writes frame to video object self._out
        
        Args:
            image (image): Original colored image taken in from camera
        
        Returns:
            bbox_radius (float): Radius of bounding circle detecting object
            len(pts_loc) (int): The number of corner points identified on object from frame
        """
        # Initialize list to save corner points and bbox radius
        bbox_radius = 0
        pts_loc = []
        
        # Take image using camera frame
        self._picam_frame()
        
        # Prepare image by masking and blurring
        image_blurred = self._prep_image()
        
        # Detect corners from image
#         __, pts_loc, __, __= cntr_bbox.corner_detect(image_blurred, self._img_frame)
        # Extract bbox radius from image
        self._img_frame, center, bbox_radius, self._success = cntr_bbox.bbox_detect(image_blurred, self._img_frame)
         
        self._logger.info(f"Radius {bbox_radius}")
        self._logger.info(f"Number of corners/edges detected are {len(pts_loc)}")
        
        return bbox_radius, center #, len(pts_loc),
    
    def _pivot(self,deg_diff,steady = False):
        """
        This function pivots robot to deseired angle

        Args:
            deg_diff (float): Amount of angular rotation to turn.
                      Default value is self._turn parameter
            
        Returns:
            None
            
        """
        self._logger.info("Inside pivot function")
        
        # To know the direction of turn
        if deg_diff >= 0:
            self._direction = 'r'
        else:
            self._direction = 'l'
        
        # Initialize change in time for derivative control
        zones = [0.3, 0.8, 1]
        
        # Resume threading to record yaw
#         self._pause_event.set()
#         time.sleep(1)
#         self._yaw = self.queue.get()
        yaw_start = self._imu_serial()
        # Angle diff initialized
        yaw_diff = deg_diff
#         self._yaw = self._yaw + deg_diff
        yaw_final = yaw_start + deg_diff
        
        yaw_diff_old = yaw_final - yaw_start
        self._logger.info(f"Needed final yaw is {yaw_final} degrees")
        yaw_current = yaw_start
        # Lower and upper bounds of dutycycle initialized
        lower_dc = 65
        upper_dc = 90
        # Divide the distance traveled into three zones
        trip = [x * yaw_diff for x in zones]
        # Variables for PID control
        derivative = 0
        integral = 0

        
        start = time.time()
        while True:
            try:
                yaw_diff = yaw_final - self._imu_serial()
                prcnt = 1 - yaw_diff/deg_diff
#                 self._logger.info(f"Percent encoder diff: {prcnt}")
                # PID Controller for overall travel of robot              
#                 derivative = yaw_diff - yaw_diff_old
#                 integral += yaw_diff
# 
#                 output = yaw_diff * self._kp + (derivative * self._kd) + integral * self._ki
#                 yaw_current += output
#                 prcnt = yaw_current / yaw_final
#                 elif self._imu_serial() > yaw_start+trip[2]:
#                 self._dutycycle[self._direction] = z3
                
                if prcnt >= zones[0] and prcnt <= zones[1]:
                    self._dutycycle[self._direction] = self.duty[self._direction] - 25
                elif prcnt > zones[1]:
                    self._dutycycle[self._direction] = self.duty[self._direction] - 5
                    
#                 self._dutycycle[self._direction] -= prcnt * self.duty[self._direction]              
                self._dutycycle[self._direction] = np.clip(self._dutycycle[self._direction], lower_dc, upper_dc)
    #                 if abs(yaw_diff) < 5:
    #                     self._dutycycle[self._direction] = 60
                
                if (abs(yaw_diff) >= 0.6 and (abs(yaw_diff-yaw_diff_old) < 5)):
    #                     self._logger.info(f"Current yaw {self.queue.get()}; Angle turning towards {yaw_final}; Error yaw {yaw_diff}")
                    self._logger.info(f"Current yaw {self._imu_serial()}; Angle turning towards {yaw_final}; Error yaw {yaw_diff}")
    #                     self._rotate()
                    if not steady:
                        if self._direction == 'r':
                            self._pivotright(self._dutycycle[self._direction]) 
                        elif self._direction == 'l':
                            self._pivotleft(self._dutycycle[self._direction])
                    else:                        
                        if self._direction == 'r':
                            self._pivotright(65) 
                        elif self._direction == 'l':
                            self._pivotleft(65)
                    
                else:
                    self._pwmZero()
                    total_turn = yaw_start - self._imu_serial()#self.queue.get() 
                    self._logger.info(f"Current yaw {self._imu_serial()}, Total turn {total_turn}, Final Angle diff {yaw_diff}")
                    self._logger.info("Robot turn done")
                    
                    if not steady:
                        # Check for overshooting
                        deg_diff = self._scan4object()
                        time.sleep(1)
                        
                        if abs(deg_diff) >= 0.6:
                            
                            # Return pwm dutcycle back to base
                            self._dutycycle[self._direction] = lower_dc
                            
                            if abs(deg_diff) >= 0.6:
                                
                                self._pivot(deg_diff, steady=True)
                    else:
                        break
                    
                    
                    self.counterFL = 0
                    self.counterBR = 0

                    # Return pwm dutcycle back to base
                    self._dutycycle[self._direction] = self.duty[self._direction]
                    
                    # Save current orientation
                    self._imu_serial()

                    self._logger.info("Final orientation reached")
                    break
            
            except KeyboardInterrupt:
                self._logger.error("Pivot Interrupted")
                self._gameover()
            except Exception as error:
                self._logger.error(f"Error {error}")
                
        #             # Pause reading yaw thread
        #             self._pause_event.clear()
        self._pwmZero()
        self._logger.info("Pivot Finished")
      
    def _go2block(self, direction='f',steady=False):
        """
        This function drives the robot in a straight path, either forward
        or reverse

        Args:
            direction: Direction of drive, forward or reverse
            
        Returns:
            None
            
        """
        self._logger.info("Going toward block (go2block)")
        
        # Divide the separation distance into three zones
        # First zone is 70% into drive
        # Second zone is 20%
        # Third zone is the last 10%
        # Each of the zones have their own dutycycle. Robot will
        # drive and stop at each zone, scan, align, and resume drive
        zones = [0.4, 0.7, 1]
        self._success = False
        # Use image from camera to estimate distance
        self._dist_estimate()
        # Check if driving slowly
        if not steady:
            # convert depth to encoder count
            orig_dist = self._cm2encoder(self._travel-20)
        else:
            # convert depth to encoder count
            orig_dist = self._cm2encoder(self._travel)
        self._logger.info(f"Driving {self._travel} forward")
        pos_encoder_orig = 0
        pos_encoder = 0
        orig_error = orig_dist
        
        self.counterFL = 0
        self.counterBR = 0
        
        # Defining the limit of dutycycle
        lower_dc = 70
        upper_dc = 100
        
        z1 = upper_dc
        z2 = z1 - 20
        z3 = z2 - 10
        
        # Divide up the path in different zones
        trip = [x * orig_dist for x in zones]
        
        # Determine if function completed successfully
        self._success = False
            
#         # Start reading from imu
#         self._pause_event.set()
        time.sleep(1)
        yaw0 = self._imu_serial()
                
        # For encoder level control
        integral_error = 0
        derivative_error = 0
        error_encoder0 = 0
        error_encoder = 0
        
        # For achieving overall distance
        integral = 0
        derivative = 0
        error0 = 0
        error = 0
        startD = 0
        
        cnt = 0
        # Assign direciton of drive from argument
        self._direction = direction
        # Compute the difference in distance between start state  and goal state        
        self._logger.info("Sending command toward thrusters")
        
        while not self._success:
#             # Use image from camera to estimate distance
#             self._dist_estimate()
#             # convert depth to encoder count
#             encoder_dist = self._cm2encoder(self._travel)
            # Start recording time for PID control
            start = time.time()                
                                
            # by considering the minimum of the two encoders
            pos_encoder = int(min(self.counterFL,self.counterBR))
            # Global error between start and goal state will be
            error = orig_dist - pos_encoder  
            prcnt = 1 - error/orig_error
            
            self._logger.info(f"Percent encoder diff: {prcnt}")
             
            if prcnt < zones[0] and self._direction == 'f' and not steady:
                if cnt == 0:
                    lower_dc = 70
                    upper_dc = 100
                    
                    error = orig_dist - pos_encoder
                    pos_encoder_orig = pos_encoder
                    
                    self.duty[self._direction] = [80,80]#prcnt * self.duty[self._direction]
                    self._dutycycle[self._direction] = self.duty[self._direction]
                    cnt += 1
                
#                     # Set pwms to zero
#                     self._pwmZero()
                
#                     # Take image and save to video object
#                     self._picam_frame()
#                     cntr_bbox.img_show("Detecting Block",self._img_frame)
#                     self._out.write(self._img_frame)
                
            elif (prcnt >= zones[0] and prcnt < zones[1]) and self._direction == 'f' and not steady:
                if cnt == 1:
                    lower_dc = 40
                    upper_dc = 65
                    
                    pos_encoder_orig += pos_encoder
                    #error = orig_dist - pos_encoder
                    
                    self.duty[self._direction] = [60,60]#prcnt * self._dutycycle[self._direction]
                    self._dutycycle[self._direction] = self.duty[self._direction]
                    cnt += 1
#                     # Take image and save to video object
#                     self._picam_frame()
#                     cntr_bbox.img_show("Detecting Block",self._img_frame)
#                     self._out.write(self._img_frame)
                
#                     # Set pwms to zero
#                     self._pwmZero()
                # Detect (and orient if need be) block again and drive forward
#                 # with newly computed distance
#                 self._scan4object()
#                 time.sleep(1)
#                 self._pivot(self._turn)
                
                # Take image and save to video object
                
#                     cntr_bbox.img_show("Detecting Block",self._img_frame)
#                     self._out.write(self._img_frame)
#                     break
                
            elif prcnt >= zones[1] or steady:
                if cnt == 2 or steady:
                    lower_dc = 22
                    upper_dc = 28
                    
                    pos_encoder_orig += pos_encoder
                    #error = orig_dist - pos_encoder
                    
                    self.duty[self._direction] = [26,26] #prcnt * self._dutycycle[self._direction]
                    self._dutycycle[self._direction] = self.duty[self._direction]
                    cnt += 1
#                     # Take image and save to video object
#                     self._picam_frame()
#                     cntr_bbox.img_show("Detecting Block",self._img_frame)
#                     self._out.write(self._img_frame)
                
#                     # Set pwms to zero
#                     self._pwmZero()
#                     time.sleep(1)
            
            # Send forward thruster commands
            #arrived = drive_robot.cmd_motors('f',self._pwms, encoder_dist, self._turn, steady=steady)
            # Count encoders for left and right motors
            if int(gpio.input(12)) != int(self.buttonBR):
                self.buttonBR = int(gpio.input(12))
                self.counterBR += 1

            if int(gpio.input(7)) != int(self.buttonFL):
                self.buttonFL  = int(gpio.input(7))
                self.counterFL += 1

#             # PD Controller for encoder balance
            error_encoder = self.counterFL - self.counterBR
#             derivative_error = error_encoder - error_encoder0
#             error_encoder0 = error_encoder
# #             
#             # PID Controller for overall travel of robot              
#             derivative = error_encoder - error_encoder0
#             integral += error_encoder
#             
#             output = error_encoder * self._kp + (derivative * self._kd) # + integral * self._ki
#             pos_encoder += output
#             prcnt = pos_encoder / int(max(self.counterFL,self.counterBR))
            
            if error_encoder > 0:
                # Give power to corresponding motors
                self._dutycycle[self._direction][0] -=  0.1 * self.duty[self._direction][0]
                self._dutycycle[self._direction][1] += 0.1 * self.duty[self._direction][1]
            else:
                # Give power to corresponding motors
                self._dutycycle[self._direction][0] += 0.1 * self.duty[self._direction][0]
                self._dutycycle[self._direction][1] -=  0.1 * self.duty[self._direction][1]
            
            self._dutycycle[self._direction] = [np.clip(self._dutycycle[self._direction][0], lower_dc, upper_dc),
                                                np.clip(self._dutycycle[self._direction][1], lower_dc, upper_dc)]
            
            self._logger.info(f"left pwm dc {self._dutycycle[self._direction][0]}, right pwm dc {self._dutycycle[self._direction][0]}")
            
#             # PID Controller for overall travel of robot              
#             derivative = error - error0
#             integral += error
#             error0 = error
#             
#             output = error * self._kp + (derivative * self._kd) + integral * self._ki
#             pos_encoder_orig += output
#             prcnt = pos_encoder_orig / orig_dist
#             
#             # Give power to corresponding motors
#             self._dutycycle[self._direction][0] -=  prcnt * self.duty[self._direction][0]
#             # Give power to corresponding motors
#             self._dutycycle[self._direction][1] -=  prcnt * self.duty[self._direction][0]
#             
#             self._dutycycle[self._direction] = [np.clip(self._dutycycle[self._direction][0], lower_dc, upper_dc),
#                                                 np.clip(self._dutycycle[self._direction][1], lower_dc, upper_dc)]

            if self._direction == 'f':
                # Drive robot towards direction
                self._forward(tuple(self._dutycycle[self._direction]))
            elif self._direction == 'rev':
                # Drive robot towards direction
                self._reverse(tuple(self._dutycycle[self._direction]))    
            
            
            if (error <= 20) and (direction in ['f','rev']):
                angle_diff = yaw0 - self._imu_serial()
                self._logger.info(f"counterBR: {self.counterBR} counterFL: {self.counterFL}")
                self._logger.info(f"Angle shifted: {angle_diff} deg ")
                self._logger.info(f"Estimated distance traveled: {self._travel} cms")
                self.counterBR = 0
                self.counterFL = 0
                self._logger.info("Arrived in front of block")
                self._distance()
                self._pwmZero()
                self._success = True
#                     self._pause_event.clear()
                
                # Take image and save to video object
                self._picam_frame()
                cntr_bbox.img_show("Detecting Block",self._img_frame)
                self._out.write(self._img_frame)
                
                break
                                
                
#                 delT = time.time() - start
#                 start = time.time()
        
#             angle_diff = yaw0 - self._imu_serial()
#             if abs(angle_diff) > 3:
#                 # Use PD controller to adjust orientation
#                 self._logger.info(f"Angle diff is {angle_diff}")
#                 self._logger.info("Adjusting to face block front")
#                 self._pwmZero()
#                 time.sleep(1)
#                 self._pivot(angle_diff)
#                 
#                 yaw0 = self._yaw
#                 break
        
        # Return pwm dutcycle back to base
        self._dutycycle[self._direction] = self.duty[self._direction]
#         self._pause_event.clear()
                    
#                 if (abs(angle_diff) > 3 and (abs(angle_diff) <10)):
#                     print(f"Angle diff is {angle_diff}")
#                     print("Adjusting to face block front")
#                     self._pwmZero()
# 
#                     self._pivot(deg_diff=angle_diff)
#                     time.sleep(1)
#                     # Access object distance from frame
#                     self._dist_estimate()
#                     cntr_bbox.img_show('Block',self._img_frame)
#                     encoder_dist = self._cm2encoder(self._travel)
#                     #align, yaw_diff = alignRobot(angle_diff)
#                     counterBR = 0
#                     counterFL = 0
#                     yaw0 = self._imu_serial()

    def _scan4object(self):
        """
        This function scans for block in environment by turning in place
        until it finds the block. If it doesn't find the block after full 360
        degree turn, it returns false to say not found
        
        Args:
            None
            
        Returns:
            None
        """
        
        self._task_now = 'scanning'
        self._actions[self._task_now] = True
        self._logger.info("Scanning for %s block started", self._block_now)
        # To keep track of pivot
        self._yaw = 0
        while True: #self._status['found']
            # Detect object in frame
            #bbox_radius, num_corners, ave_center = self._bbox_detect()
            self._picam_frame()
            self._img_frame, ave_center, bbox_radius, success = cntr_bbox.bbox_detect(self._img_frame, self._prep_image())
            
            if success not in [None,False]:
                # Object found
                diff = ave_center[0]-self.frame_center[0]
                # Convert the pixel difference to degrees
                self._turn = diff * cntr_bbox.pixel2deg
                self._success = True
                self._status['found'] = True
                self._actions[self._task_now] = False
                return self._turn
                
            else:
                # Pivot the robot to a predefined turn angle
                self._pivot(self.turn)
                # Read the distance from obstacle for reference
                self._distance()
                # Keep track of turn
                self._yaw += self.turn
                
            if self._yaw >= 360:
                self._status['found'] = False
                self._success = False
                self._logger.warning("No objects in nearby vicinity")
                return None                
    
    def _pick_up(self):
        """
        This function drives the robot to steadily approach and picks up toy block
        
        Args:
            None
            
        Returns:
            None
        """
        self._success = False
        self._task_now = 'picking'
        self._actions[self._task_now] = True
        
        # Take image and save to video object
        self._picam_frame()
        cntr_bbox.img_show("Detecting Block",self._img_frame)
        self._out.write(self._img_frame)
        
        while not self._success:
            # Open gripper
            self._servo_cntrl(self.grip_state['opened'])
            self._gripper_state = 'opened' 
            self._logger.info("Servo Opened")
            # Take image and save to video object
            self._picam_frame()
            cntr_bbox.img_show("Detecting Block",self._img_frame)
            self._out.write(self._img_frame)
            # Align robot fine turne rotation
            degree = self._scan4object()
            self._pivot(degree, steady=True)
            # Scan block and drive slowly towards it
            self._logger.info("Driving slowly toward block")
            self._go2block(steady=True)
            self._logger.info("Slow drive completed")
            
            # Take image and save to video object
            self._picam_frame()
            cntr_bbox.img_show("Detecting Block",self._img_frame)
            self._out.write(self._img_frame)
            
            # Close gripper
            self._servo_cntrl(self.grip_state['closed'])
            self._gripper_state = 'closed'
            self._logger.info("Servo Closed")
            
            # Take image and save to video object
            self._picam_frame()
            cntr_bbox.img_show("Detecting Block",self._img_frame)
            self._out.write(self._img_frame)
            
            # Check if block is gripped by taking a picture, pivoting for few degrees
            # and taking a second picture, then performing bbox detection on both
            # Take first image and mask
#             img1 = self._grip_checker()
#             # save image into video object
#             self._out.write(img1)
#             # Now run bbox detector on image and then compare that the radius of
#             # bounding box and number of detected corners are within acceptable range
#             # with second image taken below
#             radius1, center1 = self._bbox_detect()
#             # Pivot for small angle, say 5deg
#             self._pivot(10,steady=True)
#             # Take second image and mask
#             self._picam_frame()
#             # save image into video object
#             self._out.write(self._img_frame)
#             img2 = self._grip_checker()
#             # save image into video object
#             self._out.write(img2)
#             # Doing the same for second image as above
#             radius2, center2 = self._bbox_detect()
            
            #if (abs(radius2 - radius1) < 5) or (abs(center2[0] - center1[0]) < 3):
            self._logger.info(f"The {self._block_now} block has been gripped successfully")
            self._success = True
            self._status['picked_up'] = True
            self._actions[self._task_now] = False
            
                # Send an email notification here for successful grip
                
#             else:
#                 self._logger.warning("Pick up task not successful")
#                 self._success = False
#                 break

    def _go2zone(self):
        """
        This function computes the difference in pose between the robot and the construction
        zone, then estimates the displacement vector along with the angle of rotation needed
        to align the robot head  on. It then calls the pivot and go2block functions respectively
        and command the robot to enter the construction zone.
        
        Args:
            None
            
        Returns:
            None
        """
        
        # Open gripper
        self._servo_cntrl(self.grip_state['opened'])
        self._gripper_state = 'opened' 
        self._logger.info("Servo Opened")
        # Drive in reverse
        self._go2block(direction='rev')
        

    def start(self, order: List):
        """
        The sequence of actions occuring while baron robot works to pick and place
        all toy blocks to the construction zone. This funciton lists the steps needed
        for the robot to complete the orders passed to it at the beginning of competition
        
        Args:
            order: List[Blocks] list of all toy blocks needed to complete challenge
        
        Returns:
            None
        """
        
        self._order = order
        try:
            # First wait until startup QR Code has been read
#             while True:
#                 if self._wait2start():
#                     break
            
            while (not self._end_game):
                """While Robot is stil active, resume challenge"""
                while len(self._order) > 0:

                    self._block_now = self._order.pop(0)
                    self._order_no +=1

                    # Begin challenge with choesn block
                    self._logger.info("-" * 20)
                    self._logger.info(f"*** Picking Up {self._block_now} block")
                    self._logger.info("-" * 20)
                
                    # Begin by scanning for toy block object
                    degree = self._scan4object()
                    
                    # Take image and save to video object
                    cntr_bbox.img_show("Detecting Block",self._img_frame)
                    self._out.write(self._img_frame)
                    
                    # Drive toward block if successful, else pivot to
                    # a free space and drive
                    if self._success:
                        self._logger.info(f"Pivoting Robot towards {self._block_now} block")
                        self._pivot(degree)
                        self._picam_frame()
                        # Take image and save to video object
                        __, __, __,__= cntr_bbox.bbox_detect(self._img_frame, self._prep_image())
                        cntr_bbox.img_show("Detecting Block",self._img_frame)
                        self._out.write(self._img_frame)
                        
                        self._logger.info(f"Drive Robot forward to {self._block_now} block")
                        self._go2block()
                        
                        # Take image and save to video object
                        degree = self._scan4object()
                        cntr_bbox.img_show("Arrived at Block",self._img_frame)
                        self._out.write(self._img_frame)
                        
                        self._logger.info(f"Pick up {self._block_now} block with gripper")
                        self._pick_up()
                        
                        self._logger.info(f"Take {self._block_now} block to construction zone")
                        self._go2zone()
                        
                        self._logger.info(f"Place {self._block_now} block inside construction zone")
                        self._place()
                        
                        self._logger.info(f"Localize and then Return to home location to start next task")
                        self._go_home()
                        self._logger.info(f"Pick up and place the next order in the queue")
                    else:
                        break
#             # Stop thread at the end of run
#             self._event.set()
        
        except KeyboardInterrupt as error:
            self._logger.error("Error occurred: %s",error)
#             # Stop thread at the end of run
#             self._event.clear()
            time.sleep(1)
            return None
        except ValueError as error:
            self._logger.error("Error %s", error, exc_info=True)
#             # Stop thread at the end of run
#             self._event.clear()
            time.sleep(1)
            return None
        