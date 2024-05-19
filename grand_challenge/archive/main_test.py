# General Python packages
import math
import cv2
import os
import numpy as np
import matplotlib.pyplot as plot
import time
import RPi.GPIO as gpio

# This is the main interface script that will control the robot
# as it traverses in the challenge arena.

# Below are the necessary imports for its different features

# For visual perception tasks
from Perception import qr_det, cntr_bbox

# For Locomotion tasks
from Locomotion import drive_robot

# For Localization tasks
# from Localization

# For Planning and Navigation tasks
# from Planning_Navigation

# General Global variables
# Open .txt file to save data
f = open('pxlradius_distance-02.txt','w')

image = np.empty((480*640*3,),dtype=np.uint8)

detected = False
deg_diff = 0

obj = ''

sodar_dist = 0
turn = 15
yaw = 0

def orient():
    global obj
    global sodar_dist
    global turn
    global yaw
    
    print("Inside Orient function")
    total_turn = 0
    yaw = 0
    detected, deg_diff, image = cntr_bbox.scanObject(obj)

    # show the frame to our screen
    cntr_bbox.img_show(f'{obj} Block',image)

    if detected:
        # Orient robot and align towards block
        print(f'Turn toward {obj} block')
        yaw = drive_robot.pivot(deg_diff=deg_diff)
        #align, yaw_diff = drive_robot.alignRobot(deg_diff)
        time.sleep(1)
#         cntr_bbox.img_show('Block',cntr_bbox.picam_frame())
        sodar_dist = drive_robot.distance()
        print("Turn Complete")
        return True
    else:
        print("Searching for block")
        
        while True:
            sodar_dist = drive_robot.distance()
            # Orient robot and align towards block
            yaw = drive_robot.pivot(deg_diff=turn)
            if sodar_dist < 40:
                total_turn += turn
                # Orient robot and align towards block
                yaw = drive_robot.pivot(deg_diff=turn)
                sodar_dist = drive_robot.distance()
            else:
                total_turn += turn
                cntr_bbox.img_show('Block',image)
                                                   
                detected, deg_diff, image = cntr_bbox.scanObject(obj)
                
                if detected:
                    # Orient robot and align towards block
                    print('Aligning robot to block')
                    yaw = drive_robot.pivot(block_center=cntr_bbox.ave_center,deg_diff=deg_diff,pixel=True)
                    time.sleep(1)
                    cntr_bbox.img_show('Block',image)
                    sodar_dist = drive_robot.distance()
                    
                    return True
                
                elif total_turn >= 360:
                    drive_robot.pwmZero(drive_robot.pwms)
                    # Drive to a different location in the arena
                    print("Driving to a different location for a better look")    
                    return False

def go2block(check=True, steady=False):
    global sodar_dist
    global turn
    global yaw
    
    print("Inside Go to block function")
    
    total_turn = 0
    # Store ultrasonic sensor value
    sodar_dist = drive_robot.distance()
    encoder_dist = 0

    while True:
        if check:
            # Driving towards identified block
            if not steady:
                print("Regular: Estimating object distance via image")
                # Access object distance from frame
                drive_dist, image = cntr_bbox.dist_estimate()
                # Image with distance estimate
                cntr_bbox.img_show('Block Dist Est.',image)
                # convert linear distance to encoder count
                encoder_dist = drive_robot.cm2encoder(drive_dist)
                yaw = drive_robot.imu_serial()
                print("Driving forward regular speed")
            else:
                print("Steady: Estimating object distance via sodar")
                sodar_dist = drive_robot.distance()
                yaw = drive_robot.imu_serial()
                # convert linear distance to encoder count
                encoder_dist = drive_robot.cm2encoder(sodar_dist)
                print("Driving forward slow speed")
        else:
            while True:
                print("Working on Relocating...")
                if sodar_dist > 75:
                    # Command motors to drive forward to block
                    encoder_dist = drive_robot.feet2encoder(drive_robot.search_drive)
                    break
                else:    
                    # Orient robot to a fixed angle
                    yaw = drive_robot.pivot(deg_diff=turn)
                    time.sleep(1)
                    total_turn += turn
                    if total_turn >= 360:
                        turn /= 2
        
        # Send forward thruster commands
        arrived = drive_robot.cmd_motors('f',drive_robot.pwms, encoder_dist, deg_diff, steady=steady)
#         arrived = drive_robot.cmd_motors('f',drive_robot.pwms, encoder_dist, yaw, steady=steady)
        time.sleep(1)
        
        if not arrived:
            print("Reattempting Drive forward Function")
            total_turn = 0
            check = True
            continue
        print("Go to block task done")
        break
    
    return arrived

def pick_block():
    global sodar_dist
    global detected
    global deg_diff
    grip_open = False
    triple_checker = [0,0,0]
    cnt = 0
    success = False
    print("Inside pick block function")
#     # Scan and orient for reassurance
#     success = orient()
#     print("Oriented again for redundancy")
    while not success:
        #if (block_dist < 30 or drive_robot.distance() < 30) and not grip:
        # Open gripper
        drive_robot.servo_cntrl(drive_robot.opened, drive_robot.pwms[-1])
        time.sleep(1)
        grip_open = True
#         # Align again
#         yaw = drive_robot.pivot(block_center=cntr_bbox.ave_center,deg_diff=deg_diff,pixel=True)
        # Estimate separation distance from block
        block_dist, image = cntr_bbox.dist_estimate()
        cntr_bbox.img_show('Block Dist Est.',image)
        sodar_dist = drive_robot.distance()
        # Drive steadily
#         arrived = go2block(grip_open,steady=True)
        while True:
            arrived = drive_robot.cmd_motors('f',drive_robot.pwms, drive_robot.cm2encoder(block_dist), 0, steady=True)
            sodar_dist = drive_robot.distance()
            time.sleep(1)
            if (sodar_dist <= 15) or (block_dist <= 15):
                print("Closing gripper")
                # Checking with bbox of pixel
                detected, deg_diff, image = cntr_bbox.scanObject(obj)
                radius0 = cntr_bbox.bbox_radius
                # Open gripper
                drive_robot.servo_cntrl(drive_robot.closed, drive_robot.pwms[-1])
                # Pivot 2 degrees and check Sodar again
                yaw = drive_robot.pivot(deg_diff=2)
                time.sleep(1)
                # Check bbox radius again
                detected, deg_diff, image = cntr_bbox.scanObject(obj)
                radius1 = cntr_bbox.bbox_radius
                # Check pixel depth distance again
                drive_dist, image = cntr_bbox.dist_estimate()
                # Check sodar again
                sodar_dist = drive_robot.distance()
                # Take difference of reading and verify block is gripped
                if (abs(radius1 - radius0) < 2) or (drive_dist < 20) or (sodar_dist < 20):                    
                    print(f"{obj} block is gripped")
                    return True
                else:
                    print("Not certain if block is gripped. Do a reverse and restart pickup task")
                    print("Driving in reverse for few cms and restarting pickup process")
                    encoder_dist = drive_robot.cm2encoder(15)
                    arrived = drive_robot.cmd_motors('rev',drive_robot.pwms, encoder_dist, steady=True)
                    # Checking with bbox of pixel
                    detected, deg_diff, image = cntr_bbox.scanObject(obj)
                    cnt += 1
                    return False
            else:
                print("Need to get a bit close")
                cnt += 1
                
            if cnt > 2:
                print("Tried picking up block twice unsuccessfully.")
                print("Going to nearby border to localize")
                break
        print("exit out of function not gripped")
         # to     
#         else:
#             # Drive closer to block
#             sodar_dist, arrived = go2block(success,steady=True)
            
        __, __, __, success = cntr_bbox.bbox_detect()
        
    else:
        print("Block not certainly gripped. Relocalize")
        return False
    
    return True    

def main():
    global obj
    print("Inside Main Fn Block")
    block = ['red','green','blue']

    try:

        while (len(block) > 0):
            
            obj = block.pop(0)
            
            while True:
                success = orient()
                if not success:
                    break
                print("Orient Function Done")
                
                arrived = go2block()
                if not arrived:
                    break
                print("Go 2 block Function Done")
                
                print("Ready to open gripper")
                picked_up = pick_block()
                if not picked_up:
                    break
            
#             detected, deg_diff, image = cntr_bbox.scanObject(obj)
#             
#             # show the frame to our screen
#             cntr_bbox.img_show('Block',image)
#             
#             if detected:
#                 # Orient robot and align towards block
#                 print('Turn toward block')
#                 yaw = drive_robot.pivot(deg_diff=deg_diff)
#             
#             else:
#                 turn = 15
#                 while True:
#                     # Orient robot and align towards block
#                     yaw = drive_robot.pivot(deg_diff=turn)
#                     turn += 15
#                     time.sleep(1)
#                     cntr_bbox.img_show('Block',image)
#                                                        
#                     detected, deg_diff, image = cntr_bbox.scanObject(obj)
#                     
#                     if detected:
#                         # Orient robot and align towards block
#                         yaw = drive_robot.pivot(deg_diff=deg_diff)
#                         time.sleep(1)
#                         cntr_bbox.img_show('Block',image)
#                         print('Aligning robot to block')
#                         break
#                     elif turn >= 360:
#                         drive_robot.gameover(drive_robot.pwms)
#                         # Drive to a different location in the arena
#                         print("Driving to a different location for a better look")    
#                         break
            
            print("Testing check done")
            break
#         f.close()
    
    except KeyboardInterrupt:
        print("Keyboard Interrupted")
        drive_robot.gameover(drive_robot.pwms)
        for pwm in drive_robot.pwms:
            pwm.stop()
        gpio.cleanup()

if __name__ == "__main__":
    
    main()
    
