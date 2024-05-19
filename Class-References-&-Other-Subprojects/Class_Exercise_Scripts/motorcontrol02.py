import RPi.GPIO as gpio
import time
import pygame
import sys

def init():
    gpio.cleanup()
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4
    
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
    # Send all pins low & cleanup
    gameover()
    gpio.cleanup()
    
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
    gpio.cleanup()
    
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
    gpio.cleanup()
    
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
    gpio.cleanup()
    
def key_input(event):
    init()
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
    else:
        print("Invalid key pressed!!")
        
def key_pygame():
    init()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        # checking if keydown event happened or not
        if event.type == pygame.KEYDOWN:
            # checking if key "A" was pressed
            if event.key == pygame.K_w:
                forward(tf)
                print("Key w has been pressed")

            # checking if key "J" was pressed
            if event.key == pygame.K_s:
               	reverse(tf)
                print("Key s has been pressed")
            # checking if key "P" was pressed
            if event.key == pygame.K_a:
               	pivotleft(tf)
                print("Key a has been pressed")
            # checking if key "M" was pressed
            if event.key == pygame.K_d:
                pivotright(tf)
                print("Key d has been pressed")
            if event.key == pygame.K_q:
                print("Teleop quitting")
                break

def main():
    
    # initialising pygame
    #pygame.init()
    
    print("Press the bottom keys to control the robot")
    print("+++++++++++++++++++++++++")
    print(" 'w' - forward")
    print(" 's' - reverse")
    print(" 'a' - pivot left")
    print(" 'd' - pivot right")
    
    while True:
        key_press = input("Select driving mode: ")
        if key_press == 'p':
            break
        else:
            key_input(key_press)
            #key_pygame()
    gpio.cleanup()

if __name__ == "__main__":
    main()
