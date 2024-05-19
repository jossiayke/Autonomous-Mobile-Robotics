from racer.robot_old import Robot


if __name__ == "__main__":
    
    """
    Grand Challenge Competition
    
    Defines the baron robot and then initiates start
    of competition
    """
    
    # Initialize robot
    baron = Robot("Speedy Baron")
    
#     # Start imu sensor reading in parallel thread
#     baron.imu_reader.daemon = True
#     baron.imu_reader.start()
    
    # Order list of blocks to retrieve
    order = ['red','green','blue','red','green','blue','red','green','blue']
    
    # Start engine and wait for que
    baron.start(order)
    
    # Plot the robot trajectory during the challenge
    baron.plotPath()