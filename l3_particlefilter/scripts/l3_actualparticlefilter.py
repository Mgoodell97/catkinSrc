#!/usr/bin/env python3
# particle Filter

import rospy #include <ros/ros.h> cpp equivalent

# subscribe
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
#from visualization_msgs.msg import Marker

# import math or function files
import numpy as np
import matplotlib.pyplot as plt
import l3_particlefilter_functions as pf



#---------------Global Variables------------
global Robot1_pose
Robot1_pose = PoseStamped()


#-------------Functions --------------------
def robot_1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg




#-------------Main --------------------
def main():
    print('Initializing Particle Filter')
    
    # Initialize node
    rospy.init_node('l3_actualparticlefilter')
    
    # set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, robot_1_pose_cb)
    #set up publishers
    particlesMsg = particles()
    particlesPublishers = rospy.Publisher('particles', particles, queue_size=1)
    
    
    #Get parameters
    RobotID = rospy.get_param("RobotID",)
    global rate
    rate = rospy.Rate(20)   #20 hz
    
    
    # Initialize Particle filter variables
    # Number of Particles
    Np = 1000
    
    # Create environment
    xmin = 0.0
    ymin = 0.0
    xmax = 14.0/3.281   #m
    ymax = 9/3.281      #m
    
    # source location
    sLoc = [0.5, 0.5]
    
    # noise to add to particles (3%)
    xNoise = (xmax/100)*3
    yNoise = (ymax/100)*3
    
    #set parameters for wifi signal propogation
    # 2.4 Ghz -> 12.5 centimeters
    lamda = 0.125  #m
    Pt = 20    #dBm (typical for 2.4 Ghz)
    D = 1   #combined directivity
    
    # Initialize likelihood model, x, y--
    w = np.squeeze(np.ones((Np,1))/Np)
    xp = np.squeeze(np.random.uniform(xmin, xmax, (Np,1)))
    yp = np.squeeze(np.random.uniform(ymin, ymax, (Np,1)))
    
    
    # save particle locations into message
    particlesMsg.X = xp
    particlesMsg.Y = yp
    
    # publish particle message
    particlesPublishers.publish(particlesMsg)
    
    
    rospy.sleep(10)
    
    
    
    #-------- Main Loop after Initialization----------------
    # while ros is running 
    while not rospy.is_shutdown():
        
        # Find current Robot Location
        xRobot = Robot1_pose.pose.position.x
        yRobot = Robot1_pose.pose.position.y
        
        # get current distance to source
        xdiff = np.subtract(xRobot - sLoc[0])
        ydiff = np.subtract(yRobot - sLoc[1])
        dist = np.sqrt(np.square(xdiff) + np.square(ydiff))
        
        # calculate true measurement 
        hold = 4*np.multiply(np.pi, dist)
        Pr = Pt + 20 * np.log10(np.divide(lamda, hold))     #dBm
        #add 2% noise to the measurement
        sigmaNoise = (np.max(Pr)/100)*2.0
        PrN = Pr + sigmaNoise * np.random.randn()
        
        # find measurement value for each particle
        xhold = np.subtract(xp, xRobot)
        yhold = np.subtract(yp, yRobot)
        distP = np.squeeze(np.sqrt(np.square(xhold) + np.square(yhold)))
        holdP = 4*np.multiply(np.pi, distP)
        PrP = Pt + 20 * np.log10(np.divide(lamda, hold))     #dBm
        
        # find error and likelihood model
        sigma =  (np.max(PrN)/100)*3.0  #dB (x% noise)   
        w = pf.likeModel(PrN, PrP, w, sigma)
        
        [xp, yp, w] = pf.resample2D(w, xp, yp)
        
        # save particle locations into message
        particlesMsg.X = xp
        particlesMsg.Y = yp
        
        # publish particle message
        particlesPublishers.publish(particlesMsg)
        
        meanXp = np.mean(xp)
        meanYp = np.mean(yp)
        print "Source Location X:", sLoc[0], "Y:", sLoc[1]
        print "Mean Particle Location: X:", meanXp, "Y:", meanYp
        
        
        # add noise to particles
        vx = np.random.normal(0.0, xNoise, Np)
        vy = np.random.normal(0.0, yNoise, Np)
        xp = np.add(xp, vx)
        yp = np.add(yp, vy)
        
        #keep particles within bounds
        for q in range(0,Np):
            if xp[q] > xmax:
                xp[q] = xmax
            if xp[q] < xmin:
                xp[q] = xmin
            if yp[q] > ymax:
                yp[q] = ymin
            if yp[q] < ymin:
                yp[q] = ymin
                
                
                
        
        
        rate.sleep()
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass