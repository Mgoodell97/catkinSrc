#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import particles

#import numpy as np
import matplotlib.pyplot as plt

##################
# Functions
##################

def map( x,  in_min,  in_max,  out_min,  out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


##################
# Global variables
##################

global Robot1_pose
global Robot1_particles

Robot1_pose = PoseStamped()
Robot1_particles = particles()

##################
# Callbacks
##################

def Robot1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def Robot1_particles_cb(Robot1_particles_msg):
    global Robot1_particles
    Robot1_particles = Robot1_particles_msg

##################
# Main function
##################

def main():
    
    # Name node 
    rospy.init_node('plot_l3particles')
    global rate
    rate = rospy.Rate(30)

    # Map parameters
    minLimX = 0
    minLimY = 0

    #maxLimX = 120
    #maxLimY = 120
    # maxLimX = 14 / 3.28084
    # maxLimY = 9 / 3.28084
    maxLimX = 14
    maxLimY = 9

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("particles", particles, Robot1_particles_cb)

    xPltRobot1 = []
    yPltRobot1 = []


    while not rospy.is_shutdown():

        Robot1_poseX = map(Robot1_pose.pose.position.x, minLimX, maxLimX, \
                           minLimY, maxLimY)
        Robot1_poseY = map(Robot1_pose.pose.position.y, minLimX, maxLimX, \
                           minLimY, maxLimY)

        if Robot1_poseX != 0.0 and Robot1_poseY != 0.0:
            xPltRobot1.append(Robot1_poseX)
            yPltRobot1.append(Robot1_poseY)


        # Plotting stuff
        plt.clf()

        # Plot Wifi stuff
        plt.plot(xPltRobot1, yPltRobot1,"r")
        plt.plot(Robot1_poseX, Robot1_poseY,'ro',  markersize=6)
        plt.plot(Robot1_particles.X,Robot1_particles.Y,'k.')
        #Transmitter_location = [0.5,0.5]
        plt.plot(0.5,0.5,'bx')


        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid()
        plt.xlim(minLimX, maxLimX)
        plt.ylim(minLimY, maxLimY)
        plt.pause(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
