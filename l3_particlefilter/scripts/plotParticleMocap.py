#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import particles

#import numpy as np
import matplotlib.pyplot as plt

##################
# Functions
##################



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
    rate = rospy.Rate(10)

    # Map parameters
    minLimX = 0.095    #m
    minLimY = 0.295    #m
    maxLimX = 4.1656    #m
    maxLimY = 2.4003    #m


    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("particles", particles, Robot1_particles_cb)

    xPltRobot1 = []
    yPltRobot1 = []


    while not rospy.is_shutdown():

        Robot1_poseX = Robot1_pose.pose.position.x
        Robot1_poseY = Robot1_pose.pose.position.y

        if Robot1_poseX != 0.0 and Robot1_poseY != 0.0:
            xPltRobot1.append(Robot1_poseX)
            yPltRobot1.append(Robot1_poseY)


        # Plotting stuff
        plt.clf()

        # Plot Wifi stuff
        plt.plot(xPltRobot1, yPltRobot1,"r")
        plt.plot(Robot1_poseX, Robot1_poseY,'kD',  markersize=6)
        plt.plot(Robot1_particles.X, Robot1_particles.Y, 'k.')
        #Transmitter_location = [0.5,0.5]
        plt.plot(1,1,'gx')


        plt.axis("tight")  # gets rid of white border
        plt.margins(x=0)
        plt.tight_layout()
        plt.xlim(minLimX, maxLimX)
        plt.ylim(minLimY, maxLimY)
        plt.grid(True)
        plt.pause(0.01)
        rate.sleep()

        # plt.xlabel("x [m]")
        # plt.ylabel("y [m]")
        # plt.grid()
        # plt.xlim(minLimX, maxLimX)
        # plt.ylim(minLimY, maxLimY)
        # plt.pause(0.01)
        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
