#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from quadnodes.msg import gaussian

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp

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
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(30)

    # Map parameters
    minLimX = 0
    minLimY = 0

    maxLimX = 120
    maxLimY = 120
    # maxLimX = 14 / 3.28084
    # maxLimY = 9 / 3.28084

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("particles", particles, Robot1_particles_cb)

    xPltRobot1 = []
    yPltRobot1 = []


    while not rospy.is_shutdown():

        Robot1_poseXft = map(Robot1_pose.pose.position.x*3.28084,0,14,0,120)
        Robot1_poseYft = map(Robot1_pose.pose.position.y*3.28084,0,14,0,120)

        if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
            xPltRobot1.append(Robot1_poseXft)
            yPltRobot1.append(Robot1_poseYft)


        # Plotting stuff
        plt.clf()

        # Plot Wifi stuff
        plt.plot(xPltRobot1, yPltRobot1,"r")
        plt.plot(Robot1_poseXft, Robot1_poseYft,'ro',  markersize=6)
        plt.plot(Robot1_particles.X,Robot1_particles.Y,'k.')
        Transmitter_location = [20,42]
        plt.plot(20,42,'rx')


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
