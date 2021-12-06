#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from quadnodes.msg import gaussian

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp

from GaussianSensorPackage import computeGaussianPlumeMapNBNoClass

##################
# Functions
##################


##################
# Global variables
##################

global Robot1_pose
global Robot1_particles
global gaussianMsg
global current_estimate_gauss

Robot1_pose            = PoseStamped()
Robot1_particles       = particles()
current_estimate_gauss = particles()


##################
# Callbacks
##################

def Robot1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def Robot1_particles_cb(particles_msg):
    global Robot1_particles
    Robot1_particles = particles_msg

def Robot1_current_estimate_gauss_cb(particles_msg):
    global current_estimate_gauss
    current_estimate_gauss = particles_msg

##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(10)

    map_params = rospy.get_param("plotMultiPlumePF") # get params defined in launch file

    # Saves dictionary entries from launch file as variables
    for key,val in map_params.items():
        exec(key + '=val')

    maxPlot = 100       # For plotting only!
    minPlot = 0.001    # For plotting only!

    rospy.Subscriber("/mocap_node/Robot_1/pose",  PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("Robot_1/particles",         particles, Robot1_particles_cb)
    rospy.Subscriber("Robot_1/estimatedGaussian", particles, Robot1_current_estimate_gauss_cb)

    xPltRobot1 = []
    yPltRobot1 = []

    xPlumePlot = np.linspace(xmin, xmax, 200)
    yPlumePlot = np.linspace(ymin, ymax, 200)

    conArray   = np.zeros((len(yPlumePlot), len(xPlumePlot)))

    fig, ax = plt.subplots()
    while not rospy.is_shutdown():

        if current_estimate_gauss.X: # check if list is empty
            xPlumesPF     = np.array(current_estimate_gauss.X[0], dtype=np.float32)            # [m]
            yPlumesPF     = np.array(current_estimate_gauss.Y[0], dtype=np.float32)          # [m]
            zPlumesPF     = np.array(current_estimate_gauss.Z[0], dtype=np.float32)                # [m]
            thetaPlumesPF = np.array(current_estimate_gauss.theta[0], dtype=np.float32)                # [rads]
            QsPF          = np.array(current_estimate_gauss.Q[0], dtype=np.float32)  # [kg/s]
            vsPF          = np.array(current_estimate_gauss.v[0], dtype=np.float32)  # [m/s]
            DysPF         = np.array(current_estimate_gauss.Dy[0], dtype=np.float32)          # [m^2/s]
            DzsPF         = np.array(current_estimate_gauss.Dz[0], dtype=np.float32)      # [m^2/s]

            A_PF_single = np.array((xPlumesPF, yPlumesPF, zPlumesPF, thetaPlumesPF, QsPF, vsPF, DysPF, DzsPF), dtype=np.float32)
            A_PF_single = A_PF_single[:,np.newaxis]

            # Compute gaussian array
            conArray = computeGaussianPlumeMapNBNoClass(A_PF_single, (xmin, xmax), (ymin, ymax), current_estimate_gauss.Z[0], 200)

            conArray[conArray>maxPlot] = maxPlot
            conArray[conArray<minPlot] = minPlot

        Robot1_poseXft = Robot1_pose.pose.position.x
        Robot1_poseYft = Robot1_pose.pose.position.y
        if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
            xPltRobot1.append(Robot1_poseXft)
            yPltRobot1.append(Robot1_poseYft)

        plt.clf()

        # Plotting stuff
        plt.subplot("111")


        cs = plt.contour(xPlumePlot,yPlumePlot,conArray,levels=15)
        plt.plot(xPltRobot1, yPltRobot1, color='lime')
        plt.plot(Robot1_poseXft, Robot1_poseYft, 'D', markersize=10, color='lime')
        plt.plot(Robot1_particles.X,Robot1_particles.Y,'.', color='black')

        plt.axis("tight")  # gets rid of white border
        # plt.margins(x=0)
        plt.tight_layout()
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        plt.grid(True)
        plt.pause(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
