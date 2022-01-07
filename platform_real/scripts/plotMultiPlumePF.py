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

import warnings

##################
# Functions
##################


##################
# Global variables
##################

global Robot1_pose
global Robot1_particles
global current_estimate_gauss
global consumed_gauss

Robot1_pose            = PoseStamped()
Robot1_particles       = particles()
current_estimate_gauss = particles()
consumed_gauss         = particles()

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

def Robot1_consumed_gauss_cb(consumed_msg):
    global consumed_gauss
    consumed_gauss = consumed_msg

##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(10)

    map_params = rospy.get_param("plotMultiPlumePF") # get params defined in launch file
    xPlumeLoc = rospy.get_param("/xPlumeLoc")
    yPlumeLoc = rospy.get_param("/yPlumeLoc")

    # Saves dictionary entries from launch file as variables
    for key,val in map_params.items():
        exec(key + '=val')

    maxPlot = 600       # For plotting only!
    minPlot = 0.001    # For plotting only!

    rospy.Subscriber("/mocap_node/Robot_1/pose",  PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("Robot_1/particles",         particles, Robot1_particles_cb)
    rospy.Subscriber("Robot_1/estimatedGaussian", particles, Robot1_current_estimate_gauss_cb)
    rospy.Subscriber("Robot_1/consumedPlumes",    particles, Robot1_consumed_gauss_cb)

    xPltRobot1 = []
    yPltRobot1 = []

    xPlumePlot = np.linspace(xmin, xmax, 200)
    yPlumePlot = np.linspace(ymin, ymax, 200)

    conArrayPF   = np.zeros((len(yPlumePlot), len(xPlumePlot)))
    conArrayConsumed   = np.zeros((len(yPlumePlot), len(xPlumePlot)))

    fig, ax = plt.subplots()
    while not rospy.is_shutdown():
        # Plotting stuff
        plt.subplot("111")
        plt.clf()

        if current_estimate_gauss.X: # check if list is empty.
            xPlumesPF     = np.array(current_estimate_gauss.X, dtype=np.float32)            # [m]
            yPlumesPF     = np.array(current_estimate_gauss.Y, dtype=np.float32)          # [m]
            zPlumesPF     = np.array(current_estimate_gauss.Z, dtype=np.float32)                # [m]
            thetaPlumesPF = np.array(current_estimate_gauss.theta, dtype=np.float32)                # [rads]
            QsPF          = np.array(current_estimate_gauss.Q, dtype=np.float32)  # [kg/s]
            vsPF          = np.array(current_estimate_gauss.v, dtype=np.float32)  # [m/s]
            DysPF         = np.array(current_estimate_gauss.Dy, dtype=np.float32)          # [m^2/s]
            DzsPF         = np.array(current_estimate_gauss.Dz, dtype=np.float32)      # [m^2/s]

            A_PF_single = np.matrix((xPlumesPF, yPlumesPF, zPlumesPF, thetaPlumesPF, QsPF, vsPF, DysPF, DzsPF), dtype=np.float32)

            # Compute gaussian array
            conArrayPF = computeGaussianPlumeMapNBNoClass(A_PF_single, (xmin, xmax), (ymin, ymax), current_estimate_gauss.Z[0], 200)

            conArrayPF[conArrayPF>maxPlot] = maxPlot
            conArrayPF[conArrayPF<minPlot] = minPlot

        if consumed_gauss.X: # check if list is empty.
            xPlumesConsumed     = np.array(consumed_gauss.X, dtype=np.float32)            # [m]
            yPlumesConsumed     = np.array(consumed_gauss.Y, dtype=np.float32)          # [m]
            zPlumesConsumed     = np.array(consumed_gauss.Z, dtype=np.float32)                # [m]
            thetaPlumesConsumed = np.array(consumed_gauss.theta, dtype=np.float32)                # [rads]
            QsConsumed          = np.array(consumed_gauss.Q, dtype=np.float32)  # [kg/s]
            vsConsumed          = np.array(consumed_gauss.v, dtype=np.float32)  # [m/s]
            DysConsumed         = np.array(consumed_gauss.Dy, dtype=np.float32)          # [m^2/s]
            DzsConsumed         = np.array(consumed_gauss.Dz, dtype=np.float32)      # [m^2/s]

            AHat = np.matrix((xPlumesConsumed, yPlumesConsumed, zPlumesConsumed, thetaPlumesConsumed, QsConsumed, vsConsumed, DysConsumed, DzsConsumed), dtype=np.float32)

            # Compute gaussian array
            conArrayConsumed = computeGaussianPlumeMapNBNoClass(AHat, (xmin, xmax), (ymin, ymax), current_estimate_gauss.Z[0], 200)

            conArrayConsumed[conArrayConsumed>maxPlot] = maxPlot
            conArrayConsumed[conArrayConsumed<minPlot] = minPlot

        Robot1_poseXft = Robot1_pose.pose.position.x
        Robot1_poseYft = Robot1_pose.pose.position.y
        if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
            xPltRobot1.append(Robot1_poseXft)
            yPltRobot1.append(Robot1_poseYft)

        plt.plot(xPltRobot1, yPltRobot1, color='lime')
        plt.plot(Robot1_poseXft, Robot1_poseYft, '--D', markersize=10, color='lime')
        plt.plot(Robot1_particles.X,Robot1_particles.Y,'.', color='black')
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="No contour levels were found within the data range.")
            cs = plt.contour(xPlumePlot,yPlumePlot,conArrayPF,levels=15)
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="No contour levels were found within the data range.")
            cs = plt.contour(xPlumePlot,yPlumePlot,conArrayConsumed,levels=15, cmap="summer")
        plt.plot(xPlumeLoc, yPlumeLoc, 'X', markersize=10, color='red')
        if current_estimate_gauss.X: # check if list is empty
            plt.plot(current_estimate_gauss.X, current_estimate_gauss.Y, 'X', markersize=10, color='lime')


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
