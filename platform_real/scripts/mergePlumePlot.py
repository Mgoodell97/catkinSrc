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

Robot1_pose = PoseStamped()
AHat = np.array((), dtype=np.float32)

##################
# Callbacks
##################

def Robot1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def consumed_gauss_cb(consumed_msg):
    global AHat

    if consumed_msg.X: # check if list is empty.
        xPlumesConsumed     = np.array(consumed_msg.X, dtype=np.float32)            # [m]
        yPlumesConsumed     = np.array(consumed_msg.Y, dtype=np.float32)          # [m]
        zPlumesConsumed     = np.array(consumed_msg.Z, dtype=np.float32)                # [m]
        thetaPlumesConsumed = np.array(consumed_msg.theta, dtype=np.float32)                # [rads]
        QsConsumed          = np.array(consumed_msg.Q, dtype=np.float32)  # [kg/s]
        vsConsumed          = np.array(consumed_msg.v, dtype=np.float32)  # [m/s]
        DysConsumed         = np.array(consumed_msg.Dy, dtype=np.float32)          # [m^2/s]
        DzsConsumed         = np.array(consumed_msg.Dz, dtype=np.float32)      # [m^2/s]

        AHat = np.matrix((xPlumesConsumed, yPlumesConsumed, zPlumesConsumed, thetaPlumesConsumed, QsConsumed, vsConsumed, DysConsumed, DzsConsumed), dtype=np.float32)


##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(10)

    map_params = rospy.get_param("plotMergedPlumes") # get params defined in launch file
    xPlumeLoc = rospy.get_param("/xPlumeLoc", -100)
    yPlumeLoc = rospy.get_param("/yPlumeLoc", -100)

    # Saves dictionary entries from launch file as variables
    for key,val in map_params.items():
        exec(key + '=val')

    maxPlot = 600       # For plotting only!
    minPlot = 0.001    # For plotting only!

    rospy.Subscriber("/mocap_node/Robot_1/pose",  PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("consumedPlumes",    particles, consumed_gauss_cb)

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

        # Compute gaussian array
        # print(AHat.size)
        if AHat.size != 0:
            conArrayConsumed = computeGaussianPlumeMapNBNoClass(AHat, (xmin, xmax), (ymin, ymax), 0.2, 200)

        conArrayConsumed[conArrayConsumed>maxPlot] = maxPlot
        conArrayConsumed[conArrayConsumed<minPlot] = minPlot

        Robot1_poseXft = Robot1_pose.pose.position.x
        Robot1_poseYft = Robot1_pose.pose.position.y
        if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
            xPltRobot1.append(Robot1_poseXft)
            yPltRobot1.append(Robot1_poseYft)

        plt.plot(xPltRobot1, yPltRobot1, color='lime')
        plt.plot(Robot1_poseXft, Robot1_poseYft, '--D', markersize=10, color='lime')

        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="No contour levels were found within the data range.")
            cs = plt.contour(xPlumePlot,yPlumePlot,conArrayConsumed,levels=15, cmap="summer")

        # Plot real plume locations
        plt.plot(xPlumeLoc, yPlumeLoc, 'X', markersize=10, color='red')



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
