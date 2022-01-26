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
Robot2_pose = PoseStamped()
Robot3_pose = PoseStamped()
A1 = np.empty([8], dtype=np.float32)
A2 = np.empty([8], dtype=np.float32)
A3 = np.empty([8], dtype=np.float32)
A1Flag = False
A2Flag = False
A3Flag = False

##################
# Callbacks
##################

def Robot1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def Robot2_pose_cb(pose_cb_msg):
    global Robot2_pose
    Robot2_pose = pose_cb_msg

def Robot3_pose_cb(pose_cb_msg):
    global Robot3_pose
    Robot3_pose = pose_cb_msg

def gauss_est_r1(consumed_msg):
    global A1
    global A1Flag

    if consumed_msg.X: # check if list is empty. if not do this
        A1[0]     = np.array(consumed_msg.X, dtype=np.float32)            # [m]
        A1[1]     = np.array(consumed_msg.Y, dtype=np.float32)          # [m]
        A1[2]     = np.array(consumed_msg.Z, dtype=np.float32)                # [m]
        A1[3]     = np.array(consumed_msg.theta, dtype=np.float32)                # [rads]
        A1[4]     = np.array(consumed_msg.Q, dtype=np.float32)  # [kg/s]
        A1[5]     = np.array(consumed_msg.v, dtype=np.float32)  # [m/s]
        A1[6]     = np.array(consumed_msg.Dy, dtype=np.float32)          # [m^2/s]
        A1[7]     = np.array(consumed_msg.Dz, dtype=np.float32)      # [m^2/s]

        A1Flag = True
        # AHat = np.matrix((xPlumesConsumed, yPlumesConsumed, zPlumesConsumed, thetaPlumesConsumed, QsConsumed, vsConsumed, DysConsumed, DzsConsumed), dtype=np.float32)

def gauss_est_r2(consumed_msg):
    global A2
    global A2Flag

    if consumed_msg.X: # check if list is empty. if not do this
        A2[0]     = np.array(consumed_msg.X, dtype=np.float32)            # [m]
        A2[1]     = np.array(consumed_msg.Y, dtype=np.float32)          # [m]
        A2[2]     = np.array(consumed_msg.Z, dtype=np.float32)                # [m]
        A2[3]     = np.array(consumed_msg.theta, dtype=np.float32)                # [rads]
        A2[4]     = np.array(consumed_msg.Q, dtype=np.float32)  # [kg/s]
        A2[5]     = np.array(consumed_msg.v, dtype=np.float32)  # [m/s]
        A2[6]     = np.array(consumed_msg.Dy, dtype=np.float32)          # [m^2/s]
        A2[7]     = np.array(consumed_msg.Dz, dtype=np.float32)      # [m^2/s]

        A2Flag = True

def gauss_est_r3(consumed_msg):
    global A3
    global A3Flag

    if consumed_msg.X: # check if list is empty. if not do this
        A3[0]     = np.array(consumed_msg.X, dtype=np.float32)            # [m]
        A3[1]     = np.array(consumed_msg.Y, dtype=np.float32)          # [m]
        A3[2]     = np.array(consumed_msg.Z, dtype=np.float32)                # [m]
        A3[3]     = np.array(consumed_msg.theta, dtype=np.float32)                # [rads]
        A3[4]     = np.array(consumed_msg.Q, dtype=np.float32)  # [kg/s]
        A3[5]     = np.array(consumed_msg.v, dtype=np.float32)  # [m/s]
        A3[6]     = np.array(consumed_msg.Dy, dtype=np.float32)          # [m^2/s]
        A3[7]     = np.array(consumed_msg.Dz, dtype=np.float32)      # [m^2/s]

        A3Flag = True

##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(10)

    map_params = rospy.get_param("averagePlumePlot") # get params defined in launch file
    xPlumeLoc = rospy.get_param("/xPlumeLoc", -100)
    yPlumeLoc = rospy.get_param("/yPlumeLoc", -100)

    SpawnUAV1     = rospy.get_param("/subToR1Pose",False)
    SpawnUAV2     = rospy.get_param("/subToR2Pose",False)
    SpawnUAV3     = rospy.get_param("/subToR3Pose",False)
    SpawnUAV4     = rospy.get_param("/subToR4Pose",False)
    SpawnUAV5     = rospy.get_param("/subToR5Pose",False)

    # Saves dictionary entries from launch file as variables
    for key,val in map_params.items():
        exec(key + '=val')

    maxPlot = 600       # For plotting only!
    minPlot = 0.001    # For plotting only!

    rospy.Subscriber("/mocap_node/Robot_1/pose",  PoseStamped, Robot1_pose_cb)
    rospy.Subscriber("/mocap_node/Robot_2/pose",  PoseStamped, Robot2_pose_cb)
    rospy.Subscriber("/mocap_node/Robot_3/pose",  PoseStamped, Robot3_pose_cb)

    rospy.Subscriber("/Robot_1/estimatedGaussian", particles, gauss_est_r1)
    rospy.Subscriber("/Robot_2/estimatedGaussian", particles, gauss_est_r2)
    rospy.Subscriber("/Robot_3/estimatedGaussian", particles, gauss_est_r3)

    xPltRobot1 = []
    yPltRobot1 = []

    xPltRobot2 = []
    yPltRobot2 = []

    xPltRobot3 = []
    yPltRobot3 = []

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
        if A1Flag and A2Flag and A3Flag and SpawnUAV1 and SpawnUAV2 and SpawnUAV3:
            averagePlume = np.mean([A1,A2,A3], axis=0)
            AHat = np.matrix((averagePlume[0], averagePlume[1], averagePlume[2], averagePlume[3], averagePlume[4], averagePlume[5], averagePlume[6], averagePlume[7]), dtype=np.float32)
            conArrayConsumed = computeGaussianPlumeMapNBNoClass(AHat.T, (xmin, xmax), (ymin, ymax), 0.2, 200)
        elif A1Flag and SpawnUAV1:
            AHat = np.matrix((A1[0], A1[1], A1[2], A1[3], A1[4], A1[5], A1[6], A1[7]), dtype=np.float32)
            conArrayConsumed = computeGaussianPlumeMapNBNoClass(AHat.T, (xmin, xmax), (ymin, ymax), A1[2], 200)

        conArrayConsumed[conArrayConsumed>maxPlot] = maxPlot
        conArrayConsumed[conArrayConsumed<minPlot] = minPlot

        Robot1_poseXft = Robot1_pose.pose.position.x
        Robot1_poseYft = Robot1_pose.pose.position.y
        if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
            xPltRobot1.append(Robot1_poseXft)
            yPltRobot1.append(Robot1_poseYft)

        Robot2_poseXft = Robot2_pose.pose.position.x
        Robot2_poseYft = Robot2_pose.pose.position.y
        if Robot2_poseXft != 0.0 and Robot2_poseYft != 0.0:
            xPltRobot2.append(Robot2_poseXft)
            yPltRobot2.append(Robot2_poseYft)

        Robot3_poseXft = Robot3_pose.pose.position.x
        Robot3_poseYft = Robot3_pose.pose.position.y
        if Robot3_poseXft != 0.0 and Robot3_poseYft != 0.0:
            xPltRobot3.append(Robot3_poseXft)
            yPltRobot3.append(Robot3_poseYft)

        plt.plot(xPltRobot1, yPltRobot1, color='lime')
        plt.plot(Robot1_poseXft, Robot1_poseYft, '--D', markersize=10, color='lime')

        if SpawnUAV2 == 1:
            plt.plot(xPltRobot2, yPltRobot2, color='hotpink')
            plt.plot(Robot2_poseXft, Robot2_poseYft, 'D', markersize=10, color='hotpink')

        if SpawnUAV3 == 1:
            plt.plot(xPltRobot3, yPltRobot3, color='cyan')
            plt.plot(Robot3_poseXft, Robot3_poseYft, 'D', markersize=10, color='cyan')


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
