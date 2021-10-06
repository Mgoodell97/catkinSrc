#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped

global Robot1_pose
Robot1_pose = PoseStamped()

def pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg


def main():

    minLimX = 0
    minLimY = 0
    maxLimX = 14
    maxLimY = 9

    # Initalize node
    rospy.init_node('mocap_plotter')

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, pose_cb)

    global rate
    rate = rospy.Rate(20)

    xPltUAV1 = []
    yPltUAV1 = []

    while not rospy.is_shutdown():
        # Plotting stuff
        plt.clf()

        Robot1_poseXft = Robot1_pose.pose.position.x * 3.28084
        Robot1_poseYft = Robot1_pose.pose.position.y * 3.28084

        plt.subplot("111")

        plt.xlabel("x [ft]")
        plt.ylabel("y [ft]")
        plt.grid()
        plt.xlim(minLimX, maxLimX)
        plt.ylim(minLimY, maxLimY)

        xPltUAV1.append(Robot1_poseXft)
        yPltUAV1.append(Robot1_poseYft)
        # print xPltUAV1
        plt.plot(xPltUAV1, yPltUAV1,"r")
        plt.plot(Robot1_poseXft, Robot1_poseYft,'ro',  markersize=6)

        plt.pause(0.01)
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
