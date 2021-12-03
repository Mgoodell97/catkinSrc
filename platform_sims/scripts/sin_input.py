#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
import pickle
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

##################
# Functions
##################

##################
# Global variables
##################

global Robot1_pose

Robot1_pose = PoseStamped()

##################
# Callbacks
##################

def robot_1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

##################
# Main function
##################

def main():

    plotResults = False
    rospy.init_node('stepReponseXY')

    rate = rospy.Rate(100)

    startTest  = rospy.get_param("/startTest")      #  m        diffusion along z

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, robot_1_pose_cb)
    local_vel_pub = rospy.Publisher('totalVelocityCMD', Twist, queue_size=10)
    startTestFlag = False

    Step = 100
    testTime = 1.0
    timeBeforeTest = 0.5

    DesiredVel = Twist()
    DesiredVel.linear.x  = 0
    DesiredVel.linear.y  = 0
    DesiredVel.angular.z = 0

    while not rospy.get_param("/startTest"): # waits until the quad is armed before starting particle filtering
        rospy.sleep(1)
        # print(rospy.get_param("/startTest"), "   ,   ", sensorMsg_cb_flag)
        if rospy.get_param("/startTest"):
            break

    robotPoseListX = []
    robotPoseListY = []
    robotPoseListyaw = []
    timeListX = []
    timeListY = []
    timeListyaw = []
    stepListX = []
    stepListY = []
    stepListyaw = []

    # X Step
    startTime = rospy.get_rostime()
    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(timeBeforeTest)):

        DesiredVel.linear.x  = 0
        DesiredVel.linear.y  = 0
        DesiredVel.angular.z = 0
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListX.append(Robot1_pose.pose.position.x)
            timeListX.append(seconds)
            stepListX.append(0)

        local_vel_pub.publish(DesiredVel)

    startTime = rospy.get_rostime()
    testStartX = startTime

    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(testTime)):

        DesiredVel.linear.x  = Step
        DesiredVel.linear.y  = 0
        DesiredVel.angular.z = 0
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListX.append(Robot1_pose.pose.position.x)
            timeListX.append(seconds)
            stepListX.append(Step)

        local_vel_pub.publish(DesiredVel)


    DesiredVel.linear.x  = 0
    DesiredVel.linear.y  = 0
    DesiredVel.angular.z = 0

    local_vel_pub.publish(DesiredVel)

    rospy.sleep(1)

    # Y Step
    startTime = rospy.get_rostime()
    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(timeBeforeTest)):

        DesiredVel.linear.x  = 0
        DesiredVel.linear.y  = 0
        DesiredVel.angular.z = 0
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListY.append(Robot1_pose.pose.position.y)
            timeListY.append(seconds)
            stepListY.append(0)

        local_vel_pub.publish(DesiredVel)

    startTime = rospy.get_rostime()
    testStartY = startTime

    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(testTime)):

        DesiredVel.linear.x  = 0
        DesiredVel.linear.y  = Step
        DesiredVel.angular.z = 0
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            robotPoseListY.append(Robot1_pose.pose.position.y)
            timeListY.append(seconds)
            stepListY.append(Step)

        local_vel_pub.publish(DesiredVel)


    DesiredVel.linear.x  = 0
    DesiredVel.linear.y  = 0
    DesiredVel.angular.z = 0

    local_vel_pub.publish(DesiredVel)

    rospy.sleep(1)

    # Yaw Step
    startTime = rospy.get_rostime()
    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(timeBeforeTest)):

        DesiredVel.linear.x  = 0
        DesiredVel.linear.y  = 0
        DesiredVel.angular.z = 0
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            roll, pitch, yaw =euler_from_quaternion([Robot1_pose.pose.orientation.x,Robot1_pose.pose.orientation.y,Robot1_pose.pose.orientation.z,Robot1_pose.pose.orientation.w])
            robotPoseListyaw.append(yaw)
            timeListyaw.append(seconds)
            stepListyaw.append(0)

        local_vel_pub.publish(DesiredVel)

    startTime = rospy.get_rostime()
    testStartyaw = startTime

    while ((rospy.get_rostime() - startTime) < rospy.Duration.from_sec(testTime)):

        DesiredVel.linear.x  = 0
        DesiredVel.linear.y  = 0
        DesiredVel.angular.z = Step
        startTestFlag = False

        seconds = rospy.get_time()

        if Robot1_pose.pose.position.x != 0.0 and Robot1_pose.pose.position.y != 0.0:
            roll, pitch, yaw =euler_from_quaternion([Robot1_pose.pose.orientation.x,Robot1_pose.pose.orientation.y,Robot1_pose.pose.orientation.z,Robot1_pose.pose.orientation.w])
            robotPoseListyaw.append(yaw)
            timeListyaw.append(seconds)
            stepListyaw.append(Step)

        local_vel_pub.publish(DesiredVel)


    DesiredVel.linear.x  = 0
    DesiredVel.linear.y  = 0
    DesiredVel.angular.z = 0

    local_vel_pub.publish(DesiredVel)

    robotPoseListX = np.array(robotPoseListX)
    robotPoseListY = np.array(robotPoseListY)
    robotPoseListyaw = np.array(robotPoseListyaw)
    timeListX       = np.array(timeListX)
    timeListY       = np.array(timeListY)
    timeListyaw     = np.array(timeListyaw)

    robotPoseListX = robotPoseListX - robotPoseListX[0]
    robotPoseListY = robotPoseListY - robotPoseListY[0]
    robotPoseListyaw = robotPoseListyaw - robotPoseListyaw[0]
    robotPoseListyaw*=-1
    #robotPoseListyaw[robotPoseListyaw<-2]+=360

    timeListX       = (timeListX       - timeListX[0])
    timeListY       = (timeListY       - timeListY[0])
    timeListyaw      = (timeListyaw       - timeListyaw[0])

    db = {}
    db["timeListX"]   = timeListX
    db["timeListY"]   = timeListY
    db["timeListyaw"] = timeListyaw
    db["stepListX"]   = stepListX
    db["stepListY"]   = stepListY
    db["stepListyaw"] = stepListyaw
    db["testStartX"] = testStartX
    db["testStartY"] = testStartY
    db["testStartyaw"] = testStartyaw

    dbfile = open('examplePickle', 'ab')

    pickle.dump(db, dbfile)
    dbfile.close()

    if plotResults:
        while not rospy.is_shutdown():
            # Plotting stuff
            plt.clf()

            plt.subplot(231)
            plt.plot(timeListX, stepListX,'r',  markersize=6)
            plt.plot([0.5, 0.5], [-10,110],'--k')
            plt.xlabel("Time [s]")
            # plt.ylim( (-5, 110) )
            plt.grid()

            plt.subplot(232)
            plt.plot(timeListY, stepListY,'r',  markersize=6)
            plt.plot([0.5, 0.5], [-10,110],'--k')
            plt.xlabel("Time [s]")
            plt.grid()

            plt.subplot(233)
            plt.plot(timeListyaw, stepListyaw,'r',  markersize=6)
            plt.plot([0.5, 0.5], [-10,110],'--k')
            plt.xlabel("Time [s]")
            plt.grid()

            plt.subplot(234)
            plt.plot(timeListX, robotPoseListX,'r',  markersize=6)
            plt.plot([0.5, 0.5], [min(robotPoseListX)*1.1,max(robotPoseListX)*1.1],'--k')
            plt.xlabel("Time [s]")
            plt.ylabel("Distance X [m]")
            plt.grid()

            plt.subplot(235)
            plt.plot(timeListY, robotPoseListY,'r',  markersize=6)
            plt.plot([0.5, 0.5], [min(robotPoseListY)*1.1,max(robotPoseListY)*1.1],'--k')
            plt.xlabel("Time [s]")
            plt.ylabel("Distance Y [m]")
            plt.grid()

            plt.subplot(236)
            plt.plot(timeListyaw, robotPoseListyaw,'r',  markersize=6)
            plt.plot([0.5, 0.5], [min(robotPoseListyaw)*1.1,max(robotPoseListyaw)*1.1],'--k')
            plt.xlabel("Time [s]")
            plt.ylabel("Angle yaw [rad]")
            plt.grid()

            plt.pause(0.1)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
