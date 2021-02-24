#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
#import numpy
from sensor_msgs.msg import Joy
import sys


global controllerValues
controllerValues = Joy()
controllerValues.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
controllerValues.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def controller_cb(controller_sub):
    global controllerValues
    controllerValues = controller_sub

def initFunc():
    rospy.init_node('Controller') #node name is offb_node

    rospy.Subscriber("joy", Joy, controller_cb)

    global rate
    rate = rospy.Rate(20)


def main():
    initFunc()


    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        rospy.loginfo("Throttle: %f     Roll: %f        Pitch: %f",controllerValues.axes[1],-controllerValues.axes[3],controllerValues.axes[4])
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
