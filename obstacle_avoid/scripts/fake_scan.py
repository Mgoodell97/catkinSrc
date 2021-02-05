#!/usr/bin/env python

# Moving Average Filter

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np #import matrix
import genpy
import math
import random

sim = 0

def callback2(header):
    ranges = list(header.ranges)
    ii = 0
    for M in ranges:
        ranges[ii] = 30
        ii += 1
    pub_new_laser(ranges, header)



def pub_new_laser(valores, header_scan):
    idx = 0
    laser = LaserScan()
    header = Header()
    # creating header
    header.seq = idx

    ftime_now = rospy.get_time()
    header.stamp = genpy.rostime.Time(int(ftime_now) // 1, int((ftime_now % 1.0) * 1000000000))
    header.frame_id = "laser"

    laser.header = header
    laser.angle_min = header_scan.angle_min
    laser.angle_max = header_scan.angle_max
    laser.angle_increment = header_scan.angle_increment
    laser.time_increment = header_scan.time_increment
    laser.scan_time = header_scan.scan_time
    laser.range_min = header_scan.range_min
    laser.range_max = 31#header_scan.range_max
    laser.ranges = valores

    pub.publish(laser)

    idx+=1



if __name__ == '__main__':
    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=1)
    rospy.init_node('listener', anonymous=True)
    if sim == 0:
        rospy.Subscriber("agent_1/scan", LaserScan, callback2,queue_size=1 )
    else:
        rospy.Subscriber("scan", LaserScan, callback2, queue_size=1)
    rospy.spin()
