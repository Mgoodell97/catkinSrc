#!/usr/bin/env python

# Moving Average Filter

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np #import matrix
import genpy
import math
import random



def pub_new_laser():
    idx = 0
    laser = LaserScan()
    header = Header()
    # creating header
    header.seq = idx

    ftime_now = rospy.get_time()
    header.stamp = genpy.rostime.Time(int(ftime_now) // 1, int((ftime_now % 1.0) * 1000000000))
    header.frame_id = "laser"

    laser.header = header
    laser.angle_min = -2.35619449615
    laser.angle_max = 2.35619449615
    laser.angle_increment = 0.00436332309619
    laser.time_increment = 1.73611151695e-05
    laser.scan_time = 0.0250000003725
    laser.range_min = 0.019999999553
    laser.range_max = 31 #header_scan.range_max
    laser.ranges = [30]*1081

    pub.publish(laser)

    idx+=1



if __name__ == '__main__':
    pub = rospy.Publisher('scan', LaserScan, queue_size=1)
    rospy.init_node('listener', anonymous=True)
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        r.sleep()
        pub_new_laser()

    rospy.spin()
