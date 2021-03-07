#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import random

def main():
    rospy.init_node("publisher")
    pub = rospy.Publisher('sent_matrix', Float32MultiArray, queue_size=1)
    r = rospy.Rate(0.5)

    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    width = 5
    height = 3
    mat.layout.dim[0].label = "height"
    mat.layout.dim[1].label = "width"
    mat.layout.dim[0].size = height
    mat.layout.dim[1].size = width
    mat.data = [0]*height*width

    offset = mat.layout.data_offset
    while not rospy.is_shutdown():
        tmpmat = np.zeros((height,width))
        for i in range(height):
            for j in range(width):
                num = random.randrange(0,10)
                mat.data[width*i + j] = num
                tmpmat[i,j] = num
        pub.publish(mat)
        rospy.loginfo("I'm sending:")
        print tmpmat,"\r\n"
        r.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
