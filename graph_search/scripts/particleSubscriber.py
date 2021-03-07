#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

##################
# Global variables
##################

global matrix_msg
matrix_msg = Float32MultiArray()

##################
# Callbacks
##################

def matrix_cb(matrix_msg_cb):
    # print(matrix_msg_cb)
    global matrix_msg
    matrix_msg = matrix_msg_cb

##################
# Main function
##################

def main():
    rospy.init_node("subscriber")
    r = rospy.Rate(5)
    rospy.Subscriber("/UAV1/patriclesArray", Float32MultiArray, matrix_cb)

    while not rospy.is_shutdown():
        # print(matrix_msg.data)
        if len(matrix_msg.data) != 0:
            a = np.array(matrix_msg.data)
            # print(a)
            b = a.reshape(matrix_msg.layout.dim[0].size, matrix_msg.layout.dim[1].size)
            print("recieved")
            print(b)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
