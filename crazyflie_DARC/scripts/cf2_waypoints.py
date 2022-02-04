#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_from_euler

#--------------------------Global Variables ---------------------------
global desired_pose
desired_pose = PoseStamped()

global crazyflie1_pose
crazyflie1_pose = PoseStamped()

global crazyflie2_pose
crazyflie2_pose = PoseStamped()

global crazyflie3_pose
crazyflie3_pose = PoseStamped()

#------------- Callback Functions --------------------
def crazyflie1_pose_cb(pose_cb_msg):
    global crazyflie1_pose
    crazyflie1_pose = pose_cb_msg

def crazyflie2_pose_cb(pose_cb_msg):
    global crazyflie2_pose
    crazyflie2_pose = pose_cb_msg

def crazyflie3_pose_cb(pose_cb_msg):
    global crazyflie3_pose
    crazyflie3_pose = pose_cb_msg

def main():
    #Initialize node
    rospy.init_node("waypoints")

    #Get Parameters
    xd1_vec = rospy.get_param("waypoints_x_1")
    yd1_vec = rospy.get_param("waypoints_y_1")
    zd1_vec = rospy.get_param("waypoints_z_1")
    yawd1_vec = rospy.get_param("waypoints_yaw_1")

    xd2_vec = rospy.get_param("waypoints_x_2")
    yd2_vec = rospy.get_param("waypoints_y_2")
    zd2_vec = rospy.get_param("waypoints_z_2")
    yawd2_vec = rospy.get_param("waypoints_yaw_2")

    xd3_vec = rospy.get_param("waypoints_x_3")
    yd3_vec = rospy.get_param("waypoints_y_3")
    zd3_vec = rospy.get_param("waypoints_z_3")
    yawd3_vec = rospy.get_param("waypoints_yaw_3")


    #Set up subscribers
    rospy.Subscriber("/mocap_node/Crazyflie_1/pose", PoseStamped, crazyflie1_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_2/pose", PoseStamped, crazyflie2_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_3/pose", PoseStamped, crazyflie3_pose_cb)

    #Set up publishers
    desired_pos_pub = rospy.Publisher("desiredPos",PoseStamped,queue_size=100)

    while not rospy.is_shutdown():
        x1=crazyflie1_pose.pose.position.x
        y1 =crazyflie1_pose.pose.position.y
        z1 = crazyflie1_pose.pose.position.z


        x2=crazyflie2_pose.pose.position.x
        y2 =crazyflie2_pose.pose.position.y
        z2 = crazyflie2_pose.pose.position.z

        x3=crazyflie3_pose.pose.position.x
        y3 =crazyflie3_pose.pose.position.y
        z3 = crazyflie3_pose.pose.position.z

        for j in range(len(xd1_vec)):
            x1=crazyflie1_pose.pose.position.x
            y1 =crazyflie1_pose.pose.position.y
            z1 = crazyflie1_pose.pose.position.z


            x2=crazyflie2_pose.pose.position.x
            y2 =crazyflie2_pose.pose.position.y
            z2 = crazyflie2_pose.pose.position.z

            x3=crazyflie3_pose.pose.position.x
            y3 =crazyflie3_pose.pose.position.y
            z3 = crazyflie3_pose.pose.position.z

            x1_error = xd1_vec[j] - x1
            y1_error = yd1_vec[j] - y1
            z1_error = zd1_vec[j] - z1

            x2_error = xd2_vec[j] - x2
            y2_error = yd2_vec[j] - y2
            z2_error = zd2_vec[j] - z2

            x3_error = xd3_vec[j] - x3
            y3_error = yd3_vec[j] - y3
            z3_error = zd3_vec[j] - z3
            norm_error1 = np.sqrt(x1_error**2+ y1_error**2+z1_error**2)
            norm_error2 = np.sqrt(x2_error**2+ y2_error**2+z2_error**2)
            norm_error3 = np.sqrt(x3_error**2+ y3_error**2+z3_error**2)

            while not rospy.is_shutdown() and norm_error1 > 0.01 :
                x = robot_pose.pose.position.x
                y = robot_pose.pose.position.y
                DesirePos.pose.position.x = xd_vec[j]
                DesirePos.pose.position.y = yd_vec[j]
                (xq, yq, zq, wq) = quaternion_from_euler(0,0,yawd_vec[j])
                DesirePos.pose.orientation.x = xq
                DesirePos.pose.orientation.y = yq
                DesirePos.pose.orientation.z = zq
                DesirePos.pose.orientation.w = wq
                x_error = xd_vec[j] - x
                y_error = yd_vec[j] - y
                norm_error = np.sqrt(x_error**2+ y_error**2)
                desired_pos_pub.publish(DesirePos)
