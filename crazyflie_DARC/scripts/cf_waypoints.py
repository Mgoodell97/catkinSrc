#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_from_euler

#----------- Global Variables--------
global desired_pose
desired_pose = PoseStamped()
global crazyflie_pose
crazyflie_pose = PoseStamped()

#---------- Functions --------
def crazyflie_pose_cb(pose_cb_msg):
    global crazyflie_pose
    crazyflie_pose = pose_cb_msg

def main():
    #Initialize node
    rospy.init_node('waypoints')

    #Get parameters
    xd_vec = rospy.get_param("waypoints_x")
    yd_vec = rospy.get_param("waypoints_y")
    zd_vec = rospy.get_param("waypoints_z")
    yawd_vec = rospy.get_param("waypoints_yaw")
    CFID = rospy.get_param("CF_ID")

    #Set up subscriptions
    firstString = "/mocap_node/Crazyflie_"
    secondString = str(CFID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString #this allows each robot to know itself
    rospy.Subscriber(fullString, PoseStamped, crazyflie_pose_cb)


    #Set up publishers
    desired_pos_pub = rospy.Publisher("desiredPos",PoseStamped,queue_size = 100)

    #Initialize Variables
    global rate
    rate = rospy.Rate(120)
    global robot_pose
    DesirePos = PoseStamped()

    while not rospy.is_shutdown():
        # print robot_pose
        x=crazyflie_pose.pose.position.x
        y=crazyflie_pose.pose.position.y
        z=crazyflie_pose.pose.position.z
        for j in range(len(xd_vec)):
            x=crazyflie_pose.pose.position.x
            y=crazyflie_pose.pose.position.y
            z=crazyflie_pose.pose.position.z
            x_error = xd_vec[j] - x
            y_error = yd_vec[j] - y
            z_error = zd_vec[j] - z
            norm_error = np.sqrt(x_error**2+ y_error**2+z_error**2)
            while not rospy.is_shutdown() and norm_error > 0.25 :
                x=crazyflie_pose.pose.position.x
                y=crazyflie_pose.pose.position.y
                z=crazyflie_pose.pose.position.z
                DesirePos.pose.position.x = xd_vec[j]
                DesirePos.pose.position.y = yd_vec[j]
                DesirePos.pose.position.z = zd_vec[j]
                (xq, yq, zq, wq) = quaternion_from_euler(0,0,yawd_vec[j])
                DesirePos.pose.orientation.x = xq
                DesirePos.pose.orientation.y = yq
                DesirePos.pose.orientation.z = zq
                DesirePos.pose.orientation.w = wq
                x_error = xd_vec[j] - x
                y_error = yd_vec[j] - y
                z_error = zd_vec[j] - z
                norm_error = np.sqrt(x_error**2+ y_error**2+z_error**2)
                print('CFID',CFID,"Error",norm_error)
                desired_pos_pub.publish(DesirePos)
                # print "Desired Position X:", DesirePos.pose.position.x, "Y:", DesirePos.pose.position.y
                # print "Current Position X:", x, "Y:", y
                #print(xd_vec[j], yd_vec[j],zd_vec[j], yawd_vec[j], x, y,z)
                rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
