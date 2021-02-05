#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from std_srvs.srv import Empty

def setBgColorLoop():
    rospy.init_node('set_bg_color')

    rospy.wait_for_service("clear")

    rospy.set_param("/turtlesim/background_r",255);
    rospy.set_param("/turtlesim/background_g",255);
    rospy.set_param("/turtlesim/background_b",0);

    #ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
    #std_srvs::Empty srv;

    clearClient = rospy.ServiceProxy('/clear', Empty)

    clearClient()

if __name__ == '__main__':
    try:
        setBgColorLoop()
    except rospy.ROSInterruptException:
        pass
