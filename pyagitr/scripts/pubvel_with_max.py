#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import Twist #include <geometry_msgs/Twist.h>
import random

random.seed(0)
msg = Twist()

def velPublisherWithMax():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1000)
    rospy.init_node('publish_velocity', anonymous=True)
    rate = rospy.Rate(2)


    #Get the maximum velocity parameter
    #const std::string PARAM_NAME = "~max_vel";
    #double maxVel;
    max_vel = rospy.get_param('~max_vel', 'nan')

    #bool ok = ros::param::get(PARAM_NAME, maxVel);
    if max_vel == 'nan':
        rospy.logfatal('Could not get parameter ~max_vel')
        exit(1)

    while not rospy.is_shutdown():

        msg.linear.x = max_vel*random.random() # picks a random number from 0 to 1
        msg.angular.z = random.uniform(-1, 1); # picks a random number from -1 to 1
        # Publish the message.
        pub.publish(msg);

        # Send a message to rosout with the details.
        rospy.loginfo("Sending random velocity command: linear = %f  angular= %f ",msg.linear.x, msg.angular.z)
        # Wait until it's time for another iteration.
        rate.sleep();

if __name__ == '__main__':
    try:
        velPublisherWithMax()
    except rospy.ROSInterruptException:
        pass
