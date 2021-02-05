#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import random

random.seed(0)

def velPublisher():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1000)
    rospy.init_node('publish_velocity', anonymous=True)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = random.random() # picks a random number from 0 to 1
        msg.angular.z = 2*(random.random()-0.5); # picks a random number from -1 to 1
        # Publish the message.
        pub.publish(msg);

        # Send a message to rosout with the details.
        rospy.loginfo("Sending random velocity command: linear = %f  angular= %f ",msg.linear.x, msg.angular.z)
        # Wait until it's time for another iteration.
        rate.sleep();


if __name__ == '__main__':
    try:
        velPublisher()
    except rospy.ROSInterruptException:
        pass
