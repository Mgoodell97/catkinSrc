#!/usr/bin/env python

import rospy
#import<turtlesim/Spawn.h>
from turtlesim.srv import Spawn
import math

def mainLoop():
    rospy.init_node('spawn_turtle', anonymous=True) #node name is offb_node
    #ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
    spawnClient = rospy.ServiceProxy('spawn', Spawn)

    response = spawnClient(2, 2, math.pi / float(2), 'Leo')

    rospy.loginfo(response)


if __name__ == '__main__':
    try:
        mainLoop()
    except rospy.ROSInterruptException:
        pass
