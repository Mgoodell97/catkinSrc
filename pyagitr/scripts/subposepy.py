#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose


# a callback function. Executed each time a new pose message arrives
def poseMessageRecieved(msg):
    #This will look like
    #[INFO] [1592493304.000765]: Position: x = 5.544445   y = 5.544445    Direction = 0.000000
    rospy.loginfo("Position: x = %f   y = %f    Direction = %f",msg.x, msg.y,msg.theta)

    #This will look like
    #[INFO] [1592493329.881013]: /sub_to_pose_4566_1592493328308 position=(5.544444561,5.544444561) direction=0.0
    rospy.loginfo(rospy.get_caller_id() + ' position=(' + str(msg.x) + ',' + str(msg.y) + ') direction=' + str(msg.theta))

def listener():
    rospy.init_node('sub_to_pose', anonymous=True) #node name is sub_to_pose
    rospy.Subscriber("turtle1/pose", Pose, poseMessageRecieved)
    rospy.spin()

if __name__ == '__main__':
  listener()
