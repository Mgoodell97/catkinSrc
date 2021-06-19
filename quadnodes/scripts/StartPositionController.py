#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
from std_msgs.msg import Bool

def main():
    rospy.init_node('startPositionControllerNode')

    rate = rospy.Rate(1) # pub every 1 seconds

    startingPublisher = rospy.Publisher('/startTopic', Bool, queue_size=1)

    starting_Msg = Bool()
    starting_Msg.data = False

    startTime = rospy.get_rostime()

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function

        if (rospy.Time.now() - startTime > rospy.Duration.from_sec(7.0)):
            starting_Msg.data = True
        else:
            starting_Msg.data = False

        startingPublisher.publish(starting_Msg)
        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
