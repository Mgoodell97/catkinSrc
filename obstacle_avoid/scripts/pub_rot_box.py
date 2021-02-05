#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    pub1 = rospy.Publisher('rotated_box', Float64MultiArray, queue_size=1)
    r = rospy.Rate(.5)

    while not rospy.is_shutdown():
        box = Float64MultiArray()
        box.data = [0,100,100,100,0,5,2,1,2]
        pub1.publish(box)
        r.sleep()




    rospy.spin()


