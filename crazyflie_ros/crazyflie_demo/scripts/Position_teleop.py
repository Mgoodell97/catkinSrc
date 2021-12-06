#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Joy
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty as Emptymsg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
from math import *
from numpy import *

class Controller():
    def __init__(self, joy_topic, deadband):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        self._land = False
        self._takeoff = False

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        self._JoyInput = Twist()
        self._deadband = deadband
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)
        
        # subscribe to crazyflie pose
        self._pose = None
        rospy.Subscriber("pose", PoseStamped, self._poseChanged)

        # subscribe to crazyflie angle
        self._angle = Vector3
        rospy.Subscriber("angle", Vector3, self._angleChanged)

    def _joyChanged(self, data):
        # Implement deadband
        if data.axes[4] > fabs(self._deadband) or data.axes[4] < -fabs(self._deadband):
            self._JoyInput.linear.x = data.axes[4]
        else:
            self._JoyInput.linear.x = 0
        if data.axes[3] > fabs(self._deadband) or data.axes[3] < -fabs(self._deadband):
            self._JoyInput.linear.y = data.axes[3]
        else:   
            self._JoyInput.linear.y = 0
        if data.axes[1] > fabs(self._deadband) or data.axes[1] < -fabs(self._deadband):
            self._JoyInput.linear.z = data.axes[1]
        else:   
            self._JoyInput.linear.z = 0
        if data.axes[0] > fabs(self._deadband) or data.axes[0] < -fabs(self._deadband):
            self._JoyInput.angular.z = data.axes[0]
        else:   
            self._JoyInput.angular.z = 0

        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != _buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land == False:
                    self._land = True
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._takeoff == False:
                    self._takeoff = True
                if i == 6 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ACRO/roll_flip", 1))
                    rospy.set_param("ACRO/roll_flip", 1)
                    print("Roll Flip!!!")
                    self._update_params(["ACRO/roll_flip"])
                    rospy.set_param("kalman/initialX", 0)
                    self._update_params(["kalman/initialX"])
                    rospy.set_param("kalman/initialY", 0)
                    self._update_params(["kalman/initialY"])
                    rospy.sleep(1.1)
                    rospy.set_param("kalman/resetEstimation", 1)
                    self._update_params(["kalman/resetEstimation"])
                    rospy.sleep(0.1)
                    rospy.set_param("kalman/resetEstimation", 0)
                    self._update_params(["kalman/resetEstimation"])

                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ACRO/pitch_flip", 1))
                    rospy.set_param("ACRO/pitch_flip", 1)
                    print("Pitch Flip!!!")
                    self._update_params(["ACRO/pitch_flip"])

    def _poseChanged(self, data):
        self._pose = data

    def _angleChanged(self, data):
        self._angle = data


if __name__ == '__main__':
    rospy.init_node('position_teleop', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    joy_topic  = rospy.get_param("~joy_topic", "/joy")
    deadband   = rospy.get_param("~deadband", 0)
    controller = Controller(joy_topic, deadband)
    
    frequency = 50
    rate = rospy.Rate(frequency) # 10 hz

    msgPos = Position()
    msgPos.header.seq = 0
    msgPos.header.stamp = rospy.Time.now()
    msgPos.header.frame_id = worldFrame
    msgPos.x = 0.0
    msgPos.y = 0.0
    msgPos.z = 0.0
    msgPos.yaw = 0.0

    msgHov = Hover()
    msgHov.header.seq = 0
    msgHov.header.stamp = rospy.Time.now()
    msgHov.header.frame_id = worldFrame
    msgHov.yawrate = 0

    pubPos = rospy.Publisher("cmd_position", Position, queue_size=1)
    pubHov = rospy.Publisher("cmd_hover", Hover, queue_size=1)

    stop_pub = rospy.Publisher("cmd_stop", Emptymsg, queue_size=1)
    stop_msg = Emptymsg()

    update_params = rospy.ServiceProxy('update_params', UpdateParams)
    InAir = False
    
    # main loop
    while not rospy.is_shutdown():
        if controller._takeoff == True:
            update_params(["kalman/initialX"])
            rospy.sleep(0.1)
            update_params(["kalman/initialY"])
            rospy.sleep(0.1)
            rospy.set_param("kalman/resetEstimation", 1)
            update_params(["kalman/resetEstimation"])
            rospy.sleep(0.1)
            rospy.set_param("kalman/resetEstimation", 0)
            update_params(["kalman/resetEstimation"])
            rospy.sleep(0.5)
            msgPos.x = 0
            msgPos.y = 0
            msgPos.z = 0.4
            msgPos.yaw = 0.0
            msgPos.header.seq += 1
            msgPos.header.stamp = rospy.Time.now()
            controller._takeoff = False
            InAir = True
        if controller._land == True:
            msgPos.z = 0.05
            msgPos.header.seq += 1
            msgPos.header.stamp = rospy.Time.now()
            if controller._pose.pose.position.z < 0.08:
                stop_pub.publish(stop_msg)
                controller._land = False
                InAir = False
        if InAir == True:
            msgPos.x   += (controller._JoyInput.linear.x*cos(controller._angle.z/57.32) - controller._JoyInput.linear.y*sin(controller._angle.z/57.32))  * 1./frequency
            msgPos.y   += (controller._JoyInput.linear.y*cos(controller._angle.z/57.32) + controller._JoyInput.linear.x*sin(controller._angle.z/57.32))  * 1./frequency
            msgPos.z   += controller._JoyInput.linear.z  * 1./frequency
            if msgPos.z > 1.0:
                msgPos.z = 1.0
            msgPos.yaw += controller._JoyInput.angular.z * 1./frequency * 100
            pubPos.publish(msgPos)
        if rospy.is_shutdown():
            break
        rate.sleep()
    
