#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import TwistStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>

# cpp geometry_msgs::Twist msg;
# creates global variable called current_state and uses State message type
global current_state
current_state = State()
# create a simple callback which will save the current state of the autopilot. This will allow us to check connection, arming and Offboard flags
def state_cb(state_sub):
    global current_state
    current_state = state_sub

def initFunc():
    rospy.init_node('OffboardController') #node name is offb_node

    rospy.Subscriber("mavros/state", State, state_cb)

    global local_vel_pub
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1000)

    service_timeout = 30

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    global arming_client
    global set_mode_client
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    #the setpoint publishing rate MUST be faster than 2Hz
    #The px4 flight stack has a timeout of 500ms between two Offboard commands

    #cpp ros::Rate rate(20.0);
    global rate
    rate = rospy.Rate(20)
    #Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
    while not rospy.is_shutdown() and current_state.connected:
        # spinonce is not needed in python
        rate.sleep()

    #Desired xyz locations
    #Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2.
    DesiredVel = TwistStamped()
    DesiredVel.twist.linear.x = 0
    DesiredVel.twist.linear.y = 0
    DesiredVel.twist.linear.z = 1

    #send a few setpoints before starting
    #Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
    for i in range(100, 0, -1):
        local_vel_pub.publish(DesiredVel)
        rate.sleep()

def main():
    initFunc()

    last_request = rospy.get_rostime()

    DesiredVel = TwistStamped()
    DesiredVel.twist.linear.x = 0
    DesiredVel.twist.linear.y = 0
    DesiredVel.twist.linear.z = 1

    while not rospy.is_shutdown():
        if ( not current_state.mode == "OFFBOARD" and (rospy.get_rostime() - last_request) > rospy.Duration.from_sec(5.0)):
            modeResponse = set_mode_client(0,"OFFBOARD")
            if (modeResponse.mode_sent):
                rospy.loginfo("Offboard enabled")
            last_request = rospy.get_rostime()
        else:
            if (not current_state.armed and ((rospy.get_rostime() - last_request) > rospy.Duration.from_sec(5.0))):
                armResponse = arming_client(True)
                if(armResponse.success):
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.get_rostime()

        local_vel_pub.publish(DesiredVel);
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
