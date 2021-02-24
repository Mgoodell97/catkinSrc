#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>

# cpp geometry_msgs::Twist msg;
# py msg = Twist()
# creates global variable called current_state and uses State message type
global current_state
current_state = State()
# create a simple callback which will save the current state of the autopilot. This will allow us to check connection, arming and Offboard flags

def state_cb(state_sub):
    global current_state
    current_state = state_sub
    rospy.loginfo("Callback   Current mode: %s  Current state: %s" , current_state.mode, current_state.connected)


def main():
    rospy.init_node('offb_node') #node name is offb_node

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1000)

    service_timeout = 30

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    #the setpoint publishing rate MUST be faster than 2Hz
    #The px4 flight stack has a timeout of 500ms between two Offboard commands

    rate = rospy.Rate(20)
    #Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
    while not rospy.is_shutdown() and current_state.connected:
        # spinonce is not needed in python
        rate.sleep()

    #Desired xyz locations
    #Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why we set z to positive 2.
    Desiredpose = PoseStamped()
    Desiredpose.pose.position.x = 0
    Desiredpose.pose.position.y = 0
    Desiredpose.pose.position.z = 2

    #send a few setpoints before starting
    #Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
    for i in range(100, 0, -1):
        local_pos_pub.publish(Desiredpose)
        rate.sleep()

    last_request = rospy.get_rostime()

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

        local_pos_pub.publish(Desiredpose);

        #rospy.spin()
        rospy.loginfo("Whileloop   Current mode: %s  Current state: %s" , current_state.mode, current_state.connected)
        #current_state = rospy.wait_for_message("mavros/state", State, timeout=None)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
