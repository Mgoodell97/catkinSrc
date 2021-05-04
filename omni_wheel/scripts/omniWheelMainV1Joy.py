#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

desiredLocalPositionsKgains = [0.5,0.5,0.5]

# creates global variable for joystick
global controllerValues
controllerValues = Joy()
controllerValues.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
controllerValues.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def controller_cb(controller_sub):
    global controllerValues
    controllerValues = controller_sub
    # controllerValues.axes[3] = -controllerValues.axes[3] # Flip roll values


def main():
    # Initalize node
    rospy.init_node('OmniWheelMainNode')

    # Set up subscriptions
    rospy.Subscriber("joy", Joy, controller_cb)

    # Set up publishers
    local_vel_pub = rospy.Publisher('omniWheelDesiredVelocity', Twist, queue_size=100)

    global rate
    rate = rospy.Rate(20)

    DesiredVel = Twist()
    DesiredVel.linear.x = 0
    DesiredVel.linear.y = 0
    DesiredVel.linear.z = 0
    DesiredVel.angular.z = 0

    # Main loop after Initalization
    while not rospy.is_shutdown():
        # A = 0
        # B = 1
        # Y = 3
        printOnceBool = True
        desiredLocalVelocities = [0,0,0] # velovity vector for x, y, and angular velocities. For position and velocity control modes
        desiredLocalPositions = [0.0,0.0,0.0]

        if controllerValues.buttons[0] == 1: # if A is pressed go into autnomouse mode
            while not rospy.is_shutdown() and controllerValues.buttons[1] == 0 and controllerValues.buttons[3] == 0: # stay in autnomouse mode forever or until "B" or "Y" is pressed
                if printOnceBool:
                    print("In autnomouse mode")
                    printOnceBool = False

                # Do stuff here
                rate.sleep()

        elif controllerValues.buttons[1] == 1: # if B is pressed go into position mode
            while not rospy.is_shutdown() and controllerValues.buttons[0] == 0 and controllerValues.buttons[3] == 0: # stay in autnomouse mode forever or until "B" or "Y" is pressed
                if printOnceBool:
                    print("In position control mode")
                    printOnceBool = False

                desiredLocalVelocities = [-controllerValues.axes[3], controllerValues.axes[4], controllerValues.axes[0]] # roll = x, pitch = y, yaw = angular

                desiredLocalPositions[0] = desiredLocalPositions[0] + desiredLocalVelocities[0] * desiredLocalPositionsKgains[0]
                desiredLocalPositions[1] = desiredLocalPositions[1] + desiredLocalVelocities[1] * desiredLocalPositionsKgains[1]
                desiredLocalPositions[2] = desiredLocalPositions[2] + desiredLocalVelocities[2] * desiredLocalPositionsKgains[2]

                print "Desired positions X: ", desiredLocalPositions[0] , "Y: " , desiredLocalPositions[1], "w: " , desiredLocalPositions[2]

                rate.sleep()

        else:  # if no button is pressed go into velocity control mode
            while not rospy.is_shutdown() and controllerValues.buttons[0] == 0 and controllerValues.buttons[1] == 0: # stay in autnomouse mode forever or until "B" or "Y" is pressed
                if printOnceBool:
                    print("In velocity control mode")
                    printOnceBool = False

                scalingFactor = 460

                desiredLocalVelocities = [-controllerValues.axes[3], controllerValues.axes[4], controllerValues.axes[0]] # roll = x, pitch = y, yaw = angular


                # Controller frame
                #      +y
                #      ^
                #      |
                #      0 --> +x
                # Z is upward


                # Robot frame
                #      +x
                #      ^
                #      |
                #      0 --> +y
                # Z is downward

                DesiredVel.linear.x = -controllerValues.axes[3] * scalingFactor
                DesiredVel.linear.y = controllerValues.axes[4] * scalingFactor
                DesiredVel.angular.z = controllerValues.axes[0] * scalingFactor

                local_vel_pub.publish(DesiredVel)
                print "Desired velocities X: ", desiredLocalVelocities[0] * scalingFactor , "Y: " , desiredLocalVelocities[1] * scalingFactor, "w: " , desiredLocalVelocities[2] * scalingFactor

                rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
