#!/usr/bin/env python3

import logging
import time
import rospy

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller
from geometry_msgs.msg import PoseStamped

#---------------Global Variables------------
global crazyflie1_pose
crazyflie1_pose = PoseStamped()

global crazyflie3_pose
crazyflie3_pose = PoseStamped()


#-------------Functions --------------------
def crazyflie1_pose_cb(pose_cb_msg):
    global crazyflie1_pose
    crazyflie1_pose = pose_cb_msg

def crazyflie3_pose_cb(pose_cb_msg):
    global crazyflie3_pose
    crazyflie3_pose = pose_cb_msg

def main():
    print('Starting Hover ')
    rospy.Subscriber("/mocap_node/Crazyflie_1/pose", PoseStamped, crazyflie1_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_3/pose", PoseStamped, crazyflie3_pose_cb)
    # URI to the Crazyflie to connect to
    URI1 = 'radio://0/80/2M/E7E7E7E701'
    URI3 = 'radio://0/80/2M/E7E7E7E703'
    rospy.init_node('hover')
    global rate
    rate = rospy.Rate(20)   #20 hz

    logging.basicConfig(level=logging.ERROR)

    # Called when the link is established and the TOCs (that are not
    # cached) have been downloaded
    connected = Caller()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    cf = Crazyflie()
    cf3=Crazyflie()

    cf.connected.add_callback(connected)
    cf3.connected.add_callback(connected)
    cf.open_link(URI1)
    cf3.open_link(URI3)
    cf.commander.send_setpoint(0,0,0, 0)
    cf3.commander.send_setpoint(0,0,0, 0)

    while not rospy.is_shutdown():
        x_cf1 = crazyflie1_pose.pose.position.x
        y_cf1 = crazyflie1_pose.pose.position.y
        z_cf1 = crazyflie1_pose.pose.position.z

        x_cf3 = crazyflie3_pose.pose.position.x
        y_cf3 = crazyflie3_pose.pose.position.y
        z_cf3 = crazyflie3_pose.pose.position.z

        print("CF1:",x_cf1,y_cf1,z_cf1)
        print("CF3:",x_cf3,y_cf3,z_cf3)



        roll = 0.0
        pitch = 0.0
        yaw = 0
        thrust = 45000


        for _ in range(5):
        #cf.commander.send_setpoint(0,0,0, 0)
            cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            cf3.commander.send_setpoint(roll, pitch, yaw, thrust)
            x_cf1 = crazyflie1_pose.pose.position.x
            y_cf1 = crazyflie1_pose.pose.position.y
            z_cf1 = crazyflie1_pose.pose.position.z

            x_cf3 = crazyflie3_pose.pose.position.x
            y_cf3 = crazyflie3_pose.pose.position.y
            z_cf3 = crazyflie3_pose.pose.position.z

            print("CF1:",x_cf1,y_cf1,z_cf1)
            print("CF3:",x_cf3,y_cf3,z_cf3)

            time.sleep(0.1)

        for _ in range(50):
        #cf.commander.send_setpoint(0,0,0, 0)
            cf.commander.send_setpoint(roll, 10, yaw, thrust)
            cf3.commander.send_setpoint(roll, 10, yaw, thrust)
            x_cf1 = crazyflie1_pose.pose.position.x
            y_cf1 = crazyflie1_pose.pose.position.y
            z_cf1 = crazyflie1_pose.pose.position.z

            x_cf3 = crazyflie3_pose.pose.position.x
            y_cf3 = crazyflie3_pose.pose.position.y
            z_cf3 = crazyflie3_pose.pose.position.z

            print("CF1:",x_cf1,y_cf1,z_cf1)
            print("CF3:",x_cf3,y_cf3,z_cf3)

            time.sleep(0.1)

        for _ in range(50):
        #cf.commander.send_setpoint(0,0,0, 0)
            cf.commander.send_setpoint(roll, -10, yaw, thrust)
            cf3.commander.send_setpoint(roll, -10, yaw, thrust)
            x_cf1 = crazyflie1_pose.pose.position.x
            y_cf1 = crazyflie1_pose.pose.position.y
            z_cf1 = crazyflie1_pose.pose.position.z

            x_cf3 = crazyflie3_pose.pose.position.x
            y_cf3 = crazyflie3_pose.pose.position.y
            z_cf3 = crazyflie3_pose.pose.position.z

            print("CF1:",x_cf1,y_cf1,z_cf1)
            print("CF3:",x_cf3,y_cf3,z_cf3)

            time.sleep(0.1)

        #cf.commander.send_stop_setpoint()
        cf.close_link()
        cf3.close_link()
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
