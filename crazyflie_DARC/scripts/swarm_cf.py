#!/usr/bin/env python3

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import math
import numpy as np
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import Swarm
from cflib.utils.callbacks import Caller
from cflib.crazyflie.swarm import CachedCfFactory

#---------------Global Variables------------
global crazyflie1_pose
crazyflie1_pose = PoseStamped()

global crazyflie2_pose
crazyflie2_pose = PoseStamped()

global crazyflie3_pose
crazyflie3_pose = PoseStamped()
#-------------Functions --------------------
def crazyflie1_pose_cb(pose_cb_msg):
    global crazyflie1_pose
    crazyflie1_pose = pose_cb_msg

def crazyflie2_pose_cb(pose_cb_msg):
    global crazyflie2_pose
    crazyflie2_pose = pose_cb_msg

def crazyflie3_pose_cb(pose_cb_msg):
    global crazyflie3_pose
    crazyflie3_pose = pose_cb_msg

def initialize(scf):
    cf = scf.cf
    cf.commander.send_setpoint(0,0,0,0)

def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)

def run_sequence(scf,values):
    cf = scf.cf
    cf.commander.send_setpoint(values[0],values[1],values[2],values[3])

def take_off(scf):
    cf = scf.cf
    cf.commander.send_setpoint(0,0,0,45000)

def controller(xd,yd,zd,x,y,z,x_errori,y_errori,z_errori):
    Kd = 20.0 * (10**5) * 1
    Kp = 5.5 * (10**4)

    KpXY = 5
    KdXY = 1000

    x_error = xd - x
    y_error = yd - y
    z_error = zd - z

    roll = (KpXY * x_error + KdXY * (x_error-x_errori)) * 1

    pitch =  (KpXY * y_error + KdXY * (y_error-y_errori)) * 1

    thrustc = (Kp * z_error + Kd * (z_error-z_errori))*1

    thrust = 42000 +  thrustc

    #print (thrustc)

    if thrust >= 60000:
        thrust = 60000
    elif (thrust <= 10001):
        thrust = 10001



    return roll, pitch, thrust ,x_error,y_error,z_error

def main():

    # Initalize node
    rospy.init_node('swarmControl')
    print('Starting swarm ')

    global rate
    rate = rospy.Rate(120)

    #Initialize crazyflies
    URI1 = 'radio://0/80/2M/E7E7E7E701'
    URI2 ='radio://0/80/2M/E7E7E7E702'
    URI3 ='radio://0/80/2M/E7E7E7E703'
    uris = {URI1,
            URI2,
            URI3}

    rospy.Subscriber("/mocap_node/Crazyflie_1/pose", PoseStamped, crazyflie1_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_2/pose", PoseStamped, crazyflie2_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_3/pose", PoseStamped, crazyflie3_pose_cb)

    local_pose_pub = rospy.Publisher('Position', Pose, queue_size=10)
    Pose_pub = Pose()
    Pose_pub.position.x = 0 #thrust
    Pose_pub.position.y = 0 #error
    Pose_pub.position.z = 0 #z position

    yerror1 = 0
    xerror1 = 0
    zerror1 = 0

    yerror2 = 0
    xerror2 = 0
    zerror2 = 0

    yerror3 = 0
    xerror3 = 0
    zerror3 = 0
    var = 1
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print("Preparing to initialize...")
        print('Waiting for parameters to be downloaded...')
        swarm.parallel(wait_for_param_download)
        swarm.parallel(initialize)
        #swarm.parallel(take_off)
        while(not rospy.is_shutdown() ):
            print('Running Sequence')
            if var == 1:
                if crazyflie1_pose.pose.position.x != 0.0 and crazyflie1_pose.pose.position.y!=0.0:
                    xd1= crazyflie1_pose.pose.position.x
                    yd1 = crazyflie1_pose.pose.position.y
                    zd1 = 0.45
                if crazyflie2_pose.pose.position.x != 0.0 and crazyflie2_pose.pose.position.y!=0.0:
                    xd2= crazyflie2_pose.pose.position.x
                    yd2 = crazyflie2_pose.pose.position.y
                    zd2 = 0.45

                if crazyflie3_pose.pose.position.x != 0.0 and crazyflie3_pose.pose.position.y!=0.0:
                    xd3= crazyflie3_pose.pose.position.x
                    yd3 = crazyflie3_pose.pose.position.y
                    zd3 = 0.45
                var = 0

            # xd1 = 2.5
            # yd1 = 1.0
            # zd1 = 0.45
            #
            # xd2 = 1.5
            # yd2 = 0.5
            # zd2 = 0.45
            #
            # xd3 = 2.6
            # yd3 = 1.5
            # zd3 = 0.45

            x1=crazyflie1_pose.pose.position.x
            y1 =crazyflie1_pose.pose.position.y
            z1 = crazyflie1_pose.pose.position.z

            x2=crazyflie2_pose.pose.position.x
            y2 =crazyflie2_pose.pose.position.y
            z2 = crazyflie2_pose.pose.position.z

            x3=crazyflie3_pose.pose.position.x
            y3 =crazyflie3_pose.pose.position.y
            z3 = crazyflie3_pose.pose.position.z

            roll1,pitch1,thrust1,xerror1,yerror1,zerror1 = controller(xd1,yd1,zd1,x1,y1,z1,xerror1,yerror1,zerror1)
            roll2,pitch2,thrust2,xerror2,yerror2,zerror2 = controller(xd2,yd2,zd2,x2,y2,z2,xerror2,yerror2,zerror2)
            roll3,pitch3,thrust3,xerror3,yerror3,zerror3 = controller(xd3,yd3,zd3,x3,y3,z3,xerror3,yerror3,zerror3)
            print("CF1 Desired:", xd1,yd1,zd1)
            print("CF1 Actually:", x1,y1,z1)
            print("CF2 Desired:", xd2,yd2,zd2)
            print("CF2 Actually:", x2,y2,z2)
            print("CF3 Desired:", xd3,yd3,zd3)
            print("CF3 Actually:", x3,y3,z3)
            print("Roll vals",roll1,roll2,roll3)
            print("Pitch vals",pitch1,pitch2,pitch3)
            print("Thrust Vals",thrust1,thrust2,thrust3)
            values = {
            URI1:[(roll1,pitch1,0,math.floor(thrust1))],
            URI2:[(roll2,pitch2,0,math.floor(thrust2))],
            URI3:[(roll3,pitch3,0,math.floor(thrust3))]
            }
            swarm.parallel(run_sequence,values)

            Pose_pub.position.x = yerror1 #thrust
            Pose_pub.position.y = yerror3 #error
            Pose_pub.position.z = pitch1 #z position
            local_pose_pub.publish(Pose_pub)

            rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
