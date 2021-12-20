#!/usr/bin/env python3
import logging
import time
import rospy

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

def simple_connect():
    print("Yeah, I'm connected! :D")
    time.sleep(10)
    print("Now I will disconnect :'(")

def main():
    print('In main')
    rospy.init_node('connection')
    global rate
    rate = rospy.Rate(20)   #20 hz
# URI to the Crazyflie to connect to
    #global rate = rospy.Rate()
    uri = 'radio://0/80/2M/E7E7E7E701'

    while(not rospy.is_shutdown() ):
        print("in while loop")
        cflib.crtp.init_drivers()
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

            simple_connect()
        rate.sleep()

if __name__ == '__main__':
    # Initialize the low-level drivers
    try:
        main()
    except rospy.ROSInterruptException:
        pass
