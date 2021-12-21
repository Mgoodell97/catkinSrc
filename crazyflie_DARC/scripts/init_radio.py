#!/usr/bin/env python3

import rospy #include <ros/ros.h> cpp equivalent
import cflib.crtp
import logging
from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller
import time
#cflib.crtp.init_drivers()
def main():
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # #logging.basicConfig(level=logging.ERROR)
    #
    # #initialize low level drivers
    # cflib.crtp.init_drivers()
    # available = cflib.crtp.scan_interfaces()
    # print(available)
    # for i in available:
    #     print("Here1")
    #     print( "Interface with URI [%s] found and name/comment [%s]" , (i[0], i[1]))
    #
    # #time.sleep(1)
    # print("HERE")
    # #rospy.set_param('/startTest',True)
    # #print("Initialized Radio")

    rospy.spin()
if __name__ == '__main__':
    rospy.init_node('init_radio')
    try:
        main()
    except rospy.ROSInterruptException:
        pass
