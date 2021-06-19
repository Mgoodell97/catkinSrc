#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent

from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mps_driver.msg import MPS

from mavros_msgs.msg import State #include <mavros_msgs/State.h>

import numpy as np

def randomNumAtoB(a,b):
    return ((b - a) * np.random.random_sample() + a)

def main():
    rospy.init_node('Fake_pf_publisher')

    rate = rospy.Rate(1)

    PlumeType = rospy.get_param("PlumeType")       #  I'm not explaining this
    simulation = rospy.get_param("simulation")       #  I'm not explaining this

    mavrosPublisher = rospy.Publisher('mavros/state', State, queue_size=1)
    fake_mavros_Msg = State()
    fake_mavros_Msg.armed = True

    if simulation: #True
        if PlumeType == "gaussian": # if gaussian env subscribe to gaussian messages
            readingPublisher = rospy.Publisher('gaussianReading', gaussian, queue_size=1)
            locationPublisher = rospy.Publisher('true_position', PoseStamped, queue_size=1)
            fake_reading_Msg = gaussian()
            fake_location_Msg = PoseStamped()

        if PlumeType == "gaden": # if gaden subscribe to gaden messages
            readingPublisher = rospy.Publisher('Sensor_reading', gas_sensor, queue_size=1)
            locationPublisher = rospy.Publisher('Sensor_display', Marker, queue_size=1)
            fake_reading_Msg = gas_sensor()
            fake_location_Msg = Marker()
    else : #False
        readingPublisher = rospy.Publisher('MPS_Sensor_reading', MPS, queue_size=1)
        locationPublisher = rospy.Publisher('true_position', PoseStamped, queue_size=1)
        fake_reading_Msg = MPS()
        fake_location_Msg = PoseStamped()

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function

        fake_location_Msg.pose.position.x = randomNumAtoB(0,50)
        fake_location_Msg.pose.position.y = randomNumAtoB(0,50)
        fake_location_Msg.pose.position.z = 2

        if simulation: #True
            if PlumeType == "gaussian": # if gaussian env subscribe to gaussian messages
                fake_reading_Msg.kg_s = 0
                fake_reading_Msg.ppm = 0

            if PlumeType == "gaden": # if gaden subscribe to gaden messages
                fake_reading_Msg.raw = 0
        else : #False
            fake_reading_Msg.pressure = 0

        readingPublisher.publish(fake_reading_Msg)
        locationPublisher.publish(fake_location_Msg)
        mavrosPublisher.publish(fake_mavros_Msg)

        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
