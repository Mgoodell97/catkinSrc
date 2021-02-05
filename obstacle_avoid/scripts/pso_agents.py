#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import random
import numpy as np
import csv
import matplotlib.pyplot as plt
import copy
from std_msgs.msg import Header
import genpy
import time

import pso_class
import math
from mps_driver.msg import MPS
from geographic_msgs.msg import GeoPoint
from enif_iuc.msg import AgentMPS
from enif_iuc.msg import Waypoint
from enif_iuc.msg import WaypointTask
from enif_iuc.msg import AgentSource
from std_msgs.msg import Float64MultiArray

mode = rospy.get_param("mode")
bag = rospy.get_param("bag")
x_o = None
wp_info = [0,0,0,0]  # [stay_time, alt, vel, dmp_dist]
box = [(0,0), None, None]  # x y meters
old_box = box
position = np.array([0,0])
running = False
old_wp = None
current_wp = None
pso = None

wp_list = []

def latlon_2_m(lat,lon): # for Mode == 3 (mavros)
    global x_o
    if x_o is not None:
        x_m = 6371000.0 * lat * math.pi / 180
        y_m = -6371000.0 * lon * math.pi / 180 * math.cos(x_o * math.pi / 180.0)
        return [x_m, y_m]
    else:
        return [0,0]

def m_2_latlon(x,y):
    global x_o
    if x_o is not None:
        lat = 180 * x/(6371000.0 * math.pi)
        lon = 180 * y/(-6371000.0 * math.pi * math.cos(x_o * math.pi / 180.0))
        return [lat,lon]
    else:
        return [0,0]

def set_box(msg):
    global wp_info, box, x_o, stay_time, vel
    data = list(msg.data)
    if mode == 3:
        rr = rospy.Rate(1)
        while x_o is None:
            rr.sleep()
            print "waiting for x_o"
        ax = data[0]
        ay = data[1]
        temp = latlon_2_m(ax, ay)

    else:
        ax = data[0]
        ay = data[1]
        temp = [ax, ay]
    l = data[3]
    h = data[2]
    g = data[4] # rotation
    box = [(temp[0],temp[1]),h,-l] # ^x <y needs negative y length in input of pso
    stay_time = data[5]
    alt = data[6]
    vel = data[7]
    dmp_dist = data[8]
    wp_info = [stay_time, alt, vel, dmp_dist]



def send_waypoint_enif(xy, reached):
    global wp_info, old_wp, wp_list, current_wp # [stay_time, alt, vel, dmp_dist]

    if current_wp is None:
        current_wp = xy
        old_wp = current_wp
    elif current_wp[0] != xy[0] or current_wp[1] != xy[1]:
        if reached:
            old_wp = current_wp
            current_wp = xy
        else:
            current_wp = xy
            old_wp = current_wp
    wp_list = [old_wp, current_wp]

    msg = WaypointTask()
    msg.velocity = wp_info[2]
    msg.damping_distance = wp_info[3]
    list_of_wp = []
    for wp in wp_list:
        # print wp, "\n"
        if mode == 3:
            wp_latlon = m_2_latlon(wp[0], wp[1])
        else:
            wp_latlon = [wp[0], wp[1]]
        temp = Waypoint()
        temp.target_height = wp_info[1]
        temp.latitude = wp_latlon[0] # lat lon
        temp.longitude = wp_latlon[1]
        temp.staytime = wp_info[0]
        list_of_wp.append(temp)

    msg.mission_waypoint = list_of_wp
    if bag == 0:
        pub = rospy.Publisher('waypoint_list', WaypointTask, queue_size=10)
    else:
        pub = rospy.Publisher('waypoint_list_bag', WaypointTask, queue_size=10)

    pub.publish(msg)
    time.sleep(1)
    pub.publish(msg)

def update_self(msg):
    global position, pso, running
    print "update self"
    if mode == 3:
        position = latlon_2_m(msg.GPS_latitude, msg.GPS_longitude)
    elif mode == 1:
        #print "set self"
        position = [msg.local_x, msg.local_y]
    if running:

        con = msg.percentLEL
        #print "con", con
        pso.update_self(position, con)

def set_xo(msg):
    global x_o
    x_o = msg.latitude


def update_agents(msg):
    global pso
    if box[0] != (0, 0) and position[0] != 0 and pso is not None:
        agent_fit = msg.mps.percentLEL
        if mode == 3:
            agent_pos = np.array(latlon_2_m(msg.mps.GPS_latitude, msg.mps.GPS_longitude))
        elif mode == 1:
            agent_pos = np.array([msg.mps.local_x, msg.mps.local_y])

        pso.update_agents(agent_pos, agent_fit)

def update_agents_centralized(msg):
    global pso
    if box[0] != (0, 0) and position[0] != 0 and pso is not None and msg.source.altitude == 5000:
        agent_fit = msg.release_rate
        if mode == 3:
            agent_pos = np.array(latlon_2_m(msg.source.latitude, msg.mps.longitude))
        elif mode == 1:
            agent_pos = np.array([msg.mps.local_x, msg.mps.local_y])

        pso.update_agents(agent_pos, agent_fit)

if __name__ == '__main__':
    global box, old_box, old_wp

    rospy.init_node('listener', anonymous=True)
    r = rospy.Rate(1)

    rospy.Subscriber('mps_data', MPS, update_self, queue_size=1)
    rospy.Subscriber('agent_mps_data', AgentMPS, update_agents, queue_size=1)
    rospy.Subscriber("rotated_box", Float64MultiArray, set_box, queue_size=1)
    rospy.Subscriber('agent_mle_data', AgentSource, update_agents_centralized, queue_size=1)
    if mode == 3:
        rospy.Subscriber("xo", GeoPoint, set_xo, queue_size=1)

    while not rospy.is_shutdown():
        run_pso = rospy.get_param("/runAlg")
        print "run PSO?", run_pso
        if run_pso == 'PSO':
            if mode == 3:
                print "box", box, 'pos', position
                if box[0] != (0,0) and position[0] != 0:
                    running = True
                    if box != old_box:
                        print 'got a box'
                        print "box", m_2_latlon(box[0][0], box[0][1]), '\n', m_2_latlon(box[0][0] + box[1],
                                                                                        box[0][1]), '\n', m_2_latlon(
                            box[0][0] + box[1], box[0][1] + box[2]), '\n', m_2_latlon(
                            box[0][0], box[0][1] + box[2])
                        pso = pso_class.pso(box, position, stay_time, vel, use_wp=False)
                        old_box = box
                    else:
                        wp, reached = pso.get_waypoint()
                        temp = m_2_latlon(wp[0], wp[1])
                        send_waypoint_enif(wp, reached)
                    local_p, local_fit, global_p, global_fit = pso.pub_con()
                    latlon_local = m_2_latlon(local_p[0], local_p[1])
                    latlon_global = m_2_latlon(global_p[0], global_p[1])
                    best = AgentSource()
                    geo = GeoPoint()
                    geo.latitude = latlon_local[0]
                    geo.longitude = latlon_local[1]
                    geo.altitude = 5000
                    best.release_rate = local_fit
                    best.source = geo
                    best.diff_y = latlon_global[0]
                    best.diff_z = latlon_global[1]
                    best.angle = global_fit
                    pub_geo = rospy.Publisher('pf/targetGPS_', AgentSource, queue_size=1)
                    pub_geo.publish(best)

            elif mode == 1:
                print "mode 1", position

                if position[0] != 0:
                    print "box", box
                    if box[1] is not None:
                        print "box not none"
                        running = True
                        if box != old_box:
                            print 'got a box', box, position
                            pso = pso_class.pso(box, position, stay_time, vel)
                            old_box = box
                        else:
                            wp, reached = pso.get_waypoint()
                            print wp[0], ', ', wp[1]
                            send_waypoint_enif(wp, reached)
                        pso.pub_con()

        r.sleep()


    rospy.spin()


