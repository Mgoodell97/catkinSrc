#!/usr/bin/env python

# Moving Average Filter

import rospy
import genpy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
#from std_msgs.msg import Float32MultiArray
import csv
import numpy as np


import math


from mps_driver.msg import MPS
from enif_iuc.msg import AgentMPS
from enif_iuc.msg import AgentSource
# for file creation
import os
import datetime

mode = 1 #rospy.get_param("mode")
bag = 0 #rospy.get_param("bag")
fixed = False
if mode == 1:
    import matplotlib.pyplot as plt

trace = [[],[],[],[], [], []]
# for timing convergence
gBestFit = 0
gBestPos = np.array([0,0])
#
# #### Creating file for test data

if mode == 3 and bag == 0:
    working_dir = "/home/odroid/"
else:
    working_dir = "/home/matthew/bagDir/"

now = datetime.datetime.now()
test_file_dir = working_dir + "pso_data/" + str(now.year) + "-" + str(now.month) +'-' + str(now.day) + ' ' + str(now.hour) +'_' +str(now.minute) + '_' + str(now.second)
print(test_file_dir)
if not os.path.exists(test_file_dir):
    os.makedirs(test_file_dir)

file1 = open(test_file_dir + '/a1', "w")
file2 = open(test_file_dir + '/a2', "w")
file3 = open(test_file_dir + '/a3', "w")
####

#### For quad
source = [None, None]#[40.76429,-111.83490]
x_o = None


class c_map:
    '''
        Class to hold a grid map for navigation. Reads in a map.txt file of the format
        0 - free cell, x - occupied cell, g - goal location, i - initial location.
        Additionally provides a simple transition model for grid maps and a convience function
        for displaying maps.
        '''

    def __init__(self, map_path=None):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.origin = [0,0]
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.con_grid = None
        self.scale = 10
        if map_path is not None:
            self.read_map(map_path)

    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        #g = np.loadtxt(open(map_path, "rb"), delimiter=",")
        reader = csv.reader(open(map_path, "rb"))
        x = list(reader)
        self.con_grid = np.array(x).astype("float")
        self.rows = self.con_grid.shape[0]
        self.cols = self.con_grid.shape[1]

    def get_con(self, position, origin=None):
        print("get con", origin)

        if mode == 3:
            if origin[0] is not None:
                self.origin = origin
            else:
                return 0
        #print("xy", position, x_o)
        print("origin", self.origin, "pos", position)
        print("b4 scale", [(position[0] - self.origin[0]), (position[1] - self.origin[1])])
        #index = [int(round((position[0] - self.origin[0])* self.scale + 99)), int(round((position[1] - self.origin[1])* self.scale + 161))]
        index = [int(round(position[0] - self.origin[0])), int(round(position[1] - self.origin[1]))]
        print(index, self.rows, self.cols)
        if index[0] >= 0 and index[0] < self.rows and index[1] >= 0 and index[1] < self.cols:
            print(index)
            con = self.con_grid[index[0], index[1]]
            #if con < .75 and con > 0:
            #   print("below readable", con)
            #    con = 0
        else:
            print("outside")
            con = 0
        return con





def callback1(msg):
    global gBestPos, gBestFit, mode
    now = datetime.datetime.now()
    mps_data = MPS()
    #mps_data.humidity = now.
    if mode == 3:
        xy = latlon_2_m(msg.latitude, msg.longitude)
        frame = 1
        mps_data.GPS_latitude = msg.latitude
        mps_data.GPS_longitude = msg.longitude
        #xy = latlon_2_m(40.76432,-111.83497)
    elif mode == 1:
        xy = [msg.pose.position.x, msg.pose.position.y]
        print("xy", xy)
        frame = msg.header.frame_id
        #frame = int(frame[3])


    con = con_maps[now.second].get_con(xy, source)
    print("con", con)
    print("\n")

    r = rospy.Rate(10)

    mps_data.local_x = xy[0]
    mps_data.local_y = xy[1]

    mps_data.percentLEL = con

    # write to file
    #if pos.pose.orientation.x > gBestFit:
    #    print("NEW G BEST", pos.pose.orientation.x, [msg.pose.position.x, msg.pose.position.y], '\n')
    #    gBestFit = pos.pose.orientation.x
    #    gBestPos = [msg.pose.position.x, msg.pose.position.y]
    if mode == 1:
        sec = str(msg.header.stamp.secs)
        if int(frame[3]) == 1:
            file1.write(str(xy[0]) + ',' + str(xy[1]) + ',' + str(con) + ',' + str(gBestPos[0]) + ',' + str(
                gBestPos[1]) + ',' + sec + '\n')
        elif int(frame[3]) == 2:
            file2.write(str(xy[0]) + ',' + str(xy[1]) + ',' + str(con) + ',' + str(gBestPos[0]) + ',' + str(
                gBestPos[1]) + ',' + sec + '\n')
        elif int(frame[3]) == 3:
            file3.write(str(xy[0]) + ',' + str(xy[1]) + ',' + str(con) + ',' + str(gBestPos[0]) + ',' + str(
                gBestPos[1]) + ',' + sec + '\n')
    elif mode == 3:
        sec = str(msg.header.stamp.secs)
        if frame == 1:
            file1.write(str(msg.latitude) + ',' + str(msg.longitude) + ',' + str(con) + ',' + str(gBestPos[0]) + ',' + str(
                gBestPos[1]) + ',' + sec + ',' + str(x_o) +'\n')

    if mode == 1:
        agent_mps = AgentMPS()
        agent_mps.mps = mps_data
        agent_mps.agent_number = int(frame[3])
        # publish to mps_agent
        if int(frame[3]) == 1:
            r.sleep()
            #MPS1 = rospy.Publisher('uav1/mps_data', MPS, queue_size=1)

            MPS1.publish(mps_data)
            AgentMPS2.publish(agent_mps)
            AgentMPS3.publish(agent_mps)
            AgentMPS4.publish(agent_mps)
        elif int(frame[3]) == 2:
            r.sleep()
            MPS2.publish(mps_data)
            AgentMPS1.publish(agent_mps)
            AgentMPS3.publish(agent_mps)
            AgentMPS4.publish(agent_mps)
        elif int(frame[3]) == 3:
            r.sleep()
            MPS3.publish(mps_data)
            AgentMPS1.publish(agent_mps)
            AgentMPS2.publish(agent_mps)
            AgentMPS4.publish(agent_mps)
        elif int(frame[3]) == 4:
            r.sleep()
            MPS4.publish(mps_data)
            AgentMPS1.publish(agent_mps)
            AgentMPS2.publish(agent_mps)
            AgentMPS3.publish(agent_mps)

        if int(frame[3]) == 1:
            trace[0].append(msg.pose.position.x)
            trace[1].append(msg.pose.position.y)
            traj1.set_xdata(trace[0])
            traj1.set_ydata(trace[1])
        elif int(frame[3]) == 2:
            trace[2].append(msg.pose.position.x)
            trace[3].append(msg.pose.position.y)
            traj2.set_xdata(trace[2])
            traj2.set_ydata(trace[3])
        elif int(frame[3]) == 3:
            trace[4].append(msg.pose.position.x)
            trace[5].append(msg.pose.position.y)
            traj3.set_xdata(trace[4])
            traj3.set_ydata(trace[5])
    # elif mode == 3:
    #     MPS1 = rospy.Publisher('mps_data', MPS, queue_size=1)
    #
    #     MPS1.publish(mps_data)
    #
    #     r.sleep()


    #print(trace)
def plot_wp_callback1(msg):
    wp1.set_xdata(msg.linear.x)
    wp1.set_ydata(msg.linear.y)
def plot_wp_callback2(msg):
    wp2.set_xdata(msg.linear.x)
    wp2.set_ydata(msg.linear.y)
def plot_wp_callback3(msg):
    wp3.set_xdata(msg.linear.x)
    wp3.set_ydata(msg.linear.y)
def plot_wp_callback4(msg):
    wp4.set_xdata(msg.linear.x)
    wp4.set_ydata(msg.linear.y)

# plot best
def plotBest1(msg):
    pBest1.set_xdata(msg.pose.orientation.x)
    pBest1.set_ydata(msg.pose.orientation.y)
    global gBestPos, gBestFit
    gBestPos = [msg.pose.position.x, msg.pose.position.y]
    gBestFit = msg.pose.orientation.z
    #gBest.set_xdata(msg.pose.orientation.x)
    #gBest.set_ydata(msg.pose.orientation.y)
def plotBest2(msg):
    pBest2.set_xdata(msg.pose.orientation.x)
    pBest2.set_ydata(msg.pose.orientation.y)
def plotBest3(msg):
    pBest3.set_xdata(msg.pose.orientation.x)
    pBest3.set_ydata(msg.pose.orientation.y)
def plotBest4(msg):
    pBest4.set_xdata(msg.pose.orientation.x)
    pBest4.set_ydata(msg.pose.orientation.y)



def set_source(msg):
    global source, offset

    #print("source fixed", (40.76904,-111.84803))
    source = latlon_2_m(msg.source.latitude,msg.source.longitude)
    #source = latlon_2_m(40.76904,-111.84803)
def latlon_2_m(lat,lon): # for Mode == 3 (mavros)
    global x_o
    if x_o is not None:
        x_m = 6371000.0 * lat * math.pi / 180
        y_m = -6371000.0 * lon * math.pi / 180 * math.cos(x_o * math.pi / 180.0)
        return [x_m, y_m]
    else:
        return [None, None]

def set_xo(msg):
    global x_o
    x_o = msg.latitude

if __name__ == '__main__':
    con_maps = []
    for i in range(0,60):
        if 1: #not fixed:
            con_maps.append(c_map(working_dir + 'plume/' + str(i+1) + '.txt'))
        else:
            con_maps.append(c_map(working_dir + 'PSO_per/simulations/DSCC_open_plume/static/' + str(1) + '.txt')) # this won't work right now only dynamic plume
        #####
        #     source b4 offset (bottom left) , ... (-----> y)
        #     |
        #     v
        #     x     (rotate around x north y west)
        #con_maps.append(c_map('/home/darc/Dropbox/PSO/simulations/testplume/' + str(2) + '.txt'))
    rospy.init_node('listener', anonymous=True)

    #g = c_map('catkin_ws/src/obstacle_avoid/scripts/con_maps/testSite1.txt')
    if mode == 1:
        plt.imshow(con_maps[59].con_grid.transpose())
        plt.colorbar()
        plt.axis([0, 500, 0, 500])#'tight')
        plt.show(block=False)
        # uav1 = Green
        # uav2 = Red
        # uav3 = Yellow
        wp1, = plt.plot(-1, -1, 'go')
        wp2, = plt.plot(80, 40, 'ro')
        wp3, = plt.plot(0,0, 'yo')
        traj1, = plt.plot(0, 0, 'g')
        traj2, = plt.plot(0, 0, 'r')
        traj3, = plt.plot(0,0, 'y')
        pBest1, = plt.plot(0,0,'gd', markersize=9)
        pBest2, = plt.plot(0,0,'rd', markersize=9)
        pBest3, = plt.plot(0,0,'yd', markersize=9)
        #gBest, = plt.plot(0,0,'y*', markersize=10)

        plt.pause(.05)
        AgentMPS1 = rospy.Publisher('uav1/agent_mps_data', AgentMPS, queue_size=1)
        AgentMPS2 = rospy.Publisher('uav2/agent_mps_data', AgentMPS, queue_size=1)
        AgentMPS3 = rospy.Publisher('uav3/agent_mps_data', AgentMPS, queue_size=1)
        AgentMPS4 = rospy.Publisher('uav4/agent_mps_data', AgentMPS, queue_size=1)
        MPS1 = rospy.Publisher('uav1/mps_data', MPS, queue_size=1)
        MPS2 = rospy.Publisher('uav2/mps_data', MPS, queue_size=1)
        MPS3 = rospy.Publisher('uav3/mps_data', MPS, queue_size=1)
        MPS4 = rospy.Publisher('uav4/mps_data', MPS, queue_size=1)
        rospy.Subscriber("uav1/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
        rospy.Subscriber("uav2/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
        rospy.Subscriber("uav3/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
        rospy.Subscriber("uav4/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
        rospy.Subscriber("uav1/waypoint", Twist, plot_wp_callback1, queue_size=1)
        rospy.Subscriber("uav2/waypoint", Twist, plot_wp_callback2, queue_size=1)
        rospy.Subscriber("uav3/waypoint", Twist, plot_wp_callback3, queue_size=1)
        rospy.Subscriber("uav4/waypoint", Twist, plot_wp_callback4, queue_size=1)
        rospy.Subscriber("uav1/best", PoseStamped, plotBest1, queue_size=1)
        rospy.Subscriber("uav2/best", PoseStamped, plotBest2, queue_size=1)
        rospy.Subscriber("uav3/best", PoseStamped, plotBest3, queue_size=1)
        rospy.Subscriber("uav4/best", PoseStamped, plotBest4, queue_size=1)

    else:
        rospy.Subscriber("mavros/global_position/global", NavSatFix,callback1, queue_size=1)
        rospy.Subscriber("agent_source_data", AgentSource, set_source, queue_size=1)
        rospy.Subscriber("xo", GeoPoint, set_xo, queue_size=1)


    r = rospy.Rate(100)
    if mode == 1:
        while not rospy.is_shutdown():

            plt.draw()
            r.sleep()
            plt.pause(0.05)

    rospy.spin()
