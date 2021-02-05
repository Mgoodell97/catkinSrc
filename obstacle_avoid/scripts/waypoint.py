#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int64MultiArray

from mps_driver.msg import MPS
from enif_iuc.msg import AgentMPS
from enif_iuc.msg import WaypointTask

run_pso = rospy.get_param("/runAlg")
if run_pso == "PSO":
    wp_list = None

else:
    #wp_list = [0, 0, 10, 0]  # [23.17113412630456, 8.283632510694414]#[4, 4] #, 10, 10 ]
    #wp_list = None # [23.17113412630456, 8.283632510694414]#[4, 4] #, 10, 10 ]
    wp_list = [0,0,10,0,10,10,0,10]
wp_rad = 2
wp_num = 0
hold_flag = 0
heading = 0
stay_time = 10
z = 2.5
wp_vel = 1 #max 5.5
old_box = []
new_list = False
def main_cmd(msg):
    global  new_list
    if wp_list is not None:
        if new_list:
            wp_num = 0
            hold_flag = 0
            new_list = False
        rospy.sleep(.1)
        global wp_num, hold_flag, heading, stay_time, z, wp_list
        wp_i = wp_list[wp_num:wp_num+2]
        position = msg.pose.position
        lat = position.x
        lon = position.y
        xy = [lat,lon]
        # print("goal", wp_i, "true", xy)
        dis = ((xy[0] - wp_i[0])**2 + (xy[1] - wp_i[1])**2)**.5
        print(dis)
        if dis < wp_rad:
            #print("in wp_rad")
            if wp_num + 2 > len(wp_list) - 1:
                wp_num = -2
            nxt_wp = wp_list[wp_num + 2:wp_num + 4]
            #print("nxt_wp", nxt_wp)
            heading = math.atan2(nxt_wp[1] - wp_i[1], nxt_wp[0] - wp_i[0]) # in rads
            start_time = rospy.get_time()
            while rospy.get_time() - start_time < stay_time:
                rospy.sleep(.5)
                print("hold send heading ", heading)
                hold_flag = 1
                send_waypoint(wp_i[0], wp_i[1], hold_flag, heading,z,wp_vel)
            print("done holding")
            hold_flag = 0
            wp_num += 2
        send_waypoint(wp_i[0], wp_i[1], hold_flag, heading, z,wp_vel)


def set_path(msg):
    global wp_list, wp_num, stay_time
    stay_time = 0
    print("here")
    wp_list = []
    for i in msg.data:
        wp_list += [i/2.0]
    wp_num = 0
    print(wp_list)


def set_list(msg):
    global old_box, wp_list
    a1 = msg.linear.x
    a2 = msg.linear.y
    b1 = msg.angular.x
    b2 = msg.angular.y
    box = [a1,a2,b1,b2]
    if box!=old_box:
        wp_list = get_list(box)
    old_box = box


def get_list(box):

    ax = int(box[0])
    ay = int(box[1])
    bx = int(box[2])
    by = int(box[3])


    ax_m = 6371000 * math.cos(ax) * math.cos(ay)
    ay_m = 6371000 * math.cos(ax) * math.sin(ay)
    bx_m = 6371000 * math.cos(bx)  * math.cos(by)
    by_m = 6371000 * math.cos(bx) * math.sin(by)

    l = bx_m - ax_m
    h = by_m - ay_m

    l_step = l
    h_step = 2
    wp_list = []
    length = range(ax, ax+l+1, int(l_step))
    print("length", length)
    height = range(ay, ay+h, int(h_step))

    bb = 0
    for j in range(0,len(height)):
        for i in range(0,len(length)):
            if bb % 2 == 1:
                length_list = list(reversed(length))
            else:
                length_list = length
            xy = (length_list[i], height[j])
            #wp_list.append(length_list[i])
            #wp_list.append(height[j])
            wp_list.append(xy)
        bb += 1
    wp_latlon_list = []
    for wp in wp_list:
        wp_latlon_list.append()
    return wp_list


def send_waypoint(x,y,hold,heading, z, wp_vel):

    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.x = hold
    msg.angular.y= wp_vel
    msg.angular.z = heading
    print("wp", x, y)
    pub = rospy.Publisher('waypoint', Twist, queue_size=10)
    pub.publish(msg)


def set_list_from_enif(msg):
    global wp_list, wp_num, stay_time, new_list, z, wp_vel
    new_wp_list = []
    stay_time = msg.mission_waypoint[0].staytime
    z = msg.mission_waypoint[0].target_height
    wp_vel = msg.velocity
    for ii in range(len(msg.mission_waypoint)):
        new_wp_list.append(msg.mission_waypoint[ii].latitude)
        new_wp_list.append(msg.mission_waypoint[ii].longitude)
    if new_wp_list != wp_list:
        wp_list = new_wp_list
        new_list = True


if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("ground_truth_to_tf/pose", PoseStamped, main_cmd, queue_size=1)
    rospy.Subscriber("box", Twist, set_list, queue_size=1)
    rospy.Subscriber('wp_list', Int64MultiArray, set_path, queue_size=2)
    rospy.Subscriber("waypoint_list", WaypointTask, set_list_from_enif, queue_size=1)
    rospy.spin()


    ####lat = asin(z / R)
#lon = atan2(y, x)
