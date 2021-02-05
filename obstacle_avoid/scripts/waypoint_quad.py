#!/usr/bin/env python

import rospy
import time
import math
import dji_sdk.msg
from std_msgs.msg import Float64MultiArray
old_box = []

def set_list(msg):
    global old_box
    data = list(msg.data)
    ax = data[1]
    ay = data[0]
    l = data[3]
    h = data[2]
    g = data[4]
    box = [ax,ay,h,l,g]
    if box!=old_box:
        wp_list = get_list(box)
        send_waypoint(wp_list)
    old_box = box


def get_list(box):
    ax = box[0]*math.pi/180.0 #lon
    ay = box[1]*math.pi/180.0 #lat
    h = box[3]#6371000 * by - 6371000 * ay # ay by lat
    l = box[2]#6371000 * bx * math.cos(ay) -  6371000 * ax * math.cos(ay)
    #bx = box[2]
    #by = box[3]

   #center_lat = (by - ay)/2.0 + ay
    #center_lon = (bx - ax)/2.0 + ax

    print("location", ay*180/math.pi, ax*180/math.pi)
    print("h l", h, l)
    step_l = 5000000  # meter
    #step_l = l_step_m / (111699.0 * 180/math.pi)  # per step in lat
    if step_l > math.fabs(l):
        step_l = l
    step_h = 1
    #length_of_lon_deg = math.fabs(math.cos(ax)) * (111699.0 * 180/math.pi)  # meters
    #step_h = h_step_m / length_of_lon_deg  # degrees per step in lon


    g = -box[4] * math.pi/180.0  # angle of box off lat
    vect_l = [math.cos(g + math.pi/2.0) * step_l,  math.sin(g + math.pi/2.0) * step_l  ]
    vect_h =  [math.cos(g) * step_h, math.sin(g) * step_h]#
    print('Vect_l' , vect_l)
    print('vect_h', vect_h)
    wp_list_x = [ay] # initalize with bottom left corner [when at 0 deg] #TODO ask cole
    wp_list_y = [ax]

    current_x = 6371000.0 * ay # lat of start
    current_y = 6371000.0 * ax * math.cos(ay) # lon in m at start
    print("1st current", current_x, current_y)
    h_dis = 0
    ii = 2 # used a a flag to change direction every row

    while h_dis <= h:
        if ii % 2 == 1:
            s = -1
        else:
            s = 1
        l_dis = 0
        while l_dis <= math.fabs(l) - step_l:
            #print(l_dis, step_l, l)
            current_x += s * vect_l[0]
            current_y += s * vect_l[1]
            print("c_xy", current_x, current_y)
            wp_list_x += [ current_x/(6371000.0)]
            wp_list_y += [ current_y/(6371000.0*math.cos(ay))]
            l_dis += step_l
        #print(l_dis, l, step_l)
        print("step up")
        h_dis += math.fabs(step_h)
        if h_dis <= math.fabs(h):
            current_x += vect_h[0]
            current_y += vect_h[1]
            print(current_x, current_y)
            wp_list_x += [ current_x/(6371000.0)]
            wp_list_y += [ current_y/(6371000.0*math.cos(ay))]
        else:
            break

        ii += 1

    return zip(wp_list_x,wp_list_y)

def norm_pi(angle):
    while angle > math.pi:
        angle -= math.pi*2
    while angle < - math.pi:
        angle += math.pi*2
    return angle

def send_waypoint(wp_list):

    msg = dji_sdk.msg.MissionWaypointTask()
    msg.velocity_range= 1
    msg.idle_velocity = 1
    list_of_wp = []

    for wp in wp_list:
        #print(wp, "\n")
        temp =  dji_sdk.msg.MissionWaypoint()
        temp.altitude = 2
        temp.latitude = norm_pi(wp[0])*180/math.pi
        temp.longitude = norm_pi(wp[1])*180/math.pi
      	print(norm_pi(wp[0])*180/math.pi, norm_pi(wp[1])*180/math.pi)
        temp.damping_distance = .5
        temp.target_gimbal_pitch = 4
        list_of_wp.append(temp)

    msg.mission_waypoint = list_of_wp

    pub = rospy.Publisher('waypoint_list', dji_sdk.msg.MissionWaypointTask, queue_size=10)
    pub.publish(msg)
    time.sleep(1)
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rotated_box", Float64MultiArray, set_list, queue_size=1)
    rospy.spin()
