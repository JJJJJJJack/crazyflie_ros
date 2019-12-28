#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
from LPfilter import LPfilter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

_trajectory_start = False
_buttons = None
_velFilter = LPfilter(20, 0.02)

def _joyChanged(data):
    global _buttons, _trajectory_start
    #print data
    for i in range(0, len(data.buttons)):
        if _buttons == None or data.buttons[i] != _buttons[i]:
            if i == 7 and data.buttons[i] == 1:
                if _trajectory_start == False:
                    _trajectory_start = True
                    print("Start tracking trajectory...")
                else:
                    _trajectory_start = False
                    print("Stop tracking trajectory...")
    _buttons = data.buttons

def _saturate(data, upperbound, lowerbound):
    if data > upperbound:
        data = upperbound
    if data < lowerbound:
        data = lowerbound
    return data

def _getDistance(x, y):
    distance = math.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)
    return distance

def _getNorm(x, y):
    D = _getDistance(x,y)
    output = [(x[0]-y[0])/D, (x[1]-y[1])/D, (x[2]-y[2])/D]
    return output

def _getVector(base, quad, a, b):
    v = [0,0,0]
    d = _getDistance(base,quad)
    if a>3:
        a=3
    norm = _getNorm(quad, base)
    a = 0
    v[0] = 0.5*(a+1)*(norm[0])/d**1.5
    v[1] = 0.5*(a+1)*(norm[1])/d**1.5
    v[2] = 0.5*(a+1)*(norm[2])/d**1.5
    return v

def _accumulateV(acc_v, v, hand, quad):
    output = [0,0,0]
    v_abs = _getDistance(v, [0,0,0])
    if v_abs < 0.05 or _getDistance(quad, hand) > 3.5 or v_abs > 3:
        return acc_v
    elif v_abs > 2:
        v[0] = v[0]/v_abs*2
        v[1] = v[1]/v_abs*2
        v[2] = v[2]/v_abs*2
    output[0] = acc_v[0] + 0.02*v[0]
    output[1] = acc_v[1] + 0.02*v[1]
    output[2] = acc_v[2] + 0.02*v[2]
    output[0] = _saturate(output[0], 2, -2)
    output[1] = _saturate(output[1], 2, -2)
    return output

def _getVel(pose, last_pose, rate):
    global _velFilter
    pose_array = np.array(pose)
    last_pose_array = np.array(last_pose)
    if _getDistance(pose, last_pose) < 0.3:
        vel = np.multiply(np.subtract(pose_array, last_pose_array), rate)
        vel_filter = list(_velFilter.update(vel, 5, 1.0/rate))
    else:
        vel_filter = [0,0,0]
    return vel_filter

if __name__ == '__main__':
    global _trajectory_start
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    rate = rospy.Rate(r)
    
    # subscribe joystick to enable trajectory tracking or wand waving
    joy_topic = rospy.get_param("~joy_topic", "joy")
    rospy.Subscriber(joy_topic, Joy, _joyChanged)

    # read wand enable option
    wand_enable = rospy.get_param('~wand_enable', False)
    wand_name = rospy.get_param('~wand_name', "/vicon/Wand/Wand")
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    filehandle = open("/home/jack/Documents/darc/XH/data/ddrone.txt","r")
    data_str = filehandle.readline().split(" ")
    msg.pose.position.x = float(data_str[1])
    msg.pose.position.y = float(data_str[2])
    msg.pose.position.z = float(data_str[3])

    _last_trajectory_start = _trajectory_start
    _trajectory_count = 0
    quad_goal = [float(data_str[1]), float(data_str[2]), float(data_str[3])]
    last_hand = [0,0,0]
    accumulate_v = [0,0,0]
    strength = 3
    max_dis = 1.2
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        if _trajectory_start == True:
            if _last_trajectory_start == False:
                _trajectory_count = 0;
            data_str = filehandle.readline().split(" ")
            try:
                (trans_hand,rot_hand) = listener.lookupTransform("/world", "/vicon/hand/hand", rospy.Time(0))
                (trans_quad,rot_quad) = listener.lookupTransform("/world", "/vicon/Jackquad/Jackquad", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                trans_hand = [-5, -5, 0.8]
                vel = [0, 0, 0]
            if len(data_str) > 2:
                quad_goal = [float(data_str[1]), float(data_str[2]), float(data_str[3])]
            vel = _getVel(trans_hand, last_hand, r)
            v = _getVector(trans_hand, trans_quad, _getDistance(vel,[0,0,0]), strength)
            accumulate_v = _accumulateV(accumulate_v, vel, trans_hand, trans_quad)
            last_hand = trans_hand
            
            #msg.pose.position.x = _saturate(quad_goal[0] + v[0] + accumulate_v[0], 3.0, -1.6)
            #msg.pose.position.y = _saturate(quad_goal[1] + v[1] + accumulate_v[1], 1.8, -1.4)
            
            #msg.pose.position.x = _saturate(trans_hand[0]+1.5, 3.0, -1.6)
            #msg.pose.position.y = _saturate(trans_hand[1]+1.5, 1.8, -1.4)
            #msg.pose.position.z = _saturate(trans_hand[2], 2.3, 0.0)
            # Radius
            R = 1;
            trajx = R*math.sin(0.2*msg.header.seq/r)
            trajy = R*math.cos(0.2*msg.header.seq/r)
            trajz = 0.64;
            msg.pose.position.x = trajx
            msg.pose.position.y = trajy
            msg.pose.position.z = trajz
        pub.publish(msg)
        try:
            (trans_quad,rot_quad) = listener.lookupTransform("/world", "/vicon/Jackquad/Jackquad", rospy.Time(0))
            br.sendTransform((trans_quad[0],trans_quad[1],trans_quad[2]), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "quad_noROT", "/world")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        _last_trajectory_start = _trajectory_start
        _trajectory_count += 1
        rate.sleep()

