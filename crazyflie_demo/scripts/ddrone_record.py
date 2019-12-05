#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

_trajectory_start = False
_buttons = None

def _joyChanged(data):
    global _buttons, _trajectory_start
    #print data
    for i in range(0, len(data.buttons)):
        if _buttons == None or data.buttons[i] != _buttons[i]:
            if i == 7 and data.buttons[i] == 1:
                if _trajectory_start == False:
                    _trajectory_start = True
                    print("Start recording trajectory...")
                else:
                    _trajectory_start = False
                    print("Stop recording trajectory...")
    _buttons = data.buttons

def _saturate(data, upperbound, lowerbound):
    if data > upperbound:
        data = upperbound
    if data < lowerbound:
        data = lowerbound
    return data

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

    filehandle = open("/home/jack/Documents/darc/XH/data/ddrone.txt","w")

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    _last_trajectory_start = _trajectory_start
    _trajectory_count = 0
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        if _trajectory_start == True:
            if _last_trajectory_start == False:
                # Mark the starting time when first recording
                start_time = rospy.Time.now().to_sec()
            br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "goal", wand_name)
            try:
                (trans,rot) = listener.lookupTransform("/world", "goal", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            msg.pose.position.x = _saturate(trans[0], 3.1, -1.6)  #_saturate(trans[0], 1.6, -1.4)
            msg.pose.position.y = _saturate(trans[1], 1.8, -1.40)  #_saturate(trans[1], 1.25, -1.35)
            msg.pose.position.z = _saturate(trans[2], 2.3, 0.4)  #_saturate(trans[2], 2, 0.5)
            current_time = msg.header.stamp.to_sec() - start_time
            #filehandle.write(str(current_time) + " " + str(trans[0]) + " " + str(trans[1]) + " " + str(trans[2]) + "\n")
            filehandle.write(str(current_time) + " " + str(msg.pose.position.x) + " " + str(msg.pose.position.y) + " " + str(msg.pose.position.z) + "\n")
        else:
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
        pub.publish(msg)
        _last_trajectory_start = _trajectory_start
        _trajectory_count += 1
        rate.sleep()
        try:
            (trans_quad,rot_quad) = listener.lookupTransform("/world", "/vicon/Jackquad/Jackquad", rospy.Time(0))
            br.sendTransform((trans_quad[0],trans_quad[1],trans_quad[2]), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "quad_noROT", "/world")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    filehandle.close()

