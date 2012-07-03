#!/usr/bin/env python

import roslib
roslib.load_manifest('door_blast')
import rospy
import math
from sensor_msgs.msg import *
from visualization_msgs.msg import *


visualization_publisher = None

def scan_msg(msg):
    global visualization_publisher

    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "door_blast"
    marker.type = Marker.CYLINDER
    marker.action = Marker.MODIFY
    marker.id = 0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    visualization_publisher.publish(marker)

    xy = []
    angle = msg.angle_min
    for r in range(0, len(msg.ranges)):
        dist = msg.ranges[r]
        xy.append([dist * math.cos(angle), dist * math.sin(angle)])
        angle = angle + msg.angle_increment
    
    #Find the distance to the wall
    min_door = False
    for pt in xy:
        if (pt[1] < 1.0 and pt[1] > -1.0 and pt[0] > 0.01):
            if (min_door == False or min_door > pt[0]):
                min_door = pt[0]
            
    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "door_blast"
    marker.type = Marker.CUBE
    marker.action = Marker.MODIFY
    marker.id = 1
    marker.pose.position.x = min_door
    marker.pose.position.y = 0
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 1.0
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    visualization_publisher.publish(marker)

    y_pos = []
    for pt in xy:
        if (pt[1] < 0.5 and pt[1] > -0.5 and \
                pt[0] > min_door - 0.25 and pt[0] < min_door + 0.25):
            y_pos.append(pt[1])
    y_pos.sort()

    last_pos = 0
    max_delta = 0
    best = y_pos[0]
    print "Loop"
    for i in y_pos:
        delta = i - last_pos
        if (delta > max_delta):
            best = (i + last_pos) / 2
            max_delta = delta
        last_pos = i

    print best








if __name__ == '__main__':
    rospy.init_node("door_blast")
    visualization_publisher = rospy.Publisher("/visualization_markers", Marker)
    rospy.Subscriber("/base_scan", LaserScan, scan_msg)
    rospy.spin()
    
