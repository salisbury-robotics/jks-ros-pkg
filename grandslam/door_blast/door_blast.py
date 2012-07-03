#!/usr/bin/env python

import roslib
roslib.load_manifest('door_blast')
import rospy
import math
import tf
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *

visualization_publisher = None
command_vel_publisher = None
lineup = False
listener = None

def scan_msg(msg):
    global visualization_publisher
    global cmd_vel_publisher
    global lineup
    global listener

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
        if (pt[1] < 0.75 and pt[1] > -0.75 and pt[0] > 0.01):
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
    marker.scale.y = 1.2
    marker.scale.z = 1
    marker.color.a = 0.4
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    visualization_publisher.publish(marker)

    y_pos = []
    for pt in xy:
        if (pt[1] < 0.6 and pt[1] > -0.6 and \
                pt[0] > min_door - 0.25 and pt[0] < min_door + 0.25):
            y_pos.append(pt[1])
    y_pos.sort()


    last_pos = y_pos[0]
    max_delta = 0
    best = y_pos[0]
    gaps = []

    for i in y_pos:
        delta = i - last_pos
        gaps.append((delta, i + last_pos, i, last_pos))
        last_pos = i

    gaps = sorted(gaps, key=lambda gap: gap[0], reverse=True)
    gap_list = [gaps[0][2], gaps[0][3], gaps[1][2], gaps[2][2]]
    gap_list.sort()
    best = (gap_list[0] + gap_list[3]) / 2
    print best


    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "door_blast"
    marker.type = Marker.CYLINDER
    marker.action = Marker.MODIFY
    marker.id = 2
    marker.pose.position.x = min_door
    marker.pose.position.y = best
    marker.pose.position.z = 0.5
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    visualization_publisher.publish(marker)


    if (lineup):
        tol = 0.035 #FIXME param
        print best
        if (best < tol and best > -tol and False):
            lineup = False
            for n in range(0, 3):
                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = rospy.get_rostime()
                marker.ns = "door_blast"
                marker.type = Marker.CYLINDER
                marker.action = Marker.DELETE
                marker.id = n
                visualization_publisher.publish(marker)
            print "done"
            
            
            base_frame = "/base_footprint"
            odom_frame = "/odom_combined" #FIXME param
            
            while not rospy.is_shutdown(): #FIXME
                try:
                    listener.waitForTransform(base_frame, odom_frame, rospy.Time(), rospy.Duration(30))

                    goal = PoseStamped()
                    goal.header.frame_id = base_frame
                    goal.pose.position.x = 3.0
                    goal = listener.transformPose(odom_frame, goal)
                    print goal
                    break
                except:
                    rospy.logwarn("TF error") #FIXME: warning

            rate = rospy.Rate(10)
            n = 0
            while not rospy.is_shutdown(): #FIXME
                try:
                    goal_transformed = listener.transformPose(base_frame, goal)


                    print goal_transformed
                    
                    def scaleclamp(val, scale, lim):
                        t = val * scale
                        if (t > lim):
                            return lim
                        if (t < -lim):
                            return -lim
                        return t


                    msg = Twist()
                    #msg.linear.x = scaleclamp(-goal_transformed.pose.position.x, 1, 0.5)
                    p, r, y = tf.transformations.euler_from_quaternion((goal_transformed.pose.orientation.x,
                                                                        goal_transformed.pose.orientation.y,
                                                                        goal_transformed.pose.orientation.z,
                                                                        goal_transformed.pose.orientation.w))
                    msg.angular.x = 0
                    msg.angular.y = 0
                    msg.angular.z = scaleclamp(y, 1, 0.5)

                    if (y > -0.05 and y < 0.05):
                        msg.linear.y = scaleclamp(goal_transformed.pose.position.y, 1, 0.1)
                        msg.linear.x = scaleclamp(goal_transformed.pose.position.x, 1, 0.1)

                    print msg

                    cmd_vel_publisher.publish(msg)
                    


                    rate.sleep()
                    n = n + 1
                    if (n > 50):
                        rospy.sleep(2.0)
                        n = 0
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                    rospy.logwarn("TF error") #FIXME
            
        else:
            msg = Twist()
            speed_lim = 0.1 #FIXME param
            scal = 0.4
            if (best * scal > speed_lim):
                msg.linear.y = speed_lim
            elif (best * scal < -speed_lim):
                msg.linear.y = -speed_lim
            else:
                msg.linear.y = best * scal
            cmd_vel_publisher.publish(msg)
                








if __name__ == '__main__':
    rospy.init_node("door_blast")
    visualization_publisher = rospy.Publisher("/visualization_markers", Marker)
    cmd_vel_publisher = rospy.Publisher("/base_controller/command", Twist)
    listener = tf.TransformListener()
    rospy.Subscriber("/base_scan", LaserScan, scan_msg)
    lineup = True
    rospy.spin()
    
