#!/usr/bin/env python

import roslib
roslib.load_manifest('door_blast')
import rospy
import math
import tf
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from pr2_msgs.srv import *


class DoorBlast:
    def __init__(self):
        self.scan_msg = None
        self.visualization_publisher = rospy.Publisher("/visualization_markers", Marker)
        self.cmd_vel_publisher = rospy.Publisher("/base_controller/command", Twist)
        self.listener = tf.TransformListener()
        rospy.wait_for_service("/laser_tilt_controller/set_periodic_cmd")
        self.set_laser = rospy.ServiceProxy("/laser_tilt_controller/set_periodic_cmd", SetPeriodicCmd)
        rospy.Subscriber("/tilt_scan", LaserScan, self.on_scan_msg)

    
    def on_scan_msg(self, msg):
        self.scan_msg = msg

    def wait_for_scan(self, clear = True):
        if (clear):
            self.scan_msg = False
        while not rospy.is_shutdown() and not self.scan_msg:
            rospy.sleep(0.01)

    def set_laser_pos(self, pos):
        laser_per = SetPeriodicCmdRequest()
        laser_per.command.header.stamp = rospy.Time.now()
        laser_per.command.profile = "linear"
        laser_per.command.period = 1
        laser_per.command.amplitude = 0
        laser_per.command.offset = pos
        resp1 = self.set_laser(laser_per)

    def set_laser_and_wait(self, pos):
        self.set_laser_pos(pos)
        rospy.sleep(2.0)
        self.wait_for_scan()


    def get_scan_point(self, x, y, z_goal):
        self.delete_markers(self.scan_msg.header.frame_id, 3)

        self.listener.waitForTransform(self.scan_msg.header.frame_id, "/base_footprint", rospy.Time(), rospy.Duration(30))
        tf = PointStamped()
        tf.header.frame_id = self.scan_msg.header.frame_id
        tf.point.x = x
        tf.point.y = y
        tf.point.z = 0
        print "Transform point"
        print tf


        marker = Marker()
        marker.header.frame_id = self.scan_msg.header.frame_id
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "door_blast"
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY
        marker.id = 0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.visualization_publisher.publish(marker)
        
        pt_target = self.listener.transformPoint("/base_footprint", tf)
        
        pt_target.point.z = z_goal
        
        pos = 0.5
        
        while not rospy.is_shutdown():
            self.set_laser_pos(pos)
            rospy.sleep(1.0)
            self.listener.waitForTransform(self.scan_msg.header.frame_id, "/base_footprint", rospy.Time(), rospy.Duration(30))
            test = self.listener.transformPoint(self.scan_msg.header.frame_id, pt_target)
            print pos, test.point.z
            if (test.point.z < -0.3):
                pos = pos + 0.2
            elif (test.point.z > 0.3):
                pos = pos - 0.2
            elif (test.point.z < -0.05):
                pos = pos + 0.06
            elif (test.point.z > 0.05):
                pos = pos - 0.06
            elif (test.point.z < -0.01):
                pos = pos + 0.01
            elif (test.point.z > 0.01):
                pos = pos - 0.01
            else:
                break


    def run(self):
        self.set_laser_and_wait(0.2)
        min_depth = self.get_vertical_depth(self.scan_msg)

        print "Depth", min_depth
        self.get_scan_point(min_depth, 0, 0.3)

        while True:
            self.wait_for_scan()
            if (self.line_up(self.scan_msg)):
                break

        self.set_laser_and_wait(0.2)
        
        while True:
            self.wait_for_scan()
            if (self.horizontal_align(self.scan_msg, min_depth)):
                break
        self.go_forward(min_depth)
        
    def line_up(self, msg):
        xy = self.scan_to_points(msg)
        xy_c = []
        for i in xy:
            if (i[1] > -0.5 and i[1] < 0.5 and i[0] > 0.2):
                xy_c.append(i)

        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "door_blast"
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY
        marker.id = 0
        marker.pose.position.x = 1.2
        marker.pose.position.y = 0
        marker.pose.position.z = 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 1.0
        marker.scale.z = 1
        marker.color.a = 0.1
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.visualization_publisher.publish(marker)

        #print xy_c
        n = len(xy_c)
        Sx = Sy = Sxx = Syy = Sxy = 0.0
        for pt in xy_c:
            x = pt[1]
            y = pt[0]
            Sx = Sx + x
            Sy = Sy + y
            Sxx = Sxx + x*x
            Syy = Syy + y*y
            Sxy = Sxy + x*y
        det = Sxx * n - Sx * Sx
        a, b = (Sxy * n - Sy * Sx)/det, (Sxx * Sy - Sx * Sxy)/det
        meanerror = residual = 0.0
        for pt in xy_c:
            x = pt[1]
            y = pt[0]
            meanerror = meanerror + (y - Sy/n)**2
            residual = residual + (y - a * x - b)**2
        RR = 1 - residual/meanerror
        ss = residual / (n-2)
        Var_a, Var_b = ss * n / det, ss * Sxx / det
        print a, b, RR
        
        a = a - 0.1 #FIXME: param for offset

        msg = Twist()
        speed_lim = 0.15 #FIXME param                                                                                                              
        scal = 4.0
        if (-a * scal > speed_lim):
            msg.angular.z = speed_lim
        elif (-a * scal < -speed_lim):
            msg.angular.z = -speed_lim
        else:
            msg.angular.z = -a * scal

        print msg
        self.cmd_vel_publisher.publish(msg)
        
        tol = 0.01 #FIXME: param

        return a < tol and a > -tol
        

    def scan_to_points(self, msg):
        xy = []
        angle = msg.angle_min
        for r in range(0, len(msg.ranges)):
            dist = msg.ranges[r]
            xy.append([dist * math.cos(angle), dist * math.sin(angle)])
            angle = angle + msg.angle_increment
        return xy


    def get_vertical_depth(self, msg):
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
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.visualization_publisher.publish(marker)
        
        xy = self.scan_to_points(msg)
    
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
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 1
        marker.color.a = 0.4
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.visualization_publisher.publish(marker)

        return min_door

    def horizontal_align(self, msg, min_door):
        xy = self.scan_to_points(msg)
        
        y_pos = []
        for pt in xy:
            if (pt[1] < 1.0 and pt[1] > -1.0 and \
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
        best = (gaps[0][2] + gaps[0][3]) / 2
        
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
        self.visualization_publisher.publish(marker)

        
        tol = 0.01 #FIXME param
        if (best < tol and best > -tol):
            return True

        msg = Twist()
        speed_lim = 0.1 #FIXME param                                                                                                     
        scal = 0.4
        if (best * scal > speed_lim):
            msg.linear.y = speed_lim
        elif (best * scal < -speed_lim):
            msg.linear.y = -speed_lim
        else:
            msg.linear.y = best * scal
        self.cmd_vel_publisher.publish(msg)
        return False

    def delete_markers(self, frame_id, lim):
        for n in range(0, lim):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.get_rostime()
            marker.ns = "door_blast"
            marker.type = Marker.CYLINDER
            marker.action = Marker.DELETE
            marker.id = n
            self.visualization_publisher.publish(marker)

    def go_forward(self, min_door):
        print "done"
        
        
        base_frame = "/base_footprint"
        odom_frame = "/odom_combined" #FIXME param
        
        while not rospy.is_shutdown(): #FIXME
            try:
                self.listener.waitForTransform(base_frame, odom_frame, rospy.Time(), rospy.Duration(30))
                
                goal = PoseStamped()
                goal.header.frame_id = base_frame
                goal.pose.position.x = min_door + 1.5
                goal = listener.transformPose(odom_frame, goal)
                print goal
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logwarn("TF error") #FIXME: warning

        rate = rospy.Rate(10)
        while not rospy.is_shutdown(): #FIXME
            try:
                goal_transformed = self.listener.transformPose(base_frame, goal)
                
                print goal_transformed
                
                def scaleclamp(val, scale, lim):
                    t = val * scale
                    if (t > lim):
                        return lim
                    if (t < -lim):
                        return -lim
                    return t
                
                msg = Twist()
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
                
                self.cmd_vel_publisher.publish(msg)
                
                rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logwarn("TF error") #FIXME
                

if __name__ == '__main__':
    rospy.init_node("door_blast")
    blast = DoorBlast()
    blast.run()
    rospy.spin()
    