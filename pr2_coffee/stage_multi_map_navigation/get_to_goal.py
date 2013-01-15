#!/usr/bin/python

PKG = "stage_multi_map_navigation"

import roslib
roslib.load_manifest(PKG)

import unittest, sys, time
import rospy, rostest
import tf


class NavStackTest(unittest.TestCase):
    def test_run(self):
        rospy.init_node('test_nav_stack')

        self.success = False
        print "running"
        print sys.argv
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
                if (trans[0] < float(sys.argv[1]) + float(sys.argv[3])
                    and trans[0] > float(sys.argv[1]) - float(sys.argv[3])
                    and trans[1] < float(sys.argv[2]) + float(sys.argv[3])
                    and trans[1] > float(sys.argv[2]) - float(sys.argv[3])):
                    break
                print trans[0], float(sys.argv[1]) + float(sys.argv[3]), float(sys.argv[1]) - float(sys.argv[3])
                print trans[1], float(sys.argv[2]) + float(sys.argv[3]), float(sys.argv[2]) - float(sys.argv[3])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Error with transform"
            rospy.sleep(1.0)

if __name__ == '__main__':
    print "Hi"
    rostest.run(PKG, 'test_nav_stack', NavStackTest) 
