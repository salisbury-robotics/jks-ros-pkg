#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('bosch_arm_control')
import rospy
from sensor_msgs.msg import JointState
def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.name[0])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
