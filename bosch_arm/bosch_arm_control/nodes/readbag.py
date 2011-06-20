#!/usr/bin/python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('bosch_arm_control')
import rospy
import rosbag
import sys
bag = rosbag.Bag(sys.argv[1])
strs=sys.argv[1].split('.')
output=strs[0]+'.txt'
f = open(output, 'w')
for topic, msg, t in bag.read_messages(topics=['/bma180']):
    if msg.iChipSelect==0:
        t=msg.header.stamp.secs+msg.header.stamp.nsecs/1e9
        f.write('{0:.3f}\t{1}\t{2}\t{3}\n'.format(t,msg.fAcclX,msg.fAcclY,msg.fAcclZ))
bag.close()
