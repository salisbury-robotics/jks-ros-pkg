#!/usr/bin/python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('bosch_arm_control')
import rospy
import rosbag
bag = rosbag.Bag('data/touch_table.bag')
f = open('data/touch_table_acc0.txt', 'w')
for topic, msg, t in bag.read_messages(topics=['/bma180']):
    if msg.iChipSelect==0:
        t=msg.header.stamp.secs+msg.header.stamp.nsecs/1e9
        f.write('{0:.3f}\t{1}\t{2}\t{3}\n'.format(t,msg.fAcclX,msg.fAcclY,msg.fAcclZ))
bag.close()
