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
for topic, msg, t in bag.read_messages(topics=['/diagnostics']):
    time=msg.header.stamp.secs+msg.header.stamp.nsecs/1e9
    da = msg.data.replace(',','\t');
    #data = [float(i) for i in da]
    f.write('{0:.3f}\t{1}\n'.format(time,da))
bag.close()
