#--------------------------------------------------------------------
#Copyright (c) 2015
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without 
#modification, are permitted provided that the following conditions 
#are met:
#  1. Redistributions of source code must retain the above copyright 
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above 
#     copyright notice, this list of conditions and the following 
#     disclaimer in the documentation and/or other materials 
#     provided with the distribution.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
#COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
#INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
#HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
#ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#--------------------------------------------------------------------

import blast
from blast_world import *
import os, sys


ROS_TEST_ROOT = """
import time

class BlastRosTestRootAction(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        talker = self.get_library("ros_tester", "talker")
        time.sleep(3.0) #Needed for interconnect, very brittle
        iter = 0
        while True:
            print "Add ->", talker.add(iter, 42)
            iter = iter + 1
            talker.send_string("Hi, I'm a spammer " + str(iter))
            print "We have", talker.get_string()
            try:
                time.sleep(0.1)
            except:
                break

set_action_exec(BlastRosTestRootAction)
"""

ROS_TEST_TALKER = """

class RosTestTalker(BlastActionLibrary):
    def __init__(self):
        BlastActionLibrary.__init__(self, ["talker"])

    def add(self, a, b):
        rs = self.capability("talker", "add", {"a": a, "b": b})
        return rs["sum"]
    
    def send_string(self, string):
        return self.capability("talker", "send-string", {"data": string})

    def get_string(self):
        r = self.capability("talker", "get-string")
        return r["data"]

add_library("ros_tester", "talker", RosTestTalker)

"""


def make_test_types_world():
    types_world = BlastWorldTypes()
    types_world.add_robot_type(RobotType("ros_tester", {"width": 0.668, "height": 0.668, 
                                                        "image": {"image": "robot_fs/pr2/pr2_def.png",
                                                                  "priority": 0},},
                                         {}, {}))
    types_world.add_action_type(BlastAction("ros_tester.__root", ROS_TEST_ROOT,
                                            {}, "True()", "\"0\"", {}, [], {},
                                            ["ros_tester.talker"],
                                            planable = False))
    types_world.add_library_type(BlastLibrary("ros_tester.talker", ROS_TEST_TALKER,
                                              [], #No library dependices
                                              ["talker"], #Talker capability required
                                              ))
    return types_world
    
def make_test_world(root_path):
    world = BlastWorld(make_test_types_world())

    clarkcenterfirstfloor = BlastMap("clarkcenterfirstfloor", root_path,
                                         "maps/clarkcenterfirstfloor.pgm", 20.0)
    world.append_map(clarkcenterfirstfloor)

    stair4 = BlastRobot("stair4", 
                        #BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                        BlastPt(12.000, 40.957, 0.148, clarkcenterfirstfloor.map),
                        world.types.get_robot("ros_tester"))
    world.append_robot(stair4)

    return world
