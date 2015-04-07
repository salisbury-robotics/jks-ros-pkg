import blast
from blast_world import *
import os, sys


ROS_TEST_ROOT = """
import time

class BlastRosTestRootAction(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("talker", "START")
        time.sleep(3.0) #Needed for interconnect, very brittle
        iter = 0
        while True:
            rs = self.capability("talker", "add", {"a": 42, "b": iter})
            print "Add ->", rs
            iter = iter + 1
            self.capability("talker", "send-string", {"data": "Hi, I'm a spammer " + str(iter)})
            r = self.capability("talker", "get-string")
            print "We have", r
            try:
                time.sleep(0.1)
            except:
                break

set_action_exec(BlastRosTestRootAction)
"""



def make_test_types_world():
    types_world = BlastWorldTypes()
    types_world.add_robot_type(RobotType("ros_tester", {"width": 0.668, "height": 0.668, 
                                                        "image": {"image": "robot_fs/pr2/pr2_def.png",
                                                                  "priority": 0},},
                                         {}, {}))
    types_world.add_action_type(BlastAction("ros_tester.__root", ROS_TEST_ROOT,
                                            {}, "True()", "\"0\"", {}, [], {},
                                            planable = False))
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
