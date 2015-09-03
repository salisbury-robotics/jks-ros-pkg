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

import os, sys

my_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_path + "/../blast_client")

import blast_network_bridge, blast_action_exec, blast_ros


capabilities_dict = {
    "base":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [("multi_map_navigation_pr2", "forblast.launch"),],
                   "publishers":
                       {"/base_controller/command":
                            {"message": "geometry_msgs.Twist"},
                        },
                   },
         "command": {"type": "pub", "topic": "/base_controller/command"},
         },
    "move-base":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [], #Empty, should be launched by base
                   "actions":
                       {"/pr2_move_base":
                            {"action": "move_base_msgs.MoveBase"}
                        },
                   },
         "wait": {"type": "action-wait", "topic": "/pr2_move_base"},
         },
    "tilt-laser":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [], #Just need the robot launch
                   "services":
                       {"/laser_tilt_controller/set_traj_cmd":
                            {"message": "pr2_msgs.SetLaserTrajCmd", "wait": True}
                        },
                   },
         "set": {"type": "service", "name": "/laser_tilt_controller/set_traj_cmd"},
         },
    "tuck_arms":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [], #Just need the robot launch
                   "actions":
                       {"/tuck_arms":
                            {"action": "pr2_common_action_msgs.TuckArms"}
                        },
                   },
         "command": {"type": "action-send", "topic": "/tuck_arms"},
         "result": {"type": "action-result", "topic": "/tuck_arms"},
         "wait": {"type": "action-wait", "topic": "/tuck_arms"},
         "cancel": {"type": "action-cancel", "topic": "/tuck_arms"},
         
         },
    "joint_states":
        {"START": {"apt-pkgs": [], #Empty, better be installed 
                   "launch-files": [], #Just need the robot launch
                   "subscribers": {"/joint_states": 
                                   {"message": "sensor_msgs.JointState"},},
                   },
         "getstate": {"type": "sub-last", "topic": "/joint_states"},
         },
    "amcl_param": #Should only be used by __root!
        {"START": {"apt-pkgs": [], #Empty, just set param
                   "launch-files": [], #Empty, just set param
                   "params": {"/amcl/initial_pose_x": {"type": "float"},
                              "/amcl/initial_pose_y": {"type": "float"},
                              "/amcl/initial_pose_a": {"type": "float"},
                              },
                   "map": {"map": 
                           {"services": ["/dynamic_map", "/static_map",],
                            "topics": ["/map",],},
                           },
                   },
         "x": {"type": "param", "path": "/amcl/initial_pose_x"},
         "y": {"type": "param", "path": "/amcl/initial_pose_y"},
         "a": {"type": "param", "path": "/amcl/initial_pose_a"},
         "set_map": {"type": "setmap", "serve": "map"},
         "odom_loc": {"type": "tf-last", "source": "/odom_combined", "destination": "/base_footprint"},
         "get_loc": {"type": "tf-last", "source": "/map", "destination": "/base_footprint"},
         },
    "torso":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [], #Just need the robot launch
                   "actions":
                       {"/torso_controller/position_joint_action":
                            {"action": "pr2_controllers_msgs.SingleJointPosition"}
                        },
                   },
         "command": {"type": "action-send", "topic": "/torso_controller/position_joint_action"},
         "result": {"type": "action-result", "topic": "/torso_controller/position_joint_action"},
         "wait": {"type": "action-wait", "topic": "/torso_controller/position_joint_action"},
         "cancel": {"type": "action-cancel", "topic": "/torso_controller/position_joint_action"},
         },
    }

map_store = blast_network_bridge.MapStore(my_path + "/maps_client/")
action_store = blast_network_bridge.ActionStore(my_path + "/actions_client/")
library_store = blast_network_bridge.LibraryStore(my_path + "/libraries_client/")
blast_ros = blast_ros.BlastRos('blast_ros', capabilities_dict, map_store, start_ros = False)
bnb = blast_network_bridge.BlastNetworkBridge("stair4-basestation", 8080, "stair4", "pr2-cupholder",
                                              blast_ros.install_capability, blast_ros.capability_cb,
                                              map_store, action_store, library_store)

bnb.start()
bnb.wait()
bnb.stop()
if bnb.error:
    print "The system terminated with error", bnb.error
else:
    print "Terminated without error"
