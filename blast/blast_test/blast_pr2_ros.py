
import os, sys

my_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_path + "/../blast_client")

import blast_network_bridge, blast_action_exec, blast_ros


capabilities_dict = {
    "base":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [], #Just need the robot launch
                   "publishers":
                       {"/base_controller/command":
                            {"message": "geometry_msgs.Twist"},
                        },
                   },
         "command": {"type": "pub", "topic": "/base_controller/command"},
         },
    "tuck_arms":
        {"START": {"apt-pkgs": [], #Empty, better be installed
                   "launch-files": [], #Just need the robot launch
                   },
         "tuck_both": {"type": "rosrun", "package": "pr2_tuckarm", 
                       "prog": "tuck_arms.py", "args": ["-l", "t", "-r", "t", "-q"]},
         },
    "joint_states":
        {"START": {"apt-pkgs": [], #Empty, better be installed 
                   "launch-files": [], #Just need the robot launch
                   "subscribers": {"/joint_states": 
                                   {"message": "sensor_msgs.JointState"},},
                   },
         "getstate": {"type": "sub-last", "topic": "/joint_states"},
         },
    }

map_store = blast_network_bridge.MapStore(my_path + "/maps_client/")
action_store = blast_network_bridge.ActionStore(my_path + "/actions_client/")
library_store = blast_network_bridge.LibraryStore(my_path + "/libraries_client/")
blast_ros = blast_ros.BlastRos('blast_ros', capabilities_dict, start_ros = False)
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
