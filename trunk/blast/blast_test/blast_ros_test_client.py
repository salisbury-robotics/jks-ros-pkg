
import os, sys

my_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_path + "/../blast_client")

import blast_network_bridge, blast_action_exec, blast_ros


capabilities_dict = {"talker":
                         {"START": {"apt-pkgs": [], #list of apt-get packages
                                    "launch-files": [('rospy_tutorials', 'test-add-two-ints.launch'),], #(package-name, file) tuples
                                    "publishers": {"chatter": {"message": "std_msgs.String"},},
                                    "subscribers": {"chatter": {"message": "std_msgs.String"},},
                                    "services": {"/add_two_ints": {"message": "rospy_tutorials.AddTwoInts"},},
                                    },
                          "send-string": {"type": "pub", "topic": "chatter"},
                          "get-string": {"type": "sub-last", "topic": "chatter"},
                          "add": {"type": "service", "name": "/add_two_ints"},
                          },
                     }

map_store = blast_network_bridge.MapStore(my_path + "/maps_client/")
action_store = blast_network_bridge.ActionStore(my_path + "/actions_client/")
blast_ros = blast_ros.BlastRos('blast_ros', capabilities_dict, start_ros = True)
bnb = blast_network_bridge.BlastNetworkBridge("localhost", 8080, "stair4", "ros_tester",
                                              blast_ros.install_capability, blast_ros.capability_cb,
                                              map_store, action_store)

bnb.start()
bnb.wait()
bnb.stop()
if bnb.error:
    print "The system terminated with error", bnb.error
else:
    print "Terminated without error"
