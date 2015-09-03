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
library_store = blast_network_bridge.LibraryStore(my_path + "/libraries_client/")
blast_ros = blast_ros.BlastRos('blast_ros', capabilities_dict, start_ros = True)
bnb = blast_network_bridge.BlastNetworkBridge("localhost", 8080, "stair4", "ros_tester",
                                              blast_ros.install_capability, blast_ros.capability_cb,
                                              map_store, action_store, library_store)

bnb.start()
bnb.wait()
bnb.stop()
if bnb.error:
    print "The system terminated with error", bnb.error
else:
    print "Terminated without error"
