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
import time

class BlastPr2RootAction(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    
    def set_amcl(self, pos):
        self.capability("amcl_param", "x", pos.x)
        self.capability("amcl_param", "y", pos.y)
        self.capability("amcl_param", "a", pos.a)
        self.capability("amcl_param", "set_map", pos.mid)

    def kill_laser(self):
        self.capability("tilt-laser", "set", {"command": {"profile": "blended_linear", 
                                                          "position": [0.0, 0.0],
                                                          "time_from_start": [0.0, 1.0],
                                                          "max_velocity": 10.0, 
                                                          "max_acceleration": 30},})

    def wait_laser_motion(self, wait_move):
        laser_pos = None
        while True:
            r = self.capability("joint_states", "getstate")
            for name, pos in zip(r["name"], r["position"]):
                if name == 'laser_tilt_mount_joint':
                    pos_c = pos
            #print r
            #print pos_c, laser_pos
            print "Wait for the laser - wait for move:", wait_move
            time.sleep(0.1)
            if laser_pos != None:
                delta = (abs(laser_pos - pos_c) > 0.01)
                print delta, wait_move
                if delta == wait_move:
                    break
            laser_pos = pos_c

    def run(self, parameters):
        #Start a bunch of capabilities
        self.capability("joint_states", "START")

        #We set the robot's pose before starting AMCL to avoid race conditions
        pos = self.get_location()
        self.capability("amcl_param", "START")
        self.set_amcl(pos)
        self.capability("tilt-laser", "START")
        self.kill_laser()
        self.capability("base", "START")
        
        #TODO: do not allow actions during startup
        while True:
            if self.capability("amcl_param", "get_loc") != None:
                break
            else:
                time.sleep(0.1)
            print "Waiting for base transform"

        self.wait_laser_motion(True)
        self.kill_laser()
        self.wait_laser_motion(False)

        last_location = pos

        old_odom_tf = None

        while True:
            tf = self.capability("amcl_param", "get_loc")
            otf = self.capability("amcl_param", "odom_loc")

            #r = self.capability("simulator", "SIMULATE")
            r = self.capability("joint_states", "getstate")
            t = self.get_teleop()
            #print "TELEOP", t
            #print "Result", r
            if type(t) == dict:
                if "direction" in t and type(t["direction"]) == dict:
                    d = t["direction"]
                    strafe = 0
                    fb = 0
                    turn = 0
                    if d.get("left_strafe", False): strafe = 1
                    if d.get("right_strafe", False): strafe = -1
                    if d.get("forward", False): fb = 1
                    if d.get("backward", False): fb = -1
                    if d.get("left_turn", False): turn = 1
                    if d.get("right_turn", False): turn = -1
                    #self.capability("driver", "FORCE", {"strafe": strafe, "forward": fb, "turn": turn})
                    F_SCALE = 0.1
                    S_SCALE = 0.1
                    T_SCALE = 0.1
                    self.capability("base", "command", {"linear": {"x": fb * F_SCALE,
                                                                   "y": strafe * S_SCALE},
                                                        "angular": {"z": turn * T_SCALE}})
            if type(r) == dict and "position" in r and "name" in r:
                poses = {
                    "torso": ['torso_lift_joint'], # 'torso_lift_motor_screw_joint'],
                    "head": ['head_pan_joint', 'head_tilt_joint'],
                    "tilt-laser": ['laser_tilt_mount_joint'],
                    "right-arm": ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 
                                  'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 
                                  'r_forearm_roll_joint',  'r_wrist_flex_joint', 
                                  'r_wrist_roll_joint', 'r_gripper_joint'],
                    "left-arm":  ['l_shoulder_pan_joint', 'l_shoulder_lift_joint',
                                  'l_upper_arm_roll_joint','l_elbow_flex_joint', 
                                  'l_forearm_roll_joint', 'l_wrist_flex_joint',
                                  'l_wrist_roll_joint', 'l_gripper_joint'],
                    }
                

                for name, pos in zip(r["name"], r["position"]):
                    for pn in poses:
                        poses[pn] = [pos if str(x) == str(name) else x for x in poses[pn]]

                for name, val in poses.iteritems():
                    bad = False
                    for x in val:
                        if type(x) == str or type(x) == unicode:
                            bad = True
                    if not bad:
                        #print str(name), "->", val
                        self.set_robot_position(str(name), val)
                
                #Supress robot position updates from when the robot is not moving.
                #This could serve as a source of bugs
                do_tf_update = True
                if tf != None and otf != None:
                    if old_odom_tf != None:
                        dx = old_odom_tf[0][0] - otf[0][0]
                        dy = old_odom_tf[0][1] - otf[0][1]
                        da = old_odom_tf[2][2] - otf[2][2]
                        if abs(dx) < 1e-04 and abs(dy) < 1e-04 and abs(da) < 1e-04:
                            do_tf_update = False
                            #print "Supress tf"
                        #else:
                            #print "Update error", dx, dy, da
                
                if do_tf_update:
                    #print tf
                    old_odom_tf = otf
                    l_x = tf[0][0]
                    l_y = tf[0][1]
                    l_a = tf[2][2]
                    l_map = last_location.mid
                    new_loc = BlastLocation(l_x, l_y, l_a, l_map)
                    found_location = self.set_location(new_loc, last_location)
                    if found_location != True and found_location != False:
                        last_location = found_location
                        print "Locations differ"
                        print new_loc.to_dict()
                        print last_location.to_dict()
                        self.set_amcl(last_location)
                    else:
                        last_location = new_loc
            try:
                time.sleep(0.02)
            except:
                break


set_action_exec(BlastPr2RootAction)

