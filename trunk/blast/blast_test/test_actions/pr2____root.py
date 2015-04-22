import time

class BlastPr2RootAction(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("simulator", "START")
        self.capability("robot_pub", "START")
        self.capability("joint_states", "START")
        self.capability("base", "START")
        self.capability("amcl_set_location", "START")
        pos = self.get_location()
        self.capability("amcl_set_location", "VALUE", pos)
        
        while True:
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
            try:
                time.sleep(0.1)
            except:
                break


set_action_exec(BlastPr2RootAction)

