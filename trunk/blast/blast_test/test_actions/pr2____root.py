import time

class BlastPr2RootAction(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("simulator", "START")
        self.capability("robot_pub", "START")
        self.capability("amcl_set_location", "START")
        pos = self.get_location()
        self.capability("amcl_set_location", "VALUE", pos)
        
        while True:
            r = self.capability("simulator", "SIMULATE")
            r = self.capability("robot_pub", "GETSTATE")
            t = self.get_teleop()
            print "TELEOP", t
            #print "Result", r
            if type(t) == dict:
                self.capability("driver", "START")
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
                    self.capability("driver", "FORCE", {"strafe": strafe, "forward": fb, "turn": turn})
            else:
                self.capability("driver", "STOP")
            if type(r) == dict:
                for name, val in r.iteritems():
                    #print name, "->", val
                    if name == "robot_loc":
                        self.set_location(BlastLocation(val["x"], val["y"], val["a"], str(val["map"])))
                    else:
                        self.set_robot_position(str(name), val)
            try:
                time.sleep(0.1)
            except:
                break


set_action_exec(BlastPr2RootAction)

