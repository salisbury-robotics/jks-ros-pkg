import time

class BlastPr2RootAction(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("robot_pub", "START")
        self.capability("amcl_set_location", "START")
        pos = self.get_location()
        self.capability("amcl_set_location", "VALUE", pos)
        
        while True:
            r = self.capability("robot_pub", "GETSTATE")
            #print "Result", r
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

