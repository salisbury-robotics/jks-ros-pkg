import math, time

class BlastPr2DoorDragActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        #loc = self.get_location()
        #for i in xrange(0, 10):
        #    loc = loc.move(0.05, 0)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 3):
        #    loc = loc.move(0, -0.05)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 3):
        #    loc = loc.move(-0.05, 0)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 5):
        #    loc = loc.rotate( (math.pi / 2.0) / 5)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 12):
        #    loc = loc.move(0, -0.2)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 5):
        #    loc = loc.rotate( -(math.pi / 2.0) / 5)
        #    self.set_location(loc)
        #    time.sleep(0.1)

        #self.set_location(self.get_surface(parameters["door"]).locations["in_exit"])
        self.capability("amcl_set_location", "START")
        exit_loc = self.get_surface(parameters["door"]).locations["in_exit"]
        self.capability("amcl_set_location", "VALUE", exit_loc)
        self.capability("amcl_set_location", "WAIT_VALUE", exit_loc)
set_action_exec(BlastPr2DoorDragActionExec)

