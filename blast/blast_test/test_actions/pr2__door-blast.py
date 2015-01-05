import time, math

class BlastPr2DoorBlastActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        loc = self.get_location()
        for i in xrange(0, 10):
            loc = loc.move(-0.05, 0)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 3):
            loc = loc.move(0, -0.05)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 10):
            loc = loc.move(0.2, 0)
            self.set_location(loc)
            time.sleep(0.1)

        self.set_location(self.get_surface(parameters["door"]).locations["out_exit"])
set_action_exec(BlastPr2DoorBlastActionExec)