import time, math

class BlastPr2ElevatorActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        loc = self.get_location()
        for i in xrange(0, 5):
            loc = loc.move(0, 0.2)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 20):
            loc = loc.move(0.2, 0)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 10):
            loc = loc.rotate(- (math.pi/2)/10.0)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 10):
            loc = loc.rotate(-(math.pi/2)/10.0)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 20):
            loc = loc.move(0.2, 0)
            self.set_location(loc)
            time.sleep(0.1)
        for i in xrange(0, 5):
            loc = loc.move(0, 0.2)
            self.set_location(loc)
            time.sleep(0.1)
        self.set_location(parameters["outfloor"])
set_action_exec(BlastPr2ElevatorActionExec)
