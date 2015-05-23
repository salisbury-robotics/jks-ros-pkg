import time


class BlastPr2TuckBothArms(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print "Start tuck arms"
        self.capability("tuck_arms", "START")
        print "Start tuck both and wait for result"
        self.capability("tuck_arms", "wait", {"tuck_left": True, "tuck_right": True})
        print "Done!"
        time.sleep(1.0)

set_action_exec(BlastPr2TuckBothArms)
