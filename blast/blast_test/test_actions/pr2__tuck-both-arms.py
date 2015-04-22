


class BlastPr2TuckBothArms(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("tuck_arms", "START")
        self.capability("tuck_arms", "tuck_both")

set_action_exec(BlastPr2TuckBothArms)
