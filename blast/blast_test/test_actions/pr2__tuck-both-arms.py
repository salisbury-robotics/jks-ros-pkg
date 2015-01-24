


class BlastPr2TuckBothArms(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("tuck_arms", "START", None)
        self.capability("tuck_arms", "TUCK_WAIT", {"left": "tuck", "right": "tuck"})

set_action_exec(BlastPr2TuckBothArms)
