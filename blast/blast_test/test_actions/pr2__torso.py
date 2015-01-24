


class BlastPr2Torso(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print parameters
        self.capability("driver", "START")
        self.capability("driver", "WAIT_TORSO", [parameters["height"], ])
set_action_exec(BlastPr2Torso)
