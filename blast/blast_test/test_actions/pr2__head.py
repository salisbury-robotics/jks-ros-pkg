
class BlastPr2Head(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("driver", "START")
        self.capability("driver", "WAIT_HEAD", [parameters["pan"], parameters["tilt"]])
set_action_exec(BlastPr2Head)
