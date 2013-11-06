

class BlastPr2MoveActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.set_location(parameters["end"])
set_action_exec(BlastPr2MoveActionExec)


