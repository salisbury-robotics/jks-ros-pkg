

class BlastPr2ElevatorActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.set_location(parameters["outfloor"])
set_action_exec(BlastPr2ElevatorActionExec)
