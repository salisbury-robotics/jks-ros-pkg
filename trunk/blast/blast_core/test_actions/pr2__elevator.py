

class BlastPr2ElevatorActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_location(parameters["outfloor"])
set_action_exec("pr2", "elevator", BlastPr2ElevatorActionExec)
