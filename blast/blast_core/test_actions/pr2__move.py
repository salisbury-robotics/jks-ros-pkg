

class BlastPr2MoveActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_location(parameters["end"])
set_action_exec("pr2", "move", BlastPr2MoveActionExec)


