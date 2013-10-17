

class BlastPr2GrabObjectActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("left-arm", "arbitrary-object", False)
set_action_exec("pr2", "grab-object", BlastPr2GrabObjectActionExec)
