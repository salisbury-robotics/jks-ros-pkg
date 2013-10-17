
class BlastPr2GiveObjectActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("left-arm", None)
set_action_exec("pr2", "give-object", BlastPr2GiveObjectActionExec)
