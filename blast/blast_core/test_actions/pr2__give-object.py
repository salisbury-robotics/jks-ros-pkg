
class BlastPr2GiveObjectActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_position("left-arm", [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False])
        self.set_robot_holder("left-arm", None)
set_action_exec("pr2", "give-object", BlastPr2GiveObjectActionExec)
