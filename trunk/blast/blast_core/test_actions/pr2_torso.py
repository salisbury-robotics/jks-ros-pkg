


class BlastPr2Torso(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        print parameters
        self.set_robot_position("torso", [parameters["height"], ])
set_action_exec("pr2", "torso", BlastPr2Torso)
