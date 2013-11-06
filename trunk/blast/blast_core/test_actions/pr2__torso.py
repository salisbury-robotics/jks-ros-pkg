


class BlastPr2Torso(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print parameters
        self.set_robot_position("torso", [parameters["height"], ])
set_action_exec(BlastPr2Torso)
