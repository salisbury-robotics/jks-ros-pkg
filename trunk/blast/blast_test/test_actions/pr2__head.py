
class BlastPr2Head(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print parameters
        self.set_robot_position("head",
                                [parameters["pan"], 
                                 parameters["tilt"]])
set_action_exec(BlastPr2Head)
