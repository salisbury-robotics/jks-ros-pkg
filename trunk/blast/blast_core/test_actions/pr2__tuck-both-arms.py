


class BlastPr2TuckBothArms(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.set_robot_position("left-arm", [0.06024, 1.248526, 1.789070, -1.683386, 
                                                      -1.7343417, -0.0962141, -0.0864407, None])
        self.set_robot_position("right-arm", [-0.023593, 1.1072800, -1.5566882, -2.124408,
                                               -1.4175, -1.8417, 0.21436, None])
set_action_exec(BlastPr2TuckBothArms)
