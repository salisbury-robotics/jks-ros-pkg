


class BlastPr2Torso(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print parameters
        self.capability("torso", "START")
        hp = float(parameters["height"])
        self.capability("torso", "command", {"position": hp,
                                         "min_duration": 1.0,
                                         "max_velocity": 0.0,
                                         })
        torso_tol = 0.02
        while True:
            pos = self.get_robot_position("torso")
            if abs(pos["torso"] - hp) <= torso_tol:
                break
        self.capability("torso", "result", 1.0)
        self.capability("torso", "cancel")
set_action_exec(BlastPr2Torso)
