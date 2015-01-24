class BlastPr2TablePickLeft(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        obj = self.get_object(parameters["object"])
        if "table_1" != obj.parent: #For debugging always fail on table_1
            self.robot_pick_object(parameters["object"], "left-arm")
            self.capability("driver", "START")
            self.capability("driver", "WAIT_LEFT_ARM", [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False])
        else:
            self.capability("driver", "START")
            self.capability("driver", "WAIT_LEFT_ARM", [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False])
            self.delete_surface_object(parameters["object"])
            self.set_failure("no_object")
set_action_exec(BlastPr2TablePickLeft)



