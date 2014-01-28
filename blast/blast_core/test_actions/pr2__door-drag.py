
class BlastPr2DoorDragActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.set_location(self.get_surface(parameters["door"]).locations["in_exit"])
set_action_exec(BlastPr2DoorDragActionExec)

