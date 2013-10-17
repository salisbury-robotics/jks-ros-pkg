
class BlastPr2GrabObjectCupHolderActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("cup-holder", "arbitrary-object", False)
set_action_exec("pr2-cupholder", "grab-object-cupholder", BlastPr2GrabObjectCupHolderActionExec)
