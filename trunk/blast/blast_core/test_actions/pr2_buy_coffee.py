
class BlastPr2BuyCoffeeActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.plan_action("move", {"end": self.get_surface(parameters["shop"]).locations["end"] })
        self.plan_action("give-object", {"tts-text": "Money Bag"})
        self.plan_action("grab-object-cupholder", {"tts-text": "Coffee Cup"})
        self.set_robot_holder("cup-holder", "coffee_cup")
set_action_exec("pr2-cupholder", "buy_coffee", BlastPr2BuyCoffeeActionExec)
