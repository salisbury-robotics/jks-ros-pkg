
class BlastPr2BuyCoffeeActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print "+"*30, "Move to end", "+"*30
        self.plan_action("move", {"end": self.get_surface(parameters["shop"])["locations"]["end"] })
        print "+"*30, "Give money", "+"*30
        self.plan_action("give-object", {"tts-text": "Money Bag"})
        print "+"*30, "Grab object", "+"*30
        self.plan_action("grab-object", {"tts-text": "Coffee Cup"})
        self.set_robot_holder("left-arm", "coffee_cup")
set_action_exec(BlastPr2BuyCoffeeActionExec)
