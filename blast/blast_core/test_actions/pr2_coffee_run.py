
class BlastPr2CoffeeRunActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.plan_action("move", {"end": parameters["person_location"] })
        self.plan_action("grab-object", {"tts-text": "Money Bag"})
        self.set_robot_holder("left-arm", "coffee_money_bag")
        self.plan_action("buy_coffee", {"shop": parameters["shop"]})
        self.plan_action("move", {"end": parameters["person_location"] })
        self.plan_action("give-object-cupholder", {"tts-text": "Coffee Cup"})
set_action_exec("pr2-cupholder", "coffee_run", BlastPr2CoffeeRunActionExec)
