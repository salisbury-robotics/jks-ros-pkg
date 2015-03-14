
class BlastPr2CoffeeRunActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        print "-"*30, "Plan to person location", "-"*30
        self.plan_action("move", {"end": parameters["person_location"] })
        print "-"*30, "Grab money bag", "-"*30
        self.plan_action("grab-object", {"tts-text": "Money Bag"})
        self.set_robot_holder("left-arm", "coffee_money_bag")
        print "-"*30, "Execute buy coffee action", "-"*30
        self.plan_action("buy-coffee", {"shop": parameters["shop"]})
        print "-"*30, "Move to person location", "-"*30
        self.plan_action("move", {"end": parameters["person_location"] })
        print "-"*30, "Give out coffee", "-"*30
        self.plan_action("give-object", {"tts-text": "Coffee Cup"})
        print "-"*30, "Tuck", "-"*30
        self.plan_action("tuck-both-arms", {})
set_action_exec(BlastPr2CoffeeRunActionExec)
