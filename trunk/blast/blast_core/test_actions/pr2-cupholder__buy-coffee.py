
class BlastPr2BuyCoffeeActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        money_bag = self.get_robot_holder("left-arm") #Get the UID of the object to ensure the same object

        cs = self.get_surface(parameters["shop"])

        #print "+"*30, "Move to start", "+"*30
        #self.plan_action("move", {"end": cs.locations["start"] },
        #                 {"robot-holders": {"left-arm": money_bag}, })

        print "+"*30, "Move to end", "+"*30
        #self.plan_action("move", {"end": cs.locations["end"] },
        #                 {"robot-holders": {"left-arm": money_bag}, })
        self.set_location(cs.locations["end"])

        print "+"*30, "Give money", "+"*30
        self.take_action("give-object", {"tts-text": "Money Bag"})
        #                {"robot-holders": {"left-arm": money_bag},
        #                 "robot-location": cs.locations["end"], })
        
        print "+"*30, "Grab object", "+"*30
        self.take_action("grab-object", {"tts-text": "Coffee Cup"})
        #                         {"robot-location": cs.locations["end"], })
        self.set_robot_holder("left-arm", "coffee_cup")
set_action_exec(BlastPr2BuyCoffeeActionExec)
