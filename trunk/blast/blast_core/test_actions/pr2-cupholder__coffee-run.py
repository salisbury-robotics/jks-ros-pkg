
class BlastPr2CoffeeRunActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):

        #First step is to drive to the location of the person
        #And pick up the coffee cup from them
        print "-"*30, "Plan to person location", "-"*30
        self.plan_action("move", {"end": parameters["person_location"] })

        #Grab the money bag of of the human
        print "-"*30, "Grab money bag", "-"*30
        self.plan_action("grab-object", {"tts-text": "Money Bag"})
        self.set_robot_holder("left-arm", "coffee_money_bag")
        money_bag = self.get_robot_holder("left-arm") #Get the UID of the object

        #It is important for us to use world limit constraints. The reason
        #is that we don't want an action to drop and then pick up the money bag.
        #To see why we need this, imagine there's a table at the shop with a money 
        #bag on it. If for some reason travelling there without the bag is faster
        #(2 handed door opening or something) then the robot will throw out the
        #bag and grab the other one since all bags are the same as far as BLAST
        #cares. By putting in this exact constraint, we can prevent this from
        #ever happening.
        

        #Drive to the coffee shop and try to pick up the coffee cup
        #We make sure the same money bag is in the hand of the
        #robot before execution with a robot-holders world limit
        print "-"*30, "Execute buy coffee action", "-"*30
        self.plan_action("buy-coffee", {"shop": parameters["shop"]}, 
                         {"robot-holders": {"left-arm": money_bag}})
        coffee_cup = self.get_robot_holder("cupholder")

        #Now that we have purchased the coffee cup, we can drive back to
        #the person and deliver the cup
        print "-"*30, "Move to person location", "-"*30
        self.plan_action("move", {"end": parameters["person_location"] }, 
                         {"robot-holders": {"cupholder": coffee_cup}})


        print "-"*30, "Give out coffee", "-"*30
        self.plan_action("give-object", {"tts-text": "Coffee Cup"}, 
                         {"robot-holders": {"cupholder": coffee_cup}})


        print "-"*30, "Tuck", "-"*30
        self.plan_action("tuck-both-arms", {})
set_action_exec(BlastPr2CoffeeRunActionExec)
