#--------------------------------------------------------------------
#Copyright (c) 2015
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without 
#modification, are permitted provided that the following conditions 
#are met:
#  1. Redistributions of source code must retain the above copyright 
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above 
#     copyright notice, this list of conditions and the following 
#     disclaimer in the documentation and/or other materials 
#     provided with the distribution.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
#COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
#INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
#HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
#ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#--------------------------------------------------------------------

class BlastPr2BuyCoffeeActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("driver", "START")
        money_bag = self.get_robot_holder("left-arm") #Get the UID of the object to ensure the same object

        cs = self.get_surface(parameters["shop"])

        #print "+"*30, "Move to start", "+"*30
        #self.plan_action("move", {"end": cs.locations["start"] },
        #                 {"robot-holders": {"left-arm": money_bag}, })

        print "+"*30, "Move to end", "+"*30
        #self.plan_action("move", {"end": cs.locations["end"] },
        #                 {"robot-holders": {"left-arm": money_bag}, })
        lt = []
        for name, val in cs.locations.iteritems():
            if name.find("line_") == 0:
                lt.append((int(name.split("_")[1]), val))
        lt.sort(key = lambda x: x[0])
        for nc, loc in lt:
            self.capability("driver", "WAIT_ABSOLUTE_LOCATION", loc)
        self.capability("driver", "WAIT_ABSOLUTE_LOCATION", cs.locations["end"])

        print "+"*30, "Give money", "+"*30
        self.take_action("give-object", {"tts-text": "Money Bag"})
        #                {"robot-holders": {"left-arm": money_bag},
        #                 "robot-location": cs.locations["end"], })
        
        print "+"*30, "Grab object", "+"*30
        self.take_action("grab-object", {"tts-text": "Coffee Cup"})
        #                         {"robot-location": cs.locations["end"], })
        self.set_robot_holder("left-arm", "coffee_cup")
set_action_exec(BlastPr2BuyCoffeeActionExec)
