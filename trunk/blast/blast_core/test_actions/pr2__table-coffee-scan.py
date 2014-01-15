
import os

class BlastPr2TableScan(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.surface_scan(parameters["table"], ["coffee_cup", "coffee_money_bag"])

        #Test code        
        try:
            f = open(parameters["table"] + "_objects.txt", "r")
            for l in f:
                self.surface_add_object(parameters["table"], l.strip(), "0.6602,0,0.762,0,0,0")
            f.close()
            f = open(parameters["table"] + "_objects.txt", "w")
            f.close()
        except IOError:
            pass
            

set_action_exec(BlastPr2TableScan)



