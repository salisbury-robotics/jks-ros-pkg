
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
                self.surface_add_object(parameters["table"], l.strip(), BlastPos(x = 0.6602, y = 0, z = 0.762, rx = 0, ry = 0, rz = 0))
            f.close()
            f = open(parameters["table"] + "_objects.txt", "w")
            f.close()
        except IOError:
            pass
            

set_action_exec(BlastPr2TableScan)



