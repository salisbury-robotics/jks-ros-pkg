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
import time, math

class BlastPr2ElevatorActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        #loc = self.get_location()
        #for i in xrange(0, 5):
        #    loc = loc.move(0, 0.2)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 20):
        #    loc = loc.move(0.2, 0)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 10):
        #    loc = loc.rotate(- (math.pi/2)/10.0)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 10):
        #    loc = loc.rotate(-(math.pi/2)/10.0)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 20):
        #    loc = loc.move(0.2, 0)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #for i in xrange(0, 5):
        #    loc = loc.move(0, 0.2)
        #    self.set_location(loc)
        #    time.sleep(0.1)
        #self.set_location(parameters["outfloor"])
        self.capability("amcl_set_location", "START")
        self.capability("amcl_set_location", "VALUE", parameters["outfloor"])
        self.capability("amcl_set_location", "WAIT_VALUE", parameters["outfloor"])
set_action_exec(BlastPr2ElevatorActionExec)
