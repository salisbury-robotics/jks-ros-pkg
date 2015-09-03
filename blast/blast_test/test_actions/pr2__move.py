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
import math, time

class BlastPr2MoveActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)

    def run(self, parameters):
        self.capability("move-base", "START")
        self.capability("move-base", "wait", 
                        {"target_pose":
                             {"header": {"frame_id": "/map"},
                              "pose":
                                  {"position": {"x": parameters["end"].x,
                                                "y": parameters["end"].y,
                                                "z": 0.0},
                                   "orientation":
                                       {"z": math.sin(parameters["end"].a/2.0), "y": 0, "x": 0,
                                        "w": math.cos(parameters["end"].a/2.0)},
                                   }
                              }
                         })
                        #parameters["end"])
        time.sleep(1.0)
        self.capability("tilt-laser", "START")
        self.capability("tilt-laser", "set", {"command": {"profile": "blended_linear", 
                                                          "position": [0.0, 0.0],
                                                          "time_from_start": [0.0, 1.0],
                                                          "max_velocity": 10.0, 
                                                          "max_acceleration": 30},})
        time.sleep(3.0)
    
    def ok():
        start_l = self.get_location()
        end_l = parameters["end"]
        mid = BlastLocation(start_l.x, start_l.y, start_l.a, end_l.mid)
        
        turn_speed = 0.1
        move_speed = 0.1
        step_size = 0.1

        def wrap_angle(a):
            while a > +math.pi:
                a -= math.pi*2
            while a <= -math.pi:
                a += math.pi*2
            return a
        def clamp(a, mi, ma):
            if a < mi: return mi
            if a > ma: return ma
            return a
        def clamp_turn(a):
            return clamp(a, -turn_speed, turn_speed)
        def set_small(a, b):
            if abs(a - b) < 0.00001:
                return b
            return a


        while True:
            if mid.x == end_l.x and mid.y == end_l.y:
                if mid.a == end_l.a:
                    break
                else:
                    mid = mid.rotate(clamp_turn(wrap_angle(end_l.a - mid.a)))
                    mid = mid.rotateTo(set_small(mid.a, end_l.a))
            else:
                angle_to_target = wrap_angle(math.atan2(end_l.y - mid.y, end_l.x - mid.x))
                mid = mid.rotateTo(set_small(mid.a, angle_to_target))
                if mid.a != angle_to_target:
                    mid = mid.rotate(clamp_turn(wrap_angle(angle_to_target - mid.a)))
                    mid = mid.rotateTo(set_small(mid.a, angle_to_target))
                else:
                    d = math.sqrt((mid.x - end_l.x)**2 + (mid.y - end_l.y)**2)
                    if d <= move_speed:
                        mid = mid.moveTo(end_l.x, end_l.y)
                    else:
                        mid = mid.move(move_speed)
            self.set_location(mid)
            time.sleep(0.1)

        self.set_location(end_l)
set_action_exec(BlastPr2MoveActionExec)


