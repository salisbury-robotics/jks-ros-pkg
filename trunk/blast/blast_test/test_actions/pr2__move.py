import math, time

class BlastPr2MoveActionExec(BlastActionExec):
    def __init__(self):
        BlastActionExec.__init__(self)
    def run(self, parameters):
        self.capability("move_base", "START")
        self.capability("move_base", "DRIVE", parameters["end"])
    
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


