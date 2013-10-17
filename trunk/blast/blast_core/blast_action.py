import blast_world
import blast_planner

blast_action_exec_d = {}
def set_action_exec(robot_type, action_type, item):
    if not robot_type in blast_action_exec_d:
        blast_action_exec_d[robot_type] = {}
    blast_action_exec_d[robot_type][action_type] = item

class BlastActionExec:
    def __init__(self, robot, manager, guid):
        self._robot = robot
        self._manager = manager
        self._guid = guid
    
    def set_location(self, position):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._manager.world.set_robot_location(self._robot, position.copy())
        self._manager.world_unlock()

    def set_robot_holder(self, holder, ot, require_preexisting_object = True):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._manager.world.set_robot_holder(self._robot, holder, ot,
                                                 require_preexisting_object)
        self._manager.world_unlock()

    def plan_action(self, action, parameters):
        if self._manager.get_current_guid() == self._guid:
            self._manager.plan_action(self._robot, action, parameters)

    def get_surface(self, surface):
        if type(surface) != type(""):
            surface = surface.name
        return self._manager.world.get_surface(surface)
    
    def run(self, parameters):
        raise blast_world.BlastError("Empty action run")




class BlastPr2MoveActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_location(parameters["end"])
set_action_exec("pr2", "move", BlastPr2MoveActionExec)

class BlastPr2GrabObjectActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("left-arm", "arbitrary-object", False)
set_action_exec("pr2", "grab-object", BlastPr2GrabObjectActionExec)

class BlastPr2GiveObjectActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("left-arm", None)
set_action_exec("pr2", "give-object", BlastPr2GiveObjectActionExec)

class BlastPr2GiveObjectCupHolderActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("cup-holder", None)
set_action_exec("pr2", "give-object-cupholder", BlastPr2GiveObjectCupHolderActionExec)

class BlastPr2GrabObjectCupHolderActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_robot_holder("cup-holder", "arbitrary-object", False)
set_action_exec("pr2-cupholder", "grab-object-cupholder", BlastPr2GrabObjectCupHolderActionExec)


class BlastPr2DoorBlastActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_location(self.get_surface(parameters["door"]).locations["out_exit"])
set_action_exec("pr2", "door_blast", BlastPr2DoorBlastActionExec)

class BlastPr2DoorDragActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_location(self.get_surface(parameters["door"]).locations["in_exit"])
set_action_exec("pr2", "door_drag", BlastPr2DoorDragActionExec)

class BlastPr2ElevatorActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.set_location(parameters["outfloor"])
set_action_exec("pr2", "elevator", BlastPr2ElevatorActionExec)

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

class BlastPr2BuyCoffeeActionExec(BlastActionExec):
    def __init__(self, robot, manager, guid):
        BlastActionExec.__init__(self, robot, manager, guid)
    def run(self, parameters):
        self.plan_action("move", {"end": self.get_surface(parameters["shop"]).locations["end"] })
        self.plan_action("give-object", {"tts-text": "Money Bag"})
        self.plan_action("grab-object-cupholder", {"tts-text": "Coffee Cup"})
        self.set_robot_holder("cup-holder", "coffee_cup")
set_action_exec("pr2-cupholder", "buy_coffee", BlastPr2BuyCoffeeActionExec)


#Let people create planing worlds....

class BlastManager:
    def __init__(self):
        self.world = blast_planner.BlastPlannableWorld(blast_world.make_test_world())
        self.world.real_world = True
        self.world.action_callback = lambda r, a, p: self.on_action_take(r, a, p)
        self.action_stack = []
        self.action_guid = 0

    def world_lock(self):
        pass
    def world_unlock(self):
        pass
    def get_current_guid(self):
        try:
            return self.action_stack[-1]._guid
        except:
            return None

    def on_action_take(self, robot, action, parameters):
        print "Action!", robot, action, parameters
        robot_type = self.world.world.robots[robot].robot_type
        action_exec = None
        while robot_type:
            if robot_type.name in blast_action_exec_d:
                if action in blast_action_exec_d[robot_type.name]:
                    action_exec = blast_action_exec_d[robot_type.name][action]
            robot_type = robot_type.parent
        if action_exec == None:
            print "Problem - action has no exec class", robot, action
            return False
 
        print "--- Exec action", robot, "-->", action
        exe = action_exec(robot, self, self.action_guid)
        self.action_guid = self.action_guid + 1
        self.action_stack.append(exe)
        exe.run(parameters)
        self.action_stack.remove(exe)
        print "--- Done"
        return True
        

    def take_action(self, robot, action, parameters):
        if self.world.take_action(robot, action, parameters) == None:
            print "Epic fail for", robot, "-->", action
    def plan_action(self, robot, action, parameters):
        if self.world.plan_action(robot, action, parameters) == None:
            print "Epic fail for", robot, "-->", action

 #       exe = blast_action_exec.get(self.world.get

man = BlastManager()
man.plan_action("stair4", "coffee_run", {"person_location": blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor"), 
                                         "shop": "clark_peets_coffee_shop"})
