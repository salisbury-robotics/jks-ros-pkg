import blast_world
import blast_planner
import os, sys

blast_action_exec_d = {}
def set_action_exec(robot_type, action_type, item):
    if not robot_type in blast_action_exec_d:
        blast_action_exec_d[robot_type] = {}
    blast_action_exec_d[robot_type][action_type] = item

class BlastRuntimeError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
    
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

    def set_robot_position(self, pos, val):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._manager.world.set_robot_position(self._robot, pos, val)
        self._manager.world_unlock()
        
    def plan_action(self, action, parameters):
        if self._manager.get_current_guid() == self._guid:
            if not self._manager.plan_action(self._robot, action, parameters):
                raise BlastRuntimeError("Planning to run action failed")

    def get_surface(self, surface):
        if type(surface) != type(""):
            surface = surface.name
        return self._manager.world.get_surface(surface)
    
    def run(self, parameters):
        raise BlastRuntimeError("Empty action run")


def load_actions(directory):
    for f in os.listdir(directory):
        if f[-3:] == ".py":
            execfile(os.path.join(directory, f))

#Let people create planing worlds....

class BlastManager:
    def __init__(self):
        self.world = blast_planner.BlastPlannableWorld(blast_world.make_test_world())
        self.world.real_world = True
        self.world.action_callback = lambda r, a, p: self.on_action_take(r, a, p)

        oefc = self.world.action_epic_fail_callback
        def efc(r, a, p):
            oefc(r, a, p)
            sys.exit(1)
        self.world.action_epic_fail_callback = efc
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
            return False
        return True
    def plan_action(self, robot, action, parameters):
        if self.world.plan_action(robot, action, parameters) == None:
            print "Epic fail for", robot, "-->", action
            return False
        return True


if __name__ == '__main__':
    man = BlastManager()
    load_actions("test_actions")
    man.plan_action("stair4", "coffee_run", {"person_location": blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor"), 
                                             "shop": "clark_peets_coffee_shop"})
