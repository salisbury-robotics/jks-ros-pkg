import blast_world
import blast_planner
import os, sys, subprocess, json

blast_action_exec_d = {}
def set_action_exec(robot_type, action_type, item):
    if not robot_type in blast_action_exec_d:
        blast_action_exec_d[robot_type] = {}
    blast_action_exec_d[robot_type][action_type] = item

class BlastRuntimeError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
    
def thunk():
    return

class BlastActionExec:
    def __init__(self, robot, manager, guid, filenames, on_robot_change = thunk):
        self._robot = robot
        self._manager = manager
        self._guid = guid
        self._filenames = filenames
        self._on_robot_change = on_robot_change
    
    def set_location(self, position, world = None):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._get_world(world).set_robot_location(self._robot, position.copy())
        self._manager.world_unlock()
        self._on_robot_change()

    def set_robot_holder(self, holder, ot, require_preexisting_object = True, world = None):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._get_world(world).set_robot_holder(self._robot, holder, ot,
                                                         require_preexisting_object)
        self._manager.world_unlock()
        self._on_robot_change()

    def robot_transfer_holder(self, from_holder, to_holder, world = None):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._get_world(world).robot_transfer_holder(self._robot, from_holder, to_holder)
        self._manager.world_unlock()
        self._on_robot_change()

    def set_robot_position(self, pos, val, world = None):
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._get_world(world).set_robot_position(self._robot, pos, val)
        self._manager.world_unlock()
        self._on_robot_change()
        
    def plan_action(self, action, parameters, world = None):
        r = None
        if self._manager.get_current_guid() == self._guid:
            r = self._manager.plan_action(self._robot, action, parameters, True)
        print r
        if r == None:
            raise BlastRuntimeError("Planning to run action failed")
        return r

    def create_world(self, new_world, world = None):
        if type(new_world) != type(""):
            raise BlastRuntimeError("new_world name must be a string")
        self._manager.world_lock()
        if new_world in self._manager.worlds:
            name = "action_" + str(self.guid) + "_" + new_world
            self._manager.worlds[name] = self._get_world(world).copy()
            self._manager.action_worlds[self.guid] = self._manager.action_worlds.get(self.guid, []) + [name,]
        self._manager.world_unlock()

    def get_surface(self, surface, world = None):
        if type(surface) != type(""):
            surface = surface.name
        return self._get_world(world).get_surface(surface)

    def _get_world(self, world):
        if world == None: return self._manager.worlds[None]
        name = "action_" + str(self.guid) + "_" + new_world
        return self._manager.worlds[name]
    
    def run(self, parameters):
        py_file = None
        for filename in self._filenames:
            for direct in self._manager.directories:
                if os.path.exists(os.path.join(direct, filename)):
                    py_file = os.path.join(direct, filename)
                    break
            if py_file: break
        if not py_file:
            raise BlastRuntimeError("No file for action with options " + str(self._filenames) 
                                    + " in directories " + str(self._manager.directories))
        proc = subprocess.Popen(['python', "blast_action_exec.py", py_file, json.dumps(parameters)],
                                stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=sys.stdout)
        message = None

        error = False
        while True:
            result = proc.stdout.readline()
            if type(result) != type(""):
                print "Ignore packet", result
            elif result.find("GET_SURFACE") == 0:
                if result.strip().split(",")[0].strip() == "GET_SURFACE":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                res = self.get_surface(surface, world)
                if res:
                    proc.stdin.write("SURFACE" + json.dumps(self.get_surface(surface, world)).replace("\n", "\\n") + "\n")
                else:
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("PLAN_ACTION") == 0:
                if result.strip().split(",")[0].strip() == "PLAN_ACTION":
                    world = result.strip().split(",")[1].strip()
                    action = result.strip().split(",")[2].strip()
                    parameters = json.loads(",".join(result.strip().split(",")[3:]))
                else:
                    world = None
                    action = result.strip().split(",")[1].strip()
                    parameters = json.loads(",".join(result.strip().split(",")[2:]))
                try:
                    proc.stdin.write(str(self.plan_action(action, parameters, world)) + "\n")
                except BlastRuntimeError as ex:
                    print "--- Runtime error ----", ex
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("SET_ROBOT_LOCATION") == 0:
                if result.strip().split(",")[0].strip() == "SET_ROBOT_LOCATION":
                    world = result.strip().split(",")[1].strip()
                    location = json.loads(",".join(result.strip().split(",")[2:]))
                else:
                    world = None
                    location = json.loads(",".join(result.strip().split(",")[1:]))
                if self.set_location(location, world):
                    proc.stdin.write("True\n")
                else:
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("SET_ROBOT_POSITION") == 0:
                if result.strip().split(",")[0].strip() == "SET_ROBOT_POSITION":
                    world = result.strip().split(",")[1].strip()
                    position = result.strip().split(",")[2].strip()
                    state = json.loads(",".join(result.strip().split(",")[3:]))
                else:
                    world = None
                    position = result.strip().split(",")[1].strip()
                    state = json.loads(",".join(result.strip().split(",")[2:]))
                proc.stdin.write(str(self.set_robot_position(position, state, world)) + "\n")
                proc.stdin.flush()
            elif result.find("SET_ROBOT_HOLDER") == 0:
                if result.strip().split(",")[0].strip() == "SET_ROBOT_HOLDER":
                    world = result.strip().split(",")[1].strip()
                    holder = result.strip().split(",")[2].strip()
                    ot = result.strip().split(",")[3].strip()
                    req = result.strip().split(",")[4].strip() == "True"
                else:
                    world = None
                    holder = result.strip().split(",")[1].strip()
                    ot = result.strip().split(",")[2].strip()
                    req = result.strip().split(",")[3].strip() == "True"
                if ot == "None()": ot = None
                proc.stdin.write(str(self.set_robot_holder(holder, ot, req, world)) + "\n")
                proc.stdin.flush()
            elif result.find("ROBOT_TRANSFER_HOLDER") == 0:
                if result.strip().split(",")[0].strip() == "ROBOT_TRANSFER_HOLDER":
                    world = result.strip().split(",")[1].strip()
                    from_h = result.strip().split(",")[2].strip()
                    to_h = result.strip().split(",")[3].strip()
                else:
                    world = None
                    from_h = result.strip().split(",")[1].strip()
                    to_h = result.strip().split(",")[2].strip()
                proc.stdin.write(str(self.robot_transfer_holder(from_h, to_h, world)) + "\n")
                proc.stdin.flush()
            elif result.strip() == "TERMINATE":
                break
            elif result.strip() == "ERROR":
                error = True
                break
            elif result.strip() != "":
                print "Bad IPC packet", '"' + result.strip() + '"'

        if error:
            raise BlastRuntimeError("Internal action error")
        return not error

class BlastManager:
    def __init__(self, directories, world):
        self.directories = directories
        self.world = blast_planner.BlastPlannableWorld(world)
        self.world.real_world = True
        self.world.action_callback = lambda r, a, p: self.on_action_take(r, a, p)
        self.on_robot_change = lambda robot: None
        self.on_plan_action = lambda: None

        oefc = self.world.action_epic_fail_callback
        def efc(r, a, p):
            oefc(r, a, p)
            sys.exit(1)
        self.world.action_epic_fail_callback = efc
        self.action_stack = []
        self.action_guid = 0
        self.action_worlds = {}
        self.worlds = {None: self.world}

        self.implicit_plan = []



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
        action_exec = []
        while robot_type:
            action_exec.append(robot_type.name + "__" + action + ".py")
            robot_type = robot_type.parent

        print "--- Exec action", robot, "-->", action, parameters
        def rc():
            self.on_robot_change(robot)
        exe = BlastActionExec(robot, self, self.action_guid, action_exec, rc)
        self.action_guid = self.action_guid + 1
        self.action_stack.append(exe)
        
        while len(self.implicit_plan) < len(self.action_stack):
            self.implicit_plan.append([])
        self.implicit_plan[len(self.action_stack)-1] = []
        
        exe.run(parameters)
        if self.get_current_guid() in self.action_worlds:
            for world in self.action_worlds[self.get_current_guid()]:
                del self.worlds["action_" + str(self.get_current_guid()) + "_" + world]
            del self.action_worlds[self.get_current_guid()]
        self.action_stack.remove(exe)
        print "--- Done", action
        return True
        

    def take_action(self, robot, action, parameters, do_cb = False):
        if self.world.take_action(robot, action, parameters) == None:
            print "Epic fail for", robot, "-->", action
            return False
        return True
    def plan_action(self, robot, action, parameters, do_cb = False):
        if do_cb:
            sh = len(self.action_stack) - 1
            def update_cb(arg):
                self.implicit_plan[sh] = arg
                self.on_plan_action()
        else:
            update_db = lambda x: None
        res = self.world.plan_action(robot, action, parameters, execution_cb=update_cb)
        if res == None:
            print "Epic fail for", robot, "-->", action
            return None
        return res


def test_main():
    man = BlastManager(["test_actions",], blast_world.make_test_world())
    man.plan_action("stair4", "coffee-run", {"person_location": blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor"), 
                                             "shop": "clark_peets_coffee_shop"})
if __name__ == '__main__':
    test_main()

