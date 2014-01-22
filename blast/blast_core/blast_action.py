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

    def set_failure(self, mode):
        if self._manager.get_current_guid() == self._guid:
            self._manager.failure_modes[self._guid] = mode
            return True
        return None
    
    def set_location(self, position, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).set_robot_location(self._robot, position.copy())
        self._manager.world_unlock()
        self._on_robot_change()
        return r

    def set_robot_holder(self, holder, ot, require_preexisting_object = True, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).set_robot_holder(self._robot, holder, ot,
                                                        require_preexisting_object)
        self._manager.world_unlock()
        self._on_robot_change()
        return r

    def robot_transfer_holder(self, from_holder, to_holder, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).robot_transfer_holder(self._robot, from_holder, to_holder)
        self._manager.world_unlock()
        self._on_robot_change()
        return r

    def list_surface_objects(surface, world = None):
        r = ""
        if not surface in self._manager._get_world(world).world.surfaces:
            return None
        r = ",".join( ["BlastObjectRef(" + str(x.uid) + ")" for x in \
                           self._manager._get_world(world).world.surfaces[surface].objects] )
        return r

    def get_robot_holder(self, holder, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).get_robot_holder(self._robot, holder)
        self._manager.world_unlock()
        self._on_robot_change()
        return r

    def delete_surface_object(self, obj, world = None):
        obj = obj.strip().strip("BlastObjectRef()").strip()
        try:
            obj = int(obj)
        except:
            return None
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).delete_surface_object(obj)
        self._manager.world_unlock()
        self._on_robot_change()
        return r
    
    def robot_pick_object(self, uid, to_holder, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).robot_pick_object(self._robot, uid, to_holder)
        self._manager.world_unlock()
        self._on_robot_change()
        return r
        
    def robot_place_object(self, from_h, surface, pos, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).robot_place_object(self._robot, from_h, surface, pos)
        self._manager.world_unlock()
        self._on_robot_change()
        return r
        
    def add_surface_object(self, surface, object_type, pos, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).add_surface_object(surface, object_type, pos)
        self._manager.world_unlock()
        self._on_robot_change()
        return r

    def set_robot_position(self, pos, val, world = None):
        r = None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            self._get_world(world).set_robot_position(self._robot, pos, val)
        self._manager.world_unlock()
        self._on_robot_change()
        return r

    def surface_scan(self, surface, object_types, world = None):
        r= None
        self._manager.world_lock()
        if self._manager.get_current_guid() == self._guid:
            r = self._get_world(world).surface_scan(surface, object_types)
        self._manager.world_unlock()
        self._on_robot_change()
        return r
        
    def plan_action(self, action, parameters, world_limits, world = None):
        r = None
        if self._manager.get_current_guid() == self._guid:
            wl = {}
            if "robot-holders" in world_limits:
                wl["robot-holders"] = {self._robot: world_limits["robot-holders"]}
            if "robot-location" in world_limits:
                wl["robot-location"] = {self._robot: world_limits["robot-location"]}
            r = self._manager.plan_action(self._robot, action, parameters, wl, True)
        print r
        if r == None:
            raise BlastRuntimeError("Planning to run action failed")
        return r
    
    def plan_hunt(self, holder, object_type, world = None):
        r = None
        if self._manager.get_current_guid() == self._guid:
            r = self._manager.plan_hunt(self._robot, holder, object_type, True)
        print r
        if r == None:
            raise BlastRuntimeError("Planning to object hunt action failed")
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
        return True

    def get_surface(self, surface, world = None):
        if type(surface) != type(""):
            surface = surface.name
        return self._get_world(world).get_surface(surface)

    def _get_world(self, world):
        if world == None: return self._manager.worlds[None]
        name = "action_" + str(self.guid) + "_" + new_world
        return self._manager.worlds[name]

    def get_object(self, obj, world = None):
        obj = obj.strip().strip("BlastObjectRef()").strip()
        try:
            obj = int(obj)
        except:
            return None
        return self._get_world(world).get_object(obj)
    
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
            #print result
            if type(result) != type(""):
                print "Ignore packet", result
            elif result.find("DELETE_SURFACE_OBJECT") == 0:
                if result.strip().split(",")[0].strip() == "DELETE_SURFACE_OBJECT":
                    world = result.strip().split(",")[1].strip()
                    obj = result.strip().split(",")[2].strip()
                else:
                    world = None
                    obj = result.strip().split(",")[1].strip()
                res = self.delete_surface_object(obj, world)
                if res:
                    proc.stdin.write("True\n")
                else:
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("GET_OBJECT") == 0:
                if result.strip().split(",")[0].strip() == "GET_OBJECT":
                    world = result.strip().split(",")[1].strip()
                    obj = result.strip().split(",")[2].strip()
                else:
                    world = None
                    obj = result.strip().split(",")[1].strip()
                res = self.get_object(obj, world)
                if res:
                    proc.stdin.write("OBJECT" + json.dumps(res).replace("\n", "\\n") + "\n")
                else:
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("GET_SURFACE") == 0:
                if result.strip().split(",")[0].strip() == "GET_SURFACE":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                res = self.get_surface(surface, world)
                if res:
                    proc.stdin.write("SURFACE" + json.dumps(res).replace("\n", "\\n") + "\n")
                else:
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("SURFACE_SCAN") == 0:
                if result.strip().split(",")[0].strip() == "SURFACE_SCAN":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                    object_types = [str(x) for x in result.strip().split(",")[3:]]
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                    object_types = [str(x) for x in result.strip().split(",")[2:]]
                res = self.get_surface(surface, world)
                if res:
                    proc.stdin.write(str(self.surface_scan(surface, object_types, world)) + "\n")
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
                world_limits = {}
                if "world limits" in parameters: #note this contains a space so never found as param
                    world_limits = parameters["world limits"]
                if "parameter values" in parameters:
                    parameters = parameters["parameter values"]
                try:
                    proc.stdin.write(str(self.plan_action(action, parameters, world_limits, world)) + "\n")
                except BlastRuntimeError as ex:
                    print "--- Runtime error ----", ex
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("PLAN_HUNT") == 0:
                if result.strip().split(",")[0].strip() == "PLAN_HUNT":
                    world = result.strip().split(",")[1].strip()
                    holder = result.strip().split(",")[2].strip()
                    object_type = json.loads(",".join(result.strip().split(",")[3:]))
                else:
                    world = None
                    holder = result.strip().split(",")[1].strip()
                    object_type = json.loads(",".join(result.strip().split(",")[2:]))
                try:
                    proc.stdin.write(str(self.plan_hunt(holder, object_type, world)) + "\n")
                except BlastRuntimeError as ex:
                    print "--- Runtime error ----", ex
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("SET_FAILURE") == 0:
                r = self.set_failure(result.split(",")[1].strip())
                if r == None:
                    r = "None"
                elif r:
                    r = "True"
                else:
                    r = "False"
                proc.stdin.write(r + "\n")
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
            elif result.find("GET_ROBOT_HOLDER") == 0:
                if result.strip().split(",")[0].strip() == "GET_ROBOT_HOLDER":
                    world = result.strip().split(",")[1].strip()
                    holder = result.strip().split(",")[2].strip()
                else:
                    world = None
                    holder = result.strip().split(",")[1].strip()
                proc.stdin.write(str(self.get_robot_holder(holder)) + "\n")
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
            elif result.find("ROBOT_PICK_OBJECT") == 0:
                
                if result.strip().split(",")[0].strip() == "ROBOT_PICK_OBJECT":
                    world = result.strip().split(",")[1].strip()
                    object_ref = result.strip().split(",")[2].strip()
                    to_h = result.strip().split(",")[3].strip()
                else:
                    world = None
                    object_ref = result.strip().split(",")[1].strip()
                    to_h = result.strip().split(",")[2].strip()
                object_ref = int(object_ref.strip("BlastObjectRef()")) #Remove down to uid
                proc.stdin.write(str(self.robot_pick_object(object_ref, to_h, world)) + "\n")
                proc.stdin.flush()
                
            elif result.find("ROBOT_PLACE_OBJECT") == 0:
                if result.strip().split(",")[0].strip() == "ROBOT_PLACE_OBJECT":
                    world = result.strip().split(",")[1].strip()
                    from_h = result.strip().split(",")[2].strip()
                    surface = result.strip().split(",")[3].strip()
                    pos = [float(x.strip()) for x in result.strip().split(",")[4:]]
                    pos = blast_world.BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                else:
                    world = None
                    from_h = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                    pos = [float(x.strip()) for x in result.strip().split(",")[3:]]
                    pos = blast_world.BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                proc.stdin.write(str(self.robot_place_object(from_h, surface, pos, world)) + "\n")
                proc.stdin.flush()

            elif result.find("ADD_SURFACE_OBJECT") == 0:
                if result.strip().split(",")[0].strip() == "ADD_SURFACE_OBJECT":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                    object_type = result.strip().split(",")[3].strip()
                    pos = [float(x.strip()) for x in result.strip().split(",")[4:]]
                    pos = blast_world.BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                    object_type = result.strip().split(",")[2].strip()
                    pos = [float(x.strip()) for x in result.strip().split(",")[3:]]
                    pos = blast_world.BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
            
                proc.stdin.write(str(self.add_surface_object(surface, object_type, pos, world)) + "\n")
                proc.stdin.flush()

            elif result.find("LIST_SURFACE_OBJECTS") == 0:
                if result.strip().split(",")[0].strip() == "LIST_SURFACE_OBJECTS":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                proc.stdin.write(str(self.list_surface_objects(surface, world)) + "\n")
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
        self.failure_modes = {}
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
        fail = self.failure_modes.get(self.get_current_guid(), None)
        if self.get_current_guid() in self.action_worlds:
            for world in self.action_worlds[self.get_current_guid()]:
                del self.worlds["action_" + str(self.get_current_guid()) + "_" + world]
            del self.action_worlds[self.get_current_guid()]
        self.action_stack.remove(exe)
        print "--- Done", action
        #print self.world.world.to_text()
        #print "--------"
        if fail: return fail
        return True
        

    def take_action(self, robot, action, parameters, do_cb = False):
        if self.world.take_action(robot, action, parameters) == None:
            print "Epic fail for", robot, "-->", action
            return False
        return True
    def plan_action(self, robot, action, parameters, world_limits, do_cb = False):
        if do_cb:
            sh = len(self.action_stack) - 1
            def update_cb(arg):
                self.implicit_plan[sh] = arg
                self.on_plan_action()
        else:
            update_cb = lambda x: None
        res = self.world.plan_action(robot, action, parameters, world_limits, execution_cb=update_cb)
        if res == None:
            print "Failed to plan action", robot, "-->", action
            return None
        return res

    def plan_hunt(self, robot, holder, object_type, do_cb = False):
        if do_cb:
            sh = len(self.action_stack) - 1
            def update_cb(arg):
                self.implicit_plan[sh] = arg
                self.on_plan_action()
        else:
            update_cb = lambda x: None
        res = self.world.plan_hunt(robot, holder, object_type, execution_cb=update_cb)
        if res == None:
            print "Epic fail for", robot, "-->", holder, "<-", object_type
            return None
        return res



def test_main():
    import blast_world_test
    man = BlastManager(["test_actions",], blast_world_test.make_test_world())
    if not man.plan_action("stair4", "coffee-run", {"person_location": 
                                                    blast_world.BlastPt(17.460, 38.323, -2.330,
                                                                        "clarkcenterfirstfloor"), 
                                                    "shop": "clark_peets_coffee_shop"}, {}):
        return False
    a = "c66ad51b77946fb8ac04e94a732e88297ce77fae9a7ae0bdb9768b06bccf2c953810ccc6c1bf70e02ac63b3"
    a = a + "8145ce40509e7f579d13b347d085a3deb79df8e417fa7d2bd93a92d744e22058b29c0012b2f282516"
    if a != man.world.world.get_hex_hash_state():
        return False
    return True

def test_place():
    import blast_world_test
    w = open("table_1_objects.txt", "w")
    w.close()
    w = open("table_2_objects.txt", "w")
    w.write("coffee_cup\n")
    w.close()

    man = BlastManager(["test_actions",], blast_world_test.make_table_top_world())

    print "-"*120
    print " "*60, "BEFORE PICK"
    print man.world.world.to_text()
    print "-"*120

    if not man.plan_hunt("stair4", "cupholder", "coffee_cup"):
        return False

    print "-"*120
    print " "*60, "AFTER PICK"
    print man.world.world.to_text()
    print "-"*120

    #man.plan_action("stair4", "unstash-cupholder", {})
    object_uid = man.world.world.robots["stair4"].holders["cupholder"].uid
    print "Hold onto the uid of", object_uid
    r = man.plan_action("stair4", "table-place-left", {"table": "table_1", 
                                                       "position": "table_1, Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)"},
                        {"robot-holders": {"stair4": {"left-arm": object_uid}}})
    if not r:
        return False

    print "-"*120
    print " "*60, "AFTER PLACE"
    print man.world.world.to_text()
    print "-"*120

    a = "937e356e0aebbd33554a3380547260698f20b9ca7dc2c6f23361598c994f85648b069fa200933a1f2a7c8f352"
    a = a + "d13d8ca3db1b0a1dc50f28e238cd4177368f587817290159b54892eb02743b464ca71a606d13a1d"
    if a != man.world.world.get_hex_hash_state():
        print "Invalid state, failed test"
        return False

    return True

if __name__ == '__main__':
    test_main()
    #test_place()

