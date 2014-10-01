import blast_world
import blast_planner
import os, sys, subprocess, json, time, threading

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

def jsonload(strs):
    r = json.loads(strs)
    def clean(r):
        if (type(r) == type({})):
            c = {}
            for k, v in r.iteritems():
                c[str(k)] = clean(v)
            return c
        elif type(r) == type([]) or type(r) == type((0,1)):
            return [clean(x) for x in r]
        elif type(r) == type(1) or type(r) == type(0.1):
            return r
        elif r == False or r == True or r == None:
            return r
        return str(r)
    return clean(r)

def json_prepare(dt):
    if type(dt) == list:
        return [json_prepare(x) for x in dt]
    if type(dt) == dict:
        c = {}
        for k, v in dt.iteritems():
            c[json_prepare(k)] = json_prepare(v)
        return c
    if type(dt) == blast_world.BlastSurface:
        return dt.name
    if type(dt) == blast_world.BlastPt:
        return dt.to_dict()
    if type(dt) == blast_world.BlastObjectRef:
        return dt.uid
    return dt



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

    def _get_manager_world(self, world):
        if world != None:
            raise Exception("Not yet. Infact, this should probably be removed")
        w = self._manager.world.world
        self._manager.world.lock.acquire()
        return w

    def _release_manager_world(self, world):
        if world != None:
            raise Exception("Not yet. Infact, this should probably be removed")
        w = self._manager.world.world
        self._manager.world.lock.release()
        
    
    def set_location(self, position, world = None):
        r = None
        w = self._get_manager_world(world)
        w.set_robot_location(self._robot, position.copy())
        self._release_manager_world(world)
        self._on_robot_change()
        return r

    def set_robot_holder(self, holder, ot, require_preexisting_object = True, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.set_robot_holder(self._robot, holder, ot,
                               require_preexisting_object)
        self._release_manager_world(world)
        self._on_robot_change()
        return r

    def robot_transfer_holder(self, from_holder, to_holder, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.robot_transfer_holder(self._robot, from_holder, to_holder)      
        self._release_manager_world(world)
        self._on_robot_change()
        return r

    def list_surface_objects(surface, world = None):
        r = ""
        if not surface in self._manager._get_world(world).world.surfaces:
            return None
        w = self._get_manager_world(world)
        r = ",".join( ["BlastObjectRef(" + str(x.uid) + ")" \
                           for x in w.surfaces[surface].objects] )
        self._release_manager_world(world)
        return r

    def get_robot_holder(self, holder, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.get_robot_holder(self._robot, holder)
        self._release_manager_world(world)
        self._on_robot_change()
        return r

    def delete_surface_object(self, obj, world = None):
        obj = obj.strip().strip("BlastObjectRef()").strip()
        try:
            obj = int(obj)
        except:
            return None
        r = None
        w = self._get_manager_world(world)
        r = w.delete_surface_object(obj)
        self._release_manager_world(world)
        self._on_robot_change()
        return r
    
    def robot_pick_object(self, uid, to_holder, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.robot_pick_object(self._robot, uid, to_holder)
        self._release_manager_world(world)
        self._on_robot_change()
        return r
        
    def robot_place_object(self, from_h, surface, pos, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.robot_place_object(self._robot, from_h, surface, pos)
        self._release_manager_world(world)
        self._on_robot_change()
        return r
        
    def add_surface_object(self, surface, object_type, pos, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.add_surface_object(surface, object_type, pos)
        self._release_manager_world(world)
        self._on_robot_change()
        return r

    def set_robot_position(self, pos, val, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.set_robot_position(self._robot, pos, val)
        self._release_manager_world(world)
        self._on_robot_change()
        return r

    def surface_scan(self, surface, object_types, world = None):
        r= None
        w = self._get_manager_world(world)
        r = w.surface_scan(surface, object_types)
        self._release_manager_world(world)
        self._on_robot_change()
        return r
        
    def take_action(self, action, parameters, world_limits, world = None):
        r = None
        
            #wl = {}
            #if "robot-holders" in world_limits:
            #    wl["robot-holders"] = {self._robot: world_limits["robot-holders"]}
            #if "robot-location" in world_limits:
            #    wl["robot-location"] = {self._robot: world_limits["robot-location"]}
        r = self._manager.take_action(self._robot, action, parameters)
        print r
        if r == None:
            raise BlastRuntimeError("Planning to run action failed")
        return r
    
    def plan_hunt(self, holder, object_type, world = None):
        raise Exception("We need to remove this")
        r = None
        if self._manager.get_current_guid() == self._guid:
            r = self._manager.plan_hunt(self._robot, holder, object_type, True)
        if r == None:
            raise BlastRuntimeError("Planning to object hunt object failed")
        return r

    def plan_place(self, uid, surface, place, world = None):
        raise Exception("We need to remove this")
        r = None
        if self._manager.get_current_guid() == self._guid:
            r = self._manager.plan_place(uid, surface, place, True)
        if r == None:
            raise BlastRuntimeError("Planning to place object failed")
        return r

    def create_world(self, new_world, world = None):
        raise Exception("We need to remove this")
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
        w = self._get_manager_world(world)
        if type(surface) != type(""):
            surface = surface.name
        s = w.get_surface(surface)
        if s: s = s.to_dict()
        self._release_manager_world(world)
        return s

    def get_object(self, obj, world = None):
        w = self._get_manager_world(world)
        obj = obj.strip().strip("BlastObjectRef()").strip()
        try:
            obj = int(obj)
        except:
            return None
        o = w.get_object(obj)
        if o: o = o.to_dict()
        self._release_manager_world(world)
        return o
    
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
        proc = subprocess.Popen(['python', "blast_action_exec.py", py_file, json.dumps(json_prepare(parameters))],
                                stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=sys.stdout)
        message = None

        error = False
        while True:
            result = proc.stdout.readline()
            print result
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
                    proc.stdin.write("OBJECT" + json.dumps(json_prepare(res)).replace("\n", "\\n") + "\n")
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
                    proc.stdin.write("SURFACE" + json.dumps(json_prepare(res)).replace("\n", "\\n") + "\n")
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
            elif result.find("TAKE_ACTION") == 0:
                if result.strip().split(",")[0].strip() == "TAKE_ACTION":
                    world = result.strip().split(",")[1].strip()
                    action = result.strip().split(",")[2].strip()
                    parameters = jsonload(",".join(result.strip().split(",")[3:]))
                else:
                    world = None
                    action = result.strip().split(",")[1].strip()
                    parameters = jsonload(",".join(result.strip().split(",")[2:]))
                try:
                    proc.stdin.write(str(self.take_action(action, parameters, world)) + "\n")
                except BlastRuntimeError as ex:
                    print "--- Runtime error ----", ex
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("PLAN_PLACE") == 0:
                if result.strip().split(",")[0].strip() == "PLAN_PLACE":
                    world = result.strip().split(",")[1].strip()
                    uid = result.strip().split(",")[2].strip()
                    surface = result.strip().split(",")[3].strip()
                    pos = [x.strip().replace("Pos","").strip('()').strip() for x in result.strip().split(",")[4:]]
                else:
                    world = None
                    uid = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                    pos = [x.strip().replace("Pos","").strip('()').strip() for x in result.strip().split(",")[3:]]
                if pos[0] != surface:
                    proc.stdin.write("None\n")
                else:
                    try:
                        proc.stdin.write(str(self.plan_place(uid, surface, pos)) + "\n")
                    except BlastRuntimeError as ex:
                        print "--- Runtime error ----", ex
                        proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("PLAN_HUNT") == 0:
                if result.strip().split(",")[0].strip() == "PLAN_HUNT":
                    world = result.strip().split(",")[1].strip()
                    holder = result.strip().split(",")[2].strip()
                    object_type = result.strip().split(",")[3].strip()
                else:
                    world = None
                    holder = result.strip().split(",")[1].strip()
                    object_type = result.strip().split(",")[2].strip()
                try:
                    r = self.plan_hunt(holder, object_type, world)
                    if r == False or r == None:
                        proc.stdin.write(str(r) + "\n")
                    else:
                        proc.stdin.write(",".join([str(x) for x in r]) + "\n")
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
                    location = jsonload(",".join(result.strip().split(",")[2:]))
                else:
                    world = None
                    location = jsonload(",".join(result.strip().split(",")[1:]))
                if self.set_location(location, world):
                    proc.stdin.write("True\n")
                else:
                    proc.stdin.write("None\n")
                proc.stdin.flush()
            elif result.find("SET_ROBOT_POSITION") == 0:
                if result.strip().split(",")[0].strip() == "SET_ROBOT_POSITION":
                    world = result.strip().split(",")[1].strip()
                    position = result.strip().split(",")[2].strip()
                    state = jsonload(",".join(result.strip().split(",")[3:]))
                else:
                    world = None
                    position = result.strip().split(",")[1].strip()
                    state = jsonload(",".join(result.strip().split(",")[2:]))
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
                    pos = [float(str(x).strip()) for x in result.strip().split(",")[4:]]
                    pos = blast_world.BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                else:
                    world = None
                    from_h = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                    pos = [float(str(x).strip()) for x in result.strip().split(",")[3:]]
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
        self.actions_running = []
        self.action_guid = 0
        self.action_worlds = {}
        self.failure_modes = {}
        self.worlds = {None: self.world}
        self.is_stopped = False

        #Start running. Don't put code after here
        def runthread():
            self.world.run()
        self.worldthread = threading.Thread(target=runthread)
        self.worldthread.daemon = True
        self.worldthread.start()

    def __del__(self):
        self.stop()
    def stop(self):
        self.is_stopped = True
        for name, world in self.worlds.iteritems():
            world.stop()


    def take_action(self, robot, action, parameters):
        return self.on_action_take(robot, action, parameters)

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
        my_guid = self.action_guid
        exe = BlastActionExec(robot, self, my_guid, action_exec, rc)
        self.action_guid = self.action_guid + 1
        self.actions_running.append(exe)
        
        exe.run(parameters)
        fail = self.failure_modes.get(my_guid, None)
        #if self.get_current_guid() in self.action_worlds:
        #    for world in self.action_worlds[self.get_current_guid()]:
        #        del self.worlds["action_" + str(self.get_current_guid()) + "_" + world]
        #    del self.action_worlds[self.get_current_guid()]
        self.actions_running.remove(exe)
        print "--- Done", action, "Fail State:", fail
        #print self.world.world.to_text()
        #print "--------"
        if fail != False and fail != None: return fail
        print "Return true"
        return True
    
    def plan_action(self, robot, action, parameters, world_limits, do_cb = False):
        if do_cb:
            sh = len(self.action_stack) - 1
            def update_cb(arg):
                self.implicit_plan[sh] = arg
                self.on_plan_action()
        else:
            update_cb = lambda x: None
        res = self.world.plan_action(robot, action, parameters, world_limits) #, execution_cb=update_cb)
        if res == None:
            print "Failed to plan action", robot, "-->", action
            return None
        return res

    def plan_place(self, uid, surface, pos, do_cb = False):
        if do_cb:
            sh = len(self.action_stack) - 1
            def update_cb(arg):
                self.implicit_plan[sh] = arg
                self.on_plan_action()
        else:
            update_cb = lambda x: None
        res = self.world.plan_place(uid, surface, pos, execution_cb=update_cb)
        if res == None:
            print "Epic fail for place", uid, "-->", surface, "at", pos
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

    def get_program_state(self, program):
        return self.world.get_program_state(program)
    
    def wait_for_program(self, program):
        while True:
            ps = self.world.get_program_state(program)
            if ps != None:
                return ps
            time.sleep(0.01)



def test_main():
    import blast_world_test
    man = BlastManager(["test_actions",], blast_world_test.make_test_world())
    prog_id = man.plan_action("stair4", "coffee-run", {"person_location": 
                                                       blast_world.BlastPt(17.460, 38.323, -2.330,
                                                                           "clarkcenterfirstfloor"), 
                                                       "shop": "clark_peets_coffee_shop"}, {})
    if prog_id == None:
        print "Failed to plan", prog_id
        man.stop()
        return False
    if man.wait_for_program(prog_id) == False:
        print "Action execution failed"
        man.stop()
        return False
    man.stop()
    a = "0a74cbe0c1cd7804120143ffe687a63bf8baec2f00d11e47583d075b7b3fc0350a5bcf3be0a1e4d6977904"
    a = a + "dde7e04531ac7662d74ade2749085a3deb79df8e417fa7d2bd93a92d744e22058b29c0012b2f282516"
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
                                                       "position": ("table_1", "Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)")},
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

def test_multi_robot():
    import blast_world_test
    man = BlastManager(["test_actions",], blast_world_test.make_table_top_world(False))


    w = open("table_1_objects.txt", "w")
    w.close()
    w = open("table_2_objects.txt", "w")
    w.write("coffee_cup\n")
    w.write("coffee_cup\n")
    w.close()

    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    man.world.world.types.get_robot("pr2-cupholder"))
    man.world.world.append_robot(stair5)
    man.world.world.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.

    print man.world.world.to_text()

    if not man.plan_hunt("stair4", "cupholder", "coffee_cup"):
        return False


    #print man.world.world.to_text()

    #if not man.plan_hunt("stair5", "cupholder", "coffee_cup"):
    #    return False

    print man.world.world.to_text()

    return True


if __name__ == '__main__':
    print test_main()
    #print test_place()
    #print test_multi_robot()
