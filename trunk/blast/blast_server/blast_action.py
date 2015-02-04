import blast_world
import blast_planner
import SocketServer
import os, sys, subprocess, json, time, threading

blast_action_exec_d = {}
def set_action_exec(robot_type, action_type, item):
    if not robot_type in blast_action_exec_d:
        blast_action_exec_d[robot_type] = {}
    blast_action_exec_d[robot_type][action_type] = item

def enc_str(s):
    return s.replace("%", "%p").replace("\n", "%n").replace(",", "%c")
def dec_str(s):
    return s.replace("%n", "\n").replace("%c", ",").replace("%p", "%")
def is_int(x):
    try:
        x = int(x)
        return True
    except:
        return False



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
    if type(dt) == list or type(dt) == tuple:
        return [json_prepare(x) for x in dt]
    if type(dt) == dict:
        c = {}
        for k, v in dt.iteritems():
            c[json_prepare(k)] = json_prepare(v)
        return c
    if type(dt) == blast_world.BlastSurface:
        return dt.name
    if type(dt) in [blast_world.BlastPt, blast_world.BlastPos, blast_world.BlastPosIrr]:
        return dt.to_dict()
    if type(dt) == blast_world.BlastObjectRef:
        return dt.uid
    return dt



class BlastActionExec:
    def __init__(self, robot, manager, guid, action_robot_type, action, on_robot_change = thunk, on_surface_change = thunk):
        self._robot = robot
        self._manager = manager
        self._guid = guid
        self._action_robot_type = action_robot_type
        self._action = action
        self._on_robot_change = on_robot_change
        self._on_surface_change = on_surface_change

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
        
    
    def get_location(self, world = None):
        r = None
        w = self._get_manager_world(world)
        if self._robot in w.robots:
            r = w.robots[self._robot].location.to_dict()
        self._release_manager_world(world)
        return r

    def set_location(self, position, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.set_robot_location(self._robot, position.copy())
        self._release_manager_world(world)
        if r == True:
            self._on_robot_change()
            return True
        if r == False:
            return True
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
        sn = w.objects.get(obj)
        if sn: sn = sn.parent
        r = w.delete_surface_object(obj)
        self._release_manager_world(world)
        self._on_robot_change()
        if sn: self._on_surface_change(sn)
        return r
    
    def robot_pick_object(self, uid, to_holder, world = None):
        r = None
        w = self._get_manager_world(world)
        sn = w.objects.get(uid)
        if sn: sn = sn.parent
        r = w.robot_pick_object(self._robot, uid, to_holder)
        self._release_manager_world(world)
        self._on_robot_change()
        self._on_surface_change(sn)
        return r
        
    def robot_place_object(self, from_h, surface, pos, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.robot_place_object(self._robot, from_h, surface, pos)
        self._release_manager_world(world)
        self._on_robot_change()
        self._on_surface_change(surface)
        return r
        
    def add_surface_object(self, surface, object_type, pos, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.add_surface_object(surface, object_type, pos)
        self._release_manager_world(world)
        self._on_robot_change()
        self._on_surface_change(surface)
        return r

    def set_robot_position(self, pos, val, world = None):
        r = None
        w = self._get_manager_world(world)
        r = w.set_robot_position(self._robot, pos, val)
        self._release_manager_world(world)
        if r == True:
            self._on_robot_change()
        if r == False:
            return True
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
        print "Getting world"
        w = self._get_manager_world(None)
        start_robot = w.robots.get(self._robot, None)
        if start_robot == None:
            print "Could not connect to the robot"
            #TODO: EPIC FAIL
            self._release_manager_world(None)
            return
        if start_robot.is_active in [False, None, True]:
            print "Robot is not ready", start_robot.is_active
            #TODO: EPIC FAIL
            self._release_manager_world(None)
            return
        self._release_manager_world(None)
        
        print "Starting action, robot is ready"
        write_data, read_data = start_robot.is_active.start_action(self._action_robot_type, self._action, parameters)
        if read_data == None or write_data == None:
            print "Could not start action"
            #TODO: EPIC FAIL
            return
        
        w = None
                
        message = None
        error = False
        while True:
            result = read_data()
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
                    write_data("True\n")
                else:
                    write_data("None\n")
            elif result.find("GET_OBJECT") == 0:
                if result.strip().split(",")[0].strip() == "GET_OBJECT":
                    world = result.strip().split(",")[1].strip()
                    obj = result.strip().split(",")[2].strip()
                else:
                    world = None
                    obj = result.strip().split(",")[1].strip()
                res = self.get_object(obj, world)
                if res:
                    write_data("OBJECT" + json.dumps(json_prepare(res)).replace("\n", "\\n") + "\n")
                else:
                    write_data("None\n")
            elif result.find("GET_SURFACE") == 0:
                if result.strip().split(",")[0].strip() == "GET_SURFACE":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                res = self.get_surface(surface, world)
                if res:
                    write_data("SURFACE" + json.dumps(json_prepare(res)).replace("\n", "\\n") + "\n")
                else:
                    write_data("None\n")
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
                    write_data(str(self.surface_scan(surface, object_types, world)) + "\n")
                else:
                    write_data("None\n")
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
                    write_data(str(self.take_action(action, parameters, world)) + "\n")
                except BlastRuntimeError as ex:
                    print "--- Runtime error ----", ex
                    write_data("None\n")
            elif result.find("PLAN_PLACE") == 0:
                raise Exception("THIS IS DEAD")
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
                    write_data("None\n")
                else:
                    try:
                        write_data(str(self.plan_place(uid, surface, pos)) + "\n")
                    except BlastRuntimeError as ex:
                        print "--- Runtime error ----", ex
                        write_data("None\n")
            elif result.find("PLAN_HUNT") == 0:
                raise Exception("THIS IS DEAD")
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
                        write_data(str(r) + "\n")
                    else:
                        write_data(",".join([str(x) for x in r]) + "\n")
                except BlastRuntimeError as ex:
                    print "--- Runtime error ----", ex
                    write_data("None\n")
            elif result.find("SET_FAILURE") == 0:
                r = self.set_failure(result.split(",")[1].strip())
                if r == None:
                    r = "None"
                elif r:
                    r = "True"
                else:
                    r = "False"
                write_data(r + "\n")
            elif result.find("GET_ROBOT_LOCATION") == 0:
                if result.strip().split(",")[0].strip() == "GET_ROBOT_LOCATION":
                    world = result.strip().split(",")[1].strip()
                else:
                    world = None
                write_data("LOCATION" + json.dumps(json_prepare(self.get_location(world))).replace("\n", "\\n") + "\n")
            elif result.find("SET_ROBOT_LOCATION") == 0:
                if result.strip().split(",")[0].strip() == "SET_ROBOT_LOCATION":
                    world = result.strip().split(",")[1].strip()
                    location = jsonload(",".join(result.strip().split(",")[2:]))
                else:
                    world = None
                    location = jsonload(",".join(result.strip().split(",")[1:]))
                if self.set_location(location, world):
                    write_data("True\n")
                else:
                    write_data("None\n")
            elif result.find("SET_ROBOT_POSITION") == 0:
                if result.strip().split(",")[0].strip() == "SET_ROBOT_POSITION":
                    world = result.strip().split(",")[1].strip()
                    position = result.strip().split(",")[2].strip()
                    state = jsonload(",".join(result.strip().split(",")[3:]))
                else:
                    world = None
                    position = result.strip().split(",")[1].strip()
                    state = jsonload(",".join(result.strip().split(",")[2:]))
                write_data(str(self.set_robot_position(position, state, world)) + "\n")
            elif result.find("GET_ROBOT_HOLDER") == 0:
                if result.strip().split(",")[0].strip() == "GET_ROBOT_HOLDER":
                    world = result.strip().split(",")[1].strip()
                    holder = result.strip().split(",")[2].strip()
                else:
                    world = None
                    holder = result.strip().split(",")[1].strip()
                write_data(str(self.get_robot_holder(holder)) + "\n")
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
                write_data(str(self.set_robot_holder(holder, ot, req, world)) + "\n")
            elif result.find("ROBOT_TRANSFER_HOLDER") == 0:
                if result.strip().split(",")[0].strip() == "ROBOT_TRANSFER_HOLDER":
                    world = result.strip().split(",")[1].strip()
                    from_h = result.strip().split(",")[2].strip()
                    to_h = result.strip().split(",")[3].strip()
                else:
                    world = None
                    from_h = result.strip().split(",")[1].strip()
                    to_h = result.strip().split(",")[2].strip()
                write_data(str(self.robot_transfer_holder(from_h, to_h, world)) + "\n")
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
                write_data(str(self.robot_pick_object(object_ref, to_h, world)) + "\n")                
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
                write_data(str(self.robot_place_object(from_h, surface, pos, world)) + "\n")
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
                write_data(str(self.add_surface_object(surface, object_type, pos, world)) + "\n")
            elif result.find("LIST_SURFACE_OBJECTS") == 0:
                if result.strip().split(",")[0].strip() == "LIST_SURFACE_OBJECTS":
                    world = result.strip().split(",")[1].strip()
                    surface = result.strip().split(",")[2].strip()
                else:
                    world = None
                    surface = result.strip().split(",")[1].strip()
                write_data(str(self.list_surface_objects(surface, world)) + "\n")                
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

ROBOT_STATE_NONE = 0
ROBOT_STATE_NEED_INFO = 1
ROBOT_STATE_CONTROL = 2

one_manager_lock = threading.Lock()
one_manager = None
class BlastManagedRobot(SocketServer.BaseRequestHandler):
    def action_write(self, found_id, strda):
        #print "Writing data", strda
        self.lock.acquire()
        self.request.sendall(str(found_id) + "," + enc_str(strda) + "\n")
        self.lock.release()
        #print "Done"
    def action_read(self, found_id):
        while True:
            self.lock.acquire()
            if self.action_queues[found_id] != []:
                break
            self.lock.release()
            time.sleep(0.01)
            if not self.alive:
                print "Loop is no longer alive"
                return "TERMINATE"
        #Lock is still aquired from loop
        r = self.action_queues[found_id][0]
        self.action_queues[found_id] = self.action_queues[found_id][1:]
        self.lock.release()
        #print "Read from queue:", found_id, r
        return r

    def start_action(self, robot_type, action_type, parameters):
        if not self.ready_to_start or not self.alive:
            print "Robot is not ready to start action"
            return None, None
        else:
            #TODO sanitize robot_type, action_type
            self.lock.acquire()
            print "Robot sending request"
            m_id = self.action_id
            self.action_id += 1
            self.action_start_queue.append(m_id)
            self.request.sendall("START_ACTION," + robot_type + "," + action_type
                                 + "," + enc_str(json.dumps(json_prepare(parameters))) + "\n")
            self.lock.release()
            print "Waiting for response"
            found_id = None
            while self.alive and found_id == None:
                time.sleep(0.01)
                self.lock.acquire()
                if m_id in self.action_id_to_client:
                    found_id = self.action_id_to_client[m_id]
                self.lock.release()
            print "Got response", found_id
            if found_id != None:
                action_write = lambda strda: self.action_write(found_id, strda)
                action_read = lambda: self.action_read(found_id)
                return action_write, action_read
            else:
                return None, None

    def set_teleop(self, session_name, activate):
        self.lock.acquire()
        if self.teleop == session_name or self.teleop == None:
            if activate:
                self.teleop = session_name
            else:
                self.teleop = None
            self.lock.release()
            return True
        self.lock.release()
        return False

    def get_teleop(self, session_name):
        r = None
        self.lock.acquire()
        if self.teleop == session_name:
            r = True
        elif self.teleop != None:
            r = False
        self.lock.release()
        return r

    def handle(self):
        self.alive = True
        self.root_action_thread = None
        self.lock = threading.Lock()
        self.ready_to_start = False
        self.action_id = 0
        self.action_start_queue = []
        self.action_queues = {}
        self.action_id_to_client = {}
        self.teleop = None

        manager = one_manager
        if not manager:
            print "Tried to open connection, but there was no manager"
            return
        print "Starting connection"
        buff = ""

        state = 0
        robot_name = None
        # 0 = no information
        # 1 = running
        # 2 = wait for name
        # 3 = map check
        # 4 = wait for location


        while True:
            start_a = False
            if buff.find("\n") == -1:
                nxt = self.request.recv(1024)
                #print "State", state, "val", nxt
                if not nxt: break
                buff += nxt
            if buff.find("\n") != -1:
                packet = buff[0:buff.find("\n")].strip()
                buff = buff[buff.find("\n")+1:]
                
                #print "Packet", state, "data", packet
                self.lock.acquire()
                if state == 0:
                    if packet == "BLAST_ROBOT_CONNECT":
                        state = 2
                        self.request.sendall("WAIT_NAME\n")
                    else:
                        self.request.sendall("ERROR,INVALID PROTOCOL\n")
                        self.lock.release()
                        break
                elif state == 1:
                    start = packet.split(",")[0].strip()
                    #print "We recieved", packet
                    if start == "STARTED_ACTION":
                        a_id = int(packet.split(",")[1].strip())
                        m_id = int(self.action_start_queue[0])
                        self.action_start_queue = self.action_start_queue[1:]
                        self.action_queues[a_id] = []
                        self.action_id_to_client[m_id] = a_id
                    elif is_int(start):
                        a_id = int(packet.split(",")[0].strip())
                        self.action_queues[a_id].append(dec_str(",".join(packet.split(",")[1:])))
                    else:
                        self.request.sendall("ERROR,INVALID COMMAND\n")
                        self.lock.release()
                        break
                elif state == 2:
                    if packet.split(",")[0].strip() == "ROBOT":
                        robot_name = packet.split(",")[1].strip()
                        robot_type = packet.split(",")[2].strip()
                        manager.world.lock.acquire()
                        robot = manager.world.world.get_robot(robot_name)
                        if not robot:
                            print "Invalid robot"
                            self.request.sendall("ERROR,INVALID ROBOT\n")
                            manager.world.lock.release()
                            self.lock.release()
                            break
                        if robot.robot_type.name != robot_type:
                            print "Invalid robot type"
                            self.request.sendall("ERROR,INVALID TYPE\n")
                            manager.world.lock.release()
                            self.lock.release()
                            break
                        else:
                            robot.is_active = False
                            manager.world.lock.release()
                            manager.world.on_program_changed()
                            self.request.sendall("VALID_ROBOT\n")
                            state = 3
                    else:
                        self.request.sendall("ERROR,INVALID COMMAND\n")
                        self.lock.release()
                        break
                elif state == 3 and packet != "GET_LOCATION":
                    if packet == "START":
                        state = 1
                        manager.world.lock.acquire()
                        robot = manager.world.world.get_robot(robot_name)
                        if robot:
                            self.ready_to_start = True
                            robot.is_active = self
                            rt, at = manager.world.world.types.get_action_for_robot(robot.robot_type.name, "__root")
                            manager.world.lock.release()
                            manager.world.on_program_changed()
                            self.request.sendall("STARTED\n")
                            if rt != None and at != None:
                                #print "Starting root action", rt, at
                                start_a = (rt.name, at.name.split(".")[1], {})
                        else:
                            manager.world.lock.release()
                            self.request.sendall("ERROR,INVALID COMMAND\n")
                    elif packet == "LIST_MAPS":
                        ms = ["MAPS"]
                        manager.world.lock.acquire()
                        for mp in manager.world.world.maps_keysort:
                            ms.append(mp)
                            ms.append(enc_str(manager.world.world.get_map(mp).imagehash))
                        manager.world.lock.release()
                        self.request.sendall(",".join(ms) + ",\n")
                    elif packet == "LIST_ACTIONS":
                        manager.world.lock.acquire()
                        actions = manager.world.world.enumerate_robot(robot_name, False, True, True, False)
                        ac = ["ACTIONS",]
                        #print "Enumerate action for LIST_ACTIONS", robot_name, actions
                        for at, av in actions:
                            ac.append(at)
                            ac.append(av)
                        manager.world.lock.release()
                        self.request.sendall(",".join(ac) + ",\n")
                    elif packet.find("GET_MAP,") == 0:
                        mn = packet.split(",")[1].strip()
                        #print "Getting map", mn
                        mf = manager.world.world.get_map(mn)
                        manager.world.lock.acquire()
                        if mf != None:
                            #enc_str(mf.mapdata)
                            l = ",".join(["MAP", enc_str(mn), enc_str(str(mf.ppm)),
                                                           enc_str(mf.mapdata), "\n"])
                            manager.world.lock.release()
                            self.request.sendall(l)
                        else:
                            manager.world.lock.release()
                            self.request.sendall("ERROR,INVALID MAP\n")
                            self.lock.release()
                            break
                    else:
                        self.request.sendall("ERROR,INVALID COMMAND\n")
                        self.lock.release()
                        break
                elif state == 4 or (state == 3 and packet == "GET_LOCATION"):
                    if packet == "GET_LOCATION":
                        manager.robot_waiting_for(robot_name, "LOCATION")
                        self.request.sendall("WAIT_LOCATION\n")
                    elif packet == "CHECK_LOCATION":
                        self.request.sendall("LOCATION," + str(manager.robot_has(robot_name, "LOCATION")) + "\n")
                    elif packet == "START" and manager.robot_has(robot_name, "LOCATION"):
                        state = 1
                        manager.world.lock.acquire()
                        robot = manager.world.world.get_robot(robot_name)
                        if robot:
                            self.ready_to_start = True
                            robot.is_active = self
                            rt, at = manager.world.world.types.get_action_for_robot(robot.robot_type.name, "__root")
                            manager.world.lock.release()
                            manager.world.on_program_changed()
                            self.request.sendall("STARTED\n")
                            if rt != None and at != None:
                                #print "Starting root action in GET_LOC", rt, at
                                start_a = (rt.name, at.name.split(".")[1], {})
                        else:
                            manager.world.lock.release()
                            self.request.sendall("ERROR,INVALID COMMAND\n")
                    else:
                        self.request.sendall("ERROR,INVALID COMMAND\n")
                        self.lock.release()
                        break
                if start_a != False:
                    #self.start_action(start_a[0], start_a[1], start_a[2])
                    self.root_action_thread = threading.Thread(target = manager.root_action_thread,
                                                               args = (robot_name, start_a[0], start_a[1], start_a[2]))
                    self.root_action_thread.daemon = True
                    self.root_action_thread.start()
                    start_a = False
                self.lock.release()
                #print packet
        manager.world.lock.acquire()
        robot = manager.world.world.get_robot(robot_name)
        if robot:
            print "Clear robot active"
            robot.is_active = None
            manager.world.lock.release()
            manager.world.on_program_changed()
        else:
            manager.world.lock.release()
        print "Connection exit!"
        self.alive = False

        #self.state = ROBOT_STATE_NONE

    
def manager_active():
    global one_manager, one_manager_lock
    one_manager_lock.acquire()
    a = one_manager
    one_manager_lock.release()
    return a
    
class BlastManager:
    def __init__(self, directories, world):
        global one_manager, one_manager_lock
        one_manager_lock.acquire()
        if one_manager != None:
            one_manager_lock.release()
            raise Exception("There can only be one manager")
        one_manager = self
        one_manager_lock.release()
        
        self.robot_connections = {}
        
        self.server = None
        self.server_thread = None
        self.server = SocketServer.TCPServer(("localhost", 8080), BlastManagedRobot)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.world = blast_planner.BlastPlannableWorld(world)
        self.world.real_world = True
        self.world.action_callback = lambda r, a, p: self.on_action_take(r, a, p)
        self.on_robot_change = lambda robot: None
        self.on_plan_action = lambda: None
        self.on_surface_change = lambda surface: None

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
        global one_manager, one_manager_lock
        self.stop()
        if self.server:
            self.server.shutdown()
        if self.server_thread:
            self.server_thread.join()
        one_manager_lock.acquire()
        one_manager = None
        one_manager_lock.release()
    def stop(self):
        self.is_stopped = True
        for name, world in self.worlds.iteritems():
            world.stop()

    def robot_waiting_for(self, robot, forwhat):
        if forwhat != "LOCATION":
            raise Exception("You need location only forwhat")
    def robot_has(self, robot, forwhat):
        if forwhat != "LOCATION":
            raise Exception("You need location only forwhat")
    
    def take_action(self, robot, action, parameters):
        return self.on_action_take(robot, action, parameters)

    def root_action_thread(self, robot, rt, action, parameters):
        print "Root action!", robot, rt, action, parameters
        def rc():
            self.on_robot_change(robot)
        def sc(sn):
            self.on_surface_change(sn)
        my_guid = self.action_guid
        exe = BlastActionExec(robot, self, my_guid, rt, action, rc, sc)
        self.action_guid = self.action_guid + 1
        exe.run(parameters)
        print "Root action for", robot, "is offline"
        return True

    def on_action_take(self, robot, action, parameters):
        print "Action!", robot, action, parameters
        robot_type = self.world.world.robots[robot].robot_type
        action_robot_type = robot_type
        while action_robot_type:
            if (action_robot_type.name + "." + action) in self.world.world.types.actions:
                break
            action_robot_type = action_robot_type.parent

        print "--- Exec action", robot, action_robot_type.name, "-->", action, parameters
        def rc():
            self.on_robot_change(robot)
        def sc(sn):
            self.on_surface_change(sn)
        my_guid = self.action_guid
        exe = BlastActionExec(robot, self, my_guid, action_robot_type.name, action, rc, sc)
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


if __name__ == '__main__':
    print "This not intended to be a main file."
