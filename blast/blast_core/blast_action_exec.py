
import sys, json, traceback

class BlastRuntimeError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class BlastFindObjectError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

#This is important so that no commands get sent
#over stdout, avoiding problems with mixing log
#data into IPC data. All log data goes into 
#stderr along with any generated error messages.
ipc = sys.stdout
ipc_ind = sys.stdin
sys.stdout = sys.stderr

def ipc_write_packet(packet):
    ipc.write(packet + "\n")
    ipc.flush()

def ipc_packet(packet):
    ipc.write(packet + "\n")
    ipc.flush()
    #print "TO IPC", packet
    r = ipc_ind.readline()
    #print "FROM IPC", r
    return r


class BlastActionExec():
    def __init__(self):
        pass

    def json(self, d):
        return json.dumps(d)
        
    def get_surface(self, name, world=None):
        if world:
            res = ipc_packet("GET_SURFACE," + str(world) + "," + str(name) + "\n")
        else:
            res = ipc_packet("GET_SURFACE_NW," + str(name) + "\n")
        if res.find("SURFACE") == 0:
            return json.loads(res[len("SURFACE"):])
        return None
    
    def get_object(self, name, world=None):
        if world:
            res = ipc_packet("GET_OBJECT," + str(world) + "," + str(name) + "\n")
        else:
            res = ipc_packet("GET_OBJECT_NW," + str(name) + "\n")
        if res.find("OBJECT") == 0:
            return json.loads(res[len("OBJECT"):])
        return None

    def get_robot_holder(self, holder, world=None):
        if world:
            res = ipc_packet("GET_ROBOT_HOLDER," + str(world) + "," + str(holder) + "\n").strip()
        else:
            res = ipc_packet("GET_ROBOT_HOLDER_NW," + str(holder) + "\n").strip()
        if res == "None":
            res = None
        else:
            try:
                res = int(res)
            except:
                pass
        return res
        
    def set_failure(self, mode):
        res = ipc_packet("SET_FAILURE," + str(mode) + "\n").strip()
        if res == "True":
            res = True
        else:
            res = None
        return res

    def delete_surface_object(self, obj, world=None):
        if world:
            res = ipc_packet("DELETE_SURFACE_OBJECT," + str(world) + "," + str(obj) + "\n").strip()
        else:
            res = ipc_packet("DELETE_SURFACE_OBJECT_NW," + str(obj) + "\n").strip()
        if res == "True":
            res = True
        else:
            res = None
        return res
    
    def plan_action(self, action, parameters, world_limits = None, world=None):
        if world_limits != None:
            parameters = {"parameter values": parameters, "world limits": world_limits}
        if world:
            res = ipc_packet("PLAN_ACTION," + str(world) + "," + str(action) + "," + self.json(parameters) + "\n")
        else:
            res = ipc_packet("PLAN_ACTION_NW," + str(action) + "," + self.json(parameters) + "\n")
        if type(res) == type(""):
            if res.strip() == "None": res = None
        if res == None:
            raise BlastRuntimeError("Failed to plan")
        return res

    
    def plan_hunt(self, holder, object_type, world=None):
        if world:
            res = ipc_packet("PLAN_HUNT," + str(world) + "," + str(holder) + "," + str(object_type) + "\n")
        else:
            res = ipc_packet("PLAN_HUNT_NW," + str(holder) + "," + str(object_type) + "\n")
        if res.strip() == "None": res = None
        elif res.strip() == "False": res = False
        if res == None:
            raise BlastRuntimeError("Failed to plan")
        if res == False:
            raise BlastFindObjectError("Failed to find " + object_type + " and place it in " + holder)
        return res.strip().split(",")[:-1], res.strip().split(",")[-1]

    def surface_add_object(self, surface, object_type, pos, world = None):
        if type(pos) != type(""):
            pos = pos.to_text()
        pos = pos.strip().strip("BlastPos()").strip()
        pos = ",".join([str(float(str(x).strip())) for x in pos.split(",")])
        if world:
            res = ipc_packet("ADD_SURFACE_OBJECT," + str(world) + "," + str(surface) + "," + str(object_type) + "," + pos + "\n").strip()
        else:
            res = ipc_packet("ADD_SURFACE_OBJECT_NW," + str(surface) + "," + str(object_type) + "," + pos + "\n").strip()
        if res.strip() == "None": res = None
        return res
        

    def list_surface_objects(self, surface, world = None):
        if world:
            res = ipc_packet("LIST_SURFACE_OBJECTS," + str(world) + "," + str(surface) + "\n").strip()
        else:
            res = ipc_packet("LIST_SURFACE_OBJECTS_NW," + str(surface) + "\n").strip()
        if res.strip() == "None": return None
        return res.split(",")

    def surface_scan(self, surface, object_types, world=None):
        if type(object_types) != type(""):
            object_types = ",".join([str(x) for x in object_types])
        if world:
            res = ipc_packet("SURFACE_SCAN," + str(world) + "," + str(surface) + "," + str(object_types) + "\n")
        else:
            res = ipc_packet("SURFACE_SCAN_NW," + str(surface) + "," + str(object_types) + "\n")
        if res.strip() == "None": res = None
        return res
        

    def set_robot_position(self, pos, val, world=None):
        if world:
            res = ipc_packet("SET_ROBOT_POSITION," + str(world) + "," + str(pos) + "," + self.json(val) + "\n")
        else:
            res = ipc_packet("SET_ROBOT_POSITION_NW," + str(pos) + "," + self.json(val) + "\n")
        if res == "None": res = None
        return res

    def set_robot_holder(self, holder, ot, require_preexisting_object = True, world = None):
        if ot == None: ot = "None()"
        if world:
            res = ipc_packet("SET_ROBOT_HOLDER," + str(world) + "," + str(holder) + "," 
                             + str(ot) + "," + str(require_preexisting_object) + "\n")
        else:
            res = ipc_packet("SET_ROBOT_HOLDER_NW," + str(holder) + "," 
                             + str(ot) + "," + str(require_preexisting_object) + "\n")
        if res == "None": res = None
        return res

    def robot_transfer_holder(self, from_holder, to_holder, world = None):
        if world:
            res = ipc_packet("ROBOT_TRANSFER_HOLDER," + str(world) + "," 
                             + str(from_holder) + "," + str(to_holder) + "\n")
        else:
            res = ipc_packet("ROBOT_TRANSFER_HOLDER_NW," 
                             + str(from_holder) + "," + str(to_holder) + "\n")
        if res == "None": res = None
        return res

    def robot_pick_object(self, objectref, to_holder, world = None):
        if world:
            res = ipc_packet("ROBOT_PICK_OBJECT," + str(world) + "," 
                             + str(objectref) + "," + str(to_holder) + "\n")
        else:
            res = ipc_packet("ROBOT_PICK_OBJECT_NW," 
                             + str(objectref) + "," + str(to_holder) + "\n")
        if res == "None": res = None
        return res
        
    def robot_place_object(self, holder, position, world = None):
        ps = ",".join([x.strip() for x in position.replace("Pos(", "").replace(")", "").split(",")])

        if world:
            res = ipc_packet("ROBOT_PLACE_OBJECT," + str(world) + "," 
                             + str(holder) + "," + str(ps) + "\n")
        else:
            res = ipc_packet("ROBOT_PLACE_OBJECT_NW," 
                             + str(holder) + "," + str(ps) + "\n")
        if res == "None": res = None
        return res

    def set_location(self, position, world = None):
        if world:
            res = ipc_packet("SET_ROBOT_LOCATION," + str(world) + "," 
                             + self.json(position) + "\n")
        else:
            res = ipc_packet("SET_ROBOT_LOCATION_NW," + self.json(position) + "\n")
        if res == "None": res = None
        return res

    def run(self):
        raise Exception("Need to override run method")

def set_action_exec(ex):
    ex().run(parameters) #Nones for legacy

try:
    parameters = json.loads(sys.argv[2])
    execfile(sys.argv[1])
    ipc_write_packet("TERMINATE\n")
except:
    print "Exception in action:", sys.argv
    print '-'*60
    traceback.print_exc(file=sys.stdout)
    print '-'*60
    try:
        ipc_write_packet("ERROR\n")
    except:
        print "Failed to write error"














