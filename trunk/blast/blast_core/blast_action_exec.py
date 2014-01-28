
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


    
class BlastPos():
    def __init__(self, rx = None, ry = None, rz = None, x = None, y = None, z = None, surface = None, jsond = None):
        self.rx, self.ry, self.rz = rx, ry, rz
        self.x, self.y, self.z = x, y, z
        self.surface = surface
        if jsond:
            self.rx, self.ry, self.rz = jsond['rx'], jsond['ry'], jsond['rz']
            self.x, self.y, self.z = jsond['x'], jsond['y'], jsond['z']
            self.surface = jsond.get("surface", self.surface)
        if self.rx: self.rx = float(self.rx)
        if self.ry: self.ry = float(self.ry)
        if self.rz: self.rz = float(self.rz)
        if self.x: self.x = float(self.x)
        if self.y: self.y = float(self.y)
        if self.z: self.z = float(self.z)

    def __str__(self):
        return ",".join([str(x) for x in [self.surface, self.x, self.y, self.z, self.rx, self.ry, self.rz]])

class BlastObject():
    def __init__(self, uid = None, object_type = None, parent = None, pos = None, jsond = None):
        self.position = pos
        self.parent = parent
        self.uid = uid
        self.object_type = object_type
        if jsond:
            self.parent = str(jsond['parent'])
            self.position = BlastPos(jsond = jsond['position'], surface = self.parent)
            self.uid = int(jsond['uid'])
            self.object_type = str(jsond['object_type'])

#TODO: use objects for locations and surfaces.
#{u'state': u'default', u'type': u'transparent_heavy_door', u'name': u'clarkfirstfloordoor', u'locations': {u'in_entrance': {u'y': 26.643, u'x': 21.28, u'a': 0.334, u'map': u'clarkcenterfirstfloordoor'}, u'out_exit': {u'y': 26.481, u'x': 21.221, u'a': -2.855, u'map': u'clarkcenterfirstfloordoor'}, u'out_entrance': {u'y': 18.39, u'x': 20.742, u'a': -2.737, u'map': u'clarkcenterfirstfloor'}, u'in_exit': {u'y': 18.456, u'x': 20.656, u'a': 0.35, u'map': u'clarkcenterfirstfloor'}}}

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
        
    #Takes a name for a surface and gets the resulting JSON data structure.
    #Returns None if the surface does not exist.
    #TODO: describe the content of the structure.
    def get_surface(self, name, world=None):
        if world:
            res = ipc_packet("GET_SURFACE," + str(world) + "," + str(name) + "\n")
        else:
            res = ipc_packet("GET_SURFACE_NW," + str(name) + "\n")
        if res.find("SURFACE") == 0:
            return json.loads(res[len("SURFACE"):])
        return None

            
    
    #Takes a name for a object and gets the resulting BlastObject
    #Returns None if the object does not exist.
    def get_object(self, uid, world=None):
        if world:
            res = ipc_packet("GET_OBJECT," + str(world) + "," + str(uid) + "\n")
        else:
            res = ipc_packet("GET_OBJECT_NW," + str(uid) + "\n")
        if res.find("OBJECT") == 0:
            return BlastObject(jsond = json.loads(res[len("OBJECT"):]))
        return None

    #Gets the UID of the object in a holder of the robot. Takes
    #the name of the holder. Returns none if no object or invalid
    #holder, or the UID as an int.
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
    
    #Set the failure state of the action. Takes the failure mode
    #of the action. Throws a BlastRuntimeError if the failure mode
    #is invalid or another problem occurs.
    def set_failure(self, mode):
        res = ipc_packet("SET_FAILURE," + str(mode) + "\n").strip()
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to set failure mode")
        return res

    #Deletes an object from a surface in the world. Takes the UID of the
    #object as an int or string. Throws a BlastRuntimeError if the object
    #cannot be deleted or does not exist in the first place. To delete
    #objects that are in the robot's holders, use set_robot_holder with None.
    def delete_surface_object(self, obj, world=None):
        if world:
            res = ipc_packet("DELETE_SURFACE_OBJECT," + str(world) + "," + str(obj) + "\n").strip()
        else:
            res = ipc_packet("DELETE_SURFACE_OBJECT_NW," + str(obj) + "\n").strip()
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to delete object")
        return res
    
    #Plans and executes an action. It takes the name of the action as a string,
    #the parameters for the action as a dictionary (of strings, and ints) and an 
    #optional dictionary of contraints called "world_limits". The function will 
    #plan out all the actions to get to the first point where the desired action 
    #can be executed, and the plan can be arbitrarily large. In addition, at this
    #point all world_limits will be satisfied and the action will run. The
    #world_limits include "robot-location" which is a location and "robot-holders"
    #which is a dictionary keyed by holder name of object uids that have to be in
    #the holders. This is important if you want an action to take place with a very
    #specific holder.
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

    #Plans a hunt for an object of a given type. This causes the robot to iterate
    #through various plans attempting to find an object of a given type. Since all
    #objects of a given type are considered equivalent, it will grab the first one
    #it can. The function takes the name of the holder to place the object in and
    #the string name of the object type to find. It returns the uid, position, and
    #prior surface of the object. It raises a BlastRuntimeError if planning fails
    #in a matter that suggests an error with the system, and a BlastFindObjectError
    #if the object cannot actually be found.
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
        strs = [x.strip().replace("Pos", "").strip("()").strip() for x in res.strip().split(",")]
        return int(strs[0]), BlastPos(x=strs[1], y=strs[2], z=strs[3], 
                                      rx=strs[4], ry=strs[5], rz=strs[6], surface=strs[-1]), strs[-1]

    #Add an object of a given type and position to the surface. Takes the 
    #name of the surface, the name of the object_type and the position.
    #Raises a BlastRuntimeError if it fails.
    def surface_add_object(self, surface, object_type, pos, world = None):
        if type(pos) != type(""):
            pos = pos.to_text()
        pos = pos.strip().strip("BlastPos()").strip()
        pos = ",".join([str(float(str(x).strip())) for x in pos.split(",")])
        if world:
            res = ipc_packet("ADD_SURFACE_OBJECT," + str(world) + "," + str(surface) + "," + str(object_type) + "," + pos + "\n").strip()
        else:
            res = ipc_packet("ADD_SURFACE_OBJECT_NW," + str(surface) + "," + str(object_type) + "," + pos + "\n").strip()
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to add surface object")
        return res
        
    #Lists all uids of all the objects on a given surface. Takes the
    #surface and returns the list of UIDs as ints. Raises a
    #BlastRuntimeError if it fails.
    def list_surface_objects(self, surface, world = None):
        if world:
            res = ipc_packet("LIST_SURFACE_OBJECTS," + str(world) + "," + str(surface) + "\n").strip()
        else:
            res = ipc_packet("LIST_SURFACE_OBJECTS_NW," + str(surface) + "\n").strip()
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to list surface objects")
        return [int(x.strip()) for x in res.split(",")]

    #Flags to the system that a surface has been scanned for objects.
    #Takes the name of the surface, and the list of object types as
    #a list of strings or a comma-seperated string. Raises a
    #BlastRuntimeError if it fails for any reason.
    def surface_scan(self, surface, object_types, world=None):
        if type(object_types) != type(""):
            object_types = ",".join([str(x) for x in object_types])
        if world:
            res = ipc_packet("SURFACE_SCAN," + str(world) + "," + str(surface) + "," + str(object_types) + "\n")
        else:
            res = ipc_packet("SURFACE_SCAN_NW," + str(surface) + "," + str(object_types) + "\n")
        if res.strip() == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to set surface scaned")
        return res
        
    #Plans to set down an object on a surface. Takes the UID of the object,
    #the name of the surface, and the position on the surface. Raises
    #a BlastRuntimeError if it fails.
    def plan_place(self, uid, surface, pos, world=None):
        if world:
            res = ipc_packet("PLAN_PLACE," + str(world) + "," + str(uid) + "," + str(surface) + "," + str(pos))
        else:
            res = ipc_packet("PLAN_PLACE_NW," + str(uid) + "," + str(surface) + "," + str(pos))
        if res: res = res.strip().strip("()").strip()
        if res == "None": res = None
        elif res == "True": res = True
        elif res == "False": res = False
        if res == False or res == None:
            raise BlastRuntimeError("Failed to plan")
        return res
        
    #Sets the position of one of the robot's joints. Be careful with this function as
    #it can cause BLAST to declare that your application has failed when it hasn't.
    #Takes the name of the position to set and the list of joint values. Raises a
    #BlastRuntimeError if it fails.
    def set_robot_position(self, pos, val, world=None):
        if world:
            res = ipc_packet("SET_ROBOT_POSITION," + str(world) + "," + str(pos) + "," + self.json(val) + "\n")
        else:
            res = ipc_packet("SET_ROBOT_POSITION_NW," + str(pos) + "," + self.json(val) + "\n")
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to set robot position")
        return res

    #Sets what object is in the holder of the robot. Takes the holder, and the object type.
    #If require_preexisting_object is set to True, then an object must be in the holder.
    #Note that this function replaces the object, changing its UID. If it fails, it raises
    #a BlastRuntimeError. #TODO: make function return object UID.
    def set_robot_holder(self, holder, ot, require_preexisting_object = True, world = None):
        if ot == None: ot = "None()"
        if world:
            res = ipc_packet("SET_ROBOT_HOLDER," + str(world) + "," + str(holder) + "," 
                             + str(ot) + "," + str(require_preexisting_object) + "\n")
        else:
            res = ipc_packet("SET_ROBOT_HOLDER_NW," + str(holder) + "," 
                             + str(ot) + "," + str(require_preexisting_object) + "\n")
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to set robot holder")
        return res

    #Signals Transfer an object from one robot holder to another. Takes the name 
    #of the first holder and the second holder, preserves object UID. Raises a
    #BlastRuntimeError if it fails for any reason, including an empty start.
    def robot_transfer_holder(self, from_holder, to_holder, world = None):
        if world:
            res = ipc_packet("ROBOT_TRANSFER_HOLDER," + str(world) + "," 
                             + str(from_holder) + "," + str(to_holder) + "\n")
        else:
            res = ipc_packet("ROBOT_TRANSFER_HOLDER_NW," 
                             + str(from_holder) + "," + str(to_holder) + "\n")
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to transfer robot holder")
        return res

    #Signals transfer of an object from a surface and places it into the
    #holder. The function takes the uid and the holder name and raises a
    #BlastRuntimeError if it fails for any reason.
    def robot_pick_object(self, objectref, to_holder, world = None):
        if world:
            res = ipc_packet("ROBOT_PICK_OBJECT," + str(world) + "," 
                             + str(objectref) + "," + str(to_holder) + "\n")
        else:
            res = ipc_packet("ROBOT_PICK_OBJECT_NW," 
                             + str(objectref) + "," + str(to_holder) + "\n")
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to pick object")
        return res
        
    #Signals the transfer of an object from the robots holder to the
    #surface. This takes the holder and the position of the object (which
    #includes the surface). Raises a BlastRuntimeError if it fails.
    def robot_place_object(self, holder, position, world = None):
        if type(position) == type([0, 1]) or position == type((1, 2)):
            position = ",".join([str(x) for x in position])
        ps = ",".join([str(x).strip() for x in position.strip().strip('[]()').strip().replace("Pos(", "").replace(")", "").split(",")])

        print "_--" * 90
        print position, holder

        if world:
            res = ipc_packet("ROBOT_PLACE_OBJECT," + str(world) + "," 
                             + str(holder) + "," + str(ps) + "\n")
        else:
            res = ipc_packet("ROBOT_PLACE_OBJECT_NW," 
                             + str(holder) + "," + str(ps) + "\n")
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to place object")
        return res

    #Sets the location of the robot in the world. Takes the position and
    #raises a BlastRuntimeError on failure.
    def set_location(self, position, world = None):
        if world:
            res = ipc_packet("SET_ROBOT_LOCATION," + str(world) + "," 
                             + self.json(position) + "\n")
        else:
            res = ipc_packet("SET_ROBOT_LOCATION_NW," + self.json(position) + "\n")
        if res == "None": res = None
        if res == "True": res = True
        if res == "False": res = False
        if res == None or res == False:
            raise BlastRuntimeError("Failed to set location of object")
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














