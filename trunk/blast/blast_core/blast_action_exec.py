
import sys, json, traceback

class BlastRuntimeError(Exception):
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
    
    def plan_action(self, action, parameters, world=None):
        if world:
            res = ipc_packet("PLAN_ACTION," + str(world) + "," + str(action) + "," + self.json(parameters) + "\n")
        else:
            res = ipc_packet("PLAN_ACTION_NW," + str(action) + "," + self.json(parameters) + "\n")
        if res.strip() == "None": res = None
        if res == None:
            raise BlastRuntimeError("Failed to plan")
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
    ipc_write_packet("ERROR\n")














