#!/usr/bin/python
import math
import hashlib


#############TODO: FIXME: fix .equal()

class BlastAction:
    def __init__(self, name, parameters, condition, time_estimate, changes): #Name must be robot_type.action
        self.name = name
        self.robot = name.split(".")[0]
        self.parameters = parameters
        self.condition = condition
        self.time_estimate = time_estimate
        self.changes = changes
        #FIXME: keywords: robot

def make_test_actions():
    return [BlastAction("pr2.move", {"end": "Pt"},
                        ("==", "robot.location.map", "end.map"),
                        "add(mul(\"3\", dist(robot.location, end)), abs(angle_diff(robot.location.a, end.a)))",
                        {"robot.location": "end" }),
            BlastAction("pr2-cupholder.buy_coffee", {"shop": "Surface:coffee_shop"},
                        ("&&", ("==", "robot.location", "shop.locations.start"),
                         ("contains", "robot.right-arm", "coffee_money_bag")),
                        "\"1000\"",
                        {"robot.location": "shop.locations.end",
                         "robot.holders.cup-holder": "Object(\"coffee_cup\")",
                         "robot.holders.right-arm": "None()"}),
            BlastAction("pr2.door_blast", {"door": "Surface:transparent_heavy_door"},
                        ("&&", ("==", "robot.location", "door.locations.out_entrance")),
                        "\"100\"", {"robot.location": "door.locations.out_exit"}),
            BlastAction("pr2.door_drag", {"door": "Surface:transparent_heavy_door"},
                        ("&&", ("==", "robot.location", "door.locations.in_entrance")),
                        "\"100\"", {"robot.location": "door.locations.in_exit"}),
                        
            BlastAction("pr2.elevator", {"elevator": "Surface:elevator", 
                                         "infloor": "Location:elevator.floor_",
                                         "outfloor": "Location:elevator.floor_"},
                        ("&&", ("==", "robot.location", "infloor"), ("!=", "infloor", "outfloor")),
                        "\"300\"", {"robot.location": "outfloor"}),
            ]


################################################################

class BlastWorldTypes:
    def __init__(self):
        self.surfaces = {}
        self.robots = {}
        self.objects = {}
        self.actions = {}

    def add_surface_type(self, surface):
        self.surfaces[surface.name] = surface
    def get_surface(self, name):
        return self.surfaces.get(name, None)

    def add_robot_type(self, robot):
        self.robots[robot.name] = robot
    def get_robot(self, name):
        return self.robots.get(name, None)

    def add_object_type(self, obj):
        self.objects[obj.name] = obj
    def get_object(self, name):
        return self.objects.get(name, None)
    
    def add_action_type(self, action):
        self.actions[action.name] = action
    def get_action(self, name):
        return self.actions.get(name, None)

    def get_action_for_robot(self, robot_type, action):
        action_robot_type = robot_type
        action_type = None
        while not action_type:
            action_type = self.get_action(action_robot_type.name + "." + action)
            if action_robot_type.parent:
                action_robot_type = action_robot_type.parent
            elif not action_type:
                return None, None
        return action_robot_type, action_type

class SurfaceType:
    def __init__(self, name, states):
        self.name = name
        self.states = states

class RobotType:
    def __init__(self, name, holders, position_variables, parent = None):
        self.name = name
        self.holders = {}
        self.position_variables = {}
        self.parent = parent
        if parent:
            for n, d in parent.holders.iteritems():
                self.holders[n] = d
            for n, d in parent.position_variables.iteritems():
                self.position_variables[n] = d
        for n, d in holders.iteritems():
            self.holders[n] = d
        for n, d in position_variables.iteritems():
            self.position_variables[n] = d

class ObjectType:
    def __init__(self, name, parent = None):
        self.name = name
        self.parent = None

def make_test_types_world():
    types_world = BlastWorldTypes()
    
    types_world.add_object_type(ObjectType("coffee_cup"))
    types_world.add_object_type(ObjectType("coffee_money_bag"))
    
    types_world.add_surface_type(SurfaceType("coffee_shop",
                                             {"default": {"default": True, "accessible": True}}))
    types_world.add_surface_type(SurfaceType("transparent_heavy_door",
                                             {"default": {"default": True, "accessible": True}}))
    types_world.add_surface_type(SurfaceType("elevator",
                                             {"default": {"default": True, "accessible": True}}))
    ATL = 0.0001 #Arm offset tolerance
    types_world.add_robot_type(RobotType("pr2", {"left-arm": {"mass-limit": 2.5}, 
                                                 "right-arm": {"mass-limit": 2.5},
                                                 },
                                         {"torso": {False: (["torso",], [ATL,], [0.0,]), "up": [0.3,], "down": [0.0,], },
                                          "left-arm": {False: (["shoulder_pan", "shoulder_lift", "upper_arm_roll",
                                                                "elbow_flex", "forearm_roll", "r_wrist_flex",
                                                                "wrist_roll", "gripper"],
                                                               [ATL, ATL, ATL, ATL, ATL, ATL, ATL, ATL],
                                                               [0.06024, 1.248526, 1.789070, -1.683386, 
                                                                -1.7343417, -0.0962141, -0.0864407, 0.0]),
                                                       "grip": [False, False, False, False, False, False, False, 0.0],
                                                       "release": [False, False, False, False, False, False, False, 0.1],
                                                       "tucked": [0.06024, 1.248526, 1.789070, -1.683386, 
                                                                  -1.7343417, -0.0962141, -0.0864407, False],
                                                       "untucked": [0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0, False],
                                                       },
                                          "right-arm": {False: (["shoulder_pan", "shoulder_lift", "upper_arm_roll",
                                                                 "elbow_flex", "forearm_roll", "r_wrist_flex",
                                                                 "wrist_roll", "gripper"],
                                                                [ATL, ATL, ATL, ATL, ATL, ATL, ATL, ATL],
                                                                [-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                                  -1.4175, -1.8417, 0.21436, 0.0]),
                                                        "grip": [False, False, False, False, False, False, False, 0.0],
                                                        "release": [False, False, False, False, False, False, False, 0.1],
                                                        "tucked": [-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                                    -1.4175, -1.8417, 0.21436, False],
                                                        "untucked": [-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0, False],
                                                        },
                                          "head": {False: (["pan", "tilt"], [ATL, ATL], [0, 0]), "level": [0.0, 0.0]},
                                          "tilt-laser": False,
                                          }))

    types_world.add_robot_type(RobotType("pr2-cupholder", {"cup-holder": {}}, {}, types_world.get_robot("pr2")))

    [types_world.add_action_type(x) for x in make_test_actions()]
    return types_world

################################################################

class BlastPt:
    def __init__(self, x, y, a, mid):
        self.x = x
        self.y = y
        self.a = a
        self.map = mid

    def copy(self):
        return BlastPt(self.x, self.y, self.a, self.map)

    def hash_update(self, hl):
        hl.update(str(self.x))
        hl.update(str(self.y))
        hl.update(str(self.a))
        hl.update(str(self.map))

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        
        if self.x != other.x: return False
        if self.y != other.y: return False
        if self.a != other.a: return False
        if self.map != other.map: return False
        return True

    def __str__(self):
        return self.to_text()
    def __repr__(self):
        return self.to_text()

    def to_text(self):
        return "Pt(" + str(self.x) + ", " + str(self.y) \
            + ", " + str(self.a) + ", \"" + self.map + "\")"

class BlastPos:
    def __init__(self, x, y, z, rx, ry, rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def copy(self):
        return BlastPos(self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        
        if self.x != other.x: return False
        if self.y != other.y: return False
        if self.z != other.z: return False
        if self.rx != other.rx: return False
        if self.ry != other.ry: return False
        if self.rz != other.rz: return False
        return True
    
    def to_text(self):
        return "Pos(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) \
            + ", " + str(self.rx) + ", " + str(self.ry) + ", " + str(self.rz)

class BlastPosIrr:
    def __init__(self):
        pass
    def copy(self): 
        return BlastPosIrr()
    def to_text(self):
        return "PosIrr()"
    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return True

class BlastObject:
    def __init__(self, object_type, pos, parent): #Note: if pos is None, parent is a robot
        self.object_type = object_type
        self.position = pos
        self.parent = parent

    def copy(self):
        if self.position:
            return BlastObject(self.object_type, self.position.copy(), self.parent)
        return BlastObject(self.object_type, self.position, self.parent)
    
    def hash_update(self, hl):
        hl.update(self.object_type.name)
        if self.position: self.position.hash_update(hl)
        else: hl.update("None")
        if self.parent: hl.update(self.parent)

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        
        if self.object_type.name != other.object_type.name: return False
        if self.parent != other.parent: return False
        if self.position == other.position: return True
        if not self.position: return False
        return self.position.equal(other.position)

    def to_text(self):
        pt = "None"
        if self.position: pt = self.position.to_text()
        return "Object(\"" + self.object_type.name + "\", " + pt + ", \"" + self.parent + "\")"

class BlastSurface:
    def __init__(self, name, locations, surface_type, state = None):
        self.name = name
        self.locations = locations
        self.surface_type = surface_type
        self.state = state
        if not self.state:
            self.state = ""
            for state, data in surface_type.states.iteritems():
                if data.get("default", False):
                    self.state = state

    def __str__(self):
        return "Surface:" + self.name
    def __repr__(self):
        return "Surface:" + self.name
    
    def copy(self):
        lclone = {}
        for name, loc in self.locations.iteritems():
            lclone[name] = loc.copy()
        return BlastSurface(self.name, lclone, self.surface_type, self.state)

    def hash_update(self, hl):
        for name, loc in sorted(self.locations.iteritems(), key=lambda x: x[0]):
            hl.update(name)
            loc.hash_update(hl)
        hl.update(self.state)
        hl.update(self.surface_type.name)

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        
        if self.name != other.name: return False
        if self.state != other.state: return False
        if self.surface_type.name != other.surface_type.name: return False
        for name in self.locations: 
            if not name in other.locations: False
        for name in other.locations: 
            if not name in self.locations: False
            if not other.locations[name].equal(self.locations[name]): return False
        return True

    def to_text(self, loc):
        return "\t\tSurface(\"" + self.name + "\", \"" + loc + "\", " \
            + self.locations[loc].to_text() + ", \"" + self.surface_type.name \
            + "\", \"" + self.state + "\")\n"

class BlastPrimitive:
    def __init__(self, name):
        self.name = name
    def to_text():
        return self.name
    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return other.name == self.name
BLAST_FALSE = BlastPrimitive("False")
BLAST_TRUE = BlastPrimitive("True")
BLAST_NONE = BlastPrimitive("None")

class BlastMap:
    def __init__(self, mid, map_file):
        self.map = mid
        self.map_file = map_file

    def hash_update(self, hl):
        hl.update(self.map_file)

    def copy(self):
        return BlastMap(self.map, self.map_file)

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return other.map == self.map and self.map_file == other.map_file

    def to_text(self):
        r = "\tMap(\"" + str(self.map) + "\", \"" \
            + self.map_file + "\"):\n"
        return r


class BlastRobot:
    def __init__(self, name, location, robot_type):
        self.name = name
        self.robot_type = robot_type
        self.location = location
        self.holders = {}
        self.positions = {}
        for name in self.robot_type.holders:
            self.holders[name] = None
        for name, d in self.robot_type.position_variables.iteritems():
            if d:
                self.positions[name] = {}
                for i in xrange(0, len(d[False][0])): #Loop through names, etc
                    self.positions[name][d[False][0][i]] = d[False][2][i] #Set to default
            else:
                self.positions[name] = False

    def copy(self):
        copy = BlastRobot(self.name, self.location.copy(), self.robot_type)
        copy.holders = {}
        for h in self.holders:
            if self.holders[h]:
                copy.holders[h] = self.holders[h].copy()
            else:
                copy.holders[h] = None
        copy.positions = {}
        for name in self.positions:
            if self.positions[name]:
                copy.positions[name] = {}
                for joint in self.positions[name]:
                    copy.positions[name][joint] = self.positions[name][joint]
            else:
                copy.positions[name] = False
        return copy

    def hash_update(self, hl):
        hl.update(self.name)
        hl.update(self.robot_type.name)
        self.location.hash_update(hl)
        for name, value in sorted(self.holders.iteritems(), key=lambda x: x[0]):
            if value == None:
                hl.update("None")
            else:
                value.hash_update(hl)
        for name, value in sorted(self.positions.iteritems(), key=lambda x: x[0]):
            if value == False:
                hl.update("False")
            else:
                for j, sub in sorted(value.iteritems(), key=lambda x: x[0]):
                    hl.update(str(sub))

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        
        if self.robot_type.name != other.robot_type.name: return False
        if not self.location.equal(other.location): return False
        for name in other.holders:
            if not name in self.holders: return False
        for name in self.holders:
            if self.holders[name]:
                if not self.holders[name].equal(other.holders[name]): return False
            elif other.holders[name]:
                return False
        for name in other.positions:
            if not name in self.positions: return False
        for name in self.positions:
            if self.positions[name]:
                if not other.positions[name]: return False
                for joint in self.positions[name]:
                    if joint not in other.positions[name]: return False
                    if self.positions[name][joint] != other.positions[name][joint]: return False
            elif other.positions[name]:
                return False
        return True

    def to_text(self):
        r = ""
        for name in sorted(self.positions.keys()):
            if self.positions[name]:
                r = r + "\t\tRobotState(\"" + self.name + "\", \"" + name + "\", "
                r = r + ", ".join([str(x) for n, x in sorted(self.positions[name].iteritems(), key=lambda x: x[0])]) + ")\n"
        for name in sorted(self.holders.keys()):
            if self.holders[name]:
                r = r + "\t\tRobotHolder(\"" + self.name + "\", \"" + name + "\", "
                r = r + self.holders[name].to_text() + ")\n"
        return "\t\tRobot(\"" + self.name + "\", " + self.location.to_text() \
            + ", \"" + self.robot_type.name + "\")\n" + r

def paren_split(value, delim):
    subs = []
    cur = ""
    in_paren = 0
    in_app = False
    for c in value:
        if in_app:
            cur = cur + c
            if c == '"': in_app = False
        else:
            if c == '"': in_app = True
            if c == '(': in_paren = in_paren + 1
            if c == ')': in_paren = in_paren - 1
            if c == delim and in_paren == 0:
                subs.append(cur.strip())
                cur = ""
            else:
                cur = cur + c
    if cur.strip() != "":
        subs.append(cur.strip())
    return subs

class BlastWorld:
    def __init__(self, types):
        self.types = types
        self.maps = {}
        self.surfaces = {}
        self.robots = {}
        self.hash_state = None
    def copy(self):
        copy = BlastWorld(self.types)
        for arr in ["maps", "surfaces", "robots"]:
            copy_arr = getattr(copy, arr)
            for name, item in getattr(self, arr).iteritems():
                copy_arr[name] = item.copy()
        copy.hash_state = None
        return copy
    def append_map(self, mp):
        self.hash_state = None
        self.maps[mp.map] = mp
    def append_surface(self, surface):
        self.hash_state = None
        self.surfaces[surface.name] = surface
    def append_robot(self, robot):
        self.hash_state = None
        self.robots[robot.name] = robot

    def to_text(self):
        r = "World(" + ''.join('%02x' % ord(byte) for byte in self.get_hash_state()) + "):\n"
        for mid, mp in sorted(self.maps.iteritems(), key=lambda x: x[0]):
            r = r + mp.to_text()
            for name, sur in sorted(self.surfaces.iteritems(), key=lambda x: x[0]):
                for loc in sorted(sur.locations):
                    if sur.locations[loc].map == mp.map:
                        r = r + sur.to_text(loc)
            for name, robot in sorted(self.robots.iteritems(), key=lambda x: x[0]):
                if robot.location.map == mp.map:
                    r = r + robot.to_text()
        return r

    def get_hash_state(self):
        if self.hash_state:
            return self.hash_state

        hl = hashlib.sha224()
        for arr in [self.maps, self.robots, self.surfaces]:
            for name, value in sorted(arr.iteritems(), key=lambda x: x[0]):
                hl.update(name)
                value.hash_update(hl)
        self.hash_state = hl.digest()
        
        return self.hash_state

    def equal(self, other):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False


        warn = True
        hash_same = self.get_hash_state() == other.get_hash_state()

        #Comment out these two lines to verify hashing
        if self.get_hash_state() != other.get_hash_state(): return False
        return True #Comment this line to ensure, 100%, no hash collisions
        #This is to double-check to make sure hashing went as expected.
        
        for arr, oarr in [(self.robots, other.robots), (self.surfaces, other.surfaces), (self.maps, other.maps)]:
            for name, value in oarr.iteritems():
                if not name in arr:
                    if warn and hash_same: print "Hash state collision"
                    return False
            for name, value in arr.iteritems():
                if not name in oarr:
                    if warn and hash_same: print "Hash state collision"
                    return False
                if not value.equal(oarr[name]): 
                    if warn and hash_same: print "Hash state collision"
                    return False

        if not hash_same and warn: 
            print "Hashes were different, but states were same"
            print "-"*60, "self", "-"*60
            print self.to_text()
            print "-"*60, "other", "-"*60
            print other.to_text()
            print "-"*60, "delta", "-"*60
            for line_self, line_other in zip(self.to_text().split("\n"), other.to_text().split("\n")):
                if line_self.strip() != line_other.strip():
                    print "self:", line_self.strip()
                    print "other:", line_other.strip()
            print "-"*60, "done", "-"*60
        return True


    def robot_contains_condition(self, robot, condition):
        obj = None
        if condition[1].split(".")[0] == "robot":
            if not condition[1].split(".")[1] in robot.holders:
                if debug: print condition[1], "has invalid holder for robot"
                return False
            obj = robot.holders[condition[1].split(".")[1]]
        else:
            if debug: print "Invalid value for holder"
            return False
        if condition[2] == "":
            return obj == None
        if obj == None: return False #Obviously needs an object yet none
        if condition[2] == obj.object_type.name: return True
        if condition[0] == "exact-contains": return False
        t = obj.object_type
        while t.parent:
            if t.name == condition[2]: return True
            t = t.parent
        return False
    
    def take_action(self, robot_name, action, parameters, debug = False):
        #Get the robot
        robot = self.robots.get(robot_name)
        if not robot:
            if debug: print "Invalid robot:", robot_name
            return None
        
        #Get action type
        action_robot_type, action_type = self.types.get_action_for_robot(robot.robot_type, action)
        if not action_robot_type or not action_type:
            if debug:
                print "Could not find the action type", action, "for", robot.robot_type
            return None

        #Ensure that we have all the proper parameters
        for name in action_type.parameters:
            if not name in parameters:
                if debug: print "Parameter", parameter, "unspecified"
                return None


        #Check all the conditions
        def get_value(value):
            if type(value) != type(""):
                return assert_condition(value)
            if value.strip()[0] == '"':
                return value.strip()[1:-1]

            subs = paren_split(value, '.')


            val = None
            if subs[0].find('(') != -1:
                func = subs[0][0:subs[0].find("(")].strip()
                args = paren_split(subs[0][subs[0].find("(")+1:subs[0].rfind(")")], ',')
                vals = [get_value(x) for x in args]
                if func == "dist":
                    #FIXME: type check, map same check
                    dx = (vals[0].x - vals[1].x)
                    dy = (vals[0].y - vals[1].y)
                    val = math.sqrt(dx * dx + dy * dy)
                elif func == "mul":
                    val = float(vals[0])
                    for m in vals[1:]: val = val * float(m)
                elif func == "sub":
                    val = float(vals[0])
                    for m in vals[1:]: val = val - float(m)
                elif func == "add":
                    val = float(vals[0])
                    for m in vals[1:]: val = val + float(m)
                elif func == "div":
                    val = float(vals[0])
                    for m in vals[1:]: val = val / float(m)
                elif func == "abs":
                    val = float(vals[0])
                    if (val < 0): val = -val
                elif func == "angle_diff":
                    val = float(vals[0])
                    for m in vals[1:]: val = val - float(m)
                    while val >= +math.pi: val = val - math.pi*2
                    while val <= -math.pi: val = val + math.pi*2
                elif func == "Object":
                    pos = None
                    par = None
                    if len(vals) > 1: pos = vals[1]
                    if len(vals) > 2: par = vals[2]
                    val = BlastObject(self.types.get_object(vals[0]), pos, par)
                elif func == "None": 
                    val = BLAST_NONE
                elif func == "False": 
                    val = BLAST_FALSE
                elif func == "True": 
                    val = BLAST_TRUE
                else:
                    if debug: print "Invalid function:", func, "in", value
                    return None
            elif subs[0] == "robot":
                val = robot
            elif subs[0] in parameters:
                val = parameters[subs[0]]
            else:
                if debug: print "Invalid 0-th value:", value
                return None
            subs = subs[1:]
            while subs != []:
                if (type(val) == type({})):
                    if not subs[0] in val:
                        if debug: print "Value not in dictionary", subs[0], val
                        return None
                    val = val[subs[0]]
                else:
                    if not hasattr(val, subs[0]):
                        if debug: print "Value not in attributes", subs[0], val
                        return None
                    val = getattr(val, subs[0])
                subs = subs[1:]
            return val
        def assert_condition(condition):
            if (condition[0] == "==" or condition[0] == "!="):
                #if debug: print condition[1], condition[2]
                a = get_value(condition[1])
                b = get_value(condition[2])
                #if debug: print condition[0], a, b
                if condition[0] == "==": 
                    if hasattr(a, "equal"): return a.equal(b)
                    if hasattr(b, "equal"): return b.equal(a)
                    return a == b
                if condition[0] == "!=": 
                    if hasattr(a, "equal"): return not a.equal(b)
                    if hasattr(b, "equal"): return not b.equal(a)
                    return not a == b
                return False
            elif (condition[0] == "&&" or condition[0] == "||"):
                for i in xrange(1, len(condition)):
                    v = get_value(condition[i])
                    #if debug: print condition[i], v
                    if condition[0] == "&&" and not v: return False
                    if condition[0] == "||" and v: return True
                return condition[0] == "&&"
            elif (condition[0] == "contains" or condition[0] == "exact-contains"):
                return self.robot_contains_condition(robot, condition)
            else:
                print "Invalid condition", condition
            return False
        
        if not assert_condition(action_type.condition):
            if debug: print "Condition returned false"
            return None
    
        time_estimate = get_value(action_type.time_estimate)
        if debug: print "Time estimate:", time_estimate
        if time_estimate == None:
            if debug: print "Time estimate invalid"
            return None

        #Conditions met, everything is done. Now we just need to update the values
        #create a queue of transations then execute them
        transactions = []
        for var, val in action_type.changes.iteritems():
            sub = var.split(".")
            if debug: print var, "->", val
            if sub[0] == "robot":
                if not hasattr(robot, sub[1]):
                    if debug: print "Attempt to set invalid robot attribute", sub[1]
                    return None
                v = get_value(val)
                if v == None:
                    if debug: print "Invalid value for set:", val
                    return None
                if sub[1] == "holders":
                    if BLAST_NONE.equal(v):
                        v = None
                    else:
                        v.position = None
                        v.parent = robot.name + "." + sub[2]
                    transactions.append(("VAL", robot.holders, sub[2], v))
                else:
                    transactions.append(("SET", robot, sub[1], v))
            else:
                if debug: print "Update failed"
                return None

        #Execute the transactions - this is where changes actually get made to the world
        self.hash_state = None
        for x in transactions:
            if x[0] == "SET":
                setattr(x[1], x[2], x[3])
            elif x[0] == "VAL":
                x[1][x[2]] = x[3]
            else:
                print "Really bad, transaction bad:", x
                raise Exception("Bad transaction: " + str(x))

        return time_estimate

    def enumerate_action(self, robot_name, action):
        #Get the robot
        robot = self.robots.get(robot_name)
        if not robot:
            if debug: print "Invalid robot:", robot_name
            return None
        
        #Get action type
        action_robot_type, action_type = self.types.get_action_for_robot(robot.robot_type, action)
        if not action_robot_type or not action_type:
            if debug:
                print "Could not find the action type", action, "for", robot.robot_type
            return None

        #Try to optimize conditions
        req_conditions = [action_type.condition]
        conditions = [] #Conditions that absolutely must be true
        while req_conditions:
            condition = req_conditions[0]
            req_conditions = req_conditions[1:]
            if condition[0] in ['==', '!=', 'contains', 'exact-contains']:
                conditions.append(condition)
            if condition[0] == '&&':
                for i in condition[1:]:
                    req_conditions.append(i)

        #print action_type.condition, "->", conditions
        total_action_fail = False
        for condition in conditions:
            if condition[0] == "contains" or condition[0] == "exact-contains":
                if not self.robot_contains_condition(robot, condition):
                    total_action_fail = True
                    break
        if total_action_fail:
            fail_res = {}
            for key in action_type.parameters.iterkeys():
                fail_res[key] = []
            return fail_res


        def test_loc(loc, param):
            for x in conditions:
                if param in x:
                    if "robot.location" in x and x[0] == "==":
                        if not robot.location.equal(loc): return False
                if param + ".map" in x:
                    if "robot.location.map" in x and x[0] == "==":
                        if robot.location.map != loc.map: return False
            return True

        #Get all the parameters
        parameters = {}
        location_parameters = {}
        for param, ptype in action_type.parameters.iteritems():
            parameters[param] = []
            if ptype == "Pt":
                #All the robot locations
                for name, rob in self.robots.iteritems():
                    if test_loc(rob.location, param):
                        parameters[param].append(rob.location)
                #Get all the surface points
                for name, surface in self.surfaces.iteritems():
                    for loc in surface.locations.values():
                        if test_loc(loc, param):
                            parameters[param].append(loc)
            elif ptype[0:len("Surface:")] == "Surface:":
                stype = ptype.split(":")[1]
                for name, surface in self.surfaces.iteritems():
                    if surface.surface_type.name == stype:
                        #Optimize
                        good = True
                        for x in conditions:
                            if x[0] == "==" and "robot.location" in x:
                                for v in x[1:]:
                                    if v.find(param + ".locations.") == 0:
                                        if not robot.location.equal(surface.locations.get(v.split(".")[2], None)):
                                            good = False
                                            break
                        if good:
                            parameters[param].append(surface)
            elif ptype[0:len("Location:")] == "Location:":
                location_parameters[param] = ptype.split(":")[1].split(".")
            else:
                print "Error, invalid parameter type:", ptype, "for:", param
        #Handle after all others parameterized
        for param, data in location_parameters.iteritems():
            parameters[param] = {}
            for surf in parameters[data[0]]:
                parameters[param][surf] = []
                for loc, pt in surf.locations.iteritems():
                    if loc.find(data[1]) == 0:
                        if test_loc(pt, param):
                            parameters[param][surf].append(pt)
            parameters[param] = ([data[0]], parameters[param])

        return parameters


def make_test_world():
    world = BlastWorld(make_test_types_world())

    clarkcenterfirstfloordoor = BlastMap("clarkcenterfirstfloordoor", "maps/clarkcenterfirstfloordoor.pgm")
    world.append_map(clarkcenterfirstfloordoor)
    clarkcenterfirstflooroutside = BlastMap("clarkcenterfirstflooroutside", "maps/clarkcenterfirstflooroutside.pgm")
    world.append_map(clarkcenterfirstflooroutside)
    clarkcenterfirstfloor = BlastMap("clarkcenterfirstfloor", "maps/clarkcenterfirstfloor.pgm")
    world.append_map(clarkcenterfirstfloor)
    clarkcenterbasementelevator = BlastMap("clarkcenterbasementelevator", "maps/clarkcenterbasementelevator.pgm")
    world.append_map(clarkcenterbasementelevator)
    clarkcenterfirstfloorelevator = BlastMap("clarkcenterfirstfloorelevator", "maps/clarkcenterfirstfloorelevator.pgm")
    world.append_map(clarkcenterfirstfloorelevator)
    clarkcentersecondfloorelevator = BlastMap("clarkcentersecondfloorelevator", "maps/clarkcentersecondfloorelevator.pgm")
    world.append_map(clarkcentersecondfloorelevator)
    clarkcenterthirdfloorelevator = BlastMap("clarkcenterthirdfloorelevator", "maps/clarkcenterthirdfloorelevator.pgm")
    world.append_map(clarkcenterthirdfloorelevator)
    clarkcenterthirdflooroutside = BlastMap("clarkcenterthirdflooroutside", "maps/clarkcenterthirdflooroutside.pgm")
    world.append_map(clarkcenterthirdflooroutside)
    clarkcenterpeetscoffee = BlastMap("clarkcenterthirdflooroutside", "maps/clarkcenterthirdflooroutside.pgm")
    world.append_map(clarkcenterpeetscoffee)
                                 

    world.append_surface(BlastSurface("clarkfirstfloordoor", 
                                      {"in_entrance": BlastPt(21.280, 26.643, 0.334, clarkcenterfirstfloordoor.map),
                                       "in_exit": BlastPt(20.656, 18.456, 0.350, clarkcenterfirstfloor.map),
                                       "out_entrance": BlastPt(20.742, 18.390, -2.737, clarkcenterfirstfloor.map),
                                       "out_exit": BlastPt(21.221, 26.481, -2.855, clarkcenterfirstfloordoor.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkfirstflooroutsidedoor", 
                                      {"in_entrance": BlastPt(45.255, 49.637, 1.494, clarkcenterfirstflooroutside.map),
                                       "in_exit": BlastPt(21.280, 26.643, 0.334, clarkcenterfirstfloordoor.map),
                                       "out_entrance": BlastPt(21.221, 26.481, -2.855, clarkcenterfirstfloordoor.map),
                                       "out_exit": BlastPt(45.427, 49.668, -1.735, clarkcenterfirstflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkfirstfloorelevatordoor", 
                                      {"in_entrance": BlastPt(95.682, 62.549, 2.640, clarkcenterfirstflooroutside.map),
                                       "in_exit": BlastPt(49.733, 13.540, 2.619, clarkcenterfirstfloorelevator.map),
                                       "out_entrance": BlastPt(49.654, 13.662, -0.501, clarkcenterfirstfloorelevator.map),
                                       "out_exit": BlastPt(95.647, 62.677, -0.709, clarkcenterfirstflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkthirdfloorelevatordoor", 
                                      {"in_entrance": BlastPt(53.317, 26.607, -3.016, clarkcenterthirdflooroutside.map),
                                       "in_exit": BlastPt(50.452, 12.200, 3.014, clarkcenterthirdfloorelevator.map),
                                       "out_entrance": BlastPt(50.452, 12.200, -0.125, clarkcenterthirdfloorelevator.map),
                                       "out_exit": BlastPt(53.561, 26.425, 0.101, clarkcenterthirdflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkthirdpeetscoffeedoor", 
                                      {"in_entrance": BlastPt(64.375, 33.672, 1.275, clarkcenterthirdflooroutside.map),
                                       "in_exit": BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                                       "out_entrance": BlastPt(56.659, 14.061, 3.100, clarkcenterpeetscoffee.map),
                                       "out_exit": BlastPt(64.258, 33.537, -1.990, clarkcenterthirdflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkelevator", 
                                      {"floor_1": BlastPt(49.218, 19.462, -3.126, clarkcenterfirstfloorelevator.map),
                                       "floor_2": BlastPt(48.569, 17.927, -2.926, clarkcentersecondfloorelevator.map),
                                       "floor_3": BlastPt(48.569, 17.927, -2.926, clarkcenterthirdfloorelevator.map),
                                       #"floor_B": BlastPt(48.569, 17.927, -2.926, clarkcenterbasementelevator.map),
                                       },
                                      world.types.get_surface("elevator")))
    coffee_pickup = BlastSurface("clark_peets_coffee_shop", 
                                 {"start": BlastPt(56.869, 14.529, 2.511, clarkcenterpeetscoffee.map),
                                  "line_0": BlastPt(58.834, 13.630, -0.697, clarkcenterpeetscoffee.map),
                                  "line_1": BlastPt(58.782, 13.704, 2.412, clarkcenterpeetscoffee.map),
                                  "line_2": BlastPt(57.840, 14.623, 2.497, clarkcenterpeetscoffee.map),
                                  "line_3": BlastPt(57.667, 15.643, 1.731, clarkcenterpeetscoffee.map),
                                  "line_4": BlastPt(57.476, 16.429, 1.731, clarkcenterpeetscoffee.map),
                                  "line_5": BlastPt(58.521, 17.789, 0.816, clarkcenterpeetscoffee.map),
                                  "line_6": BlastPt(58.521, 17.789, -0.687, clarkcenterpeetscoffee.map),
                                  "end": BlastPt(58.521, 17.789, -0.687, clarkcenterpeetscoffee.map),
                                  },
                                 world.types.get_surface("coffee_shop"))
    world.append_surface(coffee_pickup)

    stair4 = BlastRobot("stair4", #BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                        BlastPt(12.000, 40.957, 0.148, clarkcenterfirstfloor.map),
                        world.types.get_robot("pr2-cupholder"))
    stair4.holders["right-arm"] = BlastObject(world.types.get_object("coffee_money_bag"), None, "stair4.right-arm")
    world.append_robot(stair4)

    return world

def run_test():
    world = make_test_world()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)
    
    res = world.take_action("stair4", "move", {"end": BlastPt(56.869, 14.529, 2.511, "1337")})
    print "Action result:", res
    
    print '-'*130
    print "                                                            MID-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)
    
    
    res = world.take_action("stair4", "buy_coffee", {"shop": world.surfaces["clark_peets_coffee_shop"] })
    print "Action result:", res
    
    
    print '-'*130
    print "                                                            POST-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)

    clone = world.copy()
    print '-'*130
    print "                                                            CLONE"
    print '-'*130
    print clone.to_text()
    print '-'*130
    print "Equal", clone.equal(world)
    
    print "Move:", world.enumerate_action("stair4", "move")
    print "Coffee:", world.enumerate_action("stair4", "buy_coffee")


def elevator_test():
    world = make_test_world()
    world.robots["stair4"].location = world.surfaces["clarkelevator"].locations["floor_1"].copy()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130

    print "Elevator:", world.enumerate_action("stair4", "elevator")

if __name__ == '__main__':
    #run_test()
    elevator_test()
