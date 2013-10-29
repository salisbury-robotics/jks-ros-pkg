#!/usr/bin/python
import math
import hashlib

#Permissions system

class BlastTypeError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class BlastError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class BlastAction:
    def __init__(self, name, parameters, condition, time_estimate, changes, planable = True): #Name must be robot_type.action
        self.name = name
        self.robot = name.split(".")[0]
        self.parameters = parameters
        self.condition = condition

        if "robot" in self.parameters:
            raise BlastTypeError("A parameter cannot be called 'robot' in " + name)
        for pname, ptype in self.parameters.iteritems():
            if ptype == "Pt":
                pass
            elif ptype.find("Surface:") == 0:
                pass
            elif ptype.find("Joint:") == 0:
                pass
            elif ptype.find("Location:") == 0:
                vname = ptype[ptype.find(":")+1:].split(".")[0]
                if not vname in self.parameters:
                    raise BlastTypeError("Location parameter does not reference another parameter: " + pname + " -> " + vname)
                if self.parameters[vname].find("Surface:") != 0:
                    raise BlastTypeError("Location parameter " + pname + " does not reference a surface type variable (" + vname + ")")
            elif ptype == "String":
                if planable:
                    raise BlastTypeError("Cannot plan with arbitary string type. Either declare action unplanable or add enumeration")
            else:
                raise BlastTypeError("Invalid parameter type for " + pname + ": " + ptype)

        self.validate("Invalid condition for " + name, condition)
        self.validate("Invalid time estimate for " + name, time_estimate)

        self.time_estimate = time_estimate
        self.changes = changes

        for var in self.changes:
            if var != "robot.location" and var.find("robot.holders.") != 0 and var.find("robot.positions.") != 0:
                raise BlastTypeError("Invalid variable set in " + name + ": " + var)
            self.validate("Invalid expression for variable " + var + " in " + name, self.changes[var])

        self.planable = planable
        #FIXME: keywords: robot

    def validate(self, text, exp):
        if type(exp) == type([1,]):
            return
        if type(exp) == type(""):
            if exp.strip() == "": return
            if exp.strip()[0] == "\"" and exp.strip()[-1] == "\"":
                return
            subs = paren_split(exp, '.')
            if subs[0].find('(') != -1:
                func = subs[0][0:subs[0].find("(")].strip()
                if not func in ['dist', 'mul', 'add', 'sub', 'div', 'angle_diff', 'abs', 'Object', 'None', 'False', 'True']:
                    raise BlastTypeError(text + ": invalid function: '" + func + "'")
                args = paren_split(subs[0][subs[0].find("(")+1:subs[0].rfind(")")], ',')
                for x in args: self.validate(text, x)
            else:
                if not subs[0] in self.parameters and subs[0] != "robot":
                    raise BlastTypeError(text + ": invalid variable '" + subs[0] + "'")
        elif type(exp) == type((0, 1)):
            if len(exp) < 1:
                raise BlastTypeError(text + ": empty expression")
            if exp[0] == "contains" or exp[0] == "exact-contains" or\
                    exp[0] == "not-contains" or exp[0] == "not-exact-contains":
                if len(exp) != 3:
                    raise BlastTypeError(text + ": wrong number of arguments for " + str(exp[0]) + ", must be 2")
                if len(exp[1].split(".")) != 2:
                    raise BlastTypeError(text + ": must have robot.<holder> for " + str(exp[0]))
                if exp[1].split(".")[0] != "robot":
                    raise BlastTypeError(text + ": must reference the robot (as in robot.<holder>) for " + str(exp[0]))
            elif exp[0] == "position":
                if len(exp) != 3:
                    raise BlastTypeError(text + ": wrong number of arguments for " + str(exp[0]) + ", must be 2")
                if len(exp[1].split(".")) != 2:
                    raise BlastTypeError(text + ": must have robot.<position> for " + str(exp[0]))
                if exp[1].split(".")[0] != "robot":
                    raise BlastTypeError(text + ": must reference the robot (as in robot.<position>) for " + str(exp[0]))
            elif exp[0] in ["==", "!=", "&&", "||", "not"]:
                if exp[0] == "not" and len(exp) != 2:
                    raise BlastTypeError(text + ": not must only have one argument in " + str(exp))
                for x in exp[1:]:
                    self.validate(text, x)
            else:
                raise BlastTypeError(text + ": invalid expression: " + str(exp))
        elif exp == False or exp == None:
            pass #For the position expressions
        else:
            raise BlastTypeError(text + ": Bad type for expression:" + str(exp))

#With action state output: False = could be in any state, None = left in orginal state
#Short-cut: whole thing = None or False
#With action compares: False = Don't care

def make_test_actions():
    return [BlastAction("pr2.move", {"end": "Pt"},
                        ("&&", ("==", "robot.location.map", "end.map"),
                         ("position", "robot.torso", [0.0,]),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked")),
                        "add(mul(\"8\", dist(robot.location, end)), abs(angle_diff(robot.location.a, end.a)))",
                        {"robot.location": "end" }),
            BlastAction("pr2.tuck-both-arms", {}, #Contains = temp hack for holder constraints
                        ("not", ("contains", "robot.left-arm", "coffee_cup"),), "\"10\"", #FIXME: need holder constraints
                        {"robot.positions.left-arm": [0.06024, 1.248526, 1.789070, -1.683386, 
                                                      -1.7343417, -0.0962141, -0.0864407, None],
                         "robot.positions.right-arm": [-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                        -1.4175, -1.8417, 0.21436, None]}),
            
            BlastAction("pr2-cupholder.buy_coffee", {"shop": "Surface:coffee_shop"},
                        ("&&", ("==", "robot.location", "shop.locations.start"),
                         ("contains", "robot.left-arm", "coffee_money_bag")),
                        "\"1000\"",
                        {"robot.location": "shop.locations.end",
                         "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                         "robot.positions.right-arm": "None()",
                         "robot.holders.left-arm": "Object(\"coffee_cup\")"}),
            BlastAction("pr2.door_blast", {"door": "Surface:transparent_heavy_door"},
                        ("&&", ("==", "robot.location", "door.locations.out_entrance"),
                         ("position", "robot.torso", [0.3,]),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked")),
                        "\"55\"", {"robot.location": "door.locations.out_exit"}),
            BlastAction("pr2.door_drag", {"door": "Surface:transparent_heavy_door"},
                        ("&&", ("==", "robot.location", "door.locations.in_entrance"), 
                         ("position", "robot.torso", [0.3,]),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked"),
                         ("contains", "robot.right-arm", "None()"),),
                        "\"115\"", {"robot.location": "door.locations.in_exit",
                                    "robot.positions.left-arm": False, 
                                    "robot.positions.right-arm": False}),

            BlastAction("pr2.torso", {"height": "Joint:torso.torso"},
                        "True()", "\"20\"", #FIXME
                        {"robot.positions.torso": ["height",]}),
                        
            BlastAction("pr2.elevator", {"elevator": "Surface:elevator", 
                                         "infloor": "Location:elevator.floor_",
                                         "outfloor": "Location:elevator.floor_"},
                        ("&&", ("==", "robot.location", "infloor"), ("!=", "infloor", "outfloor"),
                         ("contains", "robot.right-arm", "None()"),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.torso", [0.0,]),
                         ("position", "robot.right-arm", "tucked")),
                        "\"150\"", {"robot.location": "outfloor"}),
            
            BlastAction("pr2.grab-object", {"tts-text": "String"}, 
                        ("contains", "robot.left-arm", "None()"),
                        "\"30\"", {"robot.holders.left-arm": "Object(\"arbitrary-object\")",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   "robot.positions.right-arm": False},
                        planable = False),
            BlastAction("pr2.give-object", {"tts-text": "String"}, 
                        ("not", ("contains", "robot.left-arm", "None()")),
                        "\"30\"", {"robot.holders.left-arm": "None()",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   "robot.positions.right-arm": False},
                        planable = False),

            
            BlastAction("pr2-cupholder.stash-cupholder", {}, 
                        ("&&", ("not", ("contains", "robot.left-arm", "None()")),
                         ("contains", "robot.cup-holder", "None()")),
                        "\"30\"", {"robot.holders.cup-holder": "robot.holders.left-arm",
                                   "robot.holders.left-arm": "None()",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   "robot.positions.right-arm": False}),
            BlastAction("pr2-cupholder.unstash-cupholder", {}, 
                        ("&&", ("not", ("contains", "robot.cup-holder", "None()")),
                         ("contains", "robot.left-arm", "None()")),
                        "\"30\"", {"robot.holders.cup-holder": "None()",
                                   "robot.holders.left-arm": "robot.holders.cup-holder",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   "robot.positions.right-arm": False}),
            BlastAction("pr2-cupholder.coffee_run", {"person_location": "Pt", "shop": "Surface:coffee_shop"}, 
                        "True()", "\"10000\"", {"robot.location": "person_location"}, planable = False),
            ]


################################################################

class BlastWorldTypes:
    def __init__(self):
        self.surfaces = {}
        self.robots = {}
        self.objects = {}
        self.actions = {}

    def add_surface_type(self, surface):
        if surface.name in self.surfaces: raise BlastTypeError("Duplicate surface type: " + surface.name)
        self.surfaces[surface.name] = surface
    def get_surface(self, name):
        return self.surfaces.get(name, None)

    def add_robot_type(self, robot):
        if robot.name in self.robots: raise BlastTypeError("Duplicate robot type: " + robot.name)
        self.robots[robot.name] = robot
    def get_robot(self, name):
        return self.robots.get(name, None)

    def add_object_type(self, obj):
        if obj.name in self.objects: raise BlastTypeError("Duplicate object type: " + obj.name)
        self.objects[obj.name] = obj
    def get_object(self, name):
        return self.objects.get(name, None)
    
    def add_action_type(self, action):
        if action.name in self.actions: raise BlastTypeError("Duplicate action type: " + action.name)
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
    types_world.add_object_type(ObjectType("arbitrary-object"))
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
                                                               [False, False, False, False, False, False, False, False]),
                                                               #[0.06024, 1.248526, 1.789070, -1.683386, 
                                                               # -1.7343417, -0.0962141, -0.0864407, 0.0]),
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
                                                                [False, False, False, False, False, False, False, False]),
                                                                #[-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                                #  -1.4175, -1.8417, 0.21436, 0.0]),
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
        st = str(self.x) + str(self.y) + str(self.a) + str(self.map)
        hl.update(st)

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

blast_object_id = 0

class BlastObject:
    def __init__(self, object_type, pos, parent): #Note: if pos is None, parent is a robot
        self.object_type = object_type
        self.position = pos
        self.parent = parent
        global blast_object_id
        self.uid = blast_object_id
        blast_object_id = blast_object_id + 1

    def copy(self):
        if self.position:
            r = BlastObject(self.object_type, self.position.copy(), self.parent)
        else:
            r = BlastObject(self.object_type, self.position, self.parent)
        r.uid = self.uid
        return r
    
    def hash_update(self, hl):
        hl.update(self.object_type.name)
        if self.position: self.position.hash_update(hl)
        else: hl.update("None")
        if self.parent: hl.update(self.parent)

    def equal(self, other, tolerant = False):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        
        if self.object_type.name != other.object_type.name: return False
        if self.parent != other.parent: return False
        if not self.position: return not other.position
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

    def hash_update(self, hl, get_obj):
        for name, loc in sorted(self.locations.iteritems(), key=lambda x: x[0]):
            hl.update(name)
            loc.hash_update(hl)
        hl.update(self.state)
        hl.update(self.surface_type.name)

    def equal(self, other, get_obj, other_get_obj, tolerant = False):
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

    def hash_update(self, hl, get_obj):
        hl.update(self.map_file)

    def copy(self):
        return BlastMap(self.map, self.map_file)

    def equal(self, other, get_obj, other_get_obj, tolerant = False):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return other.map == self.map and self.map_file == other.map_file

    def to_text(self):
        r = "\tMap(\"" + str(self.map) + "\", \"" \
            + self.map_file + "\"):\n"
        return r

class BlastObjectRef:
    def __init__(self, uid):
        self.uid = uid
    def copy(self):
        return BlastObjectRef(obj)
    def equal(self, other, get_obj, other_get_obj):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return get_obj(self.uid).equal(other_get_obj(other.uid))
    def hash_update(self, hl, get_obj):
        get_obj(self.uid).hash_update(hl)
    def to_text(self):
        return "BlastObjectRef(" + str(self.uid) + ")"
        

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
        copy.holders = self.holders.copy()
        copy.positions = {}
        for name in self.positions:
            if self.positions[name]:
                copy.positions[name] = {}
                for joint in self.positions[name]:
                    copy.positions[name][joint] = self.positions[name][joint]
            else:
                copy.positions[name] = False
        return copy

    def hash_update(self, hl, get_obj):
        hl.update(self.name)
        hl.update(self.robot_type.name)
        self.location.hash_update(hl)
        for name, value in sorted(self.holders.iteritems(), key=lambda x: x[0]):
            if value == None:
                hl.update("None")
            else:
                value.hash_update(hl, get_obj)
        for name, value in sorted(self.positions.iteritems(), key=lambda x: x[0]):
            if value == False:
                hl.update("False")
            else:
                for j, sub in sorted(value.iteritems(), key=lambda x: x[0]):
                    hl.update(str(sub))

    def equal(self, other, get_obj, other_get_obj, tolerant = False):
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
                if not self.holders[name].equal(other.holders[name], get_obj, other_get_obj): return False
            elif other.holders[name]:
                return False
        for name in other.positions:
            if not name in self.positions: return False
        for name in self.positions:
            if self.positions[name]:
                if not other.positions[name]: return False

                tol = {}
                if tolerant:
                    tup = self.robot_type.position_variables[name][False]
                    for joint, toler in zip(tup[0], tup[1]):
                        tol[joint] = toler

                for joint in self.positions[name]:
                    if joint not in other.positions[name]: return False
                    if tolerant:
                        if self.positions[name][joint] != False and other.positions[name][joint] != False: 
                            d = self.positions[name][joint] - other.positions[name][joint]
                            if tol[joint] < -d or d > tol[joint]:
                                return False
                    else:
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
        self.objects = {}
        self.hash_state = None
        self.copy_on_write_optimize = False

    def get_obj(self, uid):
        return self.objects.get(uid)

    def copy(self, copy_on_write_optimize = True):
        copy = BlastWorld(self.types)
        copy.copy_on_write_optimize = copy_on_write_optimize
        for arr in ["maps", "surfaces", "robots", "objects"]:
            if copy_on_write_optimize:
                setattr(copy, arr, getattr(self, arr).copy())
            else:
                copy_arr = getattr(copy, arr)
                for name, item in getattr(self, arr).iteritems():
                    copy_arr[name] = item.copy()
        copy.hash_state = None
        return copy
    def append_map(self, mp):
        if mp.map in self.maps: raise BlastError("Duplicate map: " + mp.map)
        self.hash_state = None
        self.maps[mp.map] = mp
    def append_surface(self, surface):
        if surface.name in self.surfaces: raise BlastError("Duplicate surface: " + surface.name)
        self.hash_state = None
        self.surfaces[surface.name] = surface
    def append_robot(self, robot):
        if robot.name in self.robots: raise BlastError("Duplicate robot: " + robot.name)
        self.hash_state = None
        self.robots[robot.name] = robot
    def append_object(self, obj):
        if obj.uid in self.objects: raise BlastError("Duplicate object: " + obj.uid)
        self.hash_state = None
        self.objects[obj.uid] = obj

    def get_hex_hash_state(self):
        return ''.join('%02x' % ord(byte) for byte in self.get_hash_state())

    def to_text(self):
        r = "World(" + self.get_hex_hash_state() + "):\n"
        for uid, obj in sorted(self.objects.iteritems(), key=lambda x: x[0]):
            r = r + "\tObjectSet(" + str(uid) + ", " + obj.to_text() + ")\n"
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

    def gc_objects(self): #Delete unused objects
        objects = set()
        for robot in self.robots.itervalues():
            for obj in robot.holders.itervalues():
                if obj:
                    objects.add(obj.uid)

        for obj in self.objects.keys():
            if not obj in objects:
                del self.objects[obj]

    def get_hash_state(self):
        if self.hash_state:
            return self.hash_state

        hl = hashlib.sha224()
        get_obj = lambda x: self.get_obj(x)
        for arr in [self.maps, self.surfaces, self.robots]:
            for name, value in sorted(arr.iteritems(), key=lambda x: x[0]):
                hl.update(str(name))
                value.hash_update(hl, get_obj)
        self.hash_state = hl.digest()
        
        return self.hash_state

    def equal(self, other, tolerant = False): #Tolerant of slight variations
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False

        if not tolerant: #Hash checks only in tolerant equals
            warn = True
            hash_same = self.get_hash_state() == other.get_hash_state()

            #Comment out these two lines to verify hashing
            if not hash_same: return False
            return True #Comment this line to ensure, 100%, no hash collisions
            #This is to double-check to make sure hashing went as expected.
        else:
            warn = True
            hash_same = True
        
        get_obj = lambda x: self.get_obj(x)
        other_get_obj = lambda x: other.get_obj(x)
        for arr, oarr in [(self.robots, other.robots), (self.surfaces, other.surfaces), (self.maps, other.maps)]:
            for name, value in oarr.iteritems():
                if not name in arr:
                    if warn and hash_same: print "Hash state collision due to missing", name
                    return False
            for name, value in arr.iteritems():
                if not name in oarr:
                    if warn and hash_same: print "Hash state collision due to extra", name
                    return False
                if not value.equal(oarr[name], get_obj, other_get_obj, tolerant): 
                    if warn and hash_same: print "Hash state collision due to", name
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


    def robot_position_condition(self, robot, condition, debug, super_debug):
        if condition[1].find("robot.") != 0:
            if debug: print "Invalid position type", condition[1]
            return None
        pos = condition[1].split(".")[1]
        if not pos in robot.positions:
            if debug: print "Invalid position name", condition[1]
            return None
        if not pos in robot.robot_type.position_variables:
            if debug: print "Invalid position name", condition[1]
            return None
        
        state = robot.positions[pos]
        typ = robot.robot_type.position_variables[pos]

        name_array = typ[False][0]
        tol_array = typ[False][1]
        goal_array = []
        if type(condition[2]) == type(""):
            if not condition[2] in typ:
                if debug: print "Invalid pre-defined position state", condition[2]
                return None 
            goal_array = typ[condition[2]]
        elif type(condition[2]) == type([1,]):
            if len(condition[2]) != len(name_array):
                if debug: print "Wrong length for goal array", condition[2], "for", name_array
                return None
            goal_array = condition[2]
        else:
            if debug: print "Invalid type for position state compare", condition[2]
            return None

        for name, goal, tol in zip(name_array, goal_array, tol_array):
            if goal != False:
                if state[name] == False:
                    if super_debug: print state[name], name, "failed"
                    return False
                if goal - tol > state[name] or goal + tol < state[name]:
                    if super_debug: print name, goal, tol, state[name], "failed"
                    return False
        return True
    
    def robot_contains_condition(self, robot, condition):
        if condition[0].find("not-") == 0:
            r = self.robot_contains_condition(robot, (condition[0][4:],) + condition[1:])
            if r == True: return False
            if r == False: return True
            return r
        obj = None
        if condition[1].split(".")[0] == "robot":
            if not condition[1].split(".")[1] in robot.holders:
                if debug: print condition[1], "has invalid holder for robot"
                return False
            obj = robot.holders[condition[1].split(".")[1]]
        else:
            if debug: print "Invalid value for holder"
            return False
        #print obj, condition
        if obj:
            obj = self.objects[obj.uid]
        if condition[2] == "" or condition[2] == "None()":
            return obj == None
        if obj == None: return False #Obviously needs an object yet none
        if condition[2] == obj.object_type.name: return True
        if condition[0] == "exact-contains": return False
        t = obj.object_type
        while t.parent:
            if t.name == condition[2]: return True
            t = t.parent
        return False
    
    def take_action(self, robot_name, action, parameters, execute = True, debug = False):
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

        #Ensure that we have all the proper parameters and convert strings to locations
        clone_param = False
        for name, ptype in action_type.parameters.iteritems():
            if not name in parameters:
                if debug: print "Parameter", name, "unspecified"
                return None
            if ptype.find("Surface:") == 0:
                if type(parameters[name]) == type(""):
                    if not clone_param:
                        parameters = parameters.copy() #Avoid mutating the original dictionary
                        clone_param = True
                    parameters[name] = self.surfaces.get(parameters[name])


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
                    obj = BlastObject(self.types.get_object(vals[0]), pos, par)
                    self.append_object(obj)
                    val = BlastObjectRef(obj.uid)
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
                if subs[1] == "holders":
                    return robot.holders[subs[2]]
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
        def assert_condition(condition, super_debug=False):
            if (condition in [False, None, True]): return condition
            if (condition == "True()"): return True
            if (condition[0] == "==" or condition[0] == "!="):
                if super_debug: print condition[1], condition[2]
                a = get_value(condition[1])
                b = get_value(condition[2])
                if super_debug: print condition[0], a, b
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
                    if super_debug: print condition[i], v
                    if condition[0] == "&&" and not v: return False
                    if condition[0] == "||" and v: return True
                return condition[0] == "&&"
            elif (condition[0] == "contains" or condition[0] == "exact-contains"
                  or condition[0] == "not-contains" or condition[0] == "not-exact-contains"):
                r = self.robot_contains_condition(robot, condition)
                if super_debug: print condition, "->", r
                return r
            elif (condition[0] == "position"):
                r = self.robot_position_condition(robot, condition, debug, super_debug)
                if super_debug: print condition, "->", r
                return True
            elif (condition[0] == "not"):
                return not assert_condition(condition[1], super_debug)
            else:
                print "Invalid condition", condition
            return False
        
        if not assert_condition(action_type.condition):
            if debug: print "Condition returned false"
            if debug: assert_condition(action_type.condition, True)
            return None
    
        time_estimate = get_value(action_type.time_estimate)
        if debug: print "Time estimate:", time_estimate
        if time_estimate == None:
            if debug: print "Time estimate invalid"
            return None

        #Conditions met, everything is done. Now we just need to update the values
        #create a queue of transations then execute them
        transactions = []
        items_to_clone = {"robots": set(), "surfaces": set()}
        for var, val in action_type.changes.iteritems():
            sub = var.split(".")
            if debug: print var, "->", val
            if sub[0] == "robot":
                items_to_clone["robots"].add(robot.name)
                if not hasattr(robot, sub[1]):
                    if debug: print "Attempt to set invalid robot attribute", sub[1]
                    return None
                if type(val) == type([1,]):
                    v = [get_value(x) if x not in [None, False, True] and type(x) != type(0.01) else x for x in val]
                else:
                    v = get_value(val)
                if v == None:
                    if debug: print "Invalid value for set:", val
                    return None
                if sub[1] == "holders":
                    if BLAST_NONE.equal(v):
                        v = None
                    else:
                        self.get_obj(v.uid).position = None
                        self.get_obj(v.uid).parent = robot.name + "." + sub[2]
                    transactions.append(("VAL", ("robot", robot.name, "holders"), sub[2], v))
                elif sub[1] == "positions":
                    if not sub[2] in robot.positions:
                        if debug: print "Invalid position for robot", robot.name, sub
                        return None
                    set_d = {}
                    if type(v) == type([1,]):
                        joint_names = robot.robot_type.position_variables[sub[2]][False][0]
                        if len(joint_names) != len(v):
                            if debug: print "Invalid number of joints for array based-update", sub[2]
                            return None
                        for joint_name, value in zip(joint_names, v):
                            if value == None:
                                set_d[joint_name] = robot.positions[sub[2]][joint_name]
                            else:
                                set_d[joint_name] = value
                    elif v == None:
                        pass #The action is assumed to have left everything in the previous state
                    elif v == False:
                        for joint_name in robot.robot_type.position_variables[sub[2]][False][0]:
                            set_d[joint_name] = False
                    else:
                        if debug: print "Invalid type for positions", v
                    if v != None and set_d != {}:
                        transactions.append(("VAL", ("robot", robot.name, "positions"), sub[2], set_d))
                else:
                    transactions.append(("SET", ("robot", robot.name), sub[1], v))
            else:
                if debug: print "Update failed"
                return None

        if not execute:
            return time_estimate

        if self.copy_on_write_optimize:
            for robot_name in items_to_clone["robots"]:
                self.robots[robot_name] = self.robots[robot_name].copy()

        #Execute the transactions - this is where changes actually get made to the world
        self.hash_state = None
        for x in transactions:
            v = None
            if x[1][0] == "robot":
                v = self.robots[x[1][1]]
            for attr in x[1][2:]:
                v = getattr(v, attr)

            if x[0] == "SET":
                setattr(v, x[2], x[3])
            elif x[0] == "VAL":
                v[x[2]] = x[3]
            else:
                print "Really bad, transaction bad:", x
                raise Exception("Bad transaction: " + str(x))

        self.gc_objects()

        return time_estimate

    def enumerate_action(self, robot_name, action, extra_goals):
        #Get the robot
        robot = self.robots.get(robot_name)
        if not robot:
            if debug: print "Invalid robot:", robot_name
            return False, None
        
        #Get action type
        action_robot_type, action_type = self.types.get_action_for_robot(robot.robot_type, action)
        if not action_robot_type or not action_type:
            if debug:
                print "Could not find the action type", action, "for", robot.robot_type
            return False, None

        #Try to optimize conditions
        req_conditions = [action_type.condition]
        conditions = [] #Conditions that absolutely must be true
        while req_conditions:
            condition = req_conditions[0]
            req_conditions = req_conditions[1:]
            if condition[0] in ['==', '!=', 'contains', 'exact-contains', 'not-contains', 'not-exact-contains', 'position']:
                conditions.append(condition)
            elif condition[0] == '&&':
                for i in condition[1:]:
                    req_conditions.append(i)
            elif condition[0] == 'not':
                invert = {"==": "!=", "contains": "not-contains", "exact-contains": "not-exact-contains",
                          "!=": "==", "not-contains": "contains", "not-exact-contains": "exact-contains"}
                if condition[1][0] in invert:
                    req_conditions.append((invert[condition[1][0]],) + condition[1][1:])
            elif condition != "True()":
                print "Ignore", condition

        #print action_type.condition, "->", conditions

        #Check for cases where it is impossible to execute the action
        for condition in conditions:
            if condition[0] == "contains" or condition[0] == "exact-contains" \
                    or condition[0] == "not-contains" or condition[0] == "not-exact-contains":
                if not self.robot_contains_condition(robot, condition):
                    return False, False
            if condition[0] == "position":
                if not self.robot_position_condition(robot, condition, False, False):
                    return False, False


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
                for loc in extra_goals.get(param, []) + extra_goals.get(ptype, []):
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
            elif ptype == "String":
                parameters[param] = []
            elif ptype[0:len("Joint:")] == "Joint:":
                parameters[param] = []
                position, joint = ptype.split(":")[1].split(".")
                rtypes = [ robot.robot_type, ]
                while rtypes[-1].parent:
                    rtypes.append(rtypes[-1].parent)
                rtypes = [r.name for r in rtypes]
                for aname, action in self.types.actions.iteritems():
                    if aname.split(".")[0] in rtypes:
                        def crawl_actions(c):
                            if type(c) != type((1,)):
                                return []
                            if c[0] != "position":
                                out = []
                                for cr in c[1:]:
                                    out = out + crawl_actions(cr)
                                return out
                            if c[1].split(".")[1] == position:
                                if type(c[2]) == type([]):
                                    a = c[2][ robot.robot_type.position_variables[position][False][0].index(joint)]
                                    if (a != False and a != None) or type(a) == type(0.1):
                                        return [a,]
                            return []
                        parameters[param] = parameters[param] + crawl_actions(action.condition)
                parameters[param] = list(set(parameters[param])) #Remove duplicates
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

        return action_type.planable, parameters


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
    clarkcenterpeetscoffee = BlastMap("clarkcenterpeetscoffee", "maps/clarkcenterthirdflooroutside.pgm")
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

    stair4 = BlastRobot("stair4", 
                        #BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                        BlastPt(12.000, 40.957, 0.148, clarkcenterfirstfloor.map),
                        world.types.get_robot("pr2-cupholder"))
    world.append_robot(stair4)


    return world

def run_test():
    world = make_test_world()
    
    #Set at the shop with the initial bag
    initial_bag = BlastObject(world.types.get_object("coffee_money_bag"), None, "stair4.right-arm")
    world.append_object(initial_bag)
    world.robots["stair4"].holders["left-arm"] = BlastObjectRef(initial_bag.uid)
    world.robots["stair4"].location = BlastPt(55.840, 14.504, -0.331, "clarkcenterpeetscoffee")
    
    print "Tuck arms:", world.take_action("stair4", "tuck-both-arms", {})
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)
    
    res = world.take_action("stair4", "move", {"end": BlastPt(56.869, 14.529, 2.511, "clarkcenterpeetscoffee")})
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


    res = world.take_action("stair4", "stash-cupholder", {}, debug = True)
    print "Action result:", res

    print '-'*130
    print "                                                            POST-STASH"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)

def clone_world_test(world):
    clone = world.copy()
    print '-'*130
    print "                                                            CLONE"
    print '-'*130
    print clone.to_text()
    print '-'*130
    print "Equal", clone.equal(world)
    
    print "Move:", world.enumerate_action("stair4", "move", {})
    print "Coffee:", world.enumerate_action("stair4", "buy_coffee", {})


def elevator_test():
    world = make_test_world()
    print "Tuck arms:", world.take_action("stair4", "tuck-both-arms", {})
    world.robots["stair4"].location = world.surfaces["clarkelevator"].locations["floor_1"].copy()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130

    print "Elevator:", world.enumerate_action("stair4", "elevator", {})


def arms_test():
    world = make_test_world()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130

    print "Tuck arms:", world.take_action("stair4", "tuck-both-arms", {})

    
    print '-'*130
    print "                                                            POST-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    
def torso_test():
    world = make_test_world()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    
    print "Lift", world.take_action("stair4", "torso", {"height": 0.3})
    
    
    print '-'*130
    print "                                                            POST-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130

    print world.enumerate_action("stair4", "torso", {})

if __name__ == '__main__':
    #torso_test()
    run_test()
    #elevator_test()
    #arms_test()
