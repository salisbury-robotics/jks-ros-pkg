#!/usr/bin/python
import math
import hashlib
import json

if __name__ == '__main__': print "This is not intended to be a main file."

#Permissions system

BLAST_INFINITY = 1e10 #For objects
#FIXME: __ illegal in robot names

class BlastTypeError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class BlastError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


class BlastAction(object):
    __slots__ = ['name', 'robot', 'parameters', 'condition', 'time_estimate',
                 'changes', 'display', 'planable', 'user', 'failure_modes']

    def to_dict(self):
        return {"name": self.name, "parameters": self.parameters, "condition": self.condition, 
                "time_estimate": self.changes, "display": self.display, 
                "planable": self.planable, 'user': self.user, 'failure_modes': self.failure_modes}

    def scan_only(self):
        for var in self.changes:
            if len(var.split(".")) != 2:
                return False
            if var.split(".")[1] != "scan":
                return False
            if not var.split(".")[0] in self.parameters:
                return False
        return True

    def __init__(self, name, parameters, condition, time_estimate, 
                 changes, display, planable = True, user = False, fm = {}): #Name must be robot_type.action
        self.name = name
        self.robot = name.split(".")[0]
        self.parameters = parameters
        self.display = display
        self.condition = condition
        self.failure_modes = fm

        if "robot" in self.parameters:
            raise BlastTypeError("A parameter cannot be called 'robot' in " + name)
        surface_parameters = set()
        for pname, ptype in self.parameters.iteritems():
            if ptype == "Pt":
                pass
            elif ptype.find("Surface:") == 0:
                surface_parameters.add(pname)
            elif ptype.find("Joint:") == 0:
                pass
            elif ptype.find("Location:") == 0:
                vname = ptype[ptype.find(":")+1:].split(".")[0]
                if not vname in self.parameters:
                    raise BlastTypeError("Location parameter does not reference "
                                         + "another parameter: " + pname + " -> " + vname)
                if self.parameters[vname].find("Surface:") != 0:
                    raise BlastTypeError("Location parameter " + pname + " does not reference"
                                         + " a surface type variable (" + vname + ")")
            elif ptype == "String":
                if planable:
                    raise BlastTypeError("Cannot plan with arbitary string type. Either "
                                         + "declare action unplanable or add enumeration")
            elif ptype == "Object":
                pass
            elif ptype.find("SurfaceObject:") == 0:
                vname = ptype[ptype.find(":")+1:]
                if not vname in self.parameters:
                    raise BlastTypeError("SurfaceObject parameter does not reference "
                                         + "another parameter: " + pname + " -> " + vname)
                if self.parameters[vname].find("Surface:") != 0:
                    raise BlastTypeError("SurfaceObject parameter " + pname + " does not "
                                         + "reference a surface type variable (" + vname + ")")
            elif ptype.find("Pos:") == 0:
                vname = ptype[ptype.find(":")+1:].split(",")[0].strip()
                if not vname in self.parameters:
                    raise BlastTypeError("Pos parameter does not reference another "
                                         + "parameter: " + pname + " -> " + vname)
                if self.parameters[vname].find("Surface:") != 0:
                    raise BlastTypeError("Pos parameter " + pname + " does not reference "
                                         + "a surface type variable (" + vname + ")")
                #Fixme: the object variable could be bad
                if len(ptype[ptype.find(":")+1:].split(",")) != 8:
                    raise BlastTypeError("Pos parameter " + pname + " does not have 8"
                                         + " arguments for type (" + vname + ")")
                for f in ptype[ptype.find(":")+1:].split(",")[2:]:
                    try:
                        r = float(f.strip())
                    except:
                        raise BlastTypeError("Pos parameter " + pname + " invalid value: " + f)
            else:
                raise BlastTypeError("Invalid parameter type for " + pname + ": " + ptype)

        self.validate("Invalid condition for " + name, condition)
        self.validate("Invalid time estimate for " + name, time_estimate)

        self.time_estimate = time_estimate
        self.changes = changes

        if 'success' in fm:
            raise BlastTypeError("A failure mode cannot be called 'success'")
        
        for mode, changes in [("success", self.changes),] + fm.items():
            for var in changes:
                if var != "robot.location" and var.find("robot.holders.") != 0 and var.find("robot.positions.") != 0 \
                        and not (var.split(".")[0] in surface_parameters and var.split(".")[1].split("+")[0] == "objects") \
                        and not (var.split(".")[0] in surface_parameters and var.split(".")[1].split("-")[0] == "objects") \
                        and not (var.split(".")[0] in surface_parameters and var.split(".")[1] == "scan"):
                    raise BlastTypeError("Invalid variable set in " + name + ": " + var + " for mode: " + mode)
                if var == "robot.location" and mode == "success" and planable \
                        and (changes[var].strip() == "False()" or changes[var].strip() == "None()"):
                    raise BlastTypeError("A planable action cannot define a free location for the robot.")
                if not (var.split(".")[0] in surface_parameters and var.split(".")[1] == "scan"):
                    self.validate("Invalid expression for variable " + var + " in " + name + " for mode: " + mode,
                                  changes[var])

        self.planable = planable
        self.user = user
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


################################################################

class BlastWorldTypes(object):
    __slots__ = ['surfaces', 'robots', 'objects', 'actions', 'parameter_values_cache']
    def __init__(self):
        self.surfaces = {}
        self.robots = {}
        self.objects = {}
        self.actions = {}
        self.parameter_values_cache = {}

    def add_surface_type(self, surface):
        self.parameter_values_cache = {}
        if surface.name in self.surfaces: raise BlastTypeError("Duplicate surface type: " + surface.name)
        self.surfaces[surface.name] = surface
    def get_surface(self, name):
        return self.surfaces.get(name, None)
    
    def add_robot_type(self, robot):
        self.parameter_values_cache = {}
        if robot.name in self.robots: raise BlastTypeError("Duplicate robot type: " + robot.name)
        self.robots[robot.name] = robot
    def get_robot(self, name):
        return self.robots.get(name, None)

    def add_object_type(self, obj):
        self.parameter_values_cache = {}
        if obj.name in self.objects: raise BlastTypeError("Duplicate object type: " + obj.name)
        self.objects[obj.name] = obj
    def get_object(self, name):
        return self.objects.get(name, None)
    def add_object_tag(self, name, tag):
        if name in self.objects:
            self.objects[name].add_tag(tag)
            return False
        return True
    
    def add_action_type(self, action):
        self.parameter_values_cache = {}
        if action.name in self.actions: raise BlastTypeError("Duplicate action type: " + action.name)
        self.actions[action.name] = action
    def get_action(self, name):
        return self.actions.get(name, None)

    def get_action_for_robot(self, robot_type, action):
        if type(robot_type) == type(""):
            action_robot_type = self.get_robot(robot_type)
        else:
            action_robot_type = robot_type
        action_type = None
        while not action_type:
            action_type = self.get_action(action_robot_type.name + "." + action)
            if action_robot_type.parent:
                action_robot_type = action_robot_type.parent
            elif not action_type:
                return None, None
        return action_robot_type, action_type

class SurfaceType(object):
    __slots__ = ['name', 'states', 'planes']
    def __init__(self, name, states, planes = {}):
        self.name = name
        self.states = states
        self.planes = planes
    
class RobotType(object):
    __slots__ = ['name', 'holders', 'position_variables', 'parent', 'holders_keysort', 'position_variables_keysort',
                 'display']
    def __init__(self, name, display, holders, position_variables, parent = None):
        #TODO: define "display" values

        self.name = name
        self.holders = {}
        self.display = {}

        self.position_variables = {}
        self.parent = parent
        if parent:
            for n, d in parent.holders.iteritems():
                self.holders[n] = d
            for n, d in parent.position_variables.iteritems():
                self.position_variables[n] = d
            for n, d in parent.display.iteritems():
                self.display[n] = d
        for n, d in holders.iteritems():
            self.holders[n] = d
        for n, d in position_variables.iteritems():
            self.position_variables[n] = d
        for n, d in display.iteritems():
            self.display[n] = d
        self.position_variables_keysort = sorted(self.position_variables.keys())
        self.holders_keysort = sorted(self.holders.keys())

    def to_dict(self):
        par = None
        if self.parent: par = self.parent.name
        return {"name": self.name, "holders": self.holders, "display": self.display,
                "position_variables": self.position_variables, "parent": par}

class ObjectType(object):
    __slots__ = ["name", "parent", "tags", "motion_limits"]
    def __init__(self, name, motion_limits, parent = None):
        self.name = name
        self.tags = set()
        self.parent = None

        #Create motion limits
        self.motion_limits = motion_limits
        if parent:
            for name, value in parent.motion_limits.iteritems():
                if not name in self.motion_limits:
                    self.motion_limits[name] = value
        def_motion_limits = {"rotation_limit": math.pi, "accel_x": BLAST_INFINITY, 
                             "accel_y": BLAST_INFINITY, "accel_z": BLAST_INFINITY,
                             "bound_d": False, "bound_h": False}
        for name, value in def_motion_limits.iteritems():
            if not name in self.motion_limits:
                self.motion_limits[name] = value
    
    def add_tag(self, name):
        bad_list = ["name", "rotation_limit", "accel_x", "accel_y", "accel_z", "bound_d", "bound_h"]
        if name in bad_list:
            raise BlastTypeError(name + " is an invalid tag for objects because "
                                 + "it is a motion constraint in " + self.name)
        self.tags.add(name)

################################################################

class BlastPt(object):
    __slots__ = ['x', 'y', 'a', 'map']
    def __init__(self, x, y, a, mid):
        self.x = float(x)
        self.y = float(y)
        self.a = float(a)
        self.map = mid
        self.wrap_angle()

    def wrap_angle(self):
        while self.a > +math.pi: self.a = self.a - math.pi * 2
        while self.a < -math.pi: self.a = self.a + math.pi * 2
        
    def to_dict(self):
        return {'x': self.x, 'y': self.y, 'a': self.a, 'map': self.map}

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

class BlastPos(object):
    __slots__ = ('x', 'y', 'z', 'rx', 'ry', 'rz')
    def __init__(self, x, y, z, rx, ry, rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz

    def copy(self):
        return BlastPos(self.x, self.y, self.z, self.rx, self.ry, self.rz)

    def to_dict(self):
        return {"x": self.x, "y": self.y, "z": self.z,
                "rx": self.rx, "ry": self.ry, "rz": self.rz}

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

    def hash_update(self, hl):
        stp = str(self.x) + str(self.y) + str(self.z)
        sta = str(self.rx) + str(self.ry) + str(self.rz)
        hl.update(stp)
        hl.update(sta)
        

    def __repr__(self):
        return self.to_text()
    def __str__(self):
        return self.to_text()
    
    def to_text(self):
        return "Pos(" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) \
            + ", " + str(self.rx) + ", " + str(self.ry) + ", " + str(self.rz) + ")"

class BlastPosIrr(object):
    __slots__ = ()
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

class BlastObject(object):
    __slots__ = ('object_type', 'position', 'parent', 'uid')
    def __init__(self, object_type, pos, parent): #Note: if pos is None, parent is a robot
        self.object_type = object_type
        self.position = pos
        self.parent = parent
        global blast_object_id
        self.uid = blast_object_id
        blast_object_id = blast_object_id + 1

    def to_dict(self):
        p = None
        if self.position: p = self.position.to_dict()
        return {"object_type": self.object_type.name, "position": p,
                "uid": self.uid, "parent": self.parent}

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

class BlastSurface(object):
    __slots__ = ['name', 'locations', 'surface_type', 'state', 'locations_keysort', 'objects', 'scan']
    def __init__(self, name, locations, surface_type, state = None):
        self.name = name
        self.locations = locations
        self.locations_keysort = sorted(locations.keys())
        self.surface_type = surface_type
        self.state = state
        self.scan = set()
        self.objects = []
        if not self.state:
            self.state = ""
            for state, data in surface_type.states.iteritems():
                if data.get("default", False):
                    self.state = state

    def to_dict(self):
        lc = {}
        for name in self.locations:
            lc[name] = self.locations[name].to_dict()
        return {'name': self.name, 'locations': lc, 'state': self.state,
                'type': self.surface_type.name}
    def to_json(self):
        return json.dumps(self.to_dict())

    def __str__(self):
        return "Surface:" + self.name
    def __repr__(self):
        return "Surface:" + self.name
    
    def copy(self):
        lclone = {}
        for name, loc in self.locations.iteritems():
            lclone[name] = loc.copy()
        c = BlastSurface(self.name, lclone, self.surface_type, self.state)
        c.objects = [x.copy() for x in self.objects]
        [c.scan.add(x) for x in self.scan]
        return c


    def hash_update(self, hl, get_obj, consider_scan):
        for name in self.locations_keysort:
            hl.update(name)
            self.locations[name].hash_update(hl)
        hl.update(self.state)
        hl.update(self.surface_type.name)

        def cmpf(a, b):
            oa = get_obj(a.uid)
            ob = get_obj(b.uid)

            def ti(v):
                if (v > -1 and v < 1 and v != 0.0):
                    v = 1.0 / v
                return int(v)

            if (oa.position.x != ob.position.x): return ti(oa.position.x - ob.position.x)
            if (oa.position.y != ob.position.y): return ti(oa.position.y - ob.position.y)
            if (oa.position.z != ob.position.z): return ti(oa.position.z - ob.position.z)
            if (oa.position.rx != ob.position.rx): return ti(oa.position.rx - ob.position.rx)
            if (oa.position.ry != ob.position.ry): return ti(oa.position.ry - ob.position.ry)
            return ti(oa.position.rz - ob.position.rz)

        self.objects.sort(cmpf)
        for o in self.objects:
            o.hash_update(hl, get_obj)

        if consider_scan:
            for ot in sorted(self.scan):
                hl.update(ot)

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
            + "\", \"" + self.state + "\", \"" + ",".join(self.scan) + "\")\n"

class BlastPrimitive(object):
    __slots__ = ['name']
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

class BlastMap(object):
    __slots__ = ['map', 'map_file', 'ppm']
    def __init__(self, mid, map_file, ppm):
        self.map = mid
        self.map_file = map_file
        self.ppm = ppm

    def hash_update(self, hl, get_obj, consider_scan):
        hl.update(self.map_file + str(self.ppm))

    def copy(self):
        return BlastMap(self.map, self.map_file, self.ppm)

    def equal(self, other, get_obj, other_get_obj, tolerant = False):
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return other.map == self.map and self.map_file == other.map_file  \
            and self.ppm == other.ppm

    def to_text(self):
        r = "\tMap(\"" + str(self.map) + "\", \"" \
            + self.map_file + "\", \"" + str(self.ppm) + "\"):\n"
        return r

    def to_dict(self):
        return {"map": self.map, "map_file": self.map_file, "ppm": self.ppm}

class BlastObjectRef(object):
    __slots__ = ['uid']
    def __init__(self, uid):
        self.uid = uid
    def copy(self):
        return BlastObjectRef(self.uid)
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
    

class BlastRobot(object):
    __slots__ = ['name', 'robot_type', 'location', 'holders', 'positions']
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

    def collide(self, other): #TODO actually compare
        return other.location.equal(self.location)

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

    def hash_update(self, hl, get_obj, consider_scan):
        hl.update(self.name)
        hl.update(self.robot_type.name)
        self.location.hash_update(hl)
        for name in self.robot_type.holders_keysort:
            value = self.holders[name]
            if value == None:
                hl.update("None")
            else:
                value.hash_update(hl, get_obj)
        for name in self.robot_type.position_variables_keysort:
            value = self.positions[name]
            if value == False and type(value) == type(True):
                hl.update("False")
            else:
                for j in self.robot_type.position_variables[name][False][0]:
                    hl.update(str(value[j]))

    def reparent_objects(self, get_obj, copy_obj):
        change = False
        for name in self.robot_type.holders_keysort:
            value = self.holders[name]
            if value:
                if get_obj(value.uid).parent != self.name + "." + name:
                    copy_obj(value.uid)
                    get_obj(value.uid).parent = self.name + "." + name
                    get_obj(value.uid).position = None
                    change = True
                elif get_obj(value.uid).position:
                    copy_obj(value.uid)
                    get_obj(value.uid).position = None
                    change = True
        return change
    
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
        for name in self.robot_type.position_variables_keysort:
            if self.positions[name]:
                r = r + "\t\tRobotState(\"" + self.name + "\", \"" + name + "\", "
                r = r + ", ".join([str(x) for n, x in sorted(self.positions[name].iteritems(), key=lambda x: x[0])]) + ")\n"
        for name in self.robot_type.holders_keysort:
            if self.holders[name]:
                r = r + "\t\tRobotHolder(\"" + self.name + "\", \"" + name + "\", "
                r = r + self.holders[name].to_text() + ")\n"
        return "\t\tRobot(\"" + self.name + "\", " + self.location.to_text() \
            + ", \"" + self.robot_type.name + "\")\n" + r
    
    def to_dict(self):
        r_holder = {}
        for name in self.robot_type.holders_keysort:
            if self.holders[name] != None:
                r_holder[name] = self.holders[name].uid
            else:
                r_holder[name] = None
        return {"name": self.name, "robot_type": self.robot_type.name, 
                "location": self.location.to_dict(),
                "holders": r_holder, "positions": self.positions}

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

class BlastWorld(object):
    __slots__ = ['types', 'maps', 'surfaces', 'robots', 'objects', 'copy_on_write_optimize', 'sumh',
                 'maps_hash_state', 'surfaces_hash_state', 'robots_hash_state', 'objects_hash_state',
                 'maps_keysort', 'surfaces_keysort', 'robots_keysort', 'objects_keysort', 'consider_scan']
    def __init__(self, types):
        self.types = types
        self.maps = {}
        self.surfaces = {}
        self.robots = {}
        self.objects = {}
        self.maps_keysort = []
        self.surfaces_keysort = []
        self.robots_keysort = []
        self.objects_keysort = []
        self.maps_hash_state = None
        self.robots_hash_state = None
        self.surfaces_hash_state = None
        self.objects_hash_state = None
        self.sumh = None
        self.consider_scan = False
        self.copy_on_write_optimize = True

    def delete_surface_object(self, obj):
        if self.copy_on_write_optimize:
            self.objects[obj] = self.objects[obj].copy()
        self.objects[obj].parent = None
        self.gc_objects()
        return True

    def clear_scan(self):
        for sn in self.surfaces_keysort:
            if len(self.surfaces[sn].scan) != 0:
                if self.copy_on_write_optimize:
                    self.surfaces[sn] = self.surfaces[sn].copy()
                self.surfaces[sn].scan = set()
                self.clear_hash("surfaces")

    def scan_count(self, ot):
        c = 0
        for name, surf in self.surfaces.iteritems():
            if ot in surf.scan:
                c = c + 1
        return c


    def get_scan_actions(self):
        result_dir = {}
        for at_name, at in self.types.actions.iteritems():
            for var, value in at.changes.iteritems():
                if var.find(".scan") != 0 and var.find(".") == var.find(".scan"):
                    for ot in value.split(","):
                        s = result_dir.get(ot, set())
                        s.add(at_name)
                        result_dir[ot] = s
        return result_dir
                    
    def add_surface_object(self, surface, object_type, pos):
        if self.copy_on_write_optimize:
            self.surfaces[surface] = self.surfaces[surface].copy()
        obj_type = self.types.get_object(object_type)
        obj = BlastObject(obj_type, pos, surface)
        self.append_object(obj)
        self.surfaces[surface].objects.append(BlastObjectRef(obj.uid))
        self.gc_objects()  
        self.clear_hash("surfaces")
        return True
    
    def copy_obj(self, uid):
        if self.copy_on_write_optimize and uid in self.objects:
            self.objects[uid] = self.objects[uid].copy()
    def get_obj(self, uid):
        return self.objects.get(uid)

    def world_limit_check(self, limits):
        if "robot-holders" in limits:
            for robot_name, values in limits["robot-holders"].iteritems():
                if not robot_name in self.robots:
                    return False
                for holder, set_as in values.iteritems():
                    if not holder in self.robots[robot_name].holders:
                        return False
                    value = self.robots[robot_name].holders[holder]
                    if set_as != None and value == None: return False
                    if set_as == None and value != None: return False
                    if set_as != None and value != None:
                        if int(value.uid) != int(set_as):
                            return False
                    #We don't need to worry about the value == None and set_as == None case
        if "robot-location" in limits:
            for robot_name, value in limits["robot-location"].iteritems():
                if not robot_name in self.robots:
                    return False
                pt = BlastPt(value['x'], value['y'], value['a'], value['map'])
                if not self.robots[robot_name].location.equal(pt):
                    return False
        
        return True

    def get_robot_holder(self, robot, holder):
        a = self.robots[robot].holders[holder]
        if a == None: return None
        return a.uid

    def copy(self, copy_on_write_optimize = True):
        copy = BlastWorld(self.types)
        copy.copy_on_write_optimize = copy_on_write_optimize and self.copy_on_write_optimize
        for arr in ["maps", "surfaces", "robots", "objects"]:
            setattr(copy, arr + '_keysort', getattr(self, arr + '_keysort'))
            if copy.copy_on_write_optimize:
                setattr(copy, arr, getattr(self, arr).copy())
            else:
                copy_arr = getattr(copy, arr)
                for name, item in getattr(self, arr).iteritems():
                    copy_arr[name] = item.copy()
        copy.surfaces_hash_state = self.surfaces_hash_state
        copy.maps_hash_state = self.maps_hash_state
        copy.objects_hash_state = self.objects_hash_state
        copy.robots_hash_state = self.robots_hash_state
        copy.sumh = self.sumh
        copy.consider_scan = self.consider_scan
        return copy
    def append_map(self, mp):
        if mp.map in self.maps: raise BlastError("Duplicate map: " + mp.map)
        self.clear_hash("maps")
        self.maps[mp.map] = mp
        self.maps_keysort = sorted(self.maps.keys())
    def append_surface(self, surface):
        if surface.name in self.surfaces: raise BlastError("Duplicate surface: " + surface.name)
        self.clear_hash("surfaces")
        self.surfaces[surface.name] = surface
        self.surfaces_keysort = sorted(self.surfaces.keys())
    def append_robot(self, robot):
        if robot.name in self.robots: raise BlastError("Duplicate robot: " + robot.name)
        self.clear_hash("robots")
        self.robots[robot.name] = robot
        self.robots_keysort = sorted(self.robots.keys())
    def append_object(self, obj):
        if obj.uid in self.objects: raise BlastError("Duplicate object: " + obj.uid)
        self.clear_hash("objects")
        self.objects[obj.uid] = obj
        self.objects_keysort = sorted(self.objects.keys())

    def get_hex_hash_state(self):
        return ''.join('%02x' % ord(byte) for byte in self.get_hash_state())

    def to_text(self):
        r = "World(" + self.get_hex_hash_state() + "):\n"
        for uid in self.objects_keysort:
            r = r + "\tObjectSet(" + str(uid) + ", " + self.objects[uid].to_text() + ")\n"
        for mid in self.maps_keysort:
            mp = self.maps[mid]
            r = r + mp.to_text()
            for name in self.surfaces_keysort:
                sur = self.surfaces[name]
                for loc in sur.locations_keysort:
                    if sur.locations[loc].map == mp.map:
                        r = r + sur.to_text(loc)
            for name in self.robots_keysort:
                robot = self.robots[name]
                if robot.location.map == mp.map:
                    r = r + robot.to_text()
        return r

    def gc_objects(self): #Delete unused objects
        #print self.objects, self.objects_keysort
        objects = set()
        for robot in self.robots.itervalues():
            for obj in robot.holders.itervalues():
                if obj:
                    objects.add(obj.uid)

        for name in self.surfaces_keysort:
            surface = self.surfaces[name]
            nobjects = []
            diff = False
            for obj in surface.objects:
                if obj.uid in self.objects_keysort:
                    if self.objects[obj.uid].parent == surface.name:
                        nobjects.append(obj)
                        objects.add(obj.uid)
                    else:
                        #print "Remove", obj.uid, "for parent"
                        diff = True
                else:
                    #print "Remove", obj.uid, "for non-existence", type(self.objects_keysort[0]), type(obj.uid)
                    diff = True
            if diff:
                self.surfaces[name] = surface.copy()
                self.surfaces[name].objects = nobjects
                self.clear_hash("surfaces")
            #print self.surfaces[name].objects
                

        for obj in self.objects_keysort:
            if not obj in objects:
                del self.objects[obj]
        self.objects_keysort = sorted(self.objects.keys())

    def _set_hash(self, key, val):
        if key == "robots": self.robots_hash_state = val
        elif key == "objects": self.objects_hash_state = val
        elif key == "surfaces": self.surfaces_hash_state = val
        elif key == "maps": self.maps_hash_state = val
        else: Exception("Bad hash name" + key)
    def _get_hash(self, key):
        if key == "robots": return self.robots_hash_state
        elif key == "objects": return self.objects_hash_state
        elif key == "surfaces": return self.surfaces_hash_state
        elif key == "maps": return self.maps_hash_state
        else: Exception("Bad hash name" + key)

    def surface_scan(self, surface, object_types):
        s = self.surfaces[surface]
        for ot in object_types:
            s.scan.add(ot.strip())
        if self.consider_scan:
            self.clear_hash("surfaces")
        return True


    def set_robot_position(self, robot, position, val):
        if self.copy_on_write_optimize:
            self.robots[robot] = self.robots[robot].copy()
        if type(val) == type([]):
            for name, v in zip(self.robots[robot].robot_type.position_variables[position][False][0], val):
                if v != None:
                    self.robots[robot].positions[position][name] = float(v)
        elif type(val) == type({}):
            for name, v in val.iteritems():
                self.robots[robot].positions[position][name] = float(v)
        self.clear_hash("robots")
    
    def set_robot_location(self, robot, blast_pt):
        if self.copy_on_write_optimize:
            self.robots[robot] = self.robots[robot].copy()
        self.robots[robot].location = blast_pt.copy()
        self.clear_hash("robots")

    def robot_transfer_holder(self, robot, from_holder, to_holder):
        if self.copy_on_write_optimize:
            self.robots[robot] = self.robots[robot].copy()
        self.robots[robot].holders[to_holder] = self.robots[robot].holders[from_holder]
        self.robots[robot].holders[from_holder] = None
        self.robots[robot].reparent_objects(lambda x: self.get_obj(x), lambda x: self.copy_obj(x))
        self.clear_hash("robots")

    def robot_pick_object(self, robot, uid, to_holder):
        if self.copy_on_write_optimize:
            self.robots[robot] = self.robots[robot].copy()

        self.robots[robot].holders[to_holder] = BlastObjectRef(uid)
        self.robots[robot].reparent_objects(lambda x: self.get_obj(x), lambda x: self.copy_obj(x))
        self.gc_objects()
        self.clear_hash("robots")
        self.clear_hash("surfaces")

        
    def robot_place_object(self, robot, from_holder, surface, pos):
        if self.copy_on_write_optimize:
            self.robots[robot] = self.robots[robot].copy()
            self.surfaces[surface] = self.surfaces[surface].copy()

        uid = self.robots[robot].holders[from_holder].uid
        self.robots[robot].holders[from_holder] = None
        
        self.objects[uid].parent = surface
        self.objects[uid].position = pos.copy()
        self.surfaces[surface].objects.append(BlastObjectRef(uid))
        
        self.robots[robot].reparent_objects(lambda x: self.get_obj(x), lambda x: self.copy_obj(x))
        self.gc_objects()
        self.clear_hash("robots")
        self.clear_hash("surfaces")

    def set_robot_holder(self, robot, holder, object_type):
        if self.copy_on_write_optimize:
            self.robots[robot] = self.robots[robot].copy()
        if object_type != None:
            obj_type = self.types.get_object(object_type)
            obj = BlastObject(obj_type, None, robot + "." + holder)
            self.append_object(obj)
            self.robots[robot].holders[holder] = BlastObjectRef(obj.uid)
        else:
            self.robots[robot].holders[holder] = None
        self.gc_objects()
        self.robots[robot].reparent_objects(lambda x: self.get_obj(x), lambda x: self.copy_obj(x))
        self.clear_hash("robots")
    
    def clear_hash(self, key):
        self._set_hash(key, None)
        self.sumh = None

    def get_hash_state(self):
        if self.sumh: return self.sumh
        self.sumh = ""
        for field, arr, keysort in [("robots", self.robots, self.robots_keysort),
                                    ("surfaces", self.surfaces, self.surfaces_keysort),
                                    ("maps", self.maps, self.maps_keysort), ]:
            v = self._get_hash(field)
            if v == None:
                hl = hashlib.sha224()
                get_obj = lambda x: self.get_obj(x)
                for name in keysort:
                    hl.update(str(name))
                    arr[name].hash_update(hl, get_obj, self.consider_scan)
                v = hl.digest()
                self._set_hash(field, v)
            self.sumh = self.sumh + v
        return self.sumh

    def detect_bad_parenting(self):
        r = ""
        for name in self.robots_keysort:
            robot = self.robots[name]
            for holder in robot.robot_type.holders_keysort:
                if robot.holders[holder]:
                    if self.get_obj(robot.holders[holder].uid).parent != (name + "." + holder):
                        r = r + "stair4: " + holder + "\n"
        return r

    def equal(self, other, tolerant = False): #Tolerant of slight variations
        if self == other: return True
        if not self or not other: return False
        if type(self) != type(other): return False
        if self.__class__ != other.__class__: return False
        return self.equal_valid(other, tolerant)

    def equal_valid(self, other, tolerant = False): #Tolerant of slight variations
        if not tolerant: #Hash checks only in tolerant equals
            warn = True
            hash_same = self.get_hash_state() == other.get_hash_state()

            #Comment out these two lines to verify hashing
            if not hash_same: return False
            #return True #Comment this line to ensure, 100%, no hash collisions
            #This is to double-check to make sure hashing went as expected.
        else:
            #if self.get_hash_state() == other.get_hash_state(): return True
            
            warn = False
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
            if not (goal == False and type(goal) == type(True)):
                if state[name] == False and type(state[name]) == type(True):
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

        if type(condition[2]) == type(""):
            if obj == None: return False #Obviously needs an object yet none
            if condition[2] == obj.object_type.name: return True
            if condition[0] == "exact-contains": return False
            t = obj.object_type
            while t.parent:
                if t.name == condition[2]: return True
                t = t.parent
            return False
        else: #Motion constraints/tags
            if obj == None: return condition[2].get("accept_empty", False)
            for name, value in condition[2].iteritems():
                if name == "has_tag":
                    if type(value) == type(""):
                        value = [value,]
                    for tag in value:
                        if not tag in obj.object_type.tags:
                            return False
                elif name == "not_tag":
                    if type(value) == type(""):
                        value = [value,]
                    for tag in value:
                        if tag in obj.object_type.tags:
                            return False
                elif name in obj.object_type.motion_limits:
                    tval = obj.object_type.motion_limits[name]
                    in_region = False
                    for region in value:
                        if tval >= region[0] and tval <= region[1]:
                            in_region = True
                            break
                    if not in_region: return False
            return True
    
    def take_action(self, robot_name, action, parameters, execute = True, debug = False, failure_mode = None):

        location_do_not_cares = set()
        
        #Get the robot
        robot = self.robots.get(robot_name)
        if not robot:
            if debug: print "Invalid robot:", robot_name
            return None, None
        
        #Get action type
        action_robot_type, action_type = self.types.get_action_for_robot(robot.robot_type, action)
        if not action_robot_type or not action_type:
            if debug:
                print "Could not find the action type", action, "for", robot.robot_type
            return None, None

        #Ensure that we have all the proper parameters and convert strings to locations
        clone_param = False
        surface_parameters = set()
        for name, ptype in action_type.parameters.iteritems():
            if not name in parameters:
                if debug: print "Parameter", name, "unspecified"
                return None, None
            if ptype == "Pt" or ptype.find("Location:") == 0:
                if type(parameters[name]) == type({}):
                    if not clone_param:
                        parameters = parameters.copy() #Avoid mutating the original dictionary
                        clone_param = True
                    parameters[name] = BlastPt(float(parameters[name]["x"]), float(parameters[name]["y"]), 
                                               float(parameters[name]["a"]), parameters[name]["map"])
            elif ptype.find("Surface:") == 0:
                surface_parameters.add(name)
                if type(parameters[name]) == type("") or type(parameters[name]) == type(u""):
                    if not clone_param:
                        parameters = parameters.copy() #Avoid mutating the original dictionary
                        clone_param = True
                    parameters[name] = self.surfaces.get(parameters[name])
            elif ptype.find("SurfaceObject:") == 0:
                if type(parameters[name]) == type("") or type(parameters[name]) == type(u"") or type(parameters[name]) == type(0):
                    if not clone_param:
                        parameters = parameters.copy() #Avoid mutating the original dictionary
                        clone_param = True
                    if type(parameters[name]) == type(0):
                        if not parameters[name] in self.objects:
                            if debug: print "Non-existant object:", parameters[name]
                            return None, None
                        parameters[name] = BlastObjectRef(parameters[name])
                    elif parameters[name].strip().find("BlastObjectRef(") == 0:
                        parameters[name] = parameters[name].split("(")[1].strip().strip(')').strip()
                        try:
                            parameters[name] = int(parameters[name])
                        except:
                            if debug: print "Invalid parameter string:", parameters[name]
                            return None, None
                        if not parameters[name] in self.objects:
                            if debug: print "Non-existant object:", parameters[name]
                            return None, None
                        parameters[name] = BlastObjectRef(parameters[name])
                    else:
                        if debug: print "Invalid surface object:", parameters[name]
                        return None, None 
                    
                

        #Check all the conditions
        def get_value(value):
            if type(value) != type(""):
                return assert_condition(value)
            if value.strip()[0] == '"':
                return value.strip()[1:-1]

            subs = paren_split(value, '.')

            #print subs[0], parameters

            val = None
            if subs[0].find('(') != -1:
                func = subs[0][0:subs[0].find("(")].strip()
                args = paren_split(subs[0][subs[0].find("(")+1:subs[0].rfind(")")], ',')
                vals = [get_value(x) for x in args]
                if func == "dist":
                    #FIXME: type check, map same check
                    dx = float(vals[0].x) - float(vals[1].x)
                    dy = float(vals[0].y) - float(vals[1].y)
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
            return None, None
    
        time_estimate = get_value(action_type.time_estimate)
        if debug: print "Time estimate:", time_estimate
        if time_estimate == None:
            if debug: print "Time estimate invalid"
            return None, None

        #Conditions met, everything is done. Now we just need to update the values
        #create a queue of transations then execute them
        transactions = []
        items_to_clone = {"robots": set(), "surfaces": set()}
        robots_to_reparent = []
        
        for var, val in action_type.failure_modes.get(failure_mode, action_type.changes).iteritems():
            sub = var.split(".")
            if debug: print var, "->", val
            if sub[0] == "robot":
                items_to_clone["robots"].add(robot.name)
                if not hasattr(robot, sub[1]):
                    if debug: print "Attempt to set invalid robot attribute", sub[1]
                    return None, None
                if type(val) == type([1,]):
                    v = [get_value(x) if x not in [None, False, True] and type(x) != type(0.01) else x for x in val]
                else:
                    v = get_value(val)
                if v == None:
                    if debug: print "Invalid value for set:", val
                    return None, None
                if sub[1] == "holders":
                    if BLAST_NONE.equal(v):
                        v = None
                    transactions.append(("VAL", ("robot", robot.name, "holders"), sub[2], v)) 
                    if not robot.name in robots_to_reparent:
                        robots_to_reparent.append(robot.name)
                elif sub[1] == "positions":
                    if not sub[2] in robot.positions:
                        if debug: print "Invalid position for robot", robot.name, sub
                        return None, None
                    set_d = {}
                    if type(v) == type([1,]):
                        joint_names = robot.robot_type.position_variables[sub[2]][False][0]
                        if len(joint_names) != len(v):
                            if debug: print "Invalid number of joints for array based-update", sub[2]
                            return None, None
                        for joint_name, value in zip(joint_names, v):
                            if value == None:
                                set_d[joint_name] = robot.positions[sub[2]][joint_name]
                            else:
                                set_d[joint_name] = value
                    elif v == None or v == BLAST_NONE:
                        pass #The action is assumed to have left everything in the previous state
                    elif v == False and type(v) == type(True):
                        for joint_name in robot.robot_type.position_variables[sub[2]][False][0]:
                            set_d[joint_name] = False
                    else:
                        if debug: print "Invalid type for positions", v
                    if v != None and set_d != {}:
                        transactions.append(("VAL", ("robot", robot.name, "positions"), sub[2], set_d))
                elif sub[1].strip() == "location":
                    if (v != None and v != False and type(v) != BlastPrimitive):
                        transactions.append(("SET", ("robot", robot.name), sub[1], v))
                    else:
                        location_do_not_cares.add(robot.name)
                else:
                    transactions.append(("SET", ("robot", robot.name), sub[1], v))
            elif sub[0] in surface_parameters and len(sub) == 2 and sub[1].find("objects+") == 0:
                obj = get_value(val.split(":")[0])
                pos = get_value(val.split(":")[1])
                if type(pos) == type(""):
                    pos = (pos[0:pos.find(",")].strip(), pos[pos.find(",")+1:].strip())
                if type(pos[1]) == type(""):
                    v = [float(x.strip()) for x in pos[1].strip().strip("Pos()").strip().split(",")]
                    pos = (pos[0], BlastPos(v[0], v[1], v[2], v[3], v[4], v[5]))
                surface = get_value(sub[0])
                if surface.name != pos[0]:
                    if debug: print "Invalid surface", surface.name, "not", pos[0]
                    return None, None
                pos = pos[1]
                items_to_clone["surfaces"].add(surface.name)
                transactions.append(("ADDOBJ", ("surface", surface.name), obj, pos))
            elif sub[0] in surface_parameters and len(sub) == 2 and sub[1].find("objects-") == 0:
                obj = get_value(val)
                surface = get_value(sub[0])
                obj_s = self.get_obj(obj.uid).parent
                if surface.name != obj_s:
                    if debug: print "Invalid surface", surface.name, "not", obj_s
                    return None, None
                items_to_clone["surfaces"].add(surface.name)
                transactions.append(("DELETEOBJ", ("surface", surface.name), obj))
            elif sub[0] in surface_parameters and len(sub) == 2 and sub[1] == "scan":
                surface = get_value(sub[0])
                items_to_clone["surfaces"].add(surface.name)
                transactions.append(("SCAN", ("surface", surface.name), val))
            else:
                if debug: print "Update failed, invalid sub", sub
                return None, None

        if not execute:
            return time_estimate, None

        if self.copy_on_write_optimize:
            for robot_name in items_to_clone["robots"]:
                self.robots[robot_name] = self.robots[robot_name].copy()
            for surface_name in items_to_clone["surfaces"]:
                self.surfaces[surface_name] = self.surfaces[surface_name].copy()

        #Execute the transactions - this is where changes actually get made to the world
        for x in transactions:
            v = None
            #print x
            if x[1][0] == "robot":
                self.clear_hash("robots")
                v = self.robots[x[1][1]]
            elif x[1][0] == "surface":
                self.clear_hash("surfaces")
                v = self.surfaces[x[1][1]]
            else:
                print "Bad transaction", x
            for attr in x[1][2:]:
                v = getattr(v, attr)

            if x[0] == "SET":
                setattr(v, x[2], x[3])
            elif x[0] == "VAL":
                v[x[2]] = x[3]
            elif x[0] == "ADDOBJ":
                if self.copy_on_write_optimize:
                    self.objects[x[2].uid] = self.objects[x[2].uid].copy()
                obj = self.objects[x[2].uid]
                obj.parent = v.name
                obj.position = x[3]
                v.objects.append(BlastObjectRef(obj.uid))
            elif x[0] == "DELETEOBJ":
                if self.copy_on_write_optimize:
                    self.objects[x[2].uid] = self.objects[x[2].uid].copy()
                obj = self.objects[x[2].uid]
                obj.parent = None
                obj.position = None
            elif x[0] == "SCAN":
                [v.scan.add(t.strip()) for t in x[2].split(",")]
            else:
                print "Really bad, transaction bad:", x
                raise Exception("Bad transaction: " + str(x))

        for robot_name in self.robots_keysort: #robots_to_reparent:
            if self.robots[robot_name].reparent_objects(lambda x: self.get_obj(x), lambda x: self.copy_obj(x)):
                self.clear_hash("robots")
        self.gc_objects()

        return time_estimate, location_do_not_cares

    def enumerate_action(self, robot_name, action, extra_goals, debug = False):
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
        position_parameters = {}
        location_parameters = {}
        surfaceobject_parameters = {}
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
            elif ptype[0:len("SurfaceObject:")] == "SurfaceObject:":
                surfaceobject_parameters[param] = ptype.split(":")[1]
            elif ptype[0:len("Joint:")] == "Joint:":
                
                if param in self.types.parameter_values_cache:
                    parameters[param] = self.types.parameter_values_cache[ptype]
                else:
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
                    self.types.parameter_values_cache[ptype] = parameters[param]
            elif ptype[0:len("Pos:")] == "Pos:":
                vt = ptype.split(":")[1].split(",")
                va = [float(v.strip()) for v in vt[2:]]
                #TODO decide what to do about specified positions
                p = BlastPos(va[0], va[1], va[2], va[3], va[4], va[5])
                position_parameters[param] = (vt[0], vt[1], [p, ])
                
            else:
                print "Error, invalid parameter type:", ptype, "for:", param
        #Handle after all others parameterized
        for param, data in location_parameters.iteritems():
            parameters[param] = {}
            for surf in parameters[data[0]]:
                parameters[param][surf.name] = []
                for loc, pt in surf.locations.iteritems():
                    if loc.find(data[1]) == 0:
                        if test_loc(pt, param):
                            parameters[param][surf.name].append(pt)
            parameters[param] = ([data[0]], parameters[param])


        for param, data in position_parameters.iteritems():
            parameters[param] = {}
            for surf in parameters[data[0]]:
                parameters[param][surf.name] = []
                obj = None
                if data[1].strip().find("robot.holders.") == 0:
                    obj = robot.holders[data[1].strip().split(".")[2]]
                    if obj:
                        obj = self.objects[obj.uid]
                if not obj:
                    break

                extras = []
                extras.extend(extra_goals.get("Pos:SN:" + surf.name + ":" + obj.object_type.name, []))
                extras.extend(extra_goals.get("Pos:SU:" + surf.name + ":" + str(obj.uid), []))
                extras.extend(extra_goals.get("Pos:TN:" + surf.surface_type.name + ":" + obj.object_type.name, []))
                extras.extend(extra_goals.get("Pos:TU:" + surf.surface_type.name + ":" + str(obj.uid), []))
                extras.extend(extra_goals.get("Pos:N:" + obj.object_type.name, []))
                extras.extend(extra_goals.get("Pos:U:" + str(obj.uid), []))
                extras.extend(extra_goals.get("Pos:S:" + surf.name, []))
                extras.extend(extra_goals.get("Pos:T:" + surf.surface_type.name, []))

                for initial_pos in data[2] + extras:
                    w = 0
                    #TODO: this does not handle rotation down properly
                    wx = obj.object_type.motion_limits["bound_d"] + 0.01
                    wy = obj.object_type.motion_limits["bound_d"] + 0.01
                    while w < 30:
                        if w == 0:
                            test = [(0, 0)]
                        else:
                            test = []
                            #Make centric rectangle loops of size w around objects
                            yp = w
                            xp = 0
                            test.append((xp, yp))
                            while xp < w:
                                xp = xp + 1
                                test.append((xp, yp))
                            while yp > -w:
                                yp = yp - 1
                                test.append((xp, yp))
                            while xp > -w:
                                xp = xp - 1
                                test.append((xp, yp))
                            while yp < w:
                                yp = yp + 1
                                test.append((xp, yp))
                            while xp < -1:
                                xp = xp + 1
                                test.append((xp, yp))

                        for t in test:
                            p = initial_pos.copy()
                            p.x = p.x + wx * t[0]
                            p.y = p.y + wy * t[1]
                            
                            collision = False

                            #Does not handle rotation
                            for obj_cp in surf.objects:
                                obj_c = self.objects[obj_cp.uid]
                                dx = obj_c.position.x - p.x
                                dy = obj_c.position.y - p.y
                                dz = obj_c.position.z - p.z
                                cyl_dist = dx * dx + dy * dy
                                diam = obj_c.object_type.motion_limits['bound_d'] + obj.object_type.motion_limits['bound_d']
                                zh = obj_c.object_type.motion_limits['bound_h'] + obj.object_type.motion_limits['bound_h']
                                if (cyl_dist * 4 < diam * diam and (dz > -zh and dz < zh)):
                                    collision = True
                                    break
                            if not collision:
                                parameters[param][surf.name].append((surf.name, p))
                                w = 1000000
                                break
                        w = w + 1

            parameters[param] = ([data[0],], parameters[param])
            

        for param, data in surfaceobject_parameters.iteritems():
            parameters[param] = {}
            for surf in parameters[data]:
                parameters[param][surf.name] = [x for x in surf.objects]
            parameters[param] = ([data,], parameters[param])
        
        return action_type.planable, parameters
