#!/usr/bin/python
import math
import hashlib
import json
import cspsolver

T_STEP = 1000 #'s of a second.

if __name__ == '__main__': print "This is not intended to be a main file."

#FIXME: Permissions system

#CRITICAL NOTE: You cannot have "change" dependant on the state of
#               surfaces or upon have the workspace be state depndant.
#               They can be parameter dependant but nothing else.


BLAST_INFINITY = 1e10 #For objects
#FIXME: __ illegal in robot names

class BlastCodeError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class BlastParameterPtr(object):
    def __init__(self, parameter, sub = None, prefix = None, postfix = None):
        self.parameter = parameter
        self.sub = sub
        self.prefix = prefix
        self.postfix = postfix
    def __str__(self):
        sub = ""
        if self.sub != None: sub = ", " + str(self.sub)
        return "ParameterPtr(\"" + str(self.parameter) + sub + "\")"
    def __repr__(self): return self.__str__()



class BlastCodeStep(object):
    #Valid commands: STARTSUB, CALLSUB, GOTO, ENDSUB, IF, RETURN, FAIL, PLAN, SCAN, SCAN_STATE
    #STARTSUB indicates the name of a subroutine. It takes an arbitrary set of parameters
    #which are the names of types for the parameters of the subroutine. The main purpose of
    #this command is that when we hit it we transform into another routine. Executing this
    #command without a CALLSUB will cause the program to terminate in error.

    #CALLSUB calls out to another subroutine (a STARTSUB label) and runs that subroutine.
    #It sets a return_var to be a boolean with True or False based on failure of that
    #subroutine. The subroutine returns True if it reaches a "RETURN" and returns false if
    #it reaches a FAIL or if during runtime an action fails.

    #GOTO simply has one argument, label, and jumps to that label which must be in the
    #current subroutine.

    #IF takes condition, label_true, and label_false. Condition is a set of tuples
    #prefixed with <, >, >=, <=, ==, !=, &&, ||, or ! at 0th element. The other elements 
    #are tuples or BlastParameterPtr objects. The arguments label_true and label_false 
    #are steps to jump to in the current subroutine. If label_false is not set or is 
    #None then the system will simply step forward into the next set of actions.

    #ENDSUB represents the end of the current subroutine. If we get here we succeed.
    #FAIL causes the current subroutine to fail.
    #RETURN causes the current subroutine to succeed.
    
    #PLAN will plan out an action. Parameters are extra_steps, a set of (robot, action, 
    #parameters), extra_goals, a set of extra values for parameters, and world_limits, 
    #a set of constraints defined on the world. The system will first achieve a setup that 
    #meets the world_limits, and then try to perform the extra steps. Remeber that that 
    #PLAN failure during the planning phase are extremely costly (as the system will get 
    #hung up waiting for them to finish) so please avoid this as much as possible. 
    #Failures during replanning are not costly because the system will try to achieve 
    #the planned goal first before anything else.
    
    #There is also another flag parameter which is assume_failure. If this is set, the
    #planning process will always assume failure, and replan when the action hits. This
    #should be used sparingly, but is for conditionally dependant actions, such as when
    #hunting for objects. Generally you should never use this and using it in the store
    #will flag your application for extra review.

    #SCAN sets if we are considering scans and resets surface scan state. Note I 
    #personally do not like scan state but consider it a necessary evil and will be
    #getting rid of it as soon as I can. Using this flags you for extra review.
    #It takes to parameters - one is 'consider' a boolean enabling or disabling
    #scan state and the other is 'reset' a CSV string of things to reset.


    def acceptable_parameters(self, s):
        for param in self.parameters:
            if str(param) not in s:
                raise BlastCodeError("Parameter " + param + " is invalid for command " + self.command)

    def copy(self, prefix = "", change_names = []):
        pc = self.parameters.copy()
        def rp(c):
            print c, pc.get(c), pc.get(c) in change_names, change_names
            if c in pc:
                if pc[c] in change_names:
                    pc[c] = prefix + pc[c]
        if self.command == "IF":
            rp("label_true")
            rp("label_false")
        elif self.command == "CALLSUB":
            rp("sub")
        elif self.command == "GOTO":
            rp('label')

        lb = None
        if self.label: lb = prefix + self.label
        return BlastCodeStep(lb, self.command, pc, self.return_var)

    def __init__(self, label, command, parameters = {}, return_var = None):
        self.label = label
        self.command = str(command)
        self.parameters = parameters
        self.return_var = return_var
        if command == "STARTSUB":
            for pname, ptype in parameters.iteritems():
                pass
            if return_var != None:
                raise BlastCodeError("STARTSUB has no return")
            if type(label) != str:
                raise BlastCodeError("Need a label for STARTSUB")
            if label.strip() == "":
                raise BlastCodeError("Need a label for STARTSUB")
        elif command == "CALLSUB":
            #self.acceptable_parameters(['sub'], False)
            if not "sub" in parameters:
                raise BlastCodeError("Need 'sub' in parameters of CALLSUB")
            parameters['sub'] = str(parameters["sub"]).strip()
            if parameters["sub"] == "":
                raise BlastCodeError("Need 'sub' to be non-empty for CALLSUB")
        elif command == "GOTO":
            self.acceptable_parameters(['label'])
            if not "label" in parameters:
                raise BlastCodeError("Need 'label' in parameters of GOTO")
            parameters['label'] = str(parameters["label"]).strip()
            if parameters["label"] == "":
                raise BlastCodeError("Need 'label' to be non-empty for GOTO")
        elif command == "RETURN":
            self.acceptable_parameters([])
        elif command == "FAIL":
            self.acceptable_parameters([])
        elif command == "ENDSUB":
            self.acceptable_parameters([])
        elif command == "PLAN":
            self.acceptable_parameters(['world_limits', 'extra_steps', 'extra_goals', 'assume_failure'])
            #TODO: work it out
        elif command == "GETOBJECT":
            self.acceptable_parameters(['holder'])
        elif command == "IF":
            self.acceptable_parameters(['condition', 'label_true', 'label_false'])
            #TODO: work it out
        elif command == "SCAN":
            self.acceptable_parameters(["reset", "consider"])
        elif command == "SCAN_STATE":
            self.acceptable_parameters(["types"])
        elif command == "SCAN_MAX":
            self.acceptable_parameters(["types"])
        else:
            raise BlastCodeError("Invalid command: '" + command + "'")
            


hunt = [BlastCodeStep("hunt_objects", "STARTSUB", {'holder': 'holder', 'object_types': 'string'}),
        #Try to grab currently existing objects. This is assumed to fail
        BlastCodeStep(None, "PLAN", {'world_limits': 
                                     {'robot-holders': {BlastParameterPtr('holder', 0):
                                                            {BlastParameterPtr('holder', 1): BlastParameterPtr('object_types', prefix='TYPES:')}}},
                                     'assume_failure': True}, "plan_return"),
        BlastCodeStep(None, "IF", {"condition": BlastParameterPtr('plan_return'), 'label_true': 'win_hunt'}),
        
        BlastCodeStep(None, 'SCAN', {"reset": BlastParameterPtr('object_types')}),

        BlastCodeStep("scan_loop", 'SCAN_STATE', {'types': BlastParameterPtr('object_types')}, "n_scans_done"),
        BlastCodeStep(None, 'SCAN_MAX', {'types': BlastParameterPtr('object_types')}, "n_scans_max"),
        BlastCodeStep(None, "IF", {'condition': ('<=', BlastParameterPtr('n_scans_done'), BlastParameterPtr('n_scans_max')),
                                   'label_true': 'scan_next', 'label_false': 'exit_function'}),
        BlastCodeStep('scan_next', "PLAN", {'world_limits':
                                                {'scans': [(BlastParameterPtr('object_types'), '>', 
                                                           BlastParameterPtr('n_scans_done')),]}}, 'plan_return'),
        BlastCodeStep(None, "IF", {"condition": BlastParameterPtr('plan_return'), 
                                   'label_true': 'scan_loop', 'label_false': 'fail_hunt'}),
        #End main loop
        BlastCodeStep('exit_function', "PLAN", {'world_limits':
                                                    {'scans': [(BlastParameterPtr('object_types'), '>', 
                                                                BlastParameterPtr('n_scans_max')),]},
                                                'assume_failure': True}, 'plan_return'),
        BlastCodeStep(None, "IF", {"condition": BlastParameterPtr('plan_return'), 'label_true': 'win_hunt'}),
        BlastCodeStep('fail_hunt', 'FAIL'),
        BlastCodeStep('win_hunt', 'RETURN'),
        BlastCodeStep(None, 'ENDSUB'),
        ]


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
                 'changes', 'display', 'planable', 'user', 'failure_modes', 'workspaces', 'is_object_action']

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
                 changes, display, workspaces, planable = True, user = False, fm = {}): #Name must be robot_type.action
        self.name = name
        self.robot = name.split(".")[0]
        self.parameters = parameters
        self.display = display
        self.condition = condition
        self.workspaces = workspaces
        self.failure_modes = fm
        self.is_object_action = False

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
                if var.find("robot.holders.") == 0 or var.split(".")[1] == "scan":
                    self.is_object_action = True

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
    __slots__ = ['surfaces', 'robots', 'objects', 'actions', 'parameter_values_cache', 'robot_action_cache', 'script', 'script_indexes', 'action_for_robot_cache']
    def __init__(self):
        self.script = []
        self.script_indexes = {}
        self.surfaces = {}
        self.robots = {}
        self.objects = {}
        self.actions = {}
        self.parameter_values_cache = {}
        self.robot_action_cache = {}
        self.action_for_robot_cache = {}
        self.add_script(hunt)

    def enumerate_robot(self, robot, require_object = False):
        if require_object == False:
            rc = self.robot_action_cache.get(robot, None)
        else:
            rc = None
        if rc != None:
            return rc
        types = set()
        rt = self.robots[robot]
        while rt:
            types.add(rt.name)
            rt = rt.parent
        
        actions = []
        for name, at in self.actions.iteritems():
            rt = name.split(".")
            if rt[0] in types:
                if require_object == False or at.is_object_action:
                    if at.planable:
                        actions.append(rt[1])
        if require_object == False:
            self.robot_action_cache[robot] = actions
        return actions

    def add_script(self, s):
        for step in s:
            i = len(self.script)
            if step.label != None:
                if step.label in self.script_indexes:
                    raise BlastCodeError("Already have a label called '" + step.label + "'")
                self.script_indexes[step.label] = i
            self.script.append(step)

    def add_surface_type(self, surface):
        self.parameter_values_cache = {}
        if surface.name in self.surfaces: raise BlastTypeError("Duplicate surface type: " + surface.name)
        self.surfaces[surface.name] = surface
    def get_surface(self, name):
        return self.surfaces.get(name, None)
    
    def add_robot_type(self, robot):
        self.action_for_robot_cache = {}
        self.parameter_values_cache = {}
        self.robot_action_cache = {}
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
        self.action_for_robot_cache = {}
        self.parameter_values_cache = {}
        self.robot_action_cache = {}
        if action.name in self.actions: raise BlastTypeError("Duplicate action type: " + action.name)
        self.actions[action.name] = action
    def get_action(self, name):
        return self.actions.get(name, None)

    def get_action_for_robot(self, robot_type, action):
        cv = self.robot_action_cache.get(robot_type, None)
        if cv:
            cv = cv.get(action, None)
            if cv:
                return cv
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

        if not robot_type in self.robot_action_cache:
            self.robot_action_cache[robot_type] = {}
        self.robot_action_cache[robot_type][action] = (action_robot_type, action_type)
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
    def hash_update(self, hl):
        hl.update("PosIrr")

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
        consider_scan = True #Hack
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

            if type(oa.position) == BlastPosIrr and type(ob.position) == BlastPosIrr:
                return a.uid - b.uid
            if type(oa.position) == BlastPosIrr: return -1
            if type(ob.position) == BlastPosIrr: return 1
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
    def __str__(self): return self.to_text()
    def __repr__(self): return self.to_text()
    def to_text(self):
        return "BlastObjectRef(" + str(self.uid) + ")"
    

class BlastRobot(object):
    __slots__ = ['name', 'robot_type', 'location', 'holders', 'positions']
    def __init__(self, name, location, robot_type, do_setup = True):
        self.name = name
        self.robot_type = robot_type
        self.location = location
        self.holders = {}
        self.positions = {}
        if not do_setup: return
        for name in self.robot_type.holders.iterkeys():
            self.holders[name] = None
        for name, d in self.robot_type.position_variables.iteritems():
            if d:
                self.positions[name] = {}
                for i in xrange(0, len(d[False][0])): #Loop through names, etc
                    self.positions[name][d[False][0][i]] = d[False][2][i] #Set to default
            else:
                self.positions[name] = False

    def collide(self, other, loc = None): #TODO actually compare
        if not loc: loc = self.location
        return other.location.equal(loc)

    def copy(self):
        copy = BlastRobot(self.name, self.location.copy(), self.robot_type, do_setup = False)
        copy.holders = self.holders.copy()
        copy.positions = self.positions.copy()
        for name in self.positions.iterkeys():
            if self.positions[name]:
                copy.positions[name] = self.positions[name].copy()
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

paren_split_cache = {}

def paren_split(value, delim):
    if not delim in paren_split_cache:
        paren_split_cache[delim] = {}
    if not value in paren_split_cache[delim]:
        paren_split_cache[delim][value] = paren_split_i(value, delim)
    return paren_split_cache[delim][value]


def paren_split_i(value, delim):
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
        self.consider_scan = True #TODO: turning this off might make things more efficient.
        self.copy_on_write_optimize = True

    def enumerate_robot(self, robot, require_object = False):
        return self.types.enumerate_robot(self.robots[robot].robot_type.name, require_object = require_object)

    def delete_surface_object(self, obj):
        if self.copy_on_write_optimize:
            self.objects[obj] = self.objects[obj].copy()
        self.objects[obj].parent = None
        self.gc_objects()
        return True

    def clear_scan(self, to_value = {}):
        for sn in self.surfaces_keysort:
            if len(self.surfaces[sn].scan) != 0:
                if self.copy_on_write_optimize:
                    self.surfaces[sn] = self.surfaces[sn].copy()
                self.surfaces[sn].scan = to_value.get(sn, set()).copy()
                self.clear_hash("surfaces")

    def scan_count(self, ot):
        c = 0
        for name, surf in self.surfaces.iteritems():
            if ot in surf.scan:
                c = c + 1
        return c

    def get_scan_actions_surfaces(self):
        result_dir = {}
        for at_name, at in self.types.actions.iteritems():
            for var, value in at.changes.iteritems():
                if var.find(".scan") != 0 and var.find(".") == var.find(".scan"):
                    varname = var.split(".")[0]
                    stype = at.parameters[varname].strip().split(":")[1].strip()
                    result_dir[stype] = result_dir.get(stype, {})
                    for ot in value.split(","):
                        s = result_dir[stype].get(ot, set())
                        s.add(at_name)
                        result_dir[stype][ot] = s
        return result_dir

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

    def get_scan_state(self):
        ss = {}
        for name, surface in self.surfaces.iteritems():
            ss[name] = surface.scan.copy()
        return ss

    def count_surface_scans(self, ots):
        count = 0
        for name, surface in self.surfaces.iteritems():
            good = True
            for ot in ots:
                if not ot in surface.scan:
                    good = False
            if good: count = count + 1
        return count


    
    def world_limit_check(self, limits):
        acceptable = set(["robot-holders", "robot-location", 'scans', 'place-objects'])
        for l in limits:
            if l not in acceptable:
                raise BlastCodeError("Bad limit parameter: " + str(l) + " in " + str(limits))
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
                        if str(set_as).find("TYPES:") == 0:
                            if self.get_obj(int(value.uid)).object_type.name not in set_as[len("TYPES:"):]:
                                return False
                        elif int(value.uid) != int(set_as):
                            return False
                    #We don't need to worry about the value == None and set_as == None case
        if "robot-location" in limits:
            for robot_name, value in limits["robot-location"].iteritems():
                if not robot_name in self.robots:
                    return False
                pt = value
                if type(pt) != BlastPt:
                    pt = BlastPt(value['x'], value['y'], value['a'], value['map'])
                if not self.robots[robot_name].location.equal(pt):
                    return False
        if "scans" in limits:
            for s in limits['scans']:
                if len(s) != 3:
                    raise BlastCodeError("Invalid settings to scan limits for: " + str(s) + " need object type, (<,>,>=,<=,==) and number.")
                cm = self.count_surface_scans([p.strip() for p in s[0].split(",")])
                val = int(s[2])
                if s[1] == '==' and (not (cm == val)): return False
                if s[1] == '<=' and (not (cm <= val)): return False
                if s[1] == '>=' and (not (cm >= val)): return False
                if s[1] == '>' and (not (cm > val)): return False
                if s[1] == '<' and (not (cm < val)): return False
        if "place-objects" in limits:
            for obj in limits['place-objects']:
                objr = None
                for n in obj:
                    if not n in ["object", "surface", "position"]:
                        raise BlastCodeError("Invalid settings for place-objects: " + str(obj) + " - " + n + " is not a valid atttribute")

                if 'object' in obj:
                    objr = obj['object']
                    if type(objr) == BlastObjectRef:
                        objr = objr.uid
                    if type(objr) == int:
                        objr = self.get_obj(objr)
                else:
                    raise BlastCodeError("Invalid settings for place-objects: " + str(obj) + " has no object tag")
                if type(objr) != BlastObject:
                    raise BlastCodeError("Invalid settings for place-objects: " + str(obj) + " object is not found to be an object")

                if 'surface' in obj:
                    surf = obj['surface']
                    if type(surf) == BlastSurface: surf = surf.name
                    if not objr.parent == surf:
                        return False
                
                if 'position' in obj:
                    pos = obj['position']
                    if type(pos) == str:
                        pos = pos.strip("ABCDEFGHIJKLMNOPQRSTUVWXYZ \t,\n\rabcdefghijklmnopqrstuvwxyz()[]{}_\+=;\"\'")
                        pos = [float(x) for x in pos.split(",")]
                        if len(pos) != 6:
                            raise BlastCodeError("Invalid settings for place-objects: " + str(obj) + " has an invalid position string (need 6 values)")
                        pos = BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                    if type(pos) != BlastPos:
                        raise BlastCodeError("Invalid settings for place-objects: " + str(obj) + " has an invalid position")
                    if not pos.equal(objr.position):
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
        if condition[1][0:len("robot.")] != "robot.":
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
        if type(condition[2]) == str:
            if not condition[2] in typ:
                if debug: print "Invalid pre-defined position state", condition[2]
                return None 
            goal_array = typ[condition[2]]
        elif type(condition[2]) == list:
            if len(condition[2]) != len(name_array):
                if debug: print "Wrong length for goal array", condition[2], "for", name_array
                return None
            goal_array = condition[2]
        else:
            if debug: print "Invalid type for position state compare", condition[2]
            return None

        for name, goal, tol in zip(name_array, goal_array, tol_array):
            if goal == None: continue
            if state[name] == False and type(state[name]) == bool:
                if super_debug: print state[name], name, "failed"
                return False
            elif goal - tol > state[name] or goal + tol < state[name]:
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

    def clean_parameters(self, action_type, parameters, debug, accept_unset = False):
        #Ensure that we have all the proper parameters and convert strings to locations
        clone_param = False
        surface_parameters = set()
        for name, ptype in action_type.parameters.iteritems():
            if not name in parameters:
                if accept_unset: continue
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
        return parameters, surface_parameters

    def enumerate_robot_place(self, robot_name):
        res = {}
        for action_name in self.enumerate_robot(robot_name):
            rt = self.robots[robot_name].robot_type
            while rt != None:
                if (rt.name + "." + action_name) in self.types.actions:
                    break
                rt = rt.parent
            at = self.types.actions[rt.name + "." + action_name]

            rz = {}
            for var, value in at.changes.iteritems():
                if var.find("+") != -1:
                    surface_p = var.split(".")[0].strip()
                    pos_p = value.split(":")[1].strip()
                    holder = value.split(":")[0].strip()
                    if holder.find("robot.holders.") != 0:
                        raise Exception("Invalid holder for object: " + var + " <-" + value)
                    if surface_p not in at.parameters:
                        raise Exception("Invalid surface for object: " + var + " <- " + value)
                    if pos_p not in at.parameters:
                        raise Exception("Invalid position for object: " + var + " <- " + value)
                    holder = holder.split(".")[2].strip()
                    if holder in rz:
                        raise Exception("Double place from holder: " + var + " <- " + value)
                    rz[holder] = (surface_p, pos_p)
            if rz != {}:
                res[action_name] = rz
        return res


    def action_robot_pose(self, robot_name, action, parameters):
        debug = True
        accept_unset = False
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
        
        parameters, surface_parameters = self.clean_parameters(action_type, parameters, debug, accept_unset = True)
        if parameters == None or surface_parameters == None: return None

        csp_var = 0
        csp = [("var_0", "==", True), ]

        all_locations = [robot.location ]

        for pname, ptype in action_type.parameters.iteritems():
            if ptype == "Pt" or ptype.find("Location:") == 0:
                #FIXME LOCATION
                csp.append((pname, "==", all_locations))
            elif ptype.find("Surface:") == 0:
                st = ptype.split(":")[1]
                ss = []
                for n, s in self.surfaces.iteritems():
                    if s.surface_type.name == st:
                        ss.append(s)
                csp.append((pname, "==", ss))
            elif ptype.find("SurfaceObject:") == 0:
                csp.append((pname, "surfaceobject", ptype.split(":")[1]))
            elif ptype.find("Pos:") == 0:
                txt = ptype.split(":")[1].split(",")
                csp.append((pname, "pos", txt[0].strip(), txt[1].strip()))
            else:
                raise Exception("Invalid parameter type: " + str(ptype) + " for " + str(pname))
            
            

        def eval_condition(c, csp, csp_var):
            this_var = "var_" + str(csp_var)
            csp_var = csp_var + 1
            if c == "True()":
                csp = csp + [(this_var, "==", True),]
            elif type(c) == int or type(c) == float or type(c) == long or type(c) == bool or c == None:
                csp = csp + [(this_var, "==", c),]
            elif (c[0] == "contains" or c[0] == "exact-contains"
                  or c[0] == "not-contains" or c[0] == "not-exact-contains"):
                r = self.robot_contains_condition(robot, c)
                csp = csp + [(this_var, "==", r),]
            elif (c[0] == "&&" or c[0] == "||" or c[0] == "==" or c[0] == "!=" or c[0] == "not"):
                opmap = {"&&": "&&", "||": "||", "==": "===", "!=": "!==", "not": "not"}
                n = [this_var, opmap[c[0]]]
                for i in xrange(1, len(c)):
                    app_var, csp, csp_var = eval_condition(c[i], csp, csp_var)
                    n.append(app_var)
                csp = csp + [tuple(n), ]
            elif c == "robot.location":
                csp = csp + [(this_var, "==", "robot.location"),]
            elif type(c) == str and c.split(".")[0] in action_type.parameters:
                csp = csp + [tuple([this_var, "extract"] + c.split(".")),]
            else:
                raise Exception("Invalid condition: " + str(c))
            return this_var, csp, csp_var
            
        for param, value in parameters.iteritems():
            csp.append((param, "==", [value,]))

        #print csp

        this_var, csp, csp_var = eval_condition(action_type.condition, csp, csp_var)

        #print robot_name, action, parameters
        #for c in csp:
        #    print c

        output = []
        for d in cspsolver.solvecsp(csp):
            od = {}
            pd = {}

            for n in action_type.parameters:
                od[n] = d[n][0]
            for n in ['robot.location']:
                if n in d:
                    pd[n] = d[n][0]
            output.append((od, pd))
        
        return output
        

    def take_action(self, robot_name, action, parameters, execute = True, debug = False, failure_mode = None):
        if debug: print robot_name, action, parameters

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

        
        parameters, surface_parameters = self.clean_parameters(action_type, parameters, debug)
        if parameters == None or surface_parameters == None: return None, None

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
        time_estimate = int(math.ceil(float(time_estimate) * T_STEP))
        if time_estimate < 1: time_estimate = 1

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
                if pos[1] == False:
                    pos = (pos[0], BlastPosIrr())
                if type(pos[1]) == type(""):
                    v = [float(x.strip()) for x in pos[1].strip().strip("Pos()").strip().split(",")]
                    pos = (pos[0], BlastPos(v[0], v[1], v[2], v[3], v[4], v[5]))
                if type(pos[0]) == BlastSurface:
                    pos = (pos[0].name, pos[1])
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

    #Note this function does not use any parameters of the world, just
    #the types world. This is important for simulation reasons.
    def get_workspaces(self, robot_name, action, parameters):
        #Get the robot
        robot = self.robots.get(robot_name)
        if not robot:
            print "Invalid robot:", robot_name
            return None, None
        
        #Get action type
        action_robot_type, action_type = self.types.get_action_for_robot(robot.robot_type, action)
        if not action_robot_type or not action_type:
            print "Could not find the action type", action, "for", robot.robot_type
            return None, None
        
        result_dictionary = {}
        result_array = []
        for variable, locs in action_type.workspaces.iteritems():
            if variable != None:
                if not variable in parameters:
                    print "Invalid workspace result:", variable, "not in parameters"
                    return None, None
                for loc in locs:
                    loc_split = loc.split(".")
                    if len(loc_split) == 3 and loc_split[0] in parameters and loc_split[1] == "locations":
                        lfc = parameters[loc_split[0]]
                        if type(lfc) != BlastSurface: lfc = self.surfaces[lfc]
                        lf = lfc.locations[loc_split[2]]
                        result_dictionary[lfc.name] = result_dictionary.get(lfc.name, [])
                        result_dictionary[lfc.name].append(lf)
                        result_array.append(lf)
                    elif len(loc_split) == 1 and loc_split[0] in parameters:
                        lfc = parameters[variable]
                        if type(lfc) != BlastSurface: lfc = self.surfaces[lfc]
                        
                        lf = parameters[loc_split[0]]
                        if type(lf) == dict:
                            lf = BlastPt(lf['x'], lf['y'], lf['a'], lf['map'])
                        if type(lf) != BlastPt:
                            print "Direct parameter location failed", lf
                            return None, None
                        result_dictionary[lfc.name] = result_dictionary.get(variable, [])
                        result_dictionary[lfc.name].append(lf)
                        result_array.append(lf)
                    else:
                        print "Invalid workspace result:", loc, "is bad location"
                        return None, None
            else:
                for loc in locs:
                    if loc in parameters:
                        result_array.append(parameters[loc])
                    else:
                        print "Invalid workspace result:", loc, "location is not in parameters"
                        return None, None
        return result_dictionary, result_array
        
    def robots_coliding(self):
        for i in xrange(0, len(self.robots_keysort)):
            r1 = self.robots[self.robots_keysort[i]]
            for j in xrange(i + 1, len(self.robots_keysort)):
                r2 = self.robots[self.robots_keysort[j]]
                if r1.collide(r2): return True
        return False

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
                parameters[param][surf.name].append((surf.name, BlastPosIrr()))

            if False:
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
