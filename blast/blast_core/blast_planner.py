
import blast_world, time, json, itertools, hashlib, random, string, threading

#### Planning process
# 1. Plans are made from macro calls in the macro code structure
# 2. Plans are then executed by the reasoner.
#


def all_combinations(c, min, len):
    for r in xrange(min, len + 1):
        for i in itertools.combinations(c, r):
            yield i

def get_action_workspaces(robot, action, parameters):
    return {}

def get_workspaces(actions, time):
    surfaces = {}
    for action in action:
        if action[0] <= time and action[0] + action[1] >= time and action[2]:
            sr = get_action_workspaces(action[2][0], action[2][1], action[2][2])
            for s, vs in sr.iteritems():
                surfaces[s] = surfaces.get(s, set())
                for v in vs:
                    surfaces[s].add(v)
    return surfaces

def world_times(world):
    times = {}
    for robot in world[0].robots_keysort: times[robot] = 0
    for step in world[1]:
        r = step[2]
        if type(r) == tuple: r = r[0]
        times[r] = step[0] + step[1]
    return times
            
def lowest_t(world):
    return smallest_t(world_times(world))

def smallest_t(t):
    return min(t.itervalues())
def largest_t(t):
    return max(t.itervalues())

class BlastPlanStep(object):
    __slots__ = ['robot', 'action', 'parameters', 'initial_states',
                 'final_states', 'locs', 'id', 'gated_on']

    def __init__(self, robot, action, parameters, initial_states, final_states, locs, uid_blacklist, step_uid):
        if True:
            if len(initial_states) != len(final_states):
                raise Exception("Cannot have wrong keys in states")
            for i in initial_states:
                if not i in final_states:
                    raise Exception("Cannot have initial states in final states")
        self.robot = robot
        self.action = action
        self.parameters = parameters
        self.initial_states = initial_states
        self.final_states = final_states
        self.locs = locs
        self.gated_on = set()
        while True:
            self.id = str(step_uid) + ''.join(random.choice(string.ascii_uppercase + string.digits) for x in range(10))
            if not self.id in uid_blacklist:
                break

    def tuple(self):
        return (self.robot, self.action, self.parameters, self.initial_states, self.final_states,
                self.id)
    def __str__(self):
        return str(self.tuple())
    def __repr__(self):
        return str(self.tuple())

class Planner(object):
    __slots__ = ['initial_world', 'worlds', 'world_good', 'planned_worlds', 'good_worlds', 'code_exec',
                 'time_limit', 'worlds_tested', 'actions_tested', 'actions_finished', 'extra_goals', 
                 'action_type_debug', 'fail_debug', 'super_fail_debug', 'best_world', 'step_uid', 'extra_steps',
                 'code_exec', 'total_time_limit']

    def compare_worlds(self, wa, wb):
        if len(wa[1]) != len(wb[1]):
            return False
        for ac, bc in zip(wa[1], wb[1]):
            a = ac[2]
            b = bc[2]
            if a[0] != b[0]: return False
            if a[1] != b[1]: return False
            if type(a[2]) != type(b[2]): return False
            if type(a[2]) == str and a[2] != b[2]: return False
            if type(a[2]) == type({}) and type(b[2]) == type({}):
                if len(a[2]) != len(b[2]):
                    return False
                for k in a[2]:
                    if not k in b[2]:
                        return False
                    if type(b[2][k]) != type(a[2][k]):
                        return False
                    if type(b[2][k]) == blast_world.BlastPt or type(b[2][k]) == blast_world.BlastPos:
                        if not b[2][k].equal(a[2][k]):
                            return False
                    elif type(b[2][k]) == blast_world.BlastSurface:
                        if b[2][k].name != a[2][k].name:
                            return False
                    elif type(b[2][k]) == tuple:
                        if len(a[2][k]) != len(b[2][k]):
                            return False
                        if len(b[2][k]) == 2:
                            if type(a[2][k][0]) == type("") and type(b[2][k][0]) == type("") \
                                    and type(a[2][k][1]) == blast_world.BlastPos \
                                    and type(b[2][k][1]) == blast_world.BlastPos:
                                return False
                    elif type(b[2][k]) == type({}) or type(b[2][k]) == type([]):
                        raise Exception("Sucky value: " + str(b[2][k]))
                    elif b[2][k] != a[2][k]:
                        return False
        return True
    
    def compare_workspaces(self, comp, c):
        for name in c[0].keys():
            if name in comp[0]:
                return True
        for loc in c[1]:
            for loc_c in comp[1]:
                if loc_c.equal(loc):
                    return True
        return False

    def __init__(self, initial_world, code_exec = []):
        self.initial_world = initial_world
        self.worlds = [(initial_world, [], [], [x.copy() for x in code_exec])]
        for w in self.worlds:
            for prog in w[3]:
                prog.execute(w[0])
        self.world_good = lambda x: False
        self.planned_worlds = {} #dictionary of sets for each plan state given the already run worlds.
        self.good_worlds = []
        self.time_limit = None
        self.total_time_limit = None
        self.worlds_tested = 0
        self.actions_tested = 0
        self.actions_finished = 0
        self.step_uid = ""
        self.extra_goals = {}
        self.code_exec = None

        self.action_type_debug = {}
        self.fail_debug = {}
        self.super_fail_debug = False

        self.best_world = None

    def check_plan_state(self, w, state):
        if "world_limits" in state:
            if not w.world_limit_check(state["world_limits"]):
                return False
        return True

    #None = invalid world
    #False = not valid
    def evaluate_world(self, world, blacklist = None, debug = False):

        w = world[0].copy() #Initial world
        time = 0
        
        world[1].sort(key = lambda x: x[0])
        world[2].sort(key = lambda x: x[0])

        is_valid = True

        robot_last_times = world_times(world)
        end_of_plan = smallest_t(robot_last_times)

        w_state_ls = []
        w_state_hash = ""
        self.planned_worlds[w_state_hash] = self.planned_worlds.get(w_state_hash, set())

        for action in sorted(world[1], key = lambda x: x[0] + x[1]):
            for state in world[2]:
                if state[0] >= time and state[0] <= action[0]:
                    if not self.check_plan_state(w, state[1]):
                        #If we have this invalid state in a time when we
                        #still can plan (a rough plan), we still can and
                        #should plan based on this one. However, we can
                        #never have a complete plan at this point.
                        if action[0] < end_of_plan or time < end_of_plan:
                            is_valid = False
                            if debug: print "Made invalid by invalid state at t =", state[0]
                        else:              
                            if debug: print "Declared unrecoverable by invalid state at t =", state[0]
                            return None
            if type(action[2]) != str and type(action[2]) != tuple and blacklist != None:
                w_state_ls.append(action)
                hl = hashlib.sha224()
                hs = {}
                for t, c, s in w_state_ls:
                    hs[s.uid] = s.get_hash_state()
                for k in sorted(hs.keys()):
                    hl.update(hs[k])
                w_state_hash = hl.digest()
                self.planned_worlds[w_state_hash] = self.planned_worlds.get(w_state_hash, set())
                continue
            if type(action[2]) != tuple: continue
            time = action[0]
            change, ldc = w.take_action(action[2][0], action[2][1], action[2][2])
            if change == None:
                #If we fail to execute an action after all the plans are made
                #then we have an invalid plan because we cannot ever repair it.
                if action[0] <= end_of_plan or True:
                    if debug: print "Declared unrecoverable by action failure", action[2]
                    return None
                if debug: print "Declared invalid by action failure", action[2]
                is_valid = False


            #Test for workspace conflict. Sadly this needs to be done after computation
            #of the action so we know the time the action takes.
            work_conflict = False
            comp = w.get_workspaces(action[2][0], action[2][1], action[2][2])
            for step in world[1]:
                if type(step[2]) != tuple: continue
                if step == action: continue
                if not step[0] >= action[0] + change and not step[0] + step[1] <= action[0]:
                    if self.compare_workspaces(comp, w.get_workspaces(step[2][0], step[2][1], step[2][2])):
                        #No good, a coliding world can't be allowed. There's no way to add
                        #additional steps to get rid of the problem so we quit.
                        is_valid = False
                        if debug: print "Declared unrecoverable due to workspace overlap"
                        return None
            
            #Blacklist stuff
            if blacklist == action and blacklist != None:
                if w.get_hash_state() in self.planned_worlds[w_state_hash]:
                    return None
                self.planned_worlds[w_state_hash].add(w.get_hash_state())

            #Test for collision of two robots in the world, which means the world is invalid.
            if w.robots_coliding():
                is_valid = False
                #This can actually be resolved if the new world is fixed by changes earlier on.
                if action[0] <= end_of_plan:
                    if debug: print "Declared unrecoverable due to robot colision"
                    return None
                if debug: print "Declared invalid due to robot colision"


        #Check any remaining states
        for state in world[2]:
            if state[0] >= time:
                if not self.check_plan_state(w, state[1]):
                    if state[0] <= end_of_plan:
                        if debug: print "Declared unrecoverable due to remaining world state at t =", state[0]
                        return None
                    else:
                        if debug: print "Declared invalid due to remaining world state at t =", state[0]
                        return False
        #Worlds that are not complete cannot be true worlds
        #even though they could easily be made into such.
        #TODO, maybe not.
        #if largest_t(robot_last_times) != end_of_plan:
        #    return False
        
        #Now we have reached the end state
        if is_valid:
            for prog in world[3]:
                if prog.failed(): 
                    if debug: print "Declared unrecoverable due to program failure of", prog.uid
                    return None
            for prog in world[3]:
                if not prog.succeeded(): 
                    if debug: print "Declared invalid due to program still running", prog.uid
                    return False
            
            plan_time = largest_t(robot_last_times)

            last_times_nowait = {}
            for a in world[1]:
                if type(a[2]) == tuple:
                    last_times_nowait[a[2][0]] = a[0] + a[1]
            total_t = 0
            for t in last_times_nowait.itervalues():
                total_t = total_t + t
            
            #print "CANDIDATE", self.time_limit, plan_time, total_t
            if self.time_limit == None or self.time_limit > plan_time:
                self.time_limit = plan_time
                self.total_time_limit = total_t
                self.best_world = world
                #print "BEST WORLD", world
            if self.time_limit == plan_time and total_t < self.total_time_limit:
                self.time_limit = plan_time
                self.total_time_limit = total_t
                self.best_world = world
            return True
        return False

    def hash_world(self, world):
        hl = hashlib.sha224()

        #hl.update(world[0].get_hash_state()) #All the same for now, no need to waste CPU
        for step in world[1]:
            hl.update(str(step[0]) + str(step[1]))
            if type(step[2]) == type(""):
                hl.update("R" + step[2])
            elif type(step[2]) == tuple:
                hl.update("A" + str(step[2]))
            else:
                step[2].get_hash_state(hl)
        for state in world[2]:
            hl.update(str(state))
        for prog in world[3]:
            prog.get_hash_state(hl)

        return hl.digest()

    def update_to_point(self, world, start, end):
        w_start = world[0].copy()
        for action in sorted(world[1], key=lambda x: x[0] + x[1]):
            if action[0] + action[1] > end: continue
            if action[0] + action[1] <= start: continue
            if type(action[2]) != tuple: continue
            change, ldc = w_start.take_action(action[2][0], action[2][1], action[2][2])
            if change == None:
                raise Exception("There should not be failing actions here. Those should be blocked by evaluate_world. " + str(action))
        return w_start
        

    def get_next_worlds(self, world):
        robot_last_times = world_times(world)
        end_of_plan = smallest_t(robot_last_times)

        w_start = self.update_to_point(world, 0, end_of_plan)

        #Try to generate new worlds based on robots. These new worlds could be duplicates, so de-duplication
        #must be done by the parent world.
        for robot_name, robot in w_start.robots.iteritems():
            w_robot = w_start.copy()
            next_action_time = None
            for action in world[1]:
                for tv in (action[0], action[0] + action[1]):
                    if tv > robot_last_times[robot_name]:
                        if not next_action_time: next_action_time = tv
                        if tv < next_action_time: next_action_time = tv
                if action[0] + action[1] <= end_of_plan: continue
                if type(action[2]) != tuple: continue
                r, change = w_robot.take_action(action[2][0], action[2][1], action[2][2])
                if r == None:
                    raise Exception("There should not be failing actions here. Those should be blocked by evaluate_world. " + str(action))

            pdata = [prog.get_next_plan(w_robot) for prog in world[3]]

            extra_goals = {}
            for progdata in pdata:
                if type(progdata) != dict: continue
                if not 'extra_goals' in progdata: continue
                for vtype, vals in progdata['extra_goals'].iteritems():
                    extra_goals[vtype] = extra_goals.get(vtype, [])
                    extra_goals[vtype].extend(vals)
                

            #Try generating the case where the robot just sits there until the next action on another robot
            if next_action_time:
                if next_action_time > robot_last_times[robot_name]:
                    step = (robot_last_times[robot_name], next_action_time - robot_last_times[robot_name], robot_name)
                    new_world = [world[0], sorted(world[1] + [step, ], key=lambda x: x[0]), 
                                 world[2], world[3]]
                    yield new_world

            w_test = w_robot.copy()
            #Now try all actions and all parameter combinations.
            for action in w_robot.enumerate_robot(robot_name):
                planable, plist = w_robot.enumerate_action(robot_name, action, extra_goals)
                if not planable or not plist:
                    continue
                for parameters in self.parameter_iter(plist):
                    self.actions_tested = self.actions_tested + 1
                    change, location_do_not_cares = w_test.take_action(robot_name, action, parameters)
                    debug = False #w_start.surfaces["clarkelevator"].locations["floor_3"].equal(parameters.get("outfloor", None))
                    if self.action_type_debug != False:
                        self.action_type_debug[action] = self.action_type_debug.get(action, 0) + 1
                    if debug:
                        print "action", robot_name, action, parameters
                    if change == None: #failed
                        if debug: print "Action failed to execute"
                        #world_clone.take_action(robot_name, at, parameters, debug=True)  #Failed action print
                        #print "Failed", at, parameters
                        if self.fail_debug != False:
                            self.fail_debug[action] = self.fail_debug.get(action, 0) + 1
                        if self.super_fail_debug != False:
                            self.super_fail_debug[action] = self.super_fail_debug.get(action, [])
                            self.super_fail_debug[action].append(((world_test.copy(), world[1], world[2]), action, parameters))
                    else: #Action succeeded.
                        self.actions_finished = self.actions_finished + 1

                        step = (robot_last_times[robot_name], change, (robot_name, action, parameters))
                        new_world = [world[0], sorted(world[1] + [step, ], key=lambda x: x[0]), 
                                     world[2], world[3]]
                        if self.evaluate_world(new_world, blacklist = step) != None:
                            yield new_world
                        else:
                            if debug: print "World evaluated to none"
                        w_test = w_robot.copy() #Reset for next one

        #Try to update each of the progs.
        for i in xrange(0, len(world[3])):
            if world[3][i].done():
                continue
            c = world[3][i].copy()
            new_world = [world[0], [x for x in world[1]], [x for x in world[2]], [x for x in world[3]]]
            new_world[3][i] = c
            r = c.execute(w_start)
            #if c.get_hash_state() == world[3][i].get_hash_state(): continue
            if r == True:
                c.set_plan_executed(True)
                new_world[1].append((end_of_plan, 0, c.copy()))
                if self.evaluate_world(new_world) != None:
                    yield new_world
            if type(r) == dict:
                do_steps = True
                c.set_plan_executed(True)
                extra_step_times = 0
                if 'extra_steps' in r:
                    for robot_name, action, parameters in r['extra_steps']:
                        if end_of_plan != robot_last_times[robot_name]:
                            do_steps = False
                            break
                if 'extra_steps' in r and do_steps:
                    w_test = w_start.copy()
                    for robot_name, action, parameters in r['extra_steps']:
                        change, ldc = w_test.take_action(robot_name, action, parameters)
                        if change == None:
                            do_steps = False
                            break
                        extra_step_times = extra_step_times + change
                        new_world[1].append((robot_last_times[robot_name], change, (robot_name, action, parameters)))
                        robot_last_times[robot_name] = robot_last_times[robot_name] + change
                    c.execute(w_test)
                else:
                    c.execute(w_start)
                if do_steps:
                    new_world[1].append((end_of_plan + extra_step_times, 0, c.copy()))  #After execution add this in
                    new_world[2].append((end_of_plan, r))
                    if self.evaluate_world(new_world) != None:
                        yield new_world
            
    def expand(self):
        worlds = [x for x in self.worlds]

        expanded_worlds = set()

        while worlds != []:
            world = min(worlds, key = lambda x: lowest_t(x))
            worlds.remove(world)

            hworld = self.hash_world(world)
            if hworld in expanded_worlds:
                continue
            expanded_worlds.add(hworld)

            self.worlds_tested = self.worlds_tested + 1

            for next_world in self.get_next_worlds(world):
                hnext_world = self.hash_world(next_world)
                if not hnext_world in expanded_worlds:
                    worlds.append(next_world)
        print
        print self.best_world

    def parameter_iter(self, param, keys = None):
        if keys == []: 
            yield {}
            return
        if keys == None: 
            deps = {}
            #Sort the keys for dependencies
            for key in param.iterkeys():
                if type(param[key]) == type([]):
                    deps[key] = []
                else:
                    deps[key] = param[key][0]
            keys = []
            while deps.keys() != []: #Loop through deps, using deps as a list of keys
                k = deps.keys()
                while k != []: #Crawl deps
                    key = k[0]
                    k = k[1:]
                    deps_met = True
                    for d in deps[key]: #Loop through remaining keys
                        if not d in keys:
                            deps_met = False
                            break
                    if deps_met:
                        keys.append(key)
                        del deps[key]
            keys.reverse()
        if keys == []: 
            yield {}
            return 
        key = keys[0]
        keys = keys[1:]
  
        for next_d in self.parameter_iter(param, keys):
            ls = param[key]
            if type(ls) != type([]): #Handle dependent keys
                #print ls
                ls = ls[1]
                for v in param[key][0]:
                    if type(next_d[v]) == blast_world.BlastSurface:
                        ls = ls[next_d[v].name]
                    else:
                        ls = ls[next_d[v]]
            for value in ls:
                nxt = next_d.copy()
                nxt[key] = value
                yield nxt

    def to_plan_seq(self, initial_world, target):
        #FIXME: we need to deal with scans here, because we do not care
        #about them right now.
        w = initial_world.copy()

        steps_clean = []

        #We can assume here that the times are good.
        if len(target[1]) > 0:
            endtime = target[1][-1][0] + target[1][-1][1]
        else:
            endtime = 0
        add = []
        for step in self.extra_steps:
            add.append((endtime, 1, step))
            endtime = endtime + 1

        for step in target[1] + add:
            if type(step[2]) != type(""):
                #Clean up the parameters
                action_clean = {}
                for param, value in step[2][2].iteritems():
                    if value.__class__ == blast_world.BlastSurface:
                        action_clean[param] = value.name
                    elif value.__class__ == blast_world.BlastPt:
                        action_clean[param] = value.to_dict()
                    else:
                        action_clean[param] = value

                #Get the work spaces for the action. Note that there should be no work space
                #conflicts here - those are banned by planning.
                dictionary, locs = w.get_workspaces(step[2][0], step[2][1], action_clean)
                
                #Get the initial state of all surfaces used in the action
                l_init_state = {}
                for key in dictionary:
                    hl = hashlib.sha224()
                    get_obj = lambda x: w.get_obj(x)
                    w.surfaces[key].hash_update(hl, get_obj, False)
                    l_init_state[key] = hl.digest()
                
                #Trigger an action change, transfering to the next state.
                change, ldc = w.take_action(step[2][0], step[2][1], action_clean)
                if change == False or change == None:
                    return None, None, None

                #Store all the final sha224 state values.
                l_final_state = {}
                for key in dictionary:
                    hl = hashlib.sha224()
                    get_obj = lambda x: w.get_obj(x)
                    w.surfaces[key].hash_update(hl, get_obj, False)
                    l_final_state[key] = hl.digest()
                
                #create the plan step
                uids = [x.id for x in steps_clean]
                steps_clean.append(BlastPlanStep(step[2][0], step[2][1], action_clean, 
                                                 l_init_state, l_final_state, locs, uids, self.step_uid))

        print steps_clean
        #Now we need to do gating of steps based on surface state. In the future we could try
        #to prevent colision here too.
        robot_lasts = {}
        for step_i in xrange(1, len(steps_clean)):
            surface_gates = {}
            surface_stopped = set()
            step_j = step_i
            while step_j > 0:
                step_j = step_j - 1
                for s, v in steps_clean[step_i].initial_states.iteritems():
                    if s in surface_stopped: continue
                    if s in steps_clean[step_j].initial_states:
                        if steps_clean[step_j].initial_states[s] == v:
                            surface_gates[s] = steps_clean[step_j].id
                        else:
                            surface_stopped.add(s)
            for key in surface_gates: steps_clean[step_i].gated_on.add(key)
            #Add gating on all previous steps. This prevents the robot from executing
            #actions out of order, which could lead to strange results.
            if robot_lasts.get(steps_clean[step_i].robot, None) != None:
                steps_clean[step_i].gated_on.add(robot_lasts[steps_clean[step_i].robot])
            robot_lasts[steps_clean[step_i].robot] = steps_clean[step_i].id
        return w, largest_t(world_times(target)), steps_clean
        
    def plan_print(self):
        start_time = time.time()
        self.expand()
        print "Planning time:", time.time() - start_time, "seconds"

        if self.best_world:
            print "Estimated time", self.time_limit, "seconds (=", self.time_limit/60.0, "minutes)"
            print "Steps (total of", len(self.best_world[1]), "actions)"
            for i in self.best_world[1]:
                print i
        else:
            print "Failed to find a plan"
        if self.actions_tested == 0:
            print "Tried", self.worlds_tested, "worlds and", self.actions_tested, "actions"
        else:
            print "Tried", self.worlds_tested, "worlds and", self.actions_tested, "actions -", \
                self.actions_finished, "finished (a", (self.actions_finished * 100.0) / self.actions_tested, "percent success rate)"
        print "Fail debug", self.fail_debug
        print "Action type debug", self.action_type_debug
        if self.super_fail_debug:
            print "Super fail debug"
            for name, arr in self.super_fail_debug.iteritems():
                print "Failed", name, "actions"
                for fail in arr:
                    print fail[0][0].to_text()
                    print fail[0][0].robots["stair4"].location, fail

        return False


class BlastPlanGoal(object):
    #Succeed and fail are labels. If they are None, the plan progresses - this is the predicted result.
    #If they are True, the whole plan succeeds, if it is False the whole plan fails.
    #world_good is a function of a world and start world that returns true if it is what we want
    #force_replan is true if the action must be replaned. If it is "fail" then the action is
    #             predicted to fail.
    #extra_goals is an optional list of extra options based on type to functions.
    #extra_steps is an optional list of extra PlanSteps to take at the end of the mission
    #consider_scan is whether to care about scanning objects.
    def __init__(self, name, succeed, fail, world_good, force_replan = False, extra_goals = {}, extra_steps = [], consider_scan = False):
        self.name = name
        self.succeed = succeed
        self.fail = fail
        self.world_good = world_good
        self.force_replan = force_replan
        self.extra_goals = extra_goals
        self.extra_steps = extra_steps
        self.consider_scan = consider_scan

    def __str__(self):
        return self.__repr__()
    def tuple(self):
        return (self.name, self.succeed, self.fail, self.world_good, self.force_replan, self.extra_goals, self.extra_steps, self.consider_scan)
    def __repr__(self):
        return str(self.tuple())


class BlastPlan:
    def __init__(self, world, uid, world_good = None, extra_goals = None, consider_scan = None, extra_steps = None, goals = None): #Failure replan returns new steps
        self.world = world
        self.uid = uid
        self.goals = goals
        self.steps = None
        if goals == None:
            self.goals = [BlastPlanGoal(None, True, False, world_good, force_replan=False, extra_goals = extra_goals,
                                        extra_steps = extra_steps, consider_scan = consider_scan)]
        print self.goals
        self.needs_replan = True #Halt execution as soon as this is set
        self.replan_ready = True
        self.replan_gate = None #Wait until a replan is ordered. None = no replan, False == not ready, True == start replan
        self.done = False

    def plan(self, gp = 0):
        print "-"*30
        self.replan_ready = True
        while self.replan_gate != True:
            print "Waiting for replan gate", self.replan_gate
            time.sleep(0.01)
        
        self.world.planning_lock.acquire()
        self.replan_ready = False
        self.replan_gate = None
        self.needs_replan = False
        steps = []
        planning_world = self.world.get_planning_world()
        replan_force = False
        first_step = True
        while True:
            if gp >= len(self.goals):
                break
            steps.append(gp)
            print self.goals[gp]
            planning_world.consider_scan = self.goals[gp].consider_scan
            planner = Planner(planning_world)
            planner.step_uid = str(self.uid) + "/"
            planner.world_good = self.goals[gp].world_good
            planner.extra_goals = self.goals[gp].extra_goals
            planner.extra_steps = self.goals[gp].extra_steps
            world, add_est_time, add_steps = planner.plan_print()
            replan_force = replan_force or self.goals[gp].force_replan
            if self.goals[gp].force_replan and not first_step:
                steps.append("REPLAN")
            print "Result", world, add_est_time
            if world == None and add_est_time == None and add_steps == None or (self.goals[gp].force_replan == "fail" and not first_step):
                if world == None and add_est_time == None and add_steps == None:
                    pass #Do not do anything
                else:
                    steps.extend(add_steps)
                if  self.goals[gp].fail == False:
                    steps.append("FAIL")
                    if not replan_force: #If there's no way to replan the actions we are in a fail state, then fail
                        self.world.planning_lock.release()
                        return None, None, None
                    break
                elif self.goals[gp].fail == True:
                    steps.append("SUCCEED")
                    break
                elif self.goals[gp].fail == None:
                    gp = gp + 1
                else:
                    for gpc in xrange(0, len(self.goals)):
                        if self.goals[gpc].name == self.goals[gp].fail:
                            gp = gpc
                            break
            else:
                planning_world = world
                steps.extend(add_steps)
                if self.goals[gp].succeed == True:
                    steps.append("SUCCEED")
                    break
                elif self.goals[gp].succeed == True:
                    steps.append("FAIL")
                    if not replan_force:
                        self.world.planning_lock.release()
                        return None, None, None
                    break
                elif self.goals[gp].succeed == None:
                    gp = gp + 1
                else:
                    for gpc in xrange(0, len(self.succeed)):
                        if self.goals[gpc].name == self.goals[gp].succeed:
                            gp = gpc
                            break
            first_step = False

        self.steps = steps
        print "-"*30
        for i in steps:
            print i
        print "-"*30, "->"
        
        if self.needs_replan:
            print "Instant replan"
            self.world.planning_lock.release()
            return self.plan(gp = exec_gp)
        self.world.plans_planned_count = self.world.plans_planned_count + 1
        self.world.planning_lock.release()

        if self.world.no_exec_debug:
            print "WARNING: No exec debug is set for testing only. If this is not a test this is a problem."
            return True

        #Wait until all plans have finished planning before moving on
        #to start taking actions in the plan.
        while True:
            self.world.plans_lock.acquire()
            target = len([x for x in self.world.plans if not x.done])
            self.world.plans_lock.release()
            if self.world.plans_planned_count >= target:
                break
            time.sleep(0.01)

        exec_gp = False
        while True:
            flag_replan = False
            self.world.planning_lock.acquire()
            if self.steps == []:
                self.world.planning_lock.release()
                break
            action_in_queue = False
            while self.steps != []:
                action = self.steps[0]
                self.steps = self.steps[1:]
                if action == "REPLAN":
                    flag_replan = True
                    break
                elif type(action) == int:
                    exec_gp = action
                    break
                elif type(action) == str:
                    if (action == "FAIL" or action == "SUCCEED") and action_in_queue:
                        self.steps = [action,] + self.steps
                        break
                    elif action == "FAIL":
                        print "We are done due to FAIL hardcoded in to sequence"
                        self.done = True
                        self.steps = []
                        self.world.planning_lock.release()
                        return False
                    elif action == "SUCCEED":
                        print "We are done due to SUCCEED hardcoded in to sequence"
                        steps = self.steps
                        self.steps = []
                        self.done = True
                        self.world.planning_lock.release()
                        return True
                    break
                else:
                    action_in_queue = True
                    self.world.robot_locks[action.robot].acquire()
                    self.world.robot_queues[action.robot].append(action)
                    self.world.robot_locks[action.robot].release()
            self.world.planning_lock.release()
            
            while True:
                if self.needs_replan:
                    break
                still_there = False
                for robot, queue in self.world.robot_queues.iteritems():
                    self.world.robot_locks[robot].acquire()
                    for step in queue:
                        if step.id.split("/")[0] == str(self.uid):
                            still_there = True
                            break
                    self.world.robot_locks[robot].release()
                    if still_there: break

                if not self.world.real_world:
                    self.world.try_operate()

                if not still_there:
                    break

            if flag_replan:
                print "Order a replan due to flagging for the requirement"
                self.replan_ready = True
                self.world.order_replan()

            if self.needs_replan:
                return self.plan(gp = exec_gp)
                
        print "We are done with success due to reaching end of steps"
        self.done = True
        return True



class BlastCodeExec(object):
    __slots__ = ['uid', 'code', 'labels', 'environments', 'plan_executed']

    def get_hash_state(self, hl = None):
        if not hl:
            hl = hashlib.sha224()
            self.get_hash_state(hl)
            return hl.digest()
        hl.update(str(self.uid) + str(id(self.code)))
        if self.done():
            hl.update(str(self.environments))
            return
        hl.update(str(self.plan_executed))
        for e in self.environments:
            hl.update(str(e))

    def get_next_plan(self, w):
        c = self.copy()
        c.set_plan_executed(False)
        return c.execute(w)
        

    def __init__(self, uid, code, labels = None, environments = None, plan_e = False):
        self.uid = uid
        #Translate all the code
        self.code = code
        self.plan_executed = plan_e
        if labels:
            self.labels = labels
        else:
            self.labels = {}
            for i in xrange(0, len(self.code)):
                if self.code[i].label:
                    if self.code[i].label in self.labels:
                        raise blast_world.BlastCodeError("Duplicate label: " + self.code[i].label)
                    self.labels[self.code[i].label] = i

        if environments != None:
            if environments == True or environments == False:
                self.environments = environments
            else:
                self.environments = [[gp, cb, lb, vs.copy()] for gp, cb, lb, vs in environments]
        else:
            self.environments = [[0, self.code, self.labels, {}],]

    def copy(self):
        return BlastCodeExec(self.uid, self.code, self.labels, self.environments, self.plan_executed)

    def __str__(self):
        if self.done():
            return str(self.uid) + ":" + str(self.environments)
        a = []
        for e in self.environments:
            delt = ":"
            if e[1] == self.code: #TODO: this is not a good equality
                delt = "."
            a.append(str(e[0]) + delt + str(e[2]))
        return str(self.uid) + ":" + str(a)
            
    def __repr__(self):
        return str(self)

    def paste_parameters(self, p, env):
        if type(p) == dict:
            o = {}
            for n, v in p.iteritems():
                o[n] = self.paste_parameters(v, env)
            return o
        if type(p) == list:
            return [self.paste_parameters(v, env) for v in p]
        if type(p) == tuple:
            return tuple([self.paste_parameters(v, env) for v in p])
        if type(p) == blast_world.BlastParameterPtr:
            return env.get(p.parameter)
        return p

    def set_plan_executed(self, p):
        self.plan_executed = p

    def done(self):
        if self.environments == False or self.environments == True:
            return True
        return False
    def succeeded(self):
        return (self.environments == True)
    def failed(self):
        return (self.environments == False)

    def execute(self, world):
        if self.environments == False or self.environments == True:
            return self.environments
        next_step = self.environments[-1]
        ps = next_step[1][next_step[0]]
        params = self.paste_parameters(ps.parameters, next_step[3])
        #print next_step
        #print params
        if ps.command == "PLAN":
            if self.plan_executed:
                if ps.return_var:
                    next_step[3][ps.return_var] = self.plan_executed
                self.plan_executed = False
                next_step[0] = next_step[0] + 1
                return self.execute(world)
            else:
                return params
        elif ps.command == "IF":
            def condition_eval(c):
                if type(c) == bool:
                    return c
                e = [condition_eval(x) for x in c[1:]]
                print e
            if condition_eval(params["condition"]):
                if params['label_true'] in next_step[2]:
                    next_step[0] = next_step[2][params['label_true']]
                else:
                    raise blast_world.BlastCodeError("We tried to jump out in an if statement: " + params['label_true'])
            elif 'label_false' in params:
                if params['label_false'] in next_step[2]:
                    next_step[0] = next_step[2][params['label_false']]
                else:
                    raise blast_world.BlastCodeError("We tried to jump out in an if statement: " + params['label_false'])
            else:
                next_step[0] = next_step[0] + 1
            return self.execute(world)
        elif ps.command == "FAIL" or ps.command == "RETURN":
            if len(self.environments) == 1:
                self.environments = (ps.command == "RETURN")
            else:
                raise blast_world.BlastCodeError("Return is not yet supported for subroutines")
            return self.execute(world)
        else:
            raise blast_world.BlastCodeError("We don't support '" + ps.command + "'")
        raise blast_world.BlastCodeError("Command '" + ps.command + "' did not return a result, internal bug")
        


class BlastPlannableWorld:
    def __init__(self, world, real_world = False):
        self.world = world
        self.real_world = real_world

        self.lock = threading.Lock()

        self.needs_replan = False

        self.robot_actions = {}
        self.robot_action_queues = {}
        
        self.code_exec = []
        self.code_exec_uid = 0

    def append_plan(self, code, wait_for_plan = True):
        self.lock.acquire()
        exc = BlastCodeExec(self.code_exec_uid, code)
        self.code_exec_uid = self.code_exec_uid + 1
        self.code_exec.append(exc)
        self.needs_replan = True
        self.lock.release()

    def try_exec(self):
        self.lock.acquire()
        if self.needs_replan:
            still_running = False
            for robot in self.world.robots_keysort:
                self.robot_action_queues[robot] = []
                if self.robot_actions.get(robot) != None:
                    still_running = True
                    #Here is where we would try to interrupt the action
            if not still_running:
                planner = Planner(self.world, [x.copy() for x in self.code_exec])
                planner.plan_print()
        self.lock.release()
        return

    







#This world implements action queues and the like. It also does action
#gating so things execute in order.

class BlastPlannableWorldOld:
    def order_replan(self):
        #Tell all the plans that they need to re-plan. Also delete all done plans.
        self.plans_lock.acquire()
        self.plans_planned_count = 0
        self.plans = [x for x in self.plans if not x.done]
        self.plan_ids = set([int(x.uid) for x in self.plans])
        for plan in self.plans:
            plan.replan_gate = False
            plan.needs_replan = True
        
        #Wait for all plans to stop
        stuck = True
        while stuck and not self.no_exec_debug:
            stuck = False
            for plan in self.plans:
                if not plan.replan_ready:
                    stuck = True
                    break
            if stuck:
                print "Waiting for replan_ready"
                time.sleep(0.01)
        #Now all plans should be waiting for the replan_gate triggers

        #Cancel all actions
        self.planning_lock.acquire()
        for robot in self.world.robots_keysort:
            self.robot_locks[robot].acquire()
            self.robot_queues[robot] = []
            #TODO --> We should attempt to interrupt the current action here when we implment that.
            self.robot_locks[robot].release()
        self.planning_lock.release()

        #Wait for the actions to stop
        stuck = True
        while stuck:
            stuck = False
            for robot in self.world.robots_keysort:
                if self.robot_current_actions[robot] != None:
                    stuck = True
                    break
            if stuck:
                print "Waiting for a robot to finish"
                time.sleep(0.01)
        self.plans_lock.release()

        #Now we can release the replan gates. Note that this causes an all-out free-for all
        #for the lock, so the plans get executed in random order. In the future we may not
        #want this because we want to prioritize plans.
        for plan in self.plans:
            plan.replan_gate = True



    def __init__(self, world):
        self.world = world
        self.no_exec_debug = False #Used by tests with the actual plan info prevents execution of the test.
        self.real_world = False
        self.plan_steps = {}
        self.post_exec_world = None
        self.action_callback = lambda r, a, p: True
        def action_e_fail(r, a, p): 
            print "Action failed epically", r, "-->", a
            print "With parameters: ", p
        self.action_epic_fail_callback = action_e_fail

        #When the planning lock is in place, no actions can
        #be started. This is so the planner can plan with a
        #fixed world. Unfortunately this makes things slow
        #because we cannot have action execution and build up
        #at the same time, but alas that is the price we pay.
        self.planning_lock = threading.Lock()

        #Synchronization method to know when to restart execution
        #after replanning events.
        self.plans_planned_count = 0

        #This is the modification lock. When in place, no
        #modifications can be made to self.world. This is
        #important to prevent e.g. two actions from 
        #conflicting and crashing the system.
        self.lock = threading.Lock()

        #The queue of "PlanStep" objects for the robots. Note 
        #that all events here have a planning event that 
        #introduced them associated so they can be unplanned 
        #in the event of a failure.
        self.robot_queues = {}

        #The locks for the robot queues. You can't modify a 
        #robot's queue unless you lock it, so we don't have 
        #issues with list append and delete at the same time 
        #(list-based race conditions).
        self.robot_locks = {}

        #This is a list of the actions currently being 
        #executed by each of the robots as a plan step.
        #Note that this is used if we need to build the
        #planning world.
        self.robot_current_actions = {}

        #Zero out all the variables.
        for robot in self.world.robots_keysort:
            self.robot_current_actions[robot] = None
            self.robot_queues[robot] = []
            self.robot_locks[robot] = threading.Lock()

        #The n of the current planning event. This unique
        #ID counts up and allows cancelation of the plan.
        self.n_planning_event = 0

        #This is the list of currently executing plans.
        self.plans_lock = threading.Lock()
        self.plans = []
        self.plan_ids = set()

        #This is a list of all the executed actions
        self.executed_lock = threading.Lock()
        self.executed = []
        self.executed_ids = set()

    def try_operate(self):
        self.plans_lock.acquire()
        self.plans = [x for x in self.plans if not x.done]
        self.plan_ids = set([int(x.uid) for x in self.plans])
        self.plans_lock.release()

        self.executed_lock.acquire()
        for x in self.executed: self.executed_ids.add(x) #TODO: memory leak
        self.executed = [x for x in self.executed if int(x.id.split("/")[0]) in self.plan_ids]
        self.executed_lock.release()

        self.planning_lock.acquire()
        for robot in self.world.robots_keysort:
            dorel = True
            self.robot_locks[robot].acquire()
            if self.robot_current_actions[robot] == None and len(self.robot_queues[robot]) > 0:
                step = self.robot_queues[robot][0]
                failed = False
                for gate in step.gated_on:
                    if not gate in self.executed_ids:
                        failed = True
                        break
                if not failed:
                    self.robot_current_actions[robot] = step
                    self.robot_queues[robot] = self.robot_queues[robot][1:]
                    self.robot_locks[robot].release()
                    dorel = False
                    self.take_action(robot, step.action, step.parameters) #Nulls current action when done
            if dorel:
                self.robot_locks[robot].release()
        self.planning_lock.release()
        
        
    #Gets the current planning world, needs to be called with
    #planning_lock or bad things might happen.
    def get_planning_world(self):
        self.lock.acquire()
        clone_world = self.world.copy()
        self.lock.release()
        for robot, action in self.robot_current_actions.iteritems():
            if action != None:
                raise Exception("Needs to be properly implemented")
        return clone_world

    def copy(self):
        #FIXME: this could use a lot of work. This whole process
        #needs to be re-thought
        raise Exception("Need to work on this it is a possible source of bugs.")
        c = BlastPlannableWorld(self.world.copy())
        c.real_world = False
        return c

    def plan_hunt(self, robot, holder, object_type, world_good = lambda w: True):
        def world_has_object(w, sw):
            if w.robots[robot].holders[holder] != None:
                o = w.objects[w.robots[robot].holders[holder].uid]
                if o.object_type.name == object_type:
                    return world_good(w)
            return False
        def world_scan_inc(w, sw):
            return w.scan_count(object_type) > sw.scan_count(object_type)

        goals = [ BlastPlanGoal(None, True, None, world_has_object),
                  BlastPlanGoal("Scan", None, "Last-Check", world_scan_inc, consider_scan = True),
                  BlastPlanGoal(None, True, "Scan", world_has_object, force_replan = "fail"),
                  BlastPlanGoal("Last-Check", True, False, world_has_object, force_replan = True),
                  ]
        

        self.plans_lock.acquire()
        plan = BlastPlan(self, self.next_plan(), goals = goals)
        self.plans.append(plan)
        self.plans_lock.release()
        self.order_replan()
        return plan.plan()
        
    def next_plan(self):
        scp = self.n_planning_event = self.n_planning_event + 1
        return scp - 1

    def plan(self, world_good, extra_goals = {}, extra_steps = []):
        self.plans_lock.acquire()
        plan = BlastPlan(self, self.next_plan(), world_good, extra_goals, False, extra_steps)
        self.plans.append(plan)
        self.plans_lock.release()
        self.order_replan()
        return plan.plan()


    def plan_to_location(self, robot, location):
        if self.world.robots[robot].location.equal(location):
            print "Already at location"
            return
        return self.plan(lambda w, sw: w.robots[robot].location.equal(location), 
                         {"Pt": [location,]})

    
    #API actions ---------------------------
    def robot_transfer_holder(self, robot, from_holder, to_holder):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Set robot holder invalid robot", robot
            self.lock.release()
            return False
        if not from_holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", from_holder, "for robot", robot
            self.lock.release()
            return False
        if not to_holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", to_holder, "for robot", robot
            self.lock.release()
            return False
        self.world.robot_transfer_holder(robot, from_holder, to_holder)
        self.lock.release()
        return True
    
    def robot_pick_object(self, robot, uid, to_holder):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Robot pick object invalid robot", robot
            self.lock.release()
            return False
        if not uid in self.world.objects_keysort:
            print "Robot pick object invalid object", uid, "for robot", robot
            self.lock.release()
            return False
        if not to_holder in self.world.robots[robot].holders:
            print "Robot pick object invalid holder", to_holder, "for robot", robot
            self.lock.release()
            return False
        self.world.robot_pick_object(robot, uid, to_holder)
        self.lock.release()
        return True
    
    def robot_place_object(self, robot, from_holder, surface, pos):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Robot place object invalid robot", robot
            self.lock.release()
            return False
        if not surface in self.world.surfaces:
            print "Robot place object invalid surface", surface, "for robot", robot
            self.lock.release()
            return False
        if not from_holder in self.world.robots[robot].holders:
            print "Robot place object invalid holder", to_holder, "for robot", robot
            self.lock.release()
            return False
        if self.world.robots[robot].holders == None:
            print "Robot place object empty holder", to_holder, "for robot", robot
            self.lock.release()
            return
        self.world.robot_place_object(robot, from_holder, surface, pos)
        self.lock.release()
        return True
    
    def set_robot_holder(self, robot, holder, object_type, require_preexisting_object = True):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Set robot holder invalid robot", robot
            self.lock.release()
            return False
        if not holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", holder, "for robot", robot
            self.lock.release()
            return False
        if require_preexisting_object:
            if self.world.robots[robot].holders[holder] == None:
                print "Robot holder empty, set requires object for holder", holder, "for robot", robot
                self.lock.release()
                return False
        self.world.set_robot_holder(robot, holder, object_type)
        self.lock.release()
        return True

    def get_robot_holder(self, robot, holder):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Set robot holder invalid robot", robot
            self.lock.release()
            return False
        if not holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", holder, "for robot", robot
            self.lock.release()
            return False
        self.lock.release()
        return self.world.get_robot_holder(robot, holder)
    

    def set_robot_position(self, robot, position, val):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Set robot position invalid robot", robot
            self.lock.release()
            return False
        if not position in self.world.robots[robot].positions:
            print "Set robot holder invalid position", position, "for robot", robot
            self.lock.release()
            return False
        self.world.set_robot_position(robot, position, val)
        self.lock.release()
        return True
    
    def set_robot_location(self, robot, blast_pt):
        self.lock.acquire()
        if not robot in self.world.robots:
            print "Set robot location invalid robot", robot
            self.lock.release()
            return False
        self.world.set_robot_location(robot, blast_world.BlastPt(blast_pt['x'], blast_pt['y'],
                                                                 blast_pt['a'], blast_pt['map']))
        self.lock.release()
        return True

    def get_surface(self, surface):
        self.lock.acquire()
        sur = self.world.surfaces.get(surface)
        if sur != None: sur = sur.to_dict()
        self.lock.release()
        return sur
    
    def get_object(self, obje):
        self.lock.acquire()
        obj = self.world.objects.get(obje)
        if obj != None: obj = obj.to_dict()
        self.lock.release()
        return obj

    def delete_surface_object(self, obje):
        self.lock.acquire()
        obj = self.world.objects.get(obje)
        if obj == None:
            print "Surface delete object invalid object", obje
            self.lock.release()
            return False
        r = self.world.delete_surface_object(obje)
        self.lock.release()
        return r

    def get_map(self, map):
        self.lock.acquire()
        mp = self.world.maps.get(map)
        if mp != None: mp = mp.to_dict()
        self.lock.release()
        return mp
    
    def get_robot(self, robot):
        self.lock.acquire()
        rb = self.world.robots.get(robot)
        if rb != None: rb = rb.to_dict()
        self.lock.release()
        return rb

    def surface_scan(self, surface, object_types):
        self.lock.acquire()
        if not surface in self.world.surfaces:
            print "Invalid surface for scanning", surface
            self.lock.release()
            return False
        r = self.world.surface_scan(surface, object_types)
        self.lock.release()
        return r

    def add_surface_object(self, surface, object_type, pos):
        self.lock.acquire()
        if not surface in self.world.surfaces:
            print "Invalid surface for object addition", surface
            self.lock.release()
            return False
        r = self.world.add_surface_object(surface, object_type, pos)
        self.lock.release()
        return r
        

    def plan_place(self, uid, surface, pos, plan_and_return = False, include_action = False, execution_cb = lambda x: None):
        #FIXME: OLD FASHIONED
        extras = {}

        uid = int(uid)
        surface = str(surface)
        if type(pos) != blast_world.BlastPos:
            if type(pos) == type([]):
                pos = [str(x).strip() for x in pos]
            else:
                pos = [x.strip().strip("''").strip() for x in str(pos).strip().strip("[]").split(",")]
            if pos[0] != surface: return None
            pos = [float(x) for x in pos[1:]]
            pos = blast_world.BlastPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        if type(pos) != blast_world.BlastPos: return None
        if not surface in self.world.surfaces: return None
        if not uid in self.world.objects: return None
        extras["Pos:SU:" + surface + ":" + str(uid)] = [pos,]

        def check_world(w):
            if not uid in w.objects:
                return False
            if w.objects[uid].parent != surface:
                return False
            if w.objects[uid].position == None:
                return False
            return w.objects[uid].position.equal(pos)

        r = self.plan(lambda w: check_world(w),
                      extras, plan_and_return = plan_and_return, report_plan = True, 
                      execution_cb = lambda x: execution_cb(x))
        return r
                      

    def plan_action(self, robot, action, parameters, world_limits = {}):
        #FIXME: this can create problems if parameters is an extra element
        extras = {}
        if "robot-location" in world_limits:
            for robot_name, location in world_limits["robot-location"].iteritems():
                extras["Pt"] = extras.get("Pt", [])
                if type(location) == blast_world.BlastPt:
                    extras["Pt"].append(location)
                else:
                    extras["Pt"].append(blast_world.BlastPt(location['x'], location['y'], location['a'], location['map']))

        extra_steps = [ (robot, action, parameters), ]
        r = self.plan(lambda w, sw: w.world_limit_check(world_limits), 
                      extra_goals = extras, extra_steps = extra_steps)
        #if r != None:
            #if include_action:
                #r[2].append((robot, action, parameters))
            #if not plan_and_return:
            #    worked = self.take_action(robot, action, parameters)
            #    if not worked:
            #        return None #This really shouldn't happen, because the action is tested.
            #execution_cb([])
        return r   

    def take_action(self, robot, action, parameters, debug = True):
        parameters = parameters.copy()

        def done():
            self.executed_lock.acquire()
            self.robot_locks[robot].acquire()
            if self.robot_current_actions[robot]:
                self.executed.append(self.robot_current_actions[robot])
                self.executed_ids.add(self.robot_current_actions[robot].id)
                self.robot_current_actions[robot] = None
            self.robot_locks[robot].release()
            self.executed_lock.release()
        
        for name in parameters:
            if parameters[name].__class__ == blast_world.BlastSurface:
                parameters[name] = parameters[name].name
            elif parameters[name].__class__ == blast_world.BlastObjectRef:
                parameters[name] = parameters[name].uid
            elif type(parameters[name]) == type((0, 1)):
                parameters[name] = tuple([str(x) for x in parameters[name]])
            elif hasattr(parameters[name], "to_dict"):
                parameters[name] = parameters[name].to_dict()

        if self.real_world:
            done()
            raise Exception("Not yet.")
        elif self.world.take_action(robot, action, parameters, True, debug):
            if not self.action_callback(robot, action, parameters):
                print "Action callback failed to work, this test may be broken."
                done()
                return None
            done()
            return True
        else:
            print "Blast failed to take action"
            done()
            return None
        done()
    
    #End API actions ---------------------------        

def coffee_hunt_test():
    import blast_world_test
    world = BlastPlannableWorld(blast_world_test.make_table_top_world(False))
    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")

    def ac(r, a, p): #Test add the cups
        if r == "stair4" and a == "table-coffee-scan":
            cup = blast_world.BlastObject(world.world.types.get_object("coffee_cup"),
                                          blast_world.BlastPos(0.6602, 0.0, 0.762, 0.0, 0.0, 0.0), "table_1")
            world.world.append_object(cup)
            world.world.surfaces["table_1"].objects.append(blast_world.BlastObjectRef(cup.uid))
            world.world.clear_hash("surfaces")
        return True

    world.action_callback = ac
    
    print '-'*100
    print "Plan to pick up coffee cup"
    print '-'*100
    #world.plan(lambda w: w.robots["stair4"].holders["cupholder"] != None, {})

    r = world.plan_hunt("stair4", "cupholder", "coffee_cup")
    if not r: return False
    print r


    print '-'*100
    print "Plan to put down coffee cup"
    print '-'*100
    r = world.plan_action("stair4", "table-place-left", {"table": "table_1", "position": "table_1, Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)"})
    if not r: return False
    print r
    return True


def run_test():
    import blast_world_test
    world = BlastPlannableWorld(blast_world_test.make_test_world())

    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
    
    print '-'*100
    print "Plan to pick up bag"
    print '-'*100
    r = world.plan_to_location("stair4", initial_pickup_point)
    if not r: return False
    if not world.world.robots["stair4"].location.equal(initial_pickup_point): return False

    print '-'*100
    print "Grab money bag"
    print '-'*100
    r = world.take_action("stair4", "grab-object", {"tts-text": "Money Bag"})
    if not r: return False
    r = world.set_robot_holder("stair4", "left-arm", "coffee_money_bag")
    if not r: return False

    print '-'*100
    print "Plan to buy coffee"
    print '-'*100
    r = world.plan_action("stair4", "buy-coffee", {"shop": "clark_peets_coffee_shop"})
    if not r: return False
    
    print '-'*100
    print "Plan to return"
    print '-'*100
    r = world.plan_to_location("stair4", initial_pickup_point)
    if not r: return False
    
    print '-'*100
    print "Give object back"
    print '-'*100
    r = world.take_action("stair4", "unstash-cupholder", {})
    if not r: return False
    r = world.take_action("stair4", "give-object", {"tts-text": "Coffee Cup"})
    if not r: return False
    print r
    return True

def multi_robot_test():
    import blast_world_test
    world_i = blast_world_test.make_table_top_world(True)
    
    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    world_i.types.get_robot("pr2-cupholder"))
    world_i.append_robot(stair5)
    world_i.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.
    world = BlastPlannableWorld(world_i)

    print "-"*180
    print world.world.to_text()
    print "-"*180

    
    r = world.plan_hunt("stair4", "cupholder", "coffee_cup")

    print "-"*180
    print world.world.to_text()
    print "-"*180
    
    return r
    
def overplan():
    
    import blast_world_test
    world_i = blast_world_test.make_table_top_world(False)
    
    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    world_i.types.get_robot("pr2-cupholder"))
    world_i.append_robot(stair5)
    world_i.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.
    world = BlastPlannableWorld(world_i)

    world.no_exec_debug = True

    #Plan to location.
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {'world_limits': 
                                                                {"robot-location": 
                                                                 {"stair5":  blast_world.BlastPt(15.000, 20.957, 0.148, "clarkcenterfirstfloor")},
                                                                 },
                                                                'extra_goals':
                                                                    {'Pt': [blast_world.BlastPt(15.000, 20.957, 0.148, "clarkcenterfirstfloor")]}}, 'plan_return'),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                              'label_true': "success", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success", "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL"),
                       ], False)
    
    #Make stair5 tuck arms but order stair4 to move into position before it can happen
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {'world_limits': 
                                                                {"robot-location": 
                                                                 {"stair4": blast_world.BlastPt(20.000, 20.957, 0.148, "clarkcenterfirstfloor")},
                                                                 },
                                                                'extra_steps': [("stair5", "tuck-both-arms", {})],
                                                                'extra_goals':
                                                                    {'Pt': [blast_world.BlastPt(20.000, 20.957, 0.148, "clarkcenterfirstfloor")]}}, 'plan_return'),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                              'label_true': "success", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success", "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL"),
                       ], False)

    world.try_exec()

    #r = world.plan_to_location("stair5", blast_world.BlastPt(15.000, 20.957, 0.148, "clarkcenterfirstfloor"))
    #if not r: return False

    #r = world.plan_action("stair5", "tuck-both-arms", {}, {"robot-location": {"stair4": blast_world.BlastPt(20.000, 20.957, 0.148, "clarkcenterfirstfloor")}})
    #if not r: return False

    
    r = False
    return r

if __name__ == '__main__':
    #print coffee_hunt_test()
    #print run_test()
    #print multi_robot_test()
    print overplan()
