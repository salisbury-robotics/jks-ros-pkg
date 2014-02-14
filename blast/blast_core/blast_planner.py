
import blast_world, time, json, itertools, hashlib, random, string, threading

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
        if type(r) != type(""): r = r[0]
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
    __slots__ = ['initial_world', 'worlds', 'world_good', 'planned_worlds', 'good_worlds',
                 'time_limit', 'worlds_tested', 'actions_tested', 'actions_finished', 'extra_goals', 
                 'action_type_debug', 'fail_debug', 'super_fail_debug', 'best_world', 'step_uid', 'extra_steps']
    def __init__(self, initial_world):
        self.initial_world = initial_world
        self.worlds = [(initial_world, [])]
        self.world_good = lambda x: False
        self.planned_worlds = []
        self.good_worlds = []
        self.time_limit = None
        self.worlds_tested = 0
        self.actions_tested = 0
        self.actions_finished = 0
        self.step_uid = ""
        self.extra_goals = {}

        self.action_type_debug = {}
        self.fail_debug = {}
        self.super_fail_debug = False

        self.best_world = None

    def evaluate_world(self, world, bucket):
        if self.world_good(world, self.initial_world):
            if self.extra_steps != []:
                #FIXME: TODO workspaces here
                clone = world.copy()
                for step in self.extra_steps:
                    te, ldc = clone.take_action(step[0], step[1], step[2], True, False)
                    if te == None:
                        return False
            bucket_t = largest_t(world_times(bucket))
            if self.best_world == None:
                self.time_limit = bucket_t
                self.best_world = bucket
            elif bucket_t < self.time_limit:
                self.time_limit = bucket_t
                self.best_world = bucket
            return True
        return False
                

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

    def plan_recursive(self):

        dot_graph = file("graph_test.dot", "w")
        def world_to_dot(w):
            loc_val = None
            robot = w.robots["stair4"]
            for sname, surf in w.surfaces.iteritems():
                for name, loc in surf.locations.iteritems():
                    if loc.equal(robot.location):
                        loc_val = sname + ".locations." + name
            if loc_val == None:
                loc_val = robot.location.to_text().replace("\"", "").replace("'", "")
            
            p_str = ""
            for name, pos in sorted(robot.positions.items(), key=lambda x: x[0]):
                p_str = p_str + "["
                if pos:
                    p_str = p_str + ",".join([str(p) for j, p in sorted(pos.items(), key=lambda x: x[0])])
                p_str = p_str + "]"
            return '"' + loc_val + p_str + '"'
        if dot_graph:
            dot_graph.write("digraph G {\n")


        explored_worlds = []

        do_scans = False

        for world in self.worlds:
            if world[0].consider_scan: do_scans = True
            explored_worlds.append(world[0])
            #TODO - we need to handle pre-expansion of goals already extant here.
            self.evaluate_world(world[0], world)
        

        cached_types = {}
        cached_actions = {}
        for robot_name, robot in self.worlds[0][0].robots.iteritems():
            #Find all valid robot types
            robot_types = []
            t = robot.robot_type
            while t:
                robot_types.append(t.name)
                t = t.parent
            #Find all valid actions
            action_types = []
            for name in world[0].types.actions.keys():
                if world[0].types.actions[name].scan_only() and not do_scans: continue
                if world[0].types.actions[name].planable == False: continue
                if name.split(".")[0] in robot_types:
                    if not name.split(".")[1] in action_types:
                        action_types.append(name.split(".")[1])
            cached_types[robot.name] = robot_types
            cached_actions[robot.name] = action_types
        
        # This is simply an enumerated list of all possible actions. We
        # then apply this to the entire loop.
        all_possible_actions = []
        n_robots = len(world[0].robots)
        for robot_name, robot in world[0].robots.iteritems():
            robot_types = cached_types[robot.name]
            action_types = cached_actions[robot.name]
            for at in action_types:
                all_possible_actions.append((robot_name, at))

        #Expansion proceeds by the following rules.
        #1. The world state updates via work spaces. Two robots cannot
        #   be operating in the same work space at the same time. A work
        #   space is a surface or a location in a surface.
        #2. Plans are made sequentially through time. It is important to
        #   note conditional dependencies in time that limit the speed of
        #   work. This occurs when a work space needs to be freed or in
        #   a given state for work to succeed. This occurs e.g. when an 
        #   object must be set on a table by one robot so another can
        #   pick it up. One potential difficulty here is dead-locks which
        #   could happen if one robot gets stuck waiting for another one.
        #   The solution here is replanning, as it would be very hard to
        #   enumerate all possible deadlock scenarios. For example, one
        #   such scenario is that a robot is waiting at a table for a
        #   delivery but is in turn blocking the delivery. The solution
        #   is to replan which would trigger the motion of the stopped
        #   robot.
        #3. The worlds are represented as a set of actions which are
        #   transformations of the world state. Many of the actions are
        #   simply waiting for others to finish.


        #The worlds have dictionary of queues to actions for each robot.
        #These queues in turn specify the start and end states for all
        #actions robots and relevant surfaces.



        #Worlds fall into two categories: those with smooth times and those
        #without
        
        while self.worlds != []:
            world = min(self.worlds, key = lambda x: lowest_t(x))
            self.worlds.remove(world)
            #print world_times(world)
            #for s in world[1]: print "   ", s

            if self.time_limit != None:
                if lowest_t(world) > self.time_limit:
                    self.planned_worlds.append(world)
                    #print "World ignored for timelimit"
                    continue

            #Check to see if an equal plan has been expanded already. We need
            #To consider this more.
            skip_for_equality = False
            for plan in self.planned_worlds:
                if len(plan[1]) == len(world[1]):
                    eq = True
                    for ac, bc in zip(world[1], plan[1]):
                        a = ac[2]
                        b = bc[2]
                        if a[0] != b[0]: eq = False
                        if a[1] != b[1]: eq = False
                        if type(a[2]) != type(b[2]): eq = False
                        if type(a[2]) == type("") and type(b[2]) == type(""): eq = False
                        if type(a[2]) == type({}) and type(b[2]) == type({}) and eq:
                            if len(a[2]) != len(b[2]): 
                                eq = False
                            else:
                                for k in a[2]:
                                    if not k in b[2]:
                                        eq = False
                                        break
                                    if type(b[2][k]) != type(a[2][k]):
                                        eq = False
                                        break
                                    if type(b[2][k]) == blast_world.BlastPt or type(b[2][k]) == blast_world.BlastPos:
                                        if not b[2][k].equal(a[2][k]):
                                            eq = False
                                            break
                                    elif type(b[2][k]) == blast_world.BlastSurface:
                                        if b[2][k].name != a[2][k].name:
                                            eq = False
                                            break
                                    elif type(b[2][k]) == tuple:
                                        if len(a[2][k]) != len(b[2][k]):
                                            eq = False
                                            break
                                        if len(b[2][k]) == 2:
                                            if type(a[2][k][0]) == type("") and type(b[2][k][0]) == type("") \
                                                    and type(a[2][k][1]) == blast_world.BlastPos \
                                                    and type(b[2][k][1]) == blast_world.BlastPos:
                                                eq = False
                                                break
                                    elif type(b[2][k]) == type({}) or type(b[2][k]) == type([]):
                                        raise Exception("Sucky value: " + str(b[2][k]))
                                    elif b[2][k] != a[2][k]:
                                        eq = False
                                        break
                        if not eq: 
                            break
                    if eq:
                        skip_for_equality = True
                        break
            if skip_for_equality:
                continue

            self.planned_worlds.append(world)
            self.worlds_tested = self.worlds_tested + 1

            for robot_name in world[0].robots_keysort:
                robot = world[0].robots[robot_name]
                #Build up all possible states of the world.
                start_t = world_times(world)[robot_name]
                if start_t != lowest_t(world):
                    continue
                #These robots will be gotten on the next expansion of the world.

                #Apply all the initial actions up to the planning point for this robot,
                #which is the end of its action sequence
                w_start = self.initial_world.copy()
                remaining_steps = []
                pre_exec_steps = []
                wait_until = None
                for step in world[1]: #Loop through steps
                    #step start time + length < current
                    if wait_until == None: 
                        if step[0] + step[1] > start_t:
                            wait_until = step[0] + step[1]
                    if step[0] + step[1] <= start_t:
                        pre_exec_steps.append(step)
                        if type(step[2]) != type(""):
                            w_start.take_action(step[2][0], step[2][1], step[2][2])
                    else:
                        remaining_steps.append(step)
            
                robot_types = cached_types[robot.name]
                action_types = cached_actions[robot.name]
                
                if wait_until:
                    wait = wait_until - start_t
                    if wait > 0:
                        self.worlds.append((w_start.copy(), pre_exec_steps + [(start_t, wait, robot_name),] + remaining_steps))

                if dot_graph:
                    dot_graph.write(world_to_dot(w_start) + " [shape=box]\n") #Let us know what gets expanded
                world_clone = w_start.copy()

                for at in action_types: #Limit number of actions for testing.
                    planable, pv = world_clone.enumerate_action(robot_name, at, self.extra_goals)
                    if not planable or pv == False:
                        continue
                    
                    #print robot_name, at, [x for x in self.parameter_iter(pv)]
                    for parameters in self.parameter_iter(pv):
                        self.actions_tested = self.actions_tested + 1
                        
                        change, location_do_not_cares = world_clone.take_action(robot_name, at, parameters)
                        if self.action_type_debug != False:
                            self.action_type_debug[at] = self.action_type_debug.get(at, 0) + 1
                        #print change, robot_name, at, parameters

                        debug = False #w_start.surfaces["clarkelevator"].locations["floor_3"].equal(parameters.get("outfloor", None))
                        if debug:
                            print "action", robot_name, at, parameters

                        
                        if change == None: #failed
                            if debug: print "Action failed to execute"
                            #world_clone.take_action(robot_name, at, parameters, debug=True)  #Failed action print
                            #print "Failed", at, parameters
                            if self.fail_debug != False:
                                self.fail_debug[at] = self.fail_debug.get(at, 0) + 1
                            if self.super_fail_debug != False:
                                self.super_fail_debug[at] = self.super_fail_debug.get(at, [])
                                self.super_fail_debug[at].append(((world_clone.copy(), world[1], world[2]), at, parameters))
                        else: #Action succeeded.
                            change = float(change)
                            self.actions_finished = self.actions_finished + 1

                            #Test for workspace conflict. Sadly this needs to be done after computation
                            #of the action so we know the time the action takes.
                            work_conflict = False
                            comp = world_clone.get_workspaces(robot_name, at, parameters)
                            for step in world[1]:
                                eps_after = start_t + change - step[0]
                                eps_before = step[0] + step[1] - start_t #If this is not < 0
                                #print eps_after, eps_before
                                if not step[0] >= start_t + change and not step[0] + step[1] <= start_t and type(step[2]) != type(""):
                                    c = world_clone.get_workspaces(step[2][0], step[2][1], step[2][2])
                                    for name in c[0].keys():
                                        if name in comp[0]:
                                            work_conflict = True
                                            break
                                    for loc in c[1]:
                                        if work_conflict: break
                                        for loc_c in comp[1]:
                                            if loc_c.equal(loc):
                                                work_conflict = True
                                                break
                                if work_conflict: break
                            
                            if work_conflict:
                                if debug: print "Failed for conflict"
                                failed = True
                            elif world_clone.robots_coliding():
                                if debug: print "Failed for collision"
                                failed = True
                            else:
                                #Now verify we do not duplicate a pre-existing world
                                failed = world_clone.equal(w_start)
                                steps = [(start_t, change, (robot_name, at, parameters)),]
                                steps = pre_exec_steps + steps

                                #print robot.name, w_start.robots[robot.name].location
                                self.evaluate_world(world_clone, (world_clone, steps))
                            #if failed: print "World equals the starting world"
                            if not failed: #This is when you have the rest of your actions
                                for step in remaining_steps:
                                    if type(step[2]) != type(""):
                                        if world_clone.take_action(step[2][0], step[2][1], step[2][2]) == None:
                                            if debug:
                                                print "Taking remaining step failed"
                                            failed = True
                                            break
                                        if world_clone.robots_coliding():
                                            if debug:
                                                print "Taking remaning step resulted in robot hit"
                                            failed = True
                                            break
                                        steps.append(step)
                                        self.evaluate_world(world_clone, (world_clone, steps))
                            if not failed:
                                for w_comp in explored_worlds:
                                    if w_comp.equal(world_clone):
                                        if debug:
                                            print "World is already there", [r.location.to_text() for r in world_clone.robots.itervalues()]
                                        failed = True
                                        break
                            if not failed:
                                explored_worlds.append(world_clone.copy())
                                
                                self.worlds.append((w_start, steps))
                                if dot_graph:
                                    dot_graph.write(world_to_dot(w_start) + " -> " + world_to_dot(world_clone)
                                                    + " [label=\"" + at + "\"]\n")

                            world_clone = w_start.copy()
                            
        if dot_graph:
            dot_graph.write("}\n")
            dot_graph.close()

        #Now we are done planning so we can work it out form here.
        if self.best_world:
            return self.to_plan_seq(self.initial_world, self.best_world)
        else:
            return None, None, None

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
        world, est_time, steps = self.plan_recursive()
        print "Planning time:", time.time() - start_time, "seconds"
    
        if world != None and est_time != None and steps != None:
            print "Estimated time", est_time, "seconds (=", est_time/60.0, "minutes)"
            print "Steps (total of", len(steps), "actions)"
            for i in steps:
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

        return world, est_time, steps


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




#This world implements action queues and the like. It also does action
#gating so things execute in order.

class BlastPlannableWorld:
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
    world_i = blast_world_test.make_table_top_world(True)
    
    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    world_i.types.get_robot("pr2-cupholder"))
    world_i.append_robot(stair5)
    world_i.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.
    world = BlastPlannableWorld(world_i)

    world.no_exec_debug = True
    
    r = world.plan_to_location("stair5", blast_world.BlastPt(15.000, 20.957, 0.148, "clarkcenterfirstfloor"))
    if not r: return False

    r = world.plan_action("stair5", "tuck-both-arms", {}, {"robot-location": {"stair4": blast_world.BlastPt(20.000, 20.957, 0.148, "clarkcenterfirstfloor")}})
    if not r: return False

    
    r = False
    return r

if __name__ == '__main__':
    #print coffee_hunt_test()
    #print run_test()
    #print multi_robot_test()
    print overplan()