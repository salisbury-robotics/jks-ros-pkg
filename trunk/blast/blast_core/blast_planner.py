
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
    __slots__ = ['robot', 'action', 'parameters', 'initial_states', 'final_states', 'locs', 
                 'id', 'gated_on']

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
                 'action_type_debug', 'fail_debug', 'super_fail_debug', 'best_world', 'step_uid']
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
        if self.world_good(world):
            bucket_t = largest_t(world_times(bucket))
            if self.best_world == None:
                self.time_limit = bucket_t
                self.best_world = bucket
            elif bucket_t < self.time_limit:
                self.time_limit = bucket_t
                self.best_world = bucket
                

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

        
    def plan_hunt(self, robot, holder, object_type, wg = None):
        print robot, holder, object_type

        print "Trying to pick up pre-existing object..."
        test_planner = Planner(self.worlds[0][0])
        test_planner.world_good = lambda w: w.objects[w.robots[robot].holders[holder].uid].object_type.name == object_type \
            if w.robots[robot].holders[holder] != None else False
        if wg: test_planner.world_good = wg
        plan_wg_r = test_planner.world_good
        world, est_time, steps = test_planner.plan_print()

        if world != None and est_time != None and steps != None:
            return world, est_time, steps

        #actions = world[0].get_scan_actions()[object_type]
        
        print "Nope! No object present, scanning..."

        world_no_scan = self.worlds[0][0].copy()
        world_no_scan.clear_scan()

        last_scan_count = 0

        last_world = world_no_scan
        ics = last_world.consider_scan
        last_world.consider_scan = True
        last_world.clear_hash("surfaces")
        last_time = 0
        tsteps = []

        while True:
            test_planner = Planner(last_world)
            test_planner.world_good = lambda w: w.scan_count(object_type) > last_scan_count
            world, est_time, steps = test_planner.plan_print()
            if world != None and est_time != None and steps != None:
                last_scan_count = world.scan_count(object_type)
                last_world = world
                last_time = last_time + float(est_time)
                tsteps.extend(steps)
                tsteps.append((robot, None, {"world_good": plan_wg_r}))
            else:
                if last_world.equal(world_no_scan):
                    return None
                last_world.consider_scan = ics
                last_world.clear_hash("surfaces")
                tsteps.append((robot, None, "fail"))
                return last_world, last_time, tsteps
            

        
        #for obj, action_types in ot:
                #Need a better way to schedule.
        #    for a in action_types:
        #        print a
                #new_world = (world[0].copy(), world[1] + float(1000), world[2] + [(robot_name, None, {"object-type"})], world)
                #self.worlds.append(new_world)


    def generate_equivalent_worlds(self, w):
        yield w

    def c(): #This does not seem to help, so it is commented. 
        #Returns clones of worlds with robots swapped, so name of robot is invariant

        rs = []
        for i in xrange(0, len(w.robots_keysort)):
            n1 = w.robots_keysort[i]
            for k in xrange(i + 1, len(w.robots_keysort)):
                n2 = w.robots_keysort[k]
                if w.robots[n1].robot_type == w.robots[n2].robot_type:
                    rs.append((n1, n2))
        for r in xrange(1, len(rs) + 1):
            for x in itertools.combinations(rs, r):
                clone = w.copy()
                for c in x:
                    t = clone.robots[c[0]]
                    clone.robots[c[0]] = clone.robots[c[1]]
                    clone.robots[c[1]] = t
                    clone.robots[c[0]] = clone.robots[c[0]].copy()
                    clone.robots[c[0]].name = c[0]
                    clone.robots[c[1]] = clone.robots[c[1]].copy()
                    clone.robots[c[1]].name = c[1]
                clone.clear_hash("robots")
                yield clone

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

            if self.world_good(world[0]):
                self.good_worlds.append(world)
                t = smallest_t(world[1])
                if not self.time_limit:
                    self.time_limit = t
                if t < self.time_limit:
                    self.time_limit = t
        

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
        for step in target[1]:
            print step
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

    def temp():

        while self.worlds != []:
            world = min(self.worlds, key = lambda x: smallest_t(x[1]))
            self.worlds.remove(world)
            self.planned_worlds.append(world)

            #print "-"*120
            #print world[0].scan_count("coffee_cup"), world[1]
            #for a in world[2]: print a
            #print world[0].to_text()
            #print "-"*120

            #Once we find a good world, we stop planning for worlds after that time
            if self.time_limit != None and smallest_t(world[1]) >= self.time_limit: break
            
            #Actually testing this world
            self.worlds_tested = self.worlds_tested + 1

            for robot_name, robot in world[0].robots.iteritems():
                robot_types = cached_types[robot.name]
                action_types = cached_actions[robot.name]

                #Now enumerate all possible actions
                world_clone = world[0].copy()
                #print robot_name, robot.location.to_text()

                if world[1][robot_name] < largest_t(world[1]):
                    new_time = world[1].copy()
                    new_time[robot_name] = largest_t(world[1]) - world[1][robot_name]
                    steps_c = [x for x in world[2]]
                    steps_c.append()
                    self.worlds.append((world[0], new_time, steps_c, world))

                for at in action_types:
                    planable, pv = world_clone.enumerate_action(robot_name, at, self.extra_goals)
                    if not planable or pv == False:
                        continue
                    #print robot_name, at, [x for x in self.parameter_iter(pv)]
                    for parameters in self.parameter_iter(pv):
                        self.actions_tested = self.actions_tested + 1
                        change, location_do_not_cares = world_clone.take_action(robot_name, at, parameters) 
                        #print robot.location, at, parameters, "->", change
                        if self.action_type_debug != False:
                            self.action_type_debug[at] = self.action_type_debug.get(at, 0) + 1

                        #if world_clone.detect_bad_parenting():
                        #    print "-"*60
                        #    print "Bad parenting: ", world_clone.detect_bad_parenting()
                        #    print "after action:", robot_name, at, parameters
                        #    print world_clone.to_text()
                        #    print "-"*60

                        if change == None: #failed
                            #world_clone.take_action(robot_name, at, parameters, debug=True)  #Failed action print
                            if self.fail_debug != False:
                                self.fail_debug[at] = self.fail_debug.get(at, 0) + 1
                            if self.super_fail_debug != False:
                                self.super_fail_debug[at] = self.super_fail_debug.get(at, [])
                                self.super_fail_debug[at].append(((world[0].copy(), world[1], world[2]), at, parameters))
                        else: #Action succeeded.
                            self.actions_finished = self.actions_finished + 1
                            #Now verify we do not duplicate a pre-existing world
                            failed = world_clone.equal(world[0])
                            #if not world[0].robots[robot_name].location.equal(world_clone.robots[robot_name].location) and failed:
                            #    print "Weird!"
                            #    print world[0].to_text()
                            #    print "O"*60
                            #    print world_clone.to_text()
                            #    print "X"*60
                            #if failed:
                            #    print "Failed because equal to previous world"
                            if not failed: #Compare to parent worlds
                                parent = world[3]
                                while parent and not failed:
                                    if parent[0].equal_valid(world_clone):
                                        failed = True
                                    parent = parent[3]
                            #if not failed:
                            #    for c_name, c_robo in world_clone.robots.iteritems():
                            #        for c_name2, c_robo2 in world_clone.robots.iteritems():
                            #            if c_name != c_name2:
                            #                if c_robo.collide(c_robo2):
                            #                    failed = True
                            #                    break
                            #        if failed: break
                            if not failed:
                                for world_compare in self.generate_equivalent_worlds(world_clone):
                                    for world_cmp in self.worlds:
                                        if world_cmp[0].equal_valid(world_compare):
                                            failed = True
                                            #print "Equal to another world to be tested"
                                            break
                                    if failed: break
                            if not failed:
                                for world_cmp in self.planned_worlds:
                                    if world_cmp[0].equal_valid(world_clone):
                                        failed = True
                                        #print "Equal to a previously tested world"
                                        break
                            if not failed:
                                new_time = world[1].copy()
                                new_time[robot_name] = new_time[robot_name] + float(change)

                                new_world = (world_clone, new_time, world[2] + [(robot_name, at, parameters)], world)
                                self.worlds.append(new_world)
                                if self.world_good(world_clone):
                                    if self.time_limit == None:
                                        self.time_limit = smallest_t(new_world[1])
                                    if self.time_limit > new_world[1]:
                                        self.time_limit = smallest_t(new_world[1])
                                    self.good_worlds.append(new_world)
                            
                            world_clone = world[0].copy()

        self.good_worlds.sort(key=lambda x: smallest_t(x[1]))
        if self.good_worlds != []:
            def clean_actions(actions):
                r = []
                for action in actions:
                    action_clean = {}
                    for param, value in action[2].iteritems():
                        if value.__class__ == blast_world.BlastSurface:
                            action_clean[param] = value.name
                        elif value.__class__ == blast_world.BlastPt:
                            action_clean[param] = value.to_dict()
                        else:
                            action_clean[param] = value
                    r.append((action[0], action[1], action_clean))
                return r
            return self.good_worlds[0][0], largest_t(self.good_worlds[0][1]), clean_actions(self.good_worlds[0][2])
        else:
            print "No world found"
        return None, None, None
        
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




class BlastPlan:
    def __init__(self, world, world_good, extra_goals, consider_scan, uid, extra_steps): #Failure replan returns new steps
        self.world = world
        self.uid = uid
        self.world_good = world_good
        self.extra_goals = extra_goals
        self.extra_steps = extra_steps
        self.consider_scan = consider_scan
        self.done = False

    def plan(self, plan_and_return = False, report_plan = False):
        self.world.planning_lock.acquire()
        planner = Planner(self.world.get_planning_world())
        planner.step_uid = str(self.uid) + "/"
        planner.world_good = self.world_good
        planner.extra_goals = self.extra_goals
        world, est_time, steps = planner.plan_print()
        if world == None and est_time == None and steps == None:
            self.done = True
            return False
        if plan_and_return:
            self.done = True
            return world, est_time, steps

        for step in steps:
            self.world.robot_locks[step.robot].acquire()
            self.world.robot_queues[step.robot].append(step)
            self.world.robot_locks[step.robot].release()

        self.world.planning_lock.release()

        if self.world.real_world:
            raise Exception("Not implmented yet, will need to use semaphores here")
        else:
            while True:
                still_there = False
                for robot, queue in self.world.robot_queues.iteritems():
                    self.world.robot_locks[robot].acquire()
                    for step in queue:
                        if step.id.split("/")[0] == str(self.uid):
                            still_there = True
                    self.world.robot_locks[robot].release()
                if not still_there: break
                self.world.try_operate()
                    

        
        self.done = True
        if report_plan:
            return world, est_time, steps
        return True




#This world implements action queues and the like. It also does action
#gating so things execute in order.

class BlastPlannableWorld:
    def __init__(self, world):
        self.world = world
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
        self.plans = [x for x in self.plans if x.done]
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
        clone_world = self.world.copy()
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

    #Documentation:
    #plan_and_return = instantly execute, return immediately
    #report_plan = at the end return the plan instead of True/None

    def plan_hunt(self, robot, holder, object_type, world_good = None, plan_and_return = False, 
                  report_plan = False, execution_cb = lambda x: None, internal = False):
        self.lock.acquire()
        planner = Planner(self.get_planning_world())
        r = planner.plan_hunt(robot, holder, object_type, world_good)
        if r == None:
            self.lock.release()
            return None
        world, est_time, steps = r
        def rl(world, arobot, action, parameters):
            return self.plan_hunt(robot, holder, object_type,
                                  world_good, plan_and_return,
                                  report_plan, execution_cb, internal = True)
        w = self.exec_plan(world, est_time, steps, plan_and_return, report_plan, execution_cb, rl)
        if internal:
            return w
        if not w: return False
        if len(w) != 2: return False
        w, o = w
        if not w: return False
        orf = self.world.robots[robot].holders[holder]
        if not orf: return None
        return o[orf.uid]

    def next_plan(self):
        scp = self.n_planning_event = self.n_planning_event + 1
        return scp - 1

    def plan(self, world_good, extra_goals, plan_and_return = False, report_plan = False, execution_cb = lambda x: None,
             failure_cb = lambda world, robot, action, parameters: False, extra_steps = []):
        self.plans_lock.acquire()
        plan = BlastPlan(self, world_good, extra_goals, False, self.next_plan(), extra_steps)
        self.plans.append(plan)
        self.plans_lock.release()
        return plan.plan(plan_and_return, report_plan)

        #planner = Planner(self.get_planning_world())
        #planner.world_good = world_good
        #planner.extra_goals = extra_goals
        #world, est_time, steps = planner.plan_print()
        #return self.exec_plan(world, est_time, steps, plan_and_return, report_plan, execution_cb, failure_cb)


    ######################
    def exec_plan(self, world, est_time, steps, plan_and_return = False, report_plan = False, 
                  execution_cb = lambda x: None, failure_callback = lambda world, robot, action, parameters: False):
        if plan_and_return:
            return world, est_time, steps, None

        if not (world != None and est_time != None and steps != None):
            return None

        ol = {}

        for step in steps:
            
            if step.action == None:
                print "Special step - skip"
                raise Exception("Not yet bro")
                continue
            

        
        if report_plan:
            return world, est_time, steps, ol
        return True, ol



    def crud():

        ol = {}
        if world != None and est_time != None and steps != None:
            print "Taking", len(steps), "steps"
            x = 0

            used_robots = set()

            def finish():
                for robot in used_robots:
                    self.robot_action_locks[robot].acquire()
                    self.robot_action_locks[robot].release()
                    self.robot_locks[robot].release()

            for step in steps:
                execution_cb(steps[x:])
                x = x + 1
                
                if step.action == None:
                    if step.parameters == "fail": 
                        execution_cb([])
                        finish()
                        return False
                    print "REPLAN!!!", step

                    finish()
                    
                    planner = Planner(self.world.copy())
                    planner.world_good = step.parameters["world_good"]
                    planner.extra_goals = step.parameters.get("extra_goals", {})
                    n_world, n_est_time, n_steps = planner.plan_print()
                    if n_world != None and n_est_time != None and n_steps != None:
                        print "Plan worked!!!!", n_steps
                        return self.exec_plan(n_world, n_est_time, n_steps, plan_and_return, report_plan, execution_cb)
                    else:
                        print "Replan failed, continuing default course."

                    continue

                if not step.robot in used_robots:
                    self.robot_locks[step.robot].acquire()
                    used_robots.add(step.robot)

                for uid in self.world.objects_keysort:
                    if self.world.objects[uid].position:
                        ol[uid] = (uid, self.world.objects[uid].position, self.world.objects[uid].parent)


                #Seperate thread
                r = self.take_action(step.robot, step.action, step.parameters)
                if r == None:
                    print "EPICALLY FAILED"
                    finish()
                    return None
                elif r == False:
                    print "FAILED"
                    finish()
                    return failure_callback(self.world.copy(), step.robot, step.action, step.parameters)
            execution_cb([])
            finish()
            #print world.to_text()
            if not world.equal(self.world) and not self.real_world:
                print '-'*60, "GOAL:"
                print self.world.to_text()
                print '-'*60, "GEN:"
                print world.to_text()
                print '-'*60
                print "FAILED! Big issue: took actions that did not produce the desired world state"
                return None
        else:
            print "FAILED!"
            return None

        if report_plan:
            return world, est_time, steps, ol
        return True, ol
    
    def plan_to_location(self, robot, location):
        if self.world.robots[robot].location.equal(location):
            print "Already at location"
            return
        return self.plan(lambda w: w.robots[robot].location.equal(location), 
                         {"Pt": [location,]})

    
    #API actions ---------------------------
    def robot_transfer_holder(self, robot, from_holder, to_holder):
        if not robot in self.world.robots:
            print "Set robot holder invalid robot", robot
            return False
        if not from_holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", from_holder, "for robot", robot
            return False
        if not to_holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", to_holder, "for robot", robot
            return False
        self.world.robot_transfer_holder(robot, from_holder, to_holder)
        return True
    
    def robot_pick_object(self, robot, uid, to_holder):
        if not robot in self.world.robots:
            print "Robot pick object invalid robot", robot
            return False
        if not uid in self.world.objects_keysort:
            print "Robot pick object invalid object", uid, "for robot", robot
            return False
        if not to_holder in self.world.robots[robot].holders:
            print "Robot pick object invalid holder", to_holder, "for robot", robot
            return False
        self.world.robot_pick_object(robot, uid, to_holder)
        return True
    
    def robot_place_object(self, robot, from_holder, surface, pos):
        if not robot in self.world.robots:
            print "Robot place object invalid robot", robot
            return False
        if not surface in self.world.surfaces:
            print "Robot place object invalid surface", surface, "for robot", robot
            return False
        if not from_holder in self.world.robots[robot].holders:
            print "Robot place object invalid holder", to_holder, "for robot", robot
            return False
        if self.world.robots[robot].holders == None:
            print "Robot place object empty holder", to_holder, "for robot", robot
            return
        self.world.robot_place_object(robot, from_holder, surface, pos)
        return True
    
    def set_robot_holder(self, robot, holder, object_type, require_preexisting_object = True):
        if not robot in self.world.robots:
            print "Set robot holder invalid robot", robot
            return False
        if not holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", holder, "for robot", robot
            return False
        if require_preexisting_object:
            if self.world.robots[robot].holders[holder] == None:
                print "Robot holder empty, set requires object for holder", holder, "for robot", robot
                return False
        self.world.set_robot_holder(robot, holder, object_type)
        return True

    def get_robot_holder(self, robot, holder):
        if not robot in self.world.robots:
            print "Set robot holder invalid robot", robot
            return False
        if not holder in self.world.robots[robot].holders:
            print "Set robot holder invalid holder", holder, "for robot", robot
            return False
        return self.world.get_robot_holder(robot, holder)
    

    def set_robot_position(self, robot, position, val):
        if not robot in self.world.robots:
            print "Set robot position invalid robot", robot
            return False
        if not position in self.world.robots[robot].positions:
            print "Set robot holder invalid position", position, "for robot", robot
            return False
        self.world.set_robot_position(robot, position, val)
        return True
    
    def set_robot_location(self, robot, blast_pt):
        if not robot in self.world.robots:
            print "Set robot location invalid robot", robot
            return False
        self.world.set_robot_location(robot, blast_world.BlastPt(blast_pt['x'], blast_pt['y'],
                                                                 blast_pt['a'], blast_pt['map']))
        return True

    def get_surface(self, surface):
        sur = self.world.surfaces.get(surface)
        if sur != None: sur = sur.to_dict()
        return sur
    
    def get_object(self, obje):
        obj = self.world.objects.get(obje)
        if obj != None: obj = obj.to_dict()
        return obj

    def delete_surface_object(self, obje):
        obj = self.world.objects.get(obje)
        if obj == None:
            print "Surface delete object invalid object", obje
            return False
        return self.world.delete_surface_object(obje)

    def get_map(self, map):
        mp = self.world.maps.get(map)
        if mp != None: mp = mp.to_dict()
        return mp
    
    def get_robot(self, robot):
        rb = self.world.robots.get(robot)
        if rb != None: rb = rb.to_dict()
        return rb

    def surface_scan(self, surface, object_types):
        if not surface in self.world.surfaces:
            print "Invalid surface for scanning", surface
            return False
        return self.world.surface_scan(surface, object_types)

    def add_surface_object(self, surface, object_type, pos):
        if not surface in self.world.surfaces:
            print "Invalid surface for object addition", surface
            return False
        return self.world.add_surface_object(surface, object_type, pos)
        

    def plan_place(self, uid, surface, pos, plan_and_return = False, include_action = False, execution_cb = lambda x: None):
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
                      

    def plan_action(self, robot, action, parameters, world_limits = {}, plan_and_return = False, include_action = False, execution_cb = lambda x: None):
        #FIXME: this can create problems if parameters is an extra element
        extras = {}
        if "robot-location" in world_limits:
            for robot, location in world_limits["robot-location"].iteritems():
                extras["Pt"] = extras.get("Pt", [])
                extras["Pt"].append(blast_world.BlastPt(location['x'], location['y'], location['a'], location['map']))

        extra_steps = [ (robot, action, parameters), ]
        r = self.plan(lambda w: w.world_limit_check(world_limits) and w.take_action(robot, action, parameters, False, False)[0] != None, 
                      extras, plan_and_return = plan_and_return, report_plan = True, 
                      execution_cb = lambda x: execution_cb(x + [(robot, action, parameters),]),
                      extra_steps = extra_steps
                      )
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
    
    def rest():

        if self.real_world:
            test_world = self.world.copy()
            if test_world.take_action(robot, action, parameters, False, debug) == None:
                print "Attempted to take action in a state that was not desirable"
                self.robot_action_locks[robot].release()
                return None
            else:
                action_r = self.action_callback(robot, action, parameters)
                time_el = None
                ldc = None
                if action_r == None:
                    print "Action callback failed to work, going to epic fail"
                    self.action_epic_fail_callback(robot, action, parameters)
                    self.robot_action_locks[robot].release()
                    return None
                elif action_r == True:
                    time_el, ldc = test_world.take_action(robot, action, parameters, True, debug)
                else:
                    print "Action failed in a predictable way"
                    time_el, ldc = test_world.take_action(robot, action, parameters, True, debug, action_r)
                if ldc != None:
                    for robot in ldc:
                        test_world.robots[robot].location = self.world.robots[robot].location.copy()
                if not test_world.equal(self.world, True): #Important to tolerate arm error
                    print "-"*60
                    print test_world.to_text()
                    print "-"*60
                    print self.world.to_text()
                    print "-"*60
                    print "The output world differs, epic fail!"
                    print "-"*60, "delta", "-"*60
                    for line_self, line_other in zip(test_world.to_text().split("\n"), self.world.to_text().split("\n")):
                        if line_self.strip() != line_other.strip():
                            print "correct:", line_self.strip()
                            print "output:", line_other.strip()
                    print "-"*60, "done", "-"*60
                    self.action_epic_fail_callback(robot, action, parameters)
                    self.robot_action_locks[robot].release()
                    return None
                if action_r != True:
                    self.robot_action_locks[robot].release()
                    return False
            self.robot_action_locks[robot].release()
            return True
        elif self.world.take_action(robot, action, parameters, True, debug):
            if not self.action_callback(robot, action, parameters):
                print "Action callback failed to work, this test may be broken."
                self.robot_action_locks[robot].release()
                return None
            self.robot_action_locks[robot].release()
            return True
        else:
            print "Blast failed to take action"
            self.robot_action_locks[robot].release()
            return None
        self.robot_action_locks[robot].release()
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
    world = BlastPlannableWorld(blast_world_test.make_table_top_world(True))
    
    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    world.world.types.get_robot("pr2-cupholder"))
    world.world.append_robot(stair5)
    world.world.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.

    print "-"*180
    print world.world.to_text()
    print "-"*180

    
    r = world.plan_hunt("stair4", "cupholder", "coffee_cup")

    print "-"*180
    print world.world.to_text()
    print "-"*180
    
    return r
    

if __name__ == '__main__':
    #coffee_hunt_test()
    print run_test()
    #multi_robot_test()
