
import blast_world, time, json, itertools, hashlib, random, string, threading, thread
import heapq, traceback

#### Planning process
# 1. Plans are made from macro calls in the macro code structure
# 2. Plans are then executed by the reasoner.
#

motion_plan_misses = 0
motion_plan_hits = 0
def bin_to_hex(hexdata):
    return ''.join('%02x' % ord(byte) for byte in str(hexdata))

def parameter_iter(param, keys = None):
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
        #print "Keys none ->", keys
    if keys == []: 
        yield {}
        return 
    key = keys[0]
    keys = keys[1:]
  
    for next_d in parameter_iter(param, keys):
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


def all_combinations(c, min, len):
    for r in xrange(min, len + 1):
        for i in itertools.combinations(c, r):
            yield i


class Planner(object):
    __slots__ = ['initial_world', 'code_exc', 'point_plans', 'world_gen_tree']
    def __init__(self, initial_world, code_exc):
        self.initial_world = initial_world.copy()
        self.code_exc = code_exc
        self.world_gen_tree = {}
        self.point_plans = {}

    def get_action_workspaces(self, robot, action, parameters):
        return self.initial_world.get_workspaces(robot, action, parameters)

    def get_workspaces(self, actions, time):
        surfaces = {None: set()}
        for action in actions:
            if action[1] != "ACTION": continue
            if action[0] <= time and action[0] + action[2] > time:
                #print time, action
                sr, loc = self.get_action_workspaces(action[3], action[4], action[5])
                for s, vs in sr.iteritems():
                    surfaces[s] = surfaces.get(s, set())
                    for v in vs:
                        surfaces[s].add(v)
                for l in loc:
                    surfaces[None].add(l)
        return surfaces


    def motion_plan_state(self, robot_name, w):
        hl = hashlib.sha224()
        get_obj = lambda x: w.get_obj(x)
        w.robots[robot_name].hash_update(hl, get_obj, False)
        return hl.digest()

    def get_cached_motion_plan(self, robot_type, start, end):
        return self.point_plans[robot_type][start][end]

    def motion_plan(self, robot_name, start_w, end_w):
        global motion_plan_hits, motion_plan_misses

        #We actually need to plan from one to the other
        #TODO: this should probably construct a world with no objects or other robots
        #TODO: also, it may have problems with actions that depend on object presence
        #      but do not move the objects. We should not allow that.
        start = self.motion_plan_state(robot_name, start_w)
        end = self.motion_plan_state(robot_name, end_w)

        robot_type = start_w.robots[robot_name].robot_type.name

        if not robot_type in self.point_plans:
            self.point_plans[robot_type] = {}
        if not start in self.point_plans[robot_type]:
            self.point_plans[robot_type][start] = {}
        if end in self.point_plans[robot_type][start]:
            r = self.point_plans[robot_type][start][end]
            #print "FAST RETURN"
            motion_plan_hits = motion_plan_hits + 1
            return r[0], r[1], robot_type, start, end
        motion_plan_misses = motion_plan_misses + 1

        return self.actual_motion_plan(robot_name, robot_type, start, end, start_w, end_w)

    def actual_motion_plan(self, robot_name, robot_type, start, end, start_w, end_w):

        worlds = [(0, (start_w, 0, []))]
        planned_worlds = []
        w_hash = {}
        best_world = None

        #TODO add all elements of end robot state
        extra_goals = {'Pt': [end_w.robots[robot_name].location.copy(),]}


        while worlds != []:
            w = heapq.heappop(worlds)[1]
            planned_worlds.append(w)

            #Avoid already done worlds. If we get to one of the already
            #done worlds but do it faster then we still plan
            hs = w[0].get_hash_state()
            if not hs in w_hash:
                w_hash[hs] = w[1]
            if w_hash[hs] >= w[1]:
                w_hash[hs] = w[1]
            else:
                continue #Already planned
            
            if best_world:
                if w[1] > best_world[1]: #Skip worlds that are after the fastest time to finish
                    continue
            if w[0].equal(end_w, tolerant=True):
            #print w[0].robots[robot_name].location.to_text()
                best_world = w
            
            for action in w[0].enumerate_robot(robot_name):
                planable, plist = w[0].enumerate_action(robot_name, action, extra_goals)
                #print robot_name, action, "->", plist
                if not planable: continue
                if plist == None or plist == False: continue
                for parameters in parameter_iter(plist):
                    #print robot_name, action, "->", parameters
                    w_next = w[0].copy()
                    change, location_do_not_cares = w_next.take_action(robot_name, action, parameters)
                    if change == None:
                        continue
                    wn = (w[1] + change, (w_next, w[1] + change, w[2] + [(action, parameters, change), ]))
                    heapq.heappush(worlds, wn)
                    
        if best_world:
            self.point_plans[robot_type][start][end] = (best_world[1], best_world[2])
        else:
            print "Failed to plan"
            print start_w.to_text(), "TO",
            print end_w.to_text()
            self.point_plans[robot_type][start][end] = (None, None)
        r = self.point_plans[robot_type][start][end]
        return r[0], r[1], robot_type, start, end

    def test_motion_plan(self):
        end_w = self.initial_world.copy()
        end_w.robots["stair4"] = end_w.robots["stair4"].copy()
        end_w.robots["stair4"].location = blast_world.BlastPt(0, 0, 0, "clarkcenterfirstfloor")
        return self.motion_plan("stair4", self.initial_world, end_w)


    #Plan formats:
    #(time, "EXEC", program_uid, end_state)
    #(time, "ACTION", length, robot, action, parameters)    runs through [time, time+length)
    #(time, "BLOCK", length, robot) runs through [time, time+length)
    
    #Generate a world at a given time. Uses optimization to speed up world selection.
    def generate_world(self, plan, at_time, do_optimize = True, ret_code_exc = False):
        steps = []
        cxc = [c.copy() for c in self.code_exc]
        prog_ord_max = 0
        for step in plan:
            if step[1] == "ACTION":
                if step[0] + step[2] <= at_time:
                    steps.append(step)
            elif step[1] == "EXEC":
                if step[0] <= at_time:
                    prog_ord_max = max(prog_ord_max, step[4])
                    steps.append(step)
            elif step[1] == "SETPLAN":
                if step[0] <= at_time:
                    prog_ord_max = max(prog_ord_max, step[5])
                    steps.append(step)
                
        # (300 - ord(x)) S -> L -> E -> A sorts so that SET, then EXEC, then ACTION
        #for x in steps: print x, x[0], x[2] #, (x[0] + x[2])
        prog_ord_m = 1
        while prog_ord_max >= prog_ord_m:
            prog_ord_m = prog_ord_m * 10
        def sort_keys(x):
            ftime = x[0]
            if x[1] == "ACTION": ftime = ftime + x[2]
            otype = 100 #(300 - ord(x[1][0]))
            if x[1] == "ACTION": otype = 200
            prog_ord = 0
            if x[1] == "EXEC": prog_ord = x[4]
            if x[1] == "SETPLAN": prog_ord = x[5]
            return ftime * prog_ord_m * 1000 + prog_ord * 1000 + otype
        steps.sort(key = sort_keys)
        w = self.initial_world
        need_clone = True
        wgt = self.world_gen_tree
        #print "---"

        opt_debug = False #Debug optimization by comparing to un-optimized
        ds = ""

        for step in steps:
            #print step
            uids = [c.uid for c in cxc]
            uids.sort()
            sstep = str(step) + str(uids)
            if opt_debug: ds = ds + "\n" + sstep
            if sstep in wgt and do_optimize:
                w = wgt[sstep][1]
                cxc = wgt[sstep][2]
                wgt = wgt[sstep][0]
                need_clone = True
                continue
            if need_clone:
                w = w.copy()
                cxc = [c.copy() for c in cxc]
                need_clone = False
            if step[1] == "ACTION":
                a, b = w.take_action(step[3], step[4], step[5])
                #print step[3:], "->", a, b
                if a == None and b == None: return None
            elif step[1] == "EXEC":
                cstep = None
                for s in cxc:
                    if s.uid == step[2]: cstep = s
                if cstep == None: raise Exception("Invalid program: " + str(step[2]))
                cstep.execute(w)
                if cstep.get_hash_state() != step[3]:
                    s = "Not specified"
                    if len(step) > 5: s = step[5]
                    for i in steps:
                        print i
                        if i == step: print "THAT WAS IT"
                    raise Exception("Bad state for program (" + bin_to_hex(cstep.get_hash_state()) \
                                        + "):\n" + cstep.to_text() + "\nGood state (" \
                                        + bin_to_hex(step[3]) + "):\n" + s)
                    return None
            elif step[1] == "SETPLAN":
                cstep = None
                for s in cxc:
                    if s.uid == step[2]: cstep = s
                if cstep == None: raise Exception("Invalid program: " + str(step[2]))
                
                if step[3] != None and step[4] != None:
                    cstep.set_plan_executed(step[3], step[4])
                elif step[3] == None and step[4] == None:
                    pass #This forces a replan
                elif step[3] == None and type(step[4]) == list:
                    cstep.set_plan_code(step[4])
                elif step[3] == None and step[4] == False:
                    cstep.set_plan_code_done()
                else:
                    for s in steps: print s
                    raise Exception("Weird program SETPLAN: " + str(step))
            else:
                raise Exception("Invalid step: " + str(step))
            if do_optimize:
                wgt[sstep] = ({}, w.copy(), [c.copy() for c in cxc])
                wgt = wgt[sstep][0]
        if need_clone:
            w = w.copy()
            cxc = [c.copy() for c in cxc]
            need_clone = False
        #For debugging, comment and false. This tests if we get a different
        #world as a result of optimization, which can happen if hashing is
        #broken
        if do_optimize and opt_debug:
            wc = self.generate_world(plan, at_time, do_optimize = False)
            if not wc.equal(w):
                raise Exception("World differs at " + str(at_time) + ":" + ds
                                + "\n---------- REAL ---------\n" + wc.to_text()
                                + "\n---------- OPTO ---------\n" + w.to_text()
                                + "\n---------- DIFF ---------\n" + w.diff(wc))
        if ret_code_exc:
            return [c.copy() for c in cxc]
        return w


    #Return a dictonary of lists of times expressed as
    #[start_time, end_time] when robots are active
    def plan_get_robot_active_times(self, plan, robots = None):
        robot_action_times = {}
        for step in plan:
            if step[1] == "ACTION" or step[1] == "BLOCK": #Is an action
                if not step[3] in robot_action_times:
                    robot_action_times[step[3]] = []
                merged = False
                for rblock in robot_action_times[step[3]]:
                    if merged: break
                    if rblock[0] == step[0] + step[2]: #block starts at end
                        rblock[0] = step[0]
                        merged = True
                    if rblock[1] == step[0]: #block is ends at start
                        rblock[1] = step[0] + step[2]
                        merged = True
                if not merged:
                    robot_action_times[step[3]].append([step[0], step[0] + step[2]])

        for robot in self.initial_world.robots_keysort:
            robot_action_times[robot] = robot_action_times.get(robot, [])
        if robots != None:
            rat = {}
            for r in robots:
                rat[r] = robot_action_times.get(r, [])
            return rat
        return robot_action_times

    def merge_plans(self, plan_a, start_plan_a, plan_b, start_plan_b, extra_step_a = None, debug = False):
        #Temporarily adjust the plans.
        plan_a = [[s[0] + start_plan_a,] + list(s[1:]) for s in plan_a]
        plan_b = [[s[0] + start_plan_b,] + list(s[1:]) for s in plan_b]

        #Ensure we don't have robot overlap
        for step_a in plan_a:
            if step_a[1] != "ACTION" and step_a[1] != "BLOCK": continue
            for step_b in plan_b:
                if step_b[1] != "ACTION" and step_b[1] != "BLOCK": continue
                if step_b[3] != step_a[3]: continue #Different robot
                if step_a[0] + step_a[2] <= step_b[0]: continue #Step is before or they meet
                if step_b[0] + step_b[2] <= step_a[0]: continue #Step is after or they meet
                if debug: print "Merge failed: robot conflict:", step_a, step_b
                return None #We have a robot trying to do two actions at the same time

        #Check to ensure that the workspaces do not overlap
        times = set([])
        for step in plan_a + plan_b:
            times.add(step[0])
            if step[1] == "ACTION" or step[1] == "BLOCK":
                times.add(step[2])
        times = list(times)
        times.sort()
        for timestep in times:
            splan_a = self.get_workspaces(plan_a, timestep)
            splan_b = self.get_workspaces(plan_b, timestep)
            for plan_a_sn in splan_a.iterkeys():
                if plan_a_sn != None:
                    if plan_a_sn in splan_b:
                        if debug: print "Merge Failed: Workspace conflict at", timestep, "over surface", plan_a_sn
                        return None
            for plan_b_sn in splan_b.iterkeys():
                if plan_b_sn != None:
                    if plan_b_sn in splan_a: 
                        if debug: print "Merge Failed: Workspace conflict at", timestep, "over surface", plan_b_sn
                        return None
            for item_a in splan_a[None]:
                for item_b in splan_b[None]:
                    if item_a.equal(item_b):
                        if debug: print "Merge Failed: Workspace conflict at", timestep, "over", item_a.to_text()
                        return None

        #Figure out the details of the extra_action_a
        merged_plan = sorted(plan_a + plan_b, key=lambda x: x[0])
        if (extra_step_a):
            if len(plan_a) > 0:
                end_plan_a = max(plan_a, key=lambda x: x[0] + x[2])
                end_plan_a = end_plan_a[0] + end_plan_a[2]
            else:
                end_plan_a = start_plan_a
            #print "GEN"
            w = self.generate_world(merged_plan, end_plan_a)
            if not w: 
                if debug: print "Merge Failed: Failed to generate word for extra_step"
                return None
            #print extra_step_a
            atime, b = w.take_action(extra_step_a[0], extra_step_a[1], extra_step_a[2])
            if atime == None and b == None: 
                if debug: print "Merge Failed: Failed to run extra step", extra_step_a
                return None
            extra = [end_plan_a, "ACTION", atime] + extra_step_a
            merged_plan.append(extra)
            merged_plan.sort(key=lambda x: x[0])
            #return self.merge_plans(plan_a + [extra, ], start_plan_a, plan_b, start_plan_b)

        #Ensure that all actions actually can be executed. This often fails if we take an
        #action that breaks the future world.
        if len(merged_plan) > 0:
            end = max(merged_plan, key=lambda x: x[0] + x[2] if x[1] == "ACTION" else x[0])
            end = end[0] + end[2]
        else:
            end = 0
        if not self.generate_world(merged_plan, end):
            if debug: print "Merge Failed: Failed to generate final world"
            return None
        
        return merged_plan

    def get_robot_min_times(self, mt, plan, robots):
        robot_action_times = self.plan_get_robot_active_times(plan, robots)
        min_times = {}
        for robot in robot_action_times:
            min_times[robot] = mt
            for rblock in robot_action_times[robot]:
                if rblock[0] <= mt and mt < rblock[1]:
                    if rblock[1] > min_times[robot]:
                        min_times[robot] = rblock[1]
        return min_times
        
    def merge_motion_plan(self, w_start, w_end, robot, min_time, cplan, extra_step = None):
        mplan = None
        tmplan = 0
        new_plan = None
        if w_end.equal(w_start):
            mplan = []
        else:
            motion_time, motion_plan, rt, start_hash, end_hash = self.motion_plan(robot, w_start, w_end)
            if motion_plan == None:
                mplan = None
            else:
                mplan = []
                tmplan = 0
                for p in motion_plan:
                    mplan.append([tmplan, "ACTION", p[2], robot, p[0], p[1]])
                    tmplan = tmplan + p[2]

        if mplan != None:
            #print mplan
            new_plan = self.merge_plans(mplan, min_time, cplan, 0, extra_step)
        return new_plan


    def get_robot_pose_world(self, w_start, robot, action, parameters, state):
        w_end = w_start.copy()
        pos_set = set()
        w_end.robots[robot] = w_end.robots[robot].copy()
        for s, v in state.iteritems():
            if s == 'robot.location':
                w_end.robots[robot].location = v.copy()
            elif len(s.split(".")) > 1 and s.split(".")[1] in w_end.robots[robot].positions:
                n = s.split(".")[1]
                d = {}
                for x in zip(w_end.robots[robot].robot_type.position_variables[n][False][0], v):
                    d[x[0]] = x[1]
                w_end.robots[robot].positions[n] = d
            else:
                raise Exception("Illegal state variable from action_robot_pose: " + s
                                + " in " + str(action) + " with " + str(parameters) 
                                + " requiring " + str(state))
        for p in w_end.robots[robot].positions.iterkeys():
            if p not in pos_set:
                w_end.robots[robot].position_do_not_care(p)
        w_end.clear_hash("robots")
        return w_end

    #This function plans to a new world state as given as a goal by a program. This
    #goal can include many fields, but mainly consists of extra_steps and world_limits.
    #The world_limits are steps we take after we achieve a goal. The world_limits are
    #conditions that must be true before we start extra_steps. Note that the world_limits
    #are not garunteed to be true after we take the extra_steps.
    #This function does a BFS plan out from the initial state taking all possible actions.
    #it may recursively plan out for program actions, and also calls for motion planning.
    #
    #plan - is the current plan
    #robots - is the list of names of robots which we plan on
    #start_time - is the time in the planning process when we start
    #goal - is the goal dictionary returned by the program
    #uid - is the UID of the program to plan for
    #prog_ord - is the ordinal number used for program step counting
    #end_time - absolute upper limit for the end time of the sequence
    def plan_to_prog(self, plan, robots, start_time, goal, uid, prog_ord, end_time = None):
        #Reset initial world hash, so that we are confident of it. This is not
        #necessary but is a nice sanity insurance mechanism
        [self.initial_world.clear_hash(x) for x in ['robots', 'surfaces', 'objects', 'maps']]
        self.initial_world.consider_scan = True
        initial_hs = self.initial_world.get_hash_state()

        debug = False

        if debug:
            print
            print
            print
            print '-'*100
            print goal
            print start_time
            print robots
            print
            for s in plan:
                print s
            print
            print
            if plan != []:
                et = plan[-1][0]
                if plan[-1][0] == "ACTION" or plan[-1][0] == "BLOCK": et = et + plan[-1][2]
                print self.generate_world(plan, et).to_text()
            else:
                print self.initial_world.to_text()
            print

        #worlds is the list of possible BFS expansion points.
        #planned_worlds is the list already planned worlds for duplicate pruning.
        #pw_hash is a list of planned worlds for placement.
        worlds = [(start_time, plan), ]
        planned_worlds = []
        pw_hash = {}

        #wl_uids is the uid list for object placement. Object placement is
        #a tricky subject because objects are referenced based on location at
        #the start of the program step. Getting the UIDs here ensures that the
        #exact object is placed, not an equivalent one.
        wl_uids = {}
        if 'world_limits' in goal and 'place-objects' in goal['world_limits']:
            for it in goal['world_limits']['place-objects']:
                wl_uids[it['object'].uid] = it
        
        #best_world is our guess of the best possible result of the action.
        #This is the world which meets the criteria but has the shortest
        #possible time to get to. If None, then no suitable world was found.
        best_world = None
        while worlds != []:
            #Find the world with the lowest time location and add it to the
            #planned_worlds list.
            world = min(worlds, key=lambda x: x[0])
            worlds.remove(world)
            planned_worlds.append(world)

            #COMMENTED DEBUG = print time and plan
            if debug:
                print 
                print world[0]
                for s in world[1]:
                    print s

            #COMMENTED DEBUG = test if initial world has weird state
            #[self.initial_world.clear_hash(x) for x in ['robots', 'surfaces', 'objects', 'maps']]
            #print "LOC", self.initial_world.robots["stair4"].location.to_text()
            #if self.initial_world.get_hash_state() != initial_hs:
            #    raise Exception("Something altered the initial world")

            #Update the time of the world - if all robots in use at
            #the time t.
            world[1].sort(key = lambda x: x[0])

            #Find the end time of all current robot actions at the
            #time specified
            min_times = self.get_robot_min_times(world[0], world[1], robots)

            #Loop through the robots and expand all those that
            #can do things at the min_time.
            for robot, min_time in min_times.iteritems():
                if best_world: #Avoid worlds after the best is found
                    if min_time > best_world[0]:
                        if debug: print "Ignore time for too late", min_time, "for", robot
                        continue
                if end_time: #Avoid worlds after the end time
                    if min_time > end_time:
                        if debug: print "Ignore end time too late", end_time, "for", robot
                        continue

                #Avoid duplications if we are a duplicate world later in time.
                w_start = self.generate_world(world[1], min_time)
                if w_start == None:
                    print "Warning, world generation failed"
                    continue
                [w_start.clear_hash(x) for x in ['robots', 'surfaces', 'objects', 'maps']]
                w_start.consider_scan = True
                hs = w_start.get_hash_state()
                if hs in pw_hash and robot in pw_hash[hs]:
                    if pw_hash[hs][robot] <= min_time:
                        if debug: print "Ignore time for duplication", min_time, "for", robot
                        continue
                    if pw_hash[hs][robot] > min_time:
                        pw_hash[hs][robot] = min_time
                else:
                    pw_hash[hs] = pw_hash.get(hs, {})
                    pw_hash[hs][robot] = min_time

                #print min_time, "W_start", w_start.surfaces['salisbury_table'].objects

                #Check the world validity
                world_is_valid = True
                if 'world_limits' in goal:
                    if not w_start.world_limit_check(goal['world_limits']):
                        world_is_valid = False
                        if debug: print "World limits failed at", min_time

                #Try to merge the extra-steps into the plan, if there are any
                world_new_plan = None
                es_time = 0
                if 'extra_steps' in goal and world_is_valid:
                    if len(goal['extra_steps']) != 1: #TODO: causes issues with maining world state between actions
                        raise Exception("We don't support multiple extra steps")

                    es_plan = []
                    es_time = 0
                    w_clone = w_start.copy()
                    #print min_time, "W_clone", w_clone.surfaces['salisbury_table'].objects
                    #Try to take each step
                    for step in goal['extra_steps']:
                        #print step
                        #print w_clone.robots["stair4"].location.to_text(), w_clone.robots["stair4"].holders
                        action_robot_type, action_type = w_clone.types.get_action_for_robot(w_clone.robots[step[0]].robot_type, step[1])
                        if action_type == None: #Action failed to load internally
                            raise Exception("Invalid action type for extra step: " + str(step))
                        if action_type.time_estimate.strip() == "True()": #action is a program action
                            print "PLAN INTERNAL", action_type.name
                            if action_type.name.find("five") != -1:
                                for x in world[1]: print x
                                print "END!!!!"
                            if debug: print "SPECIAL PLAN!!!!"
                            if debug: print "Initial time:", es_time + min_time
                            #Add the robot name and subroutine name to the action.
                            p = step[2].copy()
                            if 'sub' in p: raise Exception("We cannot have parameters called 'sub'.") #FIXME: should be compile time error
                            if 'robot' in p: raise Exception("We cannot have parameters called 'robot'.")
                            p['sub'] = "action__" + action_robot_type.name + "__" + step[1]
                            p['robot'] = step[0]
                            #Define a simple program to call the subroutine with the parameters
                            code = [blast_world.BlastCodeStep(None, "CALLSUB", p, "plan_return"),
                                    blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                                           'label_true': "success", 'label_false': 'failure'}),
                                    blast_world.BlastCodeStep("success", "RETURN"),
                                    blast_world.BlastCodeStep("failure", "FAIL"),
                                    ]
                            #Add it to the program at the proper step
                            lstep = [min_time, "SETPLAN", uid, None, code, prog_ord]
                            prog_ord = prog_ord + 1
                            if prog_ord < 0: prog_ord = 1
                            
                            #Create a child planner and have it plan the action for the new world.
                            #FIXME: this is not a true BFS expansion because we go for this first.
                            planner_child = Planner(self.initial_world, [c.copy() for c in self.code_exc])
                            planner_child.point_plans = self.point_plans #This speeds everything up.
                            etl = end_time
                            if best_world:
                                etl = best_world[0]
                            r, npo = planner_child.plan(limit_progs = [uid], start_prog_time = es_time + min_time,
                                                        initial_plan = world[1] + [lstep,], prog_ord = prog_ord,
                                                        end_time = etl)
                            if debug: print "------------------>", r
                            if r == None: #Internal plan failed, we can't actually run this action.
                                es_plan = None
                                break
                            else: #Internal plan succeeded.
                                es_plan = "TOTAL_NEW"
                                es_time = 0
                                prog_ord = npo
                                for sstep in r:
                                    if sstep[1] == "EXEC" and sstep[2] == uid:
                                        es_time = max(es_time, sstep[0])
                                es_time = es_time - min_time
                                if debug: print "Time", es_time
                                world_new_plan = r
                        else: #We have a simple action, add it to the plan.
                            length, ldc = w_clone.take_action(step[0], step[1], step[2])
                            #print "Result -> ", length, ldc
                            if length == None and ldc == None:
                                es_plan = None
                                break
                            es_plan.append([es_time, "ACTION", length, step[0], step[1], step[2]])
                            es_time = es_time + length
                
                    if es_plan == None: #We have failed to merge extra steps, world is invalud
                        if debug: print "Failed to merge extra-steps at", min_time
                        world_is_valid = False
                    elif es_plan == "TOTAL_NEW": #We have a totally new plan, but it merged
                        if debug: print "Total new plan was a success at", min_time
                    else: #We need to merge in extra-steps. TODO we should move this to the simple action step
                        #print "Attempt to merge in at", min_time, es_plan
                        world_new_plan = self.merge_plans(es_plan, min_time, world[1], 0)
                        if world_new_plan == None:
                            if debug: print "Failed to merge extra-steps at", min_time
                            world_is_valid = False
            

                #If the world is valid, update the best_world if it is faster
                if world_is_valid:
                    if debug: print "World valid at", min_time + es_time
                    if world_new_plan == None: world_new_plan = world[1]
                    #If we have no best world, then this one is the best
                    if not best_world:
                        best_world = (min_time + es_time, world_new_plan)
                        if debug: print "Is best by default"
                    #If we have a best world and this one is better, then it is best
                    if min_time + es_time < best_world[0]:
                        best_world = (min_time + es_time, world_new_plan)
                        if debug: print "Is best by time"
                    #In the event of a tie, the one with less actions wins
                    if min_time + es_time == best_world[0] and \
                            len(filter(lambda x: x[1] == "ACTION", best_world[1])) \
                            > len(filter(lambda x: x[1] == "ACTION", world_new_plan)):
                        if debug: print "Is best by action count"
                        best_world = (min_time + es_time, world_new_plan)
                
                                
                #For each robot, we have to try moving to the goal from world_limits,
                #so that we can satisfy these goals.
                if 'world_limits' in goal and 'robot-location' in goal['world_limits'] \
                        and robot in goal['world_limits']['robot-location']:
                    target_pos = goal['world_limits']['robot-location'][robot]
                    w_end = w_start.copy()
                    w_end.robots[robot] = w_end.robots[robot].copy()
                    w_end.robots[robot].location = target_pos.copy()
                    w_end.robots[robot].position_do_not_care()
                    w_end.clear_hash("robots")
                    new_plan = self.merge_motion_plan(w_start, w_end, robot, min_time, world[1])
                    if new_plan != None:
                        if debug: print "World limits move succeeded for", robot, "at", min_time
                        worlds.append((min_time, new_plan, False))
                    else:
                        if debug: print "World limits move failed for", robot, "at", min_time
                else:
                    if debug: print "No world limits move for", robot, "at", min_time

                #For each robot, we try setting down any items it is carrying that are placeable
                if 'world_limits' in goal and 'place-objects' in goal['world_limits']:
                    if debug: print "Setting down objects for", robot, "at", min_time
                    for action, locations in w_start.enumerate_robot_place(robot).iteritems():
                        parameters = {}
                        for l, params in locations.iteritems():
                            if w_start.robots[robot].holders[l] == None:
                                parameters = None
                                break
                            elif w_start.robots[robot].holders[l].uid in wl_uids:
                                pos_t = wl_uids[w_start.robots[robot].holders[l].uid]

                                surface = pos_t['surface']
                                if type(surface) == blast_world.BlastSurface:
                                    surface = surface.name
                                pos = pos_t['position']
                                #TODO: this can get weird here if pos is not a valid parameter

                                if parameters.get(params[0], surface) != surface:
                                    parameters = None #We would have to have two different surfaces
                                    break
                                parameters[params[0]] = surface
                                
                                if parameters.get(params[1], pos) != pos:
                                    parameters = None #We would have to gave two different positions
                                    break
                                parameters[params[1]] = (w_start.surfaces[surface], pos)
                            else: #There is an object, but it is wrong. This case may still help if setting two objects down at once.
                                pass
                                
                        if parameters != None and parameters != {}:
                            for nparam, state in w_start.action_robot_pose(robot, action, parameters): 
                                w_end = self.get_robot_pose_world(w_start, robot, action, nparam, state)
                                new_plan = self.merge_motion_plan(w_start, w_end, robot, min_time, world[1], [robot, action, nparam])
                                if new_plan != None:
                                    if debug: print "Successfully set down object for", robot, "at", min_time
                                    worlds.append((min_time, new_plan, False))
                else:
                    if debug: print "No object setdown for", robot, "at", min_time
                
                
                #Deal with preparation for extra steps so we can execute them properly
                #try to get the robots lined up to execute them
                if 'extra_steps' in goal:
                    for step in goal['extra_steps']:
                        if step[0] != robot: 
                            if debug: print "Extra-step ", step, "is not for", robot, "at", min_time
                            continue
                        found = False
                        for parameters, state in w_start.action_robot_pose(step[0], step[1], step[2]):
                            w_end = self.get_robot_pose_world(w_start, robot, step[1], parameters, state)
                            new_plan = self.merge_motion_plan(w_start, w_end, robot, min_time, world[1]) #, [robot, step[1], parameters])
                            if new_plan != None:
                                if debug: print "Succesful extra-step preperation for", robot, "at", min_time
                                worlds.append((min_time, new_plan, False))
                            else:
                                if debug: print "Unsuccesful extra-step preperation for", robot, "at", min_time
                            found = True
                        if not found and debug: print "Unable to succesfully prepare a pose for the extra step", step, "for", robot, "at", min_time
                else:
                    if debug: print "No extra-steps for", robot, "at", min_time

                #Loop through all actions possible that are not motion actions. For example
                #pick and place.
                for action in w_start.enumerate_robot(robot, require_object = True):
                    if debug: print "Trying", action, "for", robot, "at", min_time
                    for parameters, state in w_start.action_robot_pose(robot, action, {}): 
                        w_end = self.get_robot_pose_world(w_start, robot, action, parameters, state)
                        new_plan = self.merge_motion_plan(w_start, w_end, robot, min_time, world[1], [robot, action, parameters])
                        #print parameters, state, "->", new_plan != None
                        if new_plan != None:
                            worlds.append((min_time, new_plan, False))
                    else:
                        if debug: print "No start state for this action"
        
        #Return the best world.
        if best_world:
            return best_world[1], best_world[0], prog_ord
        return None, None, None

    #This function fills up a plan with BLOCK times that prevent
    #robot reuse. Fundementally this is an inefficiency in the
    #planner in terms of resource use, but it prevents many
    #plan from being generated that are not solvable.
    def back_fill_plan(self, plan, robot):
        time_blocks = []
        for step in plan:
            if (step[1] == "BLOCK" or step[1] == "ACTION") and step[3] == robot:
                time_blocks.append((step[0], step[2]))
        time_blocks.sort(key=lambda x: x[0])

        last_block = (0, 0)
        plan_clone = [x for x in plan]
        for block in time_blocks:
            gap = block[0] - last_block[0] - last_block[1]
            if gap > 0:
                plan_clone.append([last_block[0] + last_block[1], "BLOCK", gap, robot])
            last_block = block

        plan_clone.sort(key=lambda x: x[0])
        return plan_clone

    def plan(self, limit_progs = None, start_prog_time = 0, initial_plan = [], prog_ord = 0, end_time = None):
        code_exc = self.generate_world(initial_plan, start_prog_time, ret_code_exc = True)
        if code_exc == None:
            raise Exception("Code exec could not be generated at plan start!")

        #print len(initial_plan), prog_ord
        #for i in code_exc:
        #    print i.uid, ":", i.to_text()

        plan = [x for x in initial_plan]

        #Try to plan for each of the execs, use the first one, then fill until
        #a plan is made.
        for prog in code_exc:
            if limit_progs:
                if prog.uid not in limit_progs:
                    continue
            prog_time = start_prog_time
            is_first_plan = True
            while True:
                plan.sort(key = lambda x: x[0])
                #Check if we have a time limit, and if so if we are over it
                if end_time != None:
                    if prog_time > end_time:
                        return None, prog_ord
                #Execute the program on the world at the current program time.
                #Program time is defined as the time in the world the program is in
                goal = prog.execute(self.generate_world(plan, prog_time))
                #Append to the plan the execution of the program. This includes
                #expected state and hash information.
                plan.append([prog_time, "EXEC", prog.uid, prog.get_hash_state(), prog_ord, prog.to_text(), goal])
                prog_ord = prog_ord + 1 
                #Goal is the step returned by the program. There are many possibilities
                #including an action planning goal or a sucess or failure.
                if type(goal) == dict:
                    #Try to plan to the program state desired.
                    newplan, newprog_time, newprog_ord = self.plan_to_prog(plan, prog.robots, prog_time, goal, prog.uid, prog_ord, end_time)
                    r = True
                    if newplan == None and newprog_time == None or newprog_ord == None: #If we fail
                        r = False
                        #In the event of failure, we simply need to force a replan. The program
                        #author could specify this if we have a plan that might fail but hidden
                        #state changes could fix. For example, we scan a surface as part of an
                        #object hunt. Scanning actions assume that there are objects, so it will
                        #always require replanning at that point. Fortunately we can force a replan.
                        #It is important not to replan the first plan because there are no opportunities
                        #for hidden world state change if we try to replan at that point. This would
                        #therefore lead to an infinite loop.
                        if not is_first_plan and goal.get("failure_force_replan", False) == True:
                            plan.append([prog_time, "SETPLAN", prog.uid, None, None, prog_ord])
                            prog_ord = prog_ord + 1
                    else: #If succeed
                        plan = newplan
                        prog_time = newprog_time
                        prog_ord = newprog_ord
                    #After we are done, we need to block of robot times in gaps for
                    #the plan, so that other plannings don't try to use those robots.
                    #We clear the first plan, also add set_plan statement to the plan
                    #so that execution happens properly.
                    is_first_plan = False
                    for robot in prog.robots: plan = self.back_fill_plan(plan, robot)
                    prog.set_plan_executed(True, r)
                    plan.append([prog_time, "SETPLAN", prog.uid, True, r, prog_ord])
                    prog_ord = prog_ord + 1
                elif goal == True or goal == False: #The whole program is done. Exit the loop
                    if limit_progs and goal == False: #In limit progs
                        return None, prog_ord
                    break
                elif goal == "PLAN": #The program knows that when we get to this point, replanning will be needed.
                    plan.append([prog_time, "SETPLAN", prog.uid, None, False, prog_ord])
                    st = prog.set_plan_code_done()
                    prog_ord = prog_ord + 1
                    if limit_progs:
                        if st:
                            return plan, prog_ord
                        else:
                            return None, prog_ord
                else:
                    raise Exception("Invalid program return: " + goal)
        return plan, prog_ord
    
    def plan_print(self, limit_progs = None, start_prog_time = 0, initial_plan = []):
        plan, po = self.plan(limit_progs = limit_progs, start_prog_time = start_prog_time, initial_plan = initial_plan)

        print
        print "Final plan steps:"
        for i in plan:
            print i
        
        return plan
        
class BlastCodeObjectPtr(object):
    __slots__ = ['refc']
    def __init__(self, refc):
        self.refc = refc

    def __str__(self):
        return "BlastCodeObjectPtr(" + str(self.refc) + ")"
    def __repr__(self):
        return "BlastCodeObjectPtr(" + str(self.refc) + ")"

class BlastCodeExec(object):
    __slots__ = ['uid', 'code', 'labels', 'environments', 'plan_executed', 'robots', 'plan_res', 'object_codes', 'debug']

    def get_hash_state(self, hl = None):
        if not hl:
            hl = hashlib.sha224()
            self.get_hash_state(hl)
            return hl.digest()
        hl.update(str(self.uid))
        if self.done():
            hl.update(str(self.environments))
            return
        hl.update(str(self.plan_executed))
        hl.update(str(self.plan_res))
        for e in self.environments:
            hl.update(str(e[0]) + str(id(e[1])))
            for dictionary in e[2:]:
                if dictionary == True or dictionary == False:
                    hl.update(str(dictionary))
                    continue
                for key in sorted(dictionary.keys()):
                    t = type(dictionary[key])
                    if t == set:
                        hl.update(str(key) + ":")
                        for s in sorted(dictionary[key]):
                            hl.update(s + "|")
                        hl.update(",")
                        continue
                    elif t != str and t != int and t != float and t != bool and t != blast_world.BlastPt and t != BlastCodeObjectPtr:
                        raise Exception("Invalid type for dictionary key: " + \
                                            str(type(dictionary[key])) + ": " + str(dictionary[key]))
                    hl.update(str(key) + ":" + str(dictionary[key]) + ",")
            
    def to_text(self):
        s = str(self.uid) + ","
        if self.done():
            s = s + (str(self.environments))
            return s
        s = s + (str(self.plan_executed)) + "," + (str(self.plan_res)) + ":\n"
        for e in self.environments:
            s = s + str((e[0], e[3], "->", e[5])) + "\n"
        return s

    def get_next_plan(self, w):
        c = self.copy()
        c.set_plan_executed(False)
        return c.execute(w)

    def generate_labels(self, code):
        labels = {}
        for i in xrange(0, len(code)):
            if code[i].label:
                if code[i].label in labels:
                    raise blast_world.BlastCodeError("Duplicate label: " + code[i].label)
                labels[code[i].label] = i
        return labels
        

    def __init__(self, uid, code, robots, labels = None, environments = None, plan_e = False, plan_r = False):
        self.uid = uid
        self.robots = robots
        #Translate all the code
        self.code = code
        self.plan_executed = plan_e
        self.plan_res = plan_r
        self.debug = False
        self.object_codes = []
        if labels:
            self.labels = labels
        else:
            self.labels = self.generate_labels(self.code)

        if environments != None:
            if environments == True or environments == False:
                self.environments = environments
            else:
                self.environments = [[gp, cb, lb, vs.copy(), sc.copy(), ic] for gp, cb, lb, vs, sc, ic in environments]
        else:
            self.environments = [[0, self.code, self.labels, {}, {}, False],]

    def copy(self):
        c = BlastCodeExec(self.uid, self.code, self.robots, self.labels, self.environments, self.plan_executed, self.plan_res)
        c.object_codes = [x for x in self.object_codes]
        return c

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
                o[self.paste_parameters(n, env)] = self.paste_parameters(v, env)
            return o
        if type(p) == list:
            return [self.paste_parameters(v, env) for v in p]
        if type(p) == tuple:
            return tuple([self.paste_parameters(v, env) for v in p])
        if type(p) == blast_world.BlastParameterPtr:
            if not p.parameter in env:
                raise blast_world.BlastCodeError("Invalid variable/parameter: " + str(p.parameter))
                return
            r = env.get(p.parameter)
            if type(r) == BlastCodeObjectPtr:
                r = self.object_codes[r.refc]
                if type(r) == int:
                    r = blast_world.BlastObjectRef(r)

            if p.sub != None:
                r = r.split(".")[p.sub]
            if p.prefix != None:
                r = p.prefix + str(r)
            if p.postfix != None:
                r = str(r) + p.postfix
            return r
        return p

    def set_plan_executed(self, p, plan_res = False):
        self.plan_executed = p
        self.plan_res = plan_res

    def set_plan_code(self, code):
        self.environments.append([0, code, self.generate_labels(code), {}, {}, True])

    def set_plan_code_done(self):
        self.environments = self.environments[:-1]
        substep = self.environments[-1][1][self.environments[-1][0]] #Find the CALLSUB
        return self.environments[-1][3][substep.return_var]

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
        if next_step[0] >= len(next_step[1]):
            raise blast_world.BlastCodeError("Code segment ran with no return.")
            return
        ps = next_step[1][next_step[0]]
        params = self.paste_parameters(ps.parameters, next_step[3])
        if self.debug: print ps.command, params
        if ps.command == "PLAN":
            if self.plan_executed:
                if ps.return_var:
                    next_step[3][ps.return_var] = self.plan_res
                self.plan_executed = False
                next_step[0] = next_step[0] + 1
                next_step[4] = world.get_scan_state() #Get the scan state out of the planner
                return self.execute(world)
            else:
                world.clear_scan(next_step[4]) #Set back the scan so planning runs with our scans
                return params
        elif ps.command == "IF":
            def condition_eval(c):
                if type(c) == bool:
                    return c
                if type(c) == int or type(c) == float or type(c) == long or type(c) == str:
                    return c
                if type(c) == tuple or type(c) == list:
                    if len(c) == 0:
                        raise blast_world.BlastCodeError("Invalid code condition: " + str(c))
                    math_operators = {"<=": 2, ">=": 2, ">": 2, "<": 2, "==": -2, "+": -1, "-": 2, "/": 2, "*": -1}
                    operators = math_operators.copy()
                    if c[0] in operators:
                        if operators[c[0]] >= 0:
                            if len(c) != operators[c[0]] + 1:
                                raise blast_world.BlastCodeError("Invalid code condition arguments to '" + c[0] 
                                                                 + "': " + str(c) + " -> " + str(len(c)-1) 
                                                                 + " should be " + str(operators[c[0]]))
                        else:
                            if len(c) < -operators[c[0]] + 1:
                                raise blast_world.BlastCodeError("Invalid code condition arguments to '" + c[0] 
                                                                 + "': " + str(c) + " -> " + str(len(c)-1) 
                                                                 + " should be at least " + str(-operators[c[0]]))
                        if c[0] in math_operators:
                            for p in c[1:]:
                                if not (type(p) == int or type(p) == float or type(p) == long):
                                    raise blast_world.BlastCodeError("Invalid code condition arguments to '" + c[0] 
                                                                     + "': must be int, float, or long: '" + str(p) + "'"
                                                                     + " not " + str(type(p)))
                        if c[0] == ">=": return c[1] >= c[2]
                        if c[0] == "<=": return c[1] <= c[2]
                        if c[0] == "<": return c[1] < c[2]
                        if c[0] == ">": return c[1] > c[2]

                        
                raise blast_world.BlastCodeError("Invalid code condition: " + str(c))
            cr = condition_eval(params["condition"])
            if cr != True and cr != False:
                raise blast_world.BlastCodeError("Invalid code condition result: " + str(c) + " returned " + str(cr))
            if cr == True and 'label_true' in params:
                if params['label_true'] in next_step[2]:
                    next_step[0] = next_step[2][params['label_true']]
                else:
                    raise blast_world.BlastCodeError("We tried to jump out in an if statement: " + params['label_true'])
            elif cr == False and 'label_false' in params:
                if params['label_false'] in next_step[2]:
                    next_step[0] = next_step[2][params['label_false']]
                else:
                    raise blast_world.BlastCodeError("We tried to jump out in an if statement: " + params['label_false'])
            else:
                next_step[0] = next_step[0] + 1
            return self.execute(world)
        elif ps.command == "FAIL" or ps.command == "RETURN":
            code_plan = self.environments[-1][5]
            if len(self.environments) == 1:
                if code_plan:
                    raise blast_world.BlastCodeError("Lowest runner was")
                self.environments = (ps.command == "RETURN")
            else:
                if not code_plan:
                    self.environments = self.environments[:-1]
                    substep = self.environments[-1][1][self.environments[-1][0]] #Find the CALLSUB
                    self.environments[-1][3][substep.return_var] = (ps.command == "RETURN") #Set return
                    self.environments[-1][0] = self.environments[-1][0] + 1 #Increment the instruction pointer
                else:
                    substep = self.environments[-2][1][self.environments[-2][0]]
                    self.environments[-2][3][substep.return_var] = (ps.command == "RETURN") #Set return
            if code_plan: #This allows us to properly terminate the subplan step
                return "PLAN"
            return self.execute(world)
        elif ps.command == "SCAN":
            ot = params.get('reset', [])
            if type(ot) != list and type(ot) != set:
                ot = str(ot).split(",")
            for surface, items in next_step[4].iteritems():
                ni = []
                change = False
                for i in items:
                    if i not in ot:
                        ni.append(i)
                    else:
                        change = True
                if change:
                    next_step[4][surface] = set(items)
            next_step[0] = next_step[0] + 1
            return self.execute(world)
        elif ps.command == "SCAN_STATE":
            if not ps.return_var:
                raise blast_world.BlastCodeError("SCAN_STATE has no return variable")
            ot = params.get('types', [])
            if type(ot) != list and type(ot) != set:
                ot = str(ot).split(",")
            scanned = 0
            for surface, items in next_step[4].iteritems():
                good = True
                for oti in ot:
                    if oti not in items: good = False
                if good:
                    scanned = scanned + 1
            next_step[3][ps.return_var] = scanned
            next_step[0] = next_step[0] + 1
            return self.execute(world)
        elif ps.command == "SCAN_MAX": #TODO: this returns all scannable surfaces, but some might be unscannable if a robot cannot access them
            if not ps.return_var:
                raise blast_world.BlastCodeError("SCAN_STATE has no return variable")
            ot = params.get('types', [])
            if type(ot) != list and type(ot) != set:
                ot = str(ot).split(",")

            sa = world.get_scan_actions_surfaces()
            r = 0
            for name, surface in world.surfaces.iteritems():
                otf = set()
                for robot in self.robots:
                    ctype = world.robots[robot].robot_type
                    while ctype:
                        for otype, actions in sa.get(surface.surface_type.name, {}).iteritems():
                            for action in actions:
                                if action.split(".")[0] == ctype.name:
                                    otf.add(otype)
                        ctype = ctype.parent
                good = True
                for oti in ot:
                    if oti not in otf:
                        good = False
                if good: r = r + 1
    
            next_step[3][ps.return_var] = r
            next_step[0] = next_step[0] + 1
            return self.execute(world)
        elif ps.command == "CALLSUB":
            sub = params['sub']
            code = world.types.script
            labels = world.types.script_indexes
            if sub in next_step[2]:
                code = next_step[1]
                labels = next_step[2]

            if not sub in labels:
                raise blast_world.BlastCodeError("Subroutine calls a label that is not present: '" + sub + "'")
            ptr = labels[sub]

            startsub = code[ptr]

            if startsub.command != "STARTSUB":
                raise blast_world.BlastCodeError("Subroutine calls a label that is not a subroutine: '" + sub + "'")

            for name, ptype in startsub.parameters.iteritems():
                if name not in ps.parameters:
                    raise blast_world.BlastCodeError("Missing parameter for subroutine '" + name + "'")
            pc = params.copy()
            del pc['sub']
            self.environments.append([ptr + 1, code, labels, pc, {}, False])
            return self.execute(world)
        elif ps.command == "GOTO":
            sub = params['label']
            code = world.types.script
            labels = world.types.script_indexes
            if sub in next_step[2]:
                code = next_step[1]
                labels = next_step[2]
            ptr = labels[sub]
            next_step[0] = ptr
            return self.execute(world)
        elif ps.command == "GETOBJECT":
            if type(params.get('holder', None)) != str:
                raise blast_world.BlastCodeError("Holder must be specified for GETOBJECT as a string")
            holder = params['holder'].split(".")
            if len(holder) != 2:
                raise blast_world.BlastCodeError("Holder for GETOBJECT should be robot.holder: " + params['holder'])
            robot = holder[0]
            holder = holder[1]
            if not robot in world.robots:
                raise blast_world.BlastCodeError("Holder for GETOBJECT should be robot.holder: " + params['holder'] + " - invalid robot: " + robot)
            if not holder in world.robots[robot].holders:
                raise blast_world.BlastCodeError("Holder for GETOBJECT should be robot.holder: " + params['holder'] + " - invalid holder: " + holder)

            obj = world.robots[robot].holders[holder]
            if obj != None:
                obj = obj.uid
            
            n = len(self.object_codes)
            self.object_codes.append(obj)

            next_step[3][ps.return_var] = BlastCodeObjectPtr(n)
            next_step[0] = next_step[0] + 1
            return self.execute(world)
        elif ps.command == "SETROBOTHOLDER":
            if type(params.get('holder', None)) != str:
                raise blast_world.BlastCodeError("Holder must be specified for SETROBOTHOLDER as a string")
            holder = params['holder'].split(".")
            if len(holder) != 2:
                raise blast_world.BlastCodeError("Holder for GETOBJECT should be robot.holder: " + params['holder'])
            robot = holder[0]
            holder = holder[1]
            if not robot in world.robots:
                raise blast_world.BlastCodeError("Holder for GETOBJECT should be robot.holder: " + params['holder'] + " - invalid robot: " + robot)
            if not holder in world.robots[robot].holders:
                raise blast_world.BlastCodeError("Holder for GETOBJECT should be robot.holder: " + params['holder'] + " - invalid holder: " + holder)

            if params.get('require-preexisting', False) == False or world.robots[robot].holders[holder] != None:
                world.set_robot_holder(robot, holder, params['object-type'])
                if ps.return_var != None:
                    next_step[3][ps.return_var] = True
            else:
                if ps.return_var != None:
                    next_step[3][ps.return_var] = False
            next_step[0] = next_step[0] + 1
            return self.execute(world)
        else:
            raise blast_world.BlastCodeError("We don't support '" + ps.command + "'")
        raise blast_world.BlastCodeError("Command '" + ps.command + "' did not return a result, internal bug")
        

class BlastPlannableWorld:
    def __init__(self, world, real_world = False):
        self.world = world
        self.real_world = real_world

        self.lock = threading.Lock()

        self.times_planned = 0

        self.needs_replan = False

        self.robot_actions = {}
        self.robot_end_times = {}
        self.robot_actions_cancelled = {}
        self.plan = []
        self.old_plan = []

        #Action call back returns:
        #False/None for epic failure.
        #True for success
        #String for plain failures.
        #Exception generation is epic failure plus exception printed.

        self.action_callback = lambda r, a, p: self.test_action_callback(r, a, p)

        #Returns ignored. It is important that cancel callback is not
        #called in a world lock, because during the shutdown process
        #the cancel step could alter the world state.
        self.cancel_callback = lambda r, f: self.test_cancel_callback(r, f)
        
        self.code_exec = []
        self.finished_code = []
        self.code_exec_uid = 0

        self.epic_fail_state = False

    def test_action_callback(self, robot, action, parameters):
        print "Test action callback:", robot, action, parameters
        return True

    def test_cancel_callback(self, robot, forced):
        print "Default cancel callback for", robot, "forced?", forced
        

    def append_plan(self, code, robots, wait_for_plan = True):
        self.lock.acquire()
        exc = BlastCodeExec(self.code_exec_uid, code, robots)
        self.code_exec_uid = self.code_exec_uid + 1
        self.code_exec.append(exc)
        self.needs_replan = True
        self.lock.release()

    def run(self, exit_when_done = False):
        start_time = int(time.time() * 1000)
        time_warp = 0
        while not exit_when_done or self.code_exec != []:
            step_time = time.time()
            self.lock.acquire()
            while self.epic_fail_state:
                if exit_when_done: break
                self.lock.release()
                time.sleep(1.0)
                self.lock.acquire()
            if exit_when_done and self.epic_fail_state: 
                self.lock.release()
                break

            if self.needs_replan:
                still_running = False
                tbc = set()
                for robot in self.world.robots_keysort:
                    if self.robot_actions.get(robot) != None:
                        still_running = True
                    if self.robot_actions_cancelled.get(robot, False):
                        self.robot_actions_cancelled[robot] = True

                if len(tbc) != 0:
                    self.lock.release()
                    for robot in tbc:
                        self.cancel_callback(robot, False)
                    self.lock.acquire()

                if not still_running:
                    self.times_planned = self.times_planned + 1
                    plan_world = self.world.copy(copy_on_write_optimize = False)
                    planner = Planner(self.world, [x.copy() for x in self.code_exec])
                    self.lock.release()
                    pl = planner.plan_print()

                    if type(pl) != list:
                        print "Result -> ", pl
                    else:
                        print "Generated good plan"
                    print "Resuming execution..."
                    print
                    if pl == None or pl == False:
                        pl = [] #TODO error
                    
                    #prog_ord_max = 0
                    #plan_ord = []
                    #for step in pl:
                    #    if step[1] == "EXEC":
                    #        prog_ord_max = max(prog_ord_max, step[4] + 1)
                    #        plan_ord.append([step[4] + 1, step])
                    #    elif step[1] == "SETPLAN":
                    #        prog_ord_max = max(prog_ord_max, step[5] + 1)
                    #        plan_ord.append([step[5] + 1, step])
                    #    else:
                    #        plan_ord.append([-1, step])
                    #prog_ord_max += 3
                    #prog_ord_mul = 1
                    #while prog_ord_mul < prog_ord_max:
                    #    prog_ord_mul *= 10
                    #plan_ord = [[prog_ord_max, x[1]] if x[0] < 0 else x for x in plan_ord]
                    #plan_ord.sort(key = lambda x: x[1][0] * prog_ord_mul + x[0])
                    #for i in plan_ord:
                    #    print i[1][0], prog_ord_mul, i[0], i[1]
                    #pl = [x[1] for x in plan_ord]

                    self.lock.acquire()
                    self.needs_replan = False
                    self.plan = pl
                    self.old_plan.append([0, "REPLAN", (int(time.time() * 1000) + time_warp - start_time)])
                    start_time = int(time.time() * 1000)
                    time_warp = 0

            #Simple solution - when all actions finish, then we warp ahead.
            #In the future, we will switch to an action dependency graph model
            #in which all actions execute based
            warp = True
            for r, v in self.robot_actions.iteritems():
                if v != None:
                    warp = False
            
            if warp and self.plan != []:
                t = int(time.time() * 1000) + time_warp - start_time
                w = self.plan[0][0] - t
                if w > 0:
                    print "Warping ", w, " ms ahead"
                    self.old_plan.append([t, "WARP", w])
                    time_warp += w

            current_time = (int(time.time() * 1000) + time_warp - start_time)
            #When a robot goes on for too long, it must be stopped
            tbc = set()
            for r, t in self.robot_actions.iteritems():
                if v != None:
                    if current_time >= self.robot_end_times[r] \
                            and not self.robot_actions_cancelled[r]:
                        tbc.add(r)
                        self.robot_actions_cancelled[r] = True
            if len(tbc) != 0:
                self.needs_replan = True
                self.lock.release()
                for robot in tbc:
                    self.cancel_callback(robot, False)
                self.lock.acquire()
                
            while self.plan != [] and not self.needs_replan:
                step = self.plan[0]
                if step[0] >= current_time:
                    break #All the rest is in the future
                self.old_plan.append(step)
                self.plan = self.plan[1:]
                print "EXECUTE", step
                if step[1] == "EXEC":
                    uid = step[2]
                    for prog in self.code_exec:
                        if prog.uid == uid: break
                    prog.execute(self.world)
                    if prog.get_hash_state() != step[3]:
                        s = "Not specified"
                        if len(step) > 5: s = step[5]
                        print ("Bad state for program (" + bin_to_hex(prog.get_hash_state()) \
                                   + "):\n" + prog.to_text() + "\nGood state (" \
                                   + bin_to_hex(step[3]) + "):\n" + s)
                        self.needs_replan = True
                        sys.exit(-1)
                elif step[1] == "SETPLAN":
                    if step[3] != None and step[4] != None:
                        uid = step[2]
                        for prog in self.code_exec:
                            if prog.uid == uid: break
                        prog.set_plan_executed(step[3], step[4])
                    elif step[3] == None and step[4] == None:
                        self.needs_replan = True
                    elif step[3] == None and type(step[4]) == list:
                        prog.set_plan_code(step[4])
                    elif step[3] == None and step[4] == False:
                        prog.set_plan_code_done()
                    else:
                        raise Exception("Invalid SETPLAN: " + str(step))
                    
                elif step[1] == "ACTION":
                    if self.robot_actions.get(step[3], None) != None:
                        print "ERRRROR", step[3], "is not stopped"
                        self.robot_actions_cancelled[step[3]] = True
                        self.needs_replan = True
                        self.lock.release()
                        self.cancel_callback(step[3], True)
                        self.lock.acquire()
                        self.needs_replan = True
                    else:
                        self.robot_actions[step[3]] = (step[3], step[4], step[5]) 
                        self.robot_end_times[step[3]] = step[0] + step[2]
                        self.robot_actions_cancelled[step[3]] = False
                        thread.start_new_thread(lambda s, t, r, a, p: s.do_action(t, r, a, p),
                                                (self, step[2], step[3], step[4], step[5]))
                else:
                    raise Exception("Invalid plan step: " + str(step))

            done_code = set()
            for c in self.code_exec:
                if c.done():
                    print "Program finished:", c.uid
                    done_code.add(c)
            for c in done_code:
                self.finished_code.append(c)
                self.code_exec.remove(c)
            
            self.lock.release()
            elapsed = time.time() - step_time
            if elapsed < 0.010:
                time.sleep(0.010 - elapsed)
        return

    def print_old(self):
        print
        print
        print "Plan executed so far:"
        for o in self.old_plan:
            print o
        print

    def action_overrun(self, r):
        print "Action on", r, "took too long. Cancelling."
        self.lock.acquire()
        self.needs_replan = True
        self.robot_actions_cancelled[r] = True
        self.lock.release()
        self.cancel_callback(r, True)

    def epic_fail(self, r, a, p, rea):
        self.epic_fail_state = True

    def do_action(self, timelim, r, a, p):
        print "Start action", r, a, p, "with limit", timelim, "ms"
        self.lock.acquire()
        internal_copy = self.world.copy(copy_on_write_optimize = False)
        workspace_surfaces, workspace_points = internal_copy.get_workspaces(r, a, p)
        self.lock.release()
        print "Workspace is", workspace_surfaces, workspace_points
        try:
            acr = self.action_callback(r, a, p)
        except:
            print traceback.format_exc()
            acr = False
        print "Done action", r, a, p

        self.lock.acquire()
        if acr == False or acr == None: #Then the action failed
            print "Action failed epically", r, a, p
            self.epic_fail(r, a, p, "Callback failed epically")
        else:
            fm = None
            if acr != True: 
                print "Action failed", acr, "for", r, a, p
                fm = acr
                self.needs_replan = True
            
            if not self.real_world: #Debug action internally
                atr, d = self.world.take_action(r, a, p, failure_mode = acr)
                if atr == None or d == None:
                    print "Action failed epically", r, a, p
                    self.epic_fail(r, a, p, "Failed to run")

            get_obj = lambda x: self.world.get_obj(x)
            other_get_obj = lambda x: internal_copy.get_obj(x)
            
            atr, d = internal_copy.take_action(r, a, p, failure_mode = acr)
            if atr == None or d == None:
                print "Action failed epically", r, a, p
                self.epic_fail(r, a, p, "Action verification simulation falsification")
            
            if not self.world.robots[r].equal(internal_copy.robots[r], get_obj, other_get_obj):
                print "Action failed epically", r, a, p
                self.epic_fail(r, a, p, "Robot " + r + " was not in correct state")
            for s in workspace_surfaces:
                if not self.world.surfaces[s].equal(internal_copy.surfaces[s], get_obj, other_get_obj):
                    print "Action failed epically", r, a, p
                    self.epic_fail(r, a, p, "Surface " + s + " was not in correct state")
            
            
                
            

        self.robot_actions[r] = None
        self.robot_actions_cancelled[r] = True
        self.lock.release()

    







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

    objects_to_add = {"table_1": 1, "table_2": 1, "first_scan": None}

    def ac(r, a, p): #Test add the cups
        tn = p.get("table", None)
        if type(tn) == blast_world.BlastSurface: tn = tn.name
        if r == "stair4" and a == "table-coffee-scan":
            if not objects_to_add["first_scan"]:
                objects_to_add["first_scan"] = tn
            if objects_to_add[tn] > 0:
                world.world.add_surface_object(tn, "coffee_cup",
                                               blast_world.BlastPos(0.6602, 0.0, 0.762, 0.0, 0.0, 0.0))
                objects_to_add[tn] -= 1
        if r == "stair4" and a == "table-pick-left" and tn == objects_to_add["first_scan"]:
            print "THERE IS NO OBJECT - THE PHANTOM CUP"
            return "no_object"
        if a == "table-place-left":
            time.sleep(30)
        return True

    world.action_default = True
    world.action_callback = ac
    
    world.append_plan([blast_world.BlastCodeStep(None, "CALLSUB", {'sub': 'hunt_objects', 'object_types': "coffee_cup",
                                                                   'holder': 'stair4.cupholder'}, 'plan_return'),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                              'label_true': "success", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success", "GETOBJECT", {'holder': 'stair4.cupholder'}, 'object_n'),
                       #blast_world.BlastCodeStep(None, "PLAN", {'extra_steps': [("stair4", "table-place-left", {"table": "table_1", "position": "table_1, Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)"}),],
                       #                                         }, 'plan_return'),
                       blast_world.BlastCodeStep(None, "PLAN", {'world_limits': {'place-objects':
                                                                                     [{'object': blast_world.BlastParameterPtr('object_n'),
                                                                                       'surface': 'table_1',
                                                                                       'position': 'Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)',
                                                                                       }
                                                                                      ],
                                                                                 }}, 'plan_return'),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                              'label_true': "success2", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success2", "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL"),],
                      ["stair4",], False)

    world.run(True)

    world.print_old()

    return True


def run_test():
    import blast_world_test
    world = BlastPlannableWorld(blast_world_test.make_test_world())

    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
                       #Grab the bag
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": {"stair4": initial_pickup_point}},
                                                                "extra_steps": [("stair4", "grab-object", {"tts-text": "Money Bag"}),],}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),

                       blast_world.BlastCodeStep(None, "SETROBOTHOLDER", {"holder": "stair4.left-arm", "require-preexisting": True,
                                                                          "object-type": "coffee_money_bag"}),
                       #Buy the coffee
                       blast_world.BlastCodeStep(None, "PLAN", {"extra_steps": [("stair4", "buy-coffee", {"shop": "clark_peets_coffee_shop"}),],}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),

                       #Return the bag
                       blast_world.BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": {"stair4": initial_pickup_point}},
                                                                "extra_steps": [("stair4", "give-object", {"tts-text": "Coffee Cup"}),],}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),
                       
                       blast_world.BlastCodeStep(None, "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL")
                       ], ["stair4",], False)
    
    
    world.try_exec()

def coffee_run_exec():
    import blast_world_test
    world = BlastPlannableWorld(blast_world_test.make_test_world())
    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
    rand_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloordoor")
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": {"stair4": rand_point}}}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),
                       blast_world.BlastCodeStep(None, "PLAN", 
                                                 {"extra_steps": [("stair4", "coffee-run", 
                                                                   {"shop": "clark_peets_coffee_shop",
                                                                    "person_location": initial_pickup_point}),],},
                                                 "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),
                       
                       blast_world.BlastCodeStep(None, "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL")
                       ], ["stair4",], False)

    world.run(True)

    world.print_old()

    print "Planned:", world.times_planned
    

def five_coffee_run_exec():
    import blast_world_test
    world = BlastPlannableWorld(blast_world_test.make_test_world())
    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
    rand_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloordoor")
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": {"stair4": rand_point}}}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),
                       blast_world.BlastCodeStep(None, "PLAN", 
                                                 {"extra_steps": [("stair4", "five-coffee-run", 
                                                                   {"shop": "clark_peets_coffee_shop",
                                                                    "person_location": initial_pickup_point}),],},
                                                 "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'), "label_false": 'failure'}),
                       
                       blast_world.BlastCodeStep(None, "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL")
                       ], ["stair4",], False)

    world.run(True)

    world.print_old()

    print "Planned:", world.times_planned

def multi_robot_test():
    import blast_world_test
    world_i = blast_world_test.make_table_top_world(True)
    
    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    world_i.types.get_robot("pr2-cupholder"))
    world_i.append_robot(stair5)
    world_i.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.
    world = BlastPlannableWorld(world_i)

    #print "-"*180
    #print world.world.to_text()
    #print "-"*180

    world.append_plan([blast_world.BlastCodeStep(None, "CALLSUB", {'sub': 'hunt_objects', 'object_types': "coffee_cup",
                                                                   'holder': 'stair4.cupholder'}, 'plan_return'),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                              'label_true': "success", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success", "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL"),],
                      ["stair4", "stair5",], False)
    
    #r = world.plan_hunt("stair4", "cupholder", "coffee_cup")

    world.try_exec()

    #print "-"*180
    #print world.world.to_text()
    #print "-"*180
    
    return False



    
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
                       ], ["stair5",], False)
    
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
                       ], ["stair4", "stair5",], False)

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
    print five_coffee_run_exec()
    #print multi_robot_test()
    #print overplan()

    mpt = motion_plan_hits + motion_plan_misses
    if mpt < 1: mpt = 1 #Avoid 0 div
    print "Motion planed", mpt, "times, with", motion_plan_hits, "hits", motion_plan_hits * 1.0/mpt, "percent"
