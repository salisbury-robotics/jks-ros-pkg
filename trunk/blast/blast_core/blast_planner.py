
import blast_world, time, json

class Planner:
    def __init__(self, initial_world):
        self.worlds = [(initial_world, 0, [], None)]
        self.world_good = lambda x: False
        self.planned_worlds = []
        self.good_worlds = []
        self.time_limit = None
        self.worlds_tested = 0
        self.actions_tested = 0
        self.actions_finished = 0
        self.extra_goals = {}

        self.action_type_debug = {}
        self.fail_debug = {}
        self.super_fail_debug = False

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
                return last_world, last_time, tsteps

            

        
        #for obj, action_types in ot:
                #Need a better way to schedule.
        #    for a in action_types:
        #        print a
                #new_world = (world[0].copy(), world[1] + float(1000), world[2] + [(robot_name, None, {"object-type"})], world)
                #self.worlds.append(new_world)


    def plan_recursive(self):
        for world in self.worlds:
            if self.world_good(world[0]):
                self.good_worlds.append(world)
                if not self.time_limit:
                    self.time_limit = world[1]
                if world[1] < self.time_limit:
                    self.time_limit = world[1]

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
                if name.split(".")[0] in robot_types:
                    if not name.split(".")[1] in action_types:
                        action_types.append(name.split(".")[1])
            cached_types[robot.name] = robot_types
            cached_actions[robot.name] = action_types

        while self.worlds != []:
            world = min(self.worlds, key = lambda x: x[1])
            self.worlds.remove(world)
            self.planned_worlds.append(world)


            #print "-"*120
            #print world[0].scan_count("coffee_cup")
            #for a in world[2]: print a
            #print world[0].to_text()
            #print "-"*120

            #Once we find a good world, we stop planning for worlds after that time
            if self.time_limit != None and world[1] >= self.time_limit: break
            
            #Actually testing this world
            self.worlds_tested = self.worlds_tested + 1

            for robot_name, robot in world[0].robots.iteritems():
                robot_types = cached_types[robot.name]
                action_types = cached_actions[robot.name]

                #Now enumerate all possible actions
                world_clone = world[0].copy()
                #print robot_name, robot.location.to_text()
                for at in action_types:
                    planable, pv = world_clone.enumerate_action(robot_name, at, self.extra_goals)
                    if not planable or pv == False:
                        continue
                    #print robot_name, at, [x for x in self.parameter_iter(pv)]
                    for parameters in self.parameter_iter(pv):
                        self.actions_tested = self.actions_tested + 1
                        change = world_clone.take_action(robot_name, at, parameters) 
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
                            if not failed:
                                for world_cmp in self.worlds:
                                    if world_cmp[0].equal_valid(world_clone):
                                        failed = True
                                        #print "Equal to another world to be tested"
                                        break
                            if not failed:
                                for world_cmp in self.planned_worlds:
                                    if world_cmp[0].equal_valid(world_clone):
                                        failed = True
                                        #print "Equal to a previously tested world"
                                        break
                            if not failed:
                                new_world = (world_clone, world[1] + float(change), world[2] + [(robot_name, at, parameters)], world)
                                self.worlds.append(new_world)
                                if self.world_good(world_clone):
                                    if self.time_limit == None:
                                        self.time_limit = new_world[1]
                                    if self.time_limit > new_world[1]:
                                        self.time_limit = new_world[1]
                                    self.good_worlds.append(new_world)
                            
                            world_clone = world[0].copy()

        self.good_worlds.sort(key=lambda x: x[1])
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
            return self.good_worlds[0][0], self.good_worlds[0][1], clean_actions(self.good_worlds[0][2])
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

class BlastPlannableWorld:
    def __init__(self, world):
        self.world = world
        self.real_world = False
        self.plan_steps = {}
        self.current_plan = []

        self.display_plan = []

        self.post_exec_world = None
        self.action_callback = lambda r, a, p: True
        def action_e_fail(r, a, p): 
            print "Action failed epically", r, "-->", a
            print "With parameters: ", p
        self.action_epic_fail_callback = action_e_fail

    def copy(self):
        c = BlastPlannableWorld(self.world.copy())
        c.real_world = False
        return c

    #Documentation:
    #plan_and_return = instantly execute, return immediately
    #report_plan = at the end return the plan instead of True/None

    def plan_hunt(self, robot, holder, object_type, world_good = None, plan_and_return = False, report_plan = False, execution_cb = lambda x: None):
        planner = Planner(self.world.copy())
        world, est_time, steps = planner.plan_hunt(robot, holder, object_type, world_good)
        return self.exec_plan(world, est_time, steps, plan_and_return, report_plan, execution_cb)

    def plan(self, world_good, extra_goals, plan_and_return = False, report_plan = False, execution_cb = lambda x: None):
        planner = Planner(self.world.copy())
        planner.world_good = world_good
        planner.extra_goals = extra_goals
        world, est_time, steps = planner.plan_print()
        return self.exec_plan(world, est_time, steps, plan_and_return, report_plan, execution_cb)

    def exec_plan(self, world, est_time, steps, plan_and_return = False, report_plan = False, execution_cb = lambda x: None):
        if plan_and_return:
            return world, est_time, steps
        if world != None and est_time != None and steps != None:
            print "Taking", len(steps), "steps"
            x = 0
            for step in steps:
                if step[1] == None:
                    print "REPLAN!!!", step
                    
                    planner = Planner(self.world.copy())
                    planner.world_good = step[2]["world_good"]
                    planner.extra_goals = step[2].get("extra_goals", {})
                    n_world, n_est_time, n_steps = planner.plan_print()
                    if n_world != None and n_est_time != None and n_steps != None:
                        print "Plan worked!!!!", n_steps
                        return self.exec_plan(n_world, n_est_time, n_steps, plan_and_return, report_plan, execution_cb)
                    else:
                        print "Replan failed, continuing default course."

                    continue
                execution_cb(steps[x:])
                x = x + 1
                print step[0], step[1], step[2]
                if not self.take_action(step[0], step[1], step[2]):
                    print "FAILED"
                    return None
            execution_cb([])
            #print world.to_text()
            if not self.real_world:
                if not world.equal(self.world):
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
            return world, est_time, steps
        return True
    
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
        

    def plan_action(self, robot, action, parameters, world_limits = {}, plan_and_return = False, include_action = False, execution_cb = lambda x: None):
        #FIXME: this can create problems if parameters is an extra element
        r = self.plan(lambda w: w.world_limit_check(world_limits) and w.take_action(robot, action, parameters, False, False) != None, 
                      {}, plan_and_return = plan_and_return, report_plan = True, 
                      execution_cb = lambda x: execution_cb(x + [(robot, action, parameters),]))
        if r != None:
            if include_action:
                r[2].append((robot, action, parameters))
            if not plan_and_return:
                worked = self.take_action(robot, action, parameters)
                if not worked:
                    return None #This really shouldn't happen, because the action is tested.
            execution_cb([])
        return r   

    def take_action(self, robot, action, parameters, debug = True):
        parameters = parameters.copy()
        for name in parameters:
            if parameters[name].__class__ == blast_world.BlastSurface:
                parameters[name] = parameters[name].name
            elif parameters[name].__class__ == blast_world.BlastObjectRef:
                parameters[name] = parameters[name].to_text()
            elif hasattr(parameters[name], "to_dict"):
                parameters[name] = parameters[name].to_dict()

        if self.real_world:
            test_world = self.world.copy()
            if test_world.take_action(robot, action, parameters, True, debug) == None:
                print "Attempted to take action in a state that was not desirable"
                return None
            else:
                if not self.action_callback(robot, action, parameters):
                    print "Action callback failed to work, going to epic fail"
                    self.action_epic_fail_callback(robot, action, parameters)
                    return None
                if not test_world.equal(self.world, True): #Important to tolerate arm error
                    print "-"*60
                    print test_world.to_text()
                    print "-"*60
                    print self.world.to_text()
                    print "-"*60
                    print "The output world differs, epic fail!"
                    self.action_epic_fail_callback(robot, action, parameters)
                    return None
            return True
        elif self.world.take_action(robot, action, parameters, True, debug):
            if not self.action_callback(robot, action, parameters):
                print "Action callback failed to work, this test may be broken."
                return None
            return True
        else:
            print "Blast failed to take action"
            return None
    #End API actions ---------------------------        


if __name__ == '__main__' and True:
    world = BlastPlannableWorld(blast_world.make_table_top_world(False))
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

    print world.plan_hunt("stair4", "cupholder", "coffee_cup")


    print '-'*100
    print "Plan to put down coffee cup"
    print '-'*100
    print world.plan_action("stair4", "table-place-left", {"table": "table_1", "position": "table_1, Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)"})


if __name__ == '__main__' and False:
    world = BlastPlannableWorld(blast_world.make_test_world())

    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
    
    print '-'*100
    print "Plan to pick up bag"
    print '-'*100
    world.plan_to_location("stair4", initial_pickup_point)

    print '-'*100
    print "Grab money bag"
    print '-'*100
    world.take_action("stair4", "grab-object", {"tts-text": "Money Bag"})
    world.set_robot_holder("stair4", "left-arm", "coffee_money_bag")

    print '-'*100
    print "Plan to buy coffee"
    print '-'*100
    world.plan_action("stair4", "buy-coffee", {"shop": "clark_peets_coffee_shop"})
    
    print '-'*100
    print "Plan to return"
    print '-'*100
    world.plan_to_location("stair4", initial_pickup_point)
    
    print '-'*100
    print "Give object back"
    print '-'*100
    world.take_action("stair4", "unstash-cupholder", {})
    world.take_action("stair4", "give-object", {"tts-text": "Coffee Cup"})
