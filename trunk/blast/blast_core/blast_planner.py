
import blast_world, time

class Planner:
    def __init__(self, initial_world):
        self.worlds = [(initial_world, 0, [])]
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
            for key in param.keys():
                if type(param[key]) == type([]):
                    deps[key] = []
                else:
                    deps[key] = param[key][0]
            keys = []
            while deps.keys() != []:
                k = deps.keys()
                while k != []:
                    key = k[0]
                    k = k[1:]
                    deps_met = True
                    for d in k:
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
                ls = ls[1]
                for v in param[key][0]:
                    ls = ls[next_d[v]]
            for value in ls:
                nxt = next_d.copy()
                nxt[key] = value
                yield nxt
        
    def plan_recursive(self):
        for world in self.worlds:
            if self.world_good(world[0]):
                self.good_worlds.append(world)
                if not self.time_limit:
                    self.time_limit = world[1]
                if world[1] < self.time_limit:
                    self.time_limit = world[1]
        
        while self.worlds != []:
            world = min(self.worlds, key = lambda x: x[1])
            self.worlds.remove(world)
            self.planned_worlds.append(world)

            #Once we find a good world, we stop planning for worlds after that time
            if self.time_limit != None and world[1] >= self.time_limit: break
            
            #Actually testing this world
            self.worlds_tested = self.worlds_tested + 1

            for robot_name, robot in world[0].robots.iteritems():
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

                #Now enumerate all possible actions
                world_clone = world[0].copy()
                for at in action_types:
                    planable, pv = world_clone.enumerate_action(robot_name, at, self.extra_goals)
                    if not planable or pv == False:
                        continue
                    for parameters in self.parameter_iter(pv):
                        self.actions_tested = self.actions_tested + 1
                        change = world_clone.take_action(robot_name, at, parameters) 
                        #print robot.location, at, parameters, "->", change
                        if self.action_type_debug != False:
                            self.action_type_debug[at] = self.action_type_debug.get(at, 0) + 1
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
                            if not failed:
                                for world_cmp in self.worlds:
                                    if world_cmp[0].equal(world_clone):
                                        failed = True
                                        #print "Equal to another world to be tested"
                                        break
                            if not failed:
                                for world_cmp in self.planned_worlds:
                                    if world_cmp[0].equal(world_clone):
                                        failed = True
                                        #print "Equal to a previously tested world"
                                        break
                            if not failed:
                                #print "Succeeded"
                                new_world = (world_clone, world[1] + float(change), world[2] + [(robot_name, at, parameters)])
                                self.worlds.append(new_world)
                                if self.world_good(world_clone):
                                    if self.time_limit == False:
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
                            action_clean[param] = ("surface", value.name)
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
        self.action_callback = lambda r, a, p: True
        def action_e_fail(r, a, p): 
            print "Action failed epically", r, "-->", a
        self.action_epic_fail_callback = action_e_fail

    def plan(self, world_good, extra_goals):
        planner = Planner(self.world.copy())
        planner.world_good = world_good
        planner.extra_goals = extra_goals
        world, est_time, steps = planner.plan_print()
        if world != None and est_time != None and steps != None:
            print "Taking", len(steps), "steps"
            for step in steps:
                params = {}
                for name, value in step[2].iteritems():
                    if type(value) == type((0,1,)):
                        if value[0] == 'surface':
                            params[name] = self.world.surfaces[value[1]]
                        else:
                            params[name] = value
                    else:
                        params[name] = value
                self.take_action(step[0], step[1], params) 
            #print world.to_text()
            if not self.real_world:
                if not world.equal(self.world):
                    print "FAILED! Big issue: took actions that did not produce the desired world state"
                    return None
        else:
            print "FAILED!"
            return None
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
        
        self.world.robots[robot] = self.world.robots[robot].copy()
        self.world.robots[robot].holders[to_holder] = self.world.robots[robot].holders[from_holder]
        self.world.robots[robot].holders[from_holder] = None
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
        self.world.robots[robot] = self.world.robots[robot].copy()
        if object_type != None:
            obj_type = self.world.types.get_object(object_type)
            obj = blast_world.BlastObject(obj_type, None, robot + "." + holder)
            self.world.append_object(obj)
            self.world.robots[robot].holders[holder] = blast_world.BlastObjectRef(obj.uid)
        else:
            self.world.robots[robot].holders[holder] = None
        self.world.gc_objects()
        self.world.hash_state = None
        return True

    def set_robot_position(self, robot, position, val):
        if not robot in self.world.robots:
            print "Set robot position invalid robot", robot
            return False
        if not position in self.world.robots[robot].positions:
            print "Set robot holder invalid position", position, "for robot", robot
            return False

        if type(val) == type([]):
            for name, v in zip(self.world.robots[robot].robot_type.position_variables[position][False][0], val):
                if v != None:
                    self.world.robots[robot].positions[position][name] = v
        

    def set_robot_location(self, robot, blast_pt):
        if not robot in self.world.robots:
            print "Set robot location invalid robot", robot
            return False
        self.world.robots[robot] = self.world.robots[robot].copy()
        self.world.robots[robot].location = blast_pt.copy()
        self.world.hash_state = None
        return True

    def get_surface(self, surface):
        return self.world.surfaces.get(surface) #FIXME: convert to JSON

    def plan_action(self, robot, action, parameters):
        #FIXME: this can create problems if parameters is an extra element
        r = self.plan(lambda w: w.take_action(robot, action, parameters, False, False) != None, {})
        if r != None:
            return self.take_action(robot, action, parameters)
        return r
    
    #End API actions ---------------------------        

    def take_action(self, robot, action, parameters, debug = True):
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
                    self.action_epic_fail_callback(robot, action, parameters)
                    return None
            return True
        elif self.world.take_action(robot, action, parameters, True, debug):
            return True
        else:
            print "Blast failed to take action"
            return None



if __name__ == '__main__':
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
    world.plan_action("stair4", "buy_coffee", {"shop": "clark_peets_coffee_shop"})
    
    print '-'*100
    print "Plan to return"
    print '-'*100
    world.plan_to_location("stair4", initial_pickup_point)
    
    print '-'*100
    print "Give object back"
    print '-'*100
    world.take_action("stair4", "give-object-cupholder", {"tts-text": "Coffee Cup"})
