
import blast_world

class Planner:
    def __init__(self, initial_world):
        self.worlds = [(initial_world, 0, [])]
        self.world_good = lambda x: False
        self.planned_worlds = []
        self.good_worlds = []
        self.time_limit = False
        self.worlds_tested = 0
        self.actions_tested = 0
        self.actions_finished = 0

        self.fail_debug = {}
        self.super_fail_debug = False


    def parameter_iter(self, param, keys = None):
        if keys == []: return [{},]
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
        if keys == []: return [{},]
        key = keys[0]
        keys = keys[1:]

        out = []
        down = self.parameter_iter(param, keys)
        for next_d in down:
            ls = param[key]
            if type(ls) != type([]): #Handle dependent keys
                ls = ls[1]
                for v in param[key][0]:
                    ls = ls[next_d[v]]
            for value in ls:
                nxt = {}
                for v in next_d: nxt[v] = next_d[v]
                nxt[key] = value
                out.append(nxt)

        return out

    def plan_recursive(self):
        for world in self.worlds:
            if self.world_good(world[0]):
                self.good_worlds.append(world)
                if not self.time_limit:
                    self.time_limit = world[1]
                if world[1] < self.time_limit:
                    self.time_limit = world[1]

        while self.worlds != []:
            self.worlds_tested = self.worlds_tested + 1
            self.worlds.sort(key = lambda x: x[1]) #Sort by lowest time
            world = self.worlds[0]
            self.worlds = self.worlds[1:]
            self.planned_worlds.append(world)

            #Once we find a good world, we stop planning for worlds after that time
            #if self.time_limit and world[1] >= self.time_limit: continue

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
                    planable, pv = world_clone.enumerate_action(robot_name, at)
                    parameter_combos = []
                    if planable and pv != False:
                        parameter_combos = self.parameter_iter(pv)
                    for parameters in parameter_combos:
                        self.actions_tested = self.actions_tested + 1
                        change = world_clone.take_action(robot_name, at, parameters)
                        #print robot.location, at, parameters, "->", change
                        if change == None: #failed
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
        


if __name__ == '__main__':
    world = blast_world.make_test_world()
    planner = Planner(world)
    planner.world_good = lambda w: w.robots["stair4"].holders["cup-holder"] != None and w.robots["stair4"].location.equal(w.surfaces["clarkfirstflooroutsidedoor"].locations["in_entrance"])
    world, time, steps = planner.plan_recursive()
    if world and time and steps:
        print "Estimated time", time, "s"
        print "Steps (total of", len(steps), "actions)"
        for i in steps:
            print i
    else:
        print "Failed to find a plan"
    print "Tried", planner.worlds_tested, "worlds and", planner.actions_tested, "actions -", \
        planner.actions_finished, "finished (a", (planner.actions_finished * 100.0) / planner.actions_tested, "percent success rate)"
    print "Fail debug", planner.fail_debug
    if planner.super_fail_debug:
        print "Super fail debug"
        for name, arr in planner.super_fail_debug.iteritems():
            print "Failed", name, "actions"
            for fail in arr:
                print fail[0][0].to_text()
                print fail[0][0].robots["stair4"].location, fail

