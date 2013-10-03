
import blast_world

class Planner:
    def __init__(self, initial_world):
        self.worlds = [(initial_world, 0, [])]
        self.planned_worlds = []


    def parameter_iter(self, param, keys = None):
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

        if keys == []: return [{}]

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
        while self.worlds != []:
            self.worlds.sort(key = lambda x: x[1]) #Sort by lowest time
            world = self.worlds[0]
            self.worlds = self.worlds[1:]
            self.planned_worlds.append(world)

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
                    pv = world_clone.enumerate_action(robot_name, at)
                    for parameters in self.parameter_iter(pv):
                        change = world_clone.take_action(robot_name, at, parameters)
                        #print robot.location, at, parameters, "->", change
                        if change != None: #Action succeeded.
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
                                self.worlds.append((world_clone, world[1] + float(change), world[2] + [(robot_name, at, parameters)]))
                            
                            world_clone = world[0].copy()

        print "No world found"
        for i in self.planned_worlds:
            robot = i[0].robots["stair4"]
            #print robot.location.to_text(), [name + ":" + robot.holders[name].object_type.name if robot.holders[name] else name for name in robot.holders], "--", i[1], i[2]
        print "Done", len(self.planned_worlds)
        


if __name__ == '__main__':
    world = blast_world.make_test_world()
    planner = Planner(world)
    print planner.plan_recursive()

