import blast
import blast_world_test, blast_world
import blast_planner

#Sandbox file for planner work.





def coffee_hunt_test():
    import blast_world_test
    world = blast_planner.BlastPlannableWorld(blast_world_test.make_table_top_world(False))
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
                       blast_world.BlastCodeStep(None, "IF", {"condition": ('?', blast_world.BlastParameterPtr('plan_return')),
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
                       blast_world.BlastCodeStep(None, "IF", {"condition": ("?", blast_world.BlastParameterPtr('plan_return')),
                                                              'label_true': "success2", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success2", "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL"),],
                      ["stair4",])

    world.run(True)

    world.print_old()

    return True


def run_test():
    import blast_world_test
    world = blast_planner.BlastPlannableWorld(blast_world_test.make_test_world())

    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
                       #Grab the bag
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": {"stair4": initial_pickup_point}},
                                                                "extra_steps": [("stair4", "grab-object", {"tts-text": "Money Bag"}),],}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": ('?', blast_world.BlastParameterPtr('plan_return')), "label_false": 'failure'}),

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
                       ], ["stair4",])
    
    
    world.try_exec()

def coffee_run_exec():
    import blast_world_test
    world = blast_planner.BlastPlannableWorld(blast_world_test.make_test_world())
    initial_pickup_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloor")
    rand_point = blast_world.BlastPt(17.460, 38.323, -2.330, "clarkcenterfirstfloordoor")
    world.append_plan([blast_world.BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": {"stair4": rand_point}}}, "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": ('?', blast_world.BlastParameterPtr('plan_return')), "label_false": 'failure'}),
                       blast_world.BlastCodeStep(None, "PLAN", 
                                                 {"extra_steps": [("stair4", "coffee-run", 
                                                                   {"shop": "clark_peets_coffee_shop",
                                                                    "person_location": initial_pickup_point}),],},
                                                 "plan_return"),
                       blast_world.BlastCodeStep(None, "IF", {"condition": ('?', blast_world.BlastParameterPtr('plan_return')), "label_false": 'failure'}),
                       
                       blast_world.BlastCodeStep(None, "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL")
                       ], ["stair4",])

    world.run(True)

    world.print_old()

    print "Planned:", world.times_planned
    

def five_coffee_run_exec():
    import blast_world_test
    world = blast_planner.BlastPlannableWorld(blast_world_test.make_test_world())
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
                       ], ["stair4",])

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
    world = blast_planner.BlastPlannableWorld(world_i)

    #print "-"*180
    #print world.world.to_text()
    #print "-"*180

    world.append_plan([blast_world.BlastCodeStep(None, "CALLSUB", {'sub': 'hunt_objects', 'object_types': "coffee_cup",
                                                                   'holder': 'stair4.cupholder'}, 'plan_return'),
                       blast_world.BlastCodeStep(None, "IF", {"condition": blast_world.BlastParameterPtr('plan_return'),
                                                              'label_true': "success", 'label_false': 'failure'}),
                       blast_world.BlastCodeStep("success", "RETURN"),
                       blast_world.BlastCodeStep("failure", "FAIL"),],
                      ["stair4", "stair5",])
    
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
    world = blast_planner.BlastPlannableWorld(world_i)

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
                       ], ["stair5",])
    
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
                       ], ["stair4", "stair5",])

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
    print coffee_run_exec()
    #print five_coffee_run_exec()
    #print multi_robot_test()
    #print overplan()

    mpt = blast_planner.motion_plan_hits + blast_planner.motion_plan_misses
    if mpt < 1: mpt = 1 #Avoid 0 div
    print "Motion planed", mpt, "times, with", blast_planner.motion_plan_hits, "hits", blast_planner.motion_plan_hits * 1.0/mpt, "percent"
