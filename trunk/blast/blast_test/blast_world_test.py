import blast
from blast_world import *
import os, sys



coffee_code = """
#Code for the action__pr2-cupholder__coffee-run = action pr2-cupholder.coffee-run. Parameters 'robot', 'shop' and 'person_location'
STARTSUB:
    STARTSUB {'robot': 'robot', 'person_location': 'Pt', 'shop': 'surface'};
    bag_location = CALLSUB {'sub': 'hunt_objects', 
                            'object_types': "empty_ziplock_1L_bag",
                            'holder': robot + '.left-arm'};

    #Fail if we can't find the empty bag
    IF ('==', ('?', bag_location), FALSE) FAIL;

    #At this point, the bag is in the arm, so we need to get the type of bag.
    empty_bag = GETOBJECT {'holder': robot + '.left-arm'};

    #Now we give the human the bag
    plan_return = PLAN {'world_limits': 
                        {'robot-holders': 
                         {robot: {"left-arm": empty_bag},},
                         "robot-location": 
                         {robot: person_location}},
                         "extra_steps": [(robot, "give-object", {"tts-text": "Empty Bag"}),],};
    IF ('==', plan_return, FALSE) FAIL;

    #Now we get back the bag
    plan_return = PLAN {"world_limits": {"robot-location": 
                                         {robot: person_location}},
                        "extra_steps": [(robot, "grab-object", {"tts-text": "Money Bag"}),],};
    IF ('==', plan_return, FALSE) FAIL;

    SETROBOTHOLDER {"holder": robot + '.left-arm',
                    "require-preexisting": True, "object-type": "coffee_money_bag"};
    
    coffee_money_bag = GETOBJECT {'holder': robot + '.left-arm'};

    plan_return = PLAN {"world_limits": {"robot-holders": 
                                         {robot:
                                          {"left-arm": coffee-run__coffee_money_bag},},
                                        },
                        "extra_steps": [(robot, "buy-coffee", {"shop": shop}),], };
    IF ('==', plan_return, FALSE) FAIL;

    coffee_cup = GETOBJECT {'holder': robot + '.left-arm'};

    #Return the coffee to the human
    plan_return = PLAN {"world_limits": {"robot-holders": 
                                          {robot:
                                           {"left-arm": coffee_cup},},
                                          "robot-location": 
                                           {robot: person_location}},
                        "extra_steps": [(robot, "give-object", {"tts-text": "Coffee Cup"}),],};
    IF ('==', plan_return, FALSE) FAIL;

    plan_return = PLAN {"world_limits": {"robot-location": 
                                         {robot: person_location}},
                        "extra_steps": [(robot, "grab-object", {"tts-text": "Empty Bag"}),],};
    IF ('==', plan_return, FALSE) FAIL; 
            
    SETROBOTHOLDER {"holder": robot + '.left-arm',
                    "require-preexisting": True, "object-type": "empty_ziplock_1L_bag"};

            #Set the bag down on the table
    coffee_money_bag = GETOBJECT {'holder': robot + '.left-arm'};

    plan_return = PLAN, {"world_limits": 
                         {"place-objects":
                          [{'object': coffee_money_bag,
                            'surfaceposition': bag_location,
                           },]},};
    IF ('==', plan_return, FALSE) FAIL; 
    RETURN;
"""

def process_code(codebase, action_name = None):
    strings_d = {}
    string_c = 0

    codebase_strip = codebase
    codebase_ns = ""
    while codebase_strip.find('\'') != 0 or codebase_strip.find('"') != 0:
        fsq = codebase_strip.find('\'')
        fdq = codebase_strip.find('"')
        str_d = None
        if fsq >= 0 and (fdq == -1 or fsq < fdq):
            str_d = '\''
        if fdq >= 0 and (fsq == -1 or fdq < fsq):
            str_d = '"'
        if str_d == None:
            codebase_ns += codebase_strip
            break
        else:
            
        
            
    
    

process_code(coffee_code, 'action__pr2-cupholder__coffee-run')

def code_action(*args, **kwargs):
    name = args[0]
    robot, act = name.split(".")

    code = None
    if args[3].strip().strip(')').strip('(').strip().lower() != "true": #Code is empty for macro
        my_path = os.path.dirname(os.path.abspath(__file__))
        codef = open(my_path + "/test_actions/" + robot + "__" + act + ".py", "r")
        code = ""
        for l in codef:
            code += l
        codef.close()
    
    argc = tuple([name, code] + list(args[1:]))
    return BlastAction(*argc, **kwargs)

def make_test_actions():
    test = [code_action("pr2.__root", {}, "True()", "\"0\"", {}, [], {}, planable = False),
            code_action("pr2.move", {"end": "Pt"},
                        ("&&", ("==", "robot.location.map", "end.map"),
                         #("!=", "robot.location", "end"),
                         ("position", "robot.torso", [0.0,]),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked")),
                        "add(mul(\"8\", dist(robot.location, end)), abs(angle_diff(robot.location.a, end.a)))",
                        {"robot.location": "end" },
                        [("line", "robot.location", "end", "00FF00"),
                         ], {None: ["end"]}),
            code_action("pr2.tuck-both-arms", {},
                        ("contains", "robot.left-arm", {"rotation_limit": [(0.5, math.pi)],
                                                        "accept_empty": True}), 
                        "\"10\"", 
                        {"robot.positions.left-arm": [0.06024, 1.248526, 1.789070, -1.683386, 
                                                      -1.7343417, -0.0962141, -0.0864407, None],
                         "robot.positions.right-arm": [-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                        -1.4175, -1.8417, 0.21436, None]},
                        [("icon", "robot.location", "action_fs/pr2/tuck_both_arms/icon.png"), ], {}),
            
            code_action("pr2-cupholder.buy-coffee", {"shop": "Surface:coffee_shop"},
                        ("&&", ("==", "robot.location", "shop.locations.start"),
                         ("contains", "robot.left-arm", "coffee_money_bag")),
                        "\"1000\"",
                        {"robot.location": "shop.locations.end",
                         "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                         "robot.positions.right-arm": "None()",
                         "robot.holders.left-arm": "Object(\"coffee_cup\")"},
                        [("line", "robot.location", "shop.locations.end"),
                         ("icon", "shop.locations.end", "action_fs/pr2/buy-coffee/icon.png"), ],
                        {"shop": ["shop.locations.end"]}, planable = False),
            code_action("pr2.door-blast", {"door": "Surface:transparent_heavy_door"},
                        ("&&", ("==", "robot.location", "door.locations.out_entrance"),
                         ("position", "robot.torso", [0.3,]),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked")),
                        "\"55\"", {"robot.location": "door.locations.out_exit"},
                        [("icon", "door.locations.out_entrance", "action_fs/pr2/door-blast/icon.png"),],
                        {"door": ["door.locations.out_entrance", "door.locations.out_exit"], }),
            code_action("pr2.door-drag", {"door": "Surface:transparent_heavy_door"},
                        ("&&", ("==", "robot.location", "door.locations.in_entrance"), 
                         ("position", "robot.torso", [0.3,]),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked"),
                         ("contains", "robot.right-arm", "None()"),),
                        "\"115\"", {"robot.location": "door.locations.in_exit",
                                    "robot.positions.left-arm": False, 
                                    "robot.positions.right-arm": False},
                        [("icon", "door.locations.in_entrance", "action_fs/pr2/door-drag/icon.png"),],
                        {"door": ["door.locations.in_entrance", "door.locations.in_exit"], }),

            code_action("pr2.torso", {"height": "Joint:torso.torso"},
                        "True()", "\"20\"", #FIXME
                        {"robot.positions.torso": ["height",]},
                        [("icon", "robot.location", "action_fs/pr2/torso/icon.png"),], {}),
            code_action("pr2.head", {"pan": "Joint:head.pan", "tilt": "Joint:head.tilt"},
                        "True()", "\"3\"",
                        {"robot.positions.head": ["pan", "tilt",]},
                        [("icon", "robot.location", "action_fs/pr2/head/icon.png"),], {}),
                        
            code_action("pr2.elevator", {"elevator": "Surface:elevator", 
                                         "infloor": "Location:elevator.floor_",
                                         "outfloor": "Location:elevator.floor_"},
                        ("&&", ("==", "robot.location", "infloor"), ("!=", "infloor", "outfloor"),
                         ("contains", "robot.right-arm", "None()"),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.torso", [0.0,]),
                         ("position", "robot.right-arm", "tucked")),
                        "\"150\"", {"robot.location": "outfloor"},
                        [("icon", "infloor", "action_fs/pr2/elevator/icon.png"),],
                        {"elevator": ["infloor", "outfloor"] }),
            
            code_action("pr2.grab-object", {"tts-text": "String"}, 
                        ("contains", "robot.left-arm", "None()"),
                        "\"30\"", {"robot.holders.left-arm": "Object(\"arbitrary-object\")",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   "robot.positions.right-arm": False},
                        [("icon", "robot.location", "action_fs/pr2/grab-object/icon.png"),], {},
                        planable = False),
            code_action("pr2.give-object", {"tts-text": "String"}, 
                        ("not", ("contains", "robot.left-arm", "None()")),
                        "\"30\"", {"robot.holders.left-arm": "None()",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   "robot.positions.right-arm": False},
                        [("icon", "robot.location", "action_fs/pr2/give-object/icon.png"),], {},
                        planable = False),

            
            code_action("pr2-cupholder.stash-cupholder", {}, 
                        ("&&", ("contains", "robot.left-arm", {"has_tag": "cupholder_object"}),
                         ("contains", "robot.cupholder", "None()")),
                        "\"30\"", {"robot.holders.cupholder": "robot.holders.left-arm",
                                   "robot.holders.left-arm": "None()",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   #"robot.positions.right-arm": None
                                   },
                        [("icon", "robot.location", "action_fs/pr2-cupholder/stash-cupholder/icon.png"),], {},),
            code_action("pr2-cupholder.unstash-cupholder", {}, 
                        ("&&", ("contains", "robot.cupholder", {"has_tag": "cupholder_object"}),
                         ("contains", "robot.left-arm",  "None()")),
                        "\"30\"", {"robot.holders.cupholder": "None()",
                                   "robot.holders.left-arm": "robot.holders.cupholder",
                                   "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                                   #"robot.positions.right-arm": None
                                   },
                        [("icon", "robot.location", "action_fs/pr2-cupholder/unstash-cupholder/icon.png"),], {},),


            code_action("pr2-cupholder.coffee-run", {"person_location": "Pt", "shop": "Surface:coffee_shop"}, 
                        "True()", "True()", {"robot.location": "False()",
                                             "robot.positions.left-arm": False, 
                                             "robot.positions.right-arm": False,}, 
                        [], {}, planable = False, user = True, 
                        fm = {"no-bag": {"robot.location": "False()", 
                                         "robot.positions.left-arm": False, 
                                         "robot.positions.right-arm": False,
                                         },
                              }),

            code_action("pr2-cupholder.five-coffee-run", {"person_location": "Pt", "shop": "Surface:coffee_shop"}, 
                        "True()", "True()", {"robot.location": "False()",
                                             "robot.positions.left-arm": False, 
                                             "robot.positions.right-arm": False,}, 
                        [], {}, planable = False, user = True, 
                        fm = {"no-bag": {"robot.location": "False()", 
                                         "robot.positions.left-arm": False, 
                                         "robot.positions.right-arm": False,
                                         },
                              }),



            code_action("pr2.table-pick-left", 
                        {"table": "Surface:table", "object": "SurfaceObject:table"}, #FIXME: motion/size limits + can't be too far back
                        
                        ("&&", ("contains", "robot.left-arm",  "None()"),
                         ("==", "robot.location", "table.locations.location")), "\"20\"",

                        {"robot.holders.left-arm": "object",
                         "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                         }, [("icon", "robot.location", "action_fs/pr2/table-pick-left/icon.png"),],
                        {"table": ["table.locations.location",], },
                        fm = {"no_object": 
                              {"robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                               "table.objects-0": "object",
                               }}),
            #BlastAction("pr2-cupholder.table-pick-right", 
            #            {"table": "Surface:table", "object": "SurfaceObject:table"},
            #            
            #            ("==", "robot.location", "table.locations.location"), "\"20\"",
#
#                        {"robot.holders.right-arm": "object",
#            "robot.positions.right-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
#                         }, []),
            
            code_action("pr2.table-place-left", 
                        {"table": "Surface:table", "position": "Pos:table, robot.holders.left-arm, 0.6602, 0, 0.762, 0, 0, 0"},
                        
                        ("&&", ("not", ("contains", "robot.left-arm", "None()")),
                         ("==", "robot.location", "table.locations.location")), "\"20\"",

                        {"table.objects+0": "robot.holders.left-arm:position",
                         "robot.holders.left-arm": "None()",
                         "robot.positions.left-arm": [0.0, -0.350, 0.0, -1.225, 3.14159, -1.65, 0.0, False], 
                         #"robot.positions.left-arm": False,
                         }, [("icon", "robot.location", "action_fs/pr2/table-place-left/icon.png"),],
                        {"table": ["table.locations.location",], },),

            
            code_action("pr2.table-coffee-scan", 
                        {"table": "Surface:table"},
                        ("&&", ("==", "robot.location", "table.locations.location"),
                         ("position", "robot.left-arm", "tucked"),
                         ("position", "robot.right-arm", "tucked")), "\"1\"",
                        {"table.scan": "coffee_cup,empty_ziplock_1L_bag" },
                        [("icon", "robot.location", "action_fs/pr2/table-coffee-scan/icon.png"),],
                        {"table": ["table.locations.location",], },),
            
            ]

    code = [#Code for the action__pr2-cupholder__five-coffee-run = action pr2-cupholder.five-coffee-run. Parameters 'robot', 'shop' and 'person_location'
            BlastCodeStep("action__pr2-cupholder__five-coffee-run", "STARTSUB", {'robot': 'robot', 'person_location': 'Pt'}),
            #Run once
            BlastCodeStep(None, "PLAN", {"extra_steps": [(BlastParameterPtr('robot'), "coffee-run", 
                                                          {"shop": BlastParameterPtr('shop'),
                                                           "person_location": BlastParameterPtr('person_location')}),]},
                          "action__pr2-cupholder__five-coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__five-coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__five-coffee-run__failure"}),
            #Run twice
            BlastCodeStep(None, "PLAN", {"extra_steps": [(BlastParameterPtr('robot'), "coffee-run", 
                                                          {"shop": BlastParameterPtr('shop'),
                                                           "person_location": BlastParameterPtr('person_location')}),]},
                          "action__pr2-cupholder__five-coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__five-coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__five-coffee-run__failure"}),

            BlastCodeStep(None, "RETURN"),
            #Run thrice
            BlastCodeStep(None, "PLAN", {"extra_steps": [(BlastParameterPtr('robot'), "coffee-run", 
                                                          {"shop": BlastParameterPtr('shop'),
                                                           "person_location": BlastParameterPtr('person_location')}),]},
                          "action__pr2-cupholder__five-coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__five-coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__five-coffee-run__failure"}),
            #Run quardic
            BlastCodeStep(None, "PLAN", {"extra_steps": [(BlastParameterPtr('robot'), "coffee-run", 
                                                          {"shop": BlastParameterPtr('shop'),
                                                           "person_location": BlastParameterPtr('person_location')}),]},
                          "action__pr2-cupholder__five-coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__five-coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__five-coffee-run__failure"}),
            #Run last time
            BlastCodeStep(None, "PLAN", {"extra_steps": [(BlastParameterPtr('robot'), "coffee-run", 
                                                          {"shop": BlastParameterPtr('shop'),
                                                           "person_location": BlastParameterPtr('person_location')}),]},
                          "action__pr2-cupholder__five-coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__five-coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__five-coffee-run__failure"}),
            
            BlastCodeStep(None, "RETURN"),
            BlastCodeStep("action__pr2-cupholder__five-coffee-run__failure", "FAIL"),
            
            
            #Code for the action__pr2-cupholder__coffee-run = action pr2-cupholder.coffee-run. Parameters 'robot', 'shop' and 'person_location'
            BlastCodeStep("action__pr2-cupholder__coffee-run", "STARTSUB", {'robot': 'robot', 'person_location': 'Pt'}),
            BlastCodeStep(None, "CALLSUB", {'sub': 'hunt_objects', 'object_types': "empty_ziplock_1L_bag",
                                            'holder': BlastParameterPtr('robot', postfix = '.left-arm')},
                          'action__pr2-cupholder__coffee-run__bag_location'),

            #Fail if we can't find the empty bag
            BlastCodeStep(None, "IF", {'condition': ('?', BlastParameterPtr('action__pr2-cupholder__coffee-run__bag_location')),
                                       'label_false': "action__pr2-cupholder__coffee-run__failure"}),
            #At this point, the bag is in the arm, so we need to get the type of bag.
            BlastCodeStep(None, "GETOBJECT", {'holder': BlastParameterPtr('robot', postfix = '.left-arm')}, 
                          "action__pr2-cupholder__coffee-run__empty_bag"),

            #Now we give the human the bag
            BlastCodeStep(None, "PLAN", {"world_limits": {"robot-holders": 
                                                          {BlastParameterPtr("robot"):
                                                               {"left-arm": BlastParameterPtr('action__pr2-cupholder__coffee-run__empty_bag')},},
                                                          "robot-location": 
                                                          {BlastParameterPtr('robot'): BlastParameterPtr('person_location')}},
                                         "extra_steps": [(BlastParameterPtr('robot'), "give-object", {"tts-text": "Empty Bag"}),],}, 
                          "action__pr2-cupholder__coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__coffee-run__failure"}),
            #Now we get back the bag
            BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": 
                                                          {BlastParameterPtr('robot'): BlastParameterPtr('person_location')}},
                                         "extra_steps": [(BlastParameterPtr('robot'), "grab-object", {"tts-text": "Money Bag"}),],}, 
                          "action__pr2-cupholder__coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__coffee-run__failure"}),
            
            BlastCodeStep(None, "SETROBOTHOLDER", {"holder": BlastParameterPtr('robot', postfix = '.left-arm'),
                                                   "require-preexisting": True, "object-type": "coffee_money_bag"}),
            BlastCodeStep(None, "GETOBJECT", {'holder': BlastParameterPtr('robot', postfix = '.left-arm')}, 
                          "action__pr2-cupholder__coffee-run__coffee_money_bag"),

            #Go for the coffee shop
            BlastCodeStep(None, "PLAN", {"world_limits": {"robot-holders": 
                                                          {BlastParameterPtr("robot"):
                                                               {"left-arm": BlastParameterPtr('action__pr2-cupholder__coffee-run__coffee_money_bag')},}
                                                          },
                                         "extra_steps": [(BlastParameterPtr('robot'), "buy-coffee", {"shop": BlastParameterPtr('shop')}),], },
                          "action__pr2-cupholder__coffee-run__plan_return"),
            BlastCodeStep(None, "GETOBJECT", {'holder': BlastParameterPtr('robot', postfix = '.left-arm')}, 
                          "action__pr2-cupholder__coffee-run__coffee_cup"),

            #Return the coffee to the human
            BlastCodeStep(None, "PLAN", {"world_limits": {"robot-holders": 
                                                          {BlastParameterPtr("robot"):
                                                               {"left-arm": BlastParameterPtr('action__pr2-cupholder__coffee-run__coffee_cup')},},
                                                          "robot-location": 
                                                          {BlastParameterPtr('robot'): BlastParameterPtr('person_location')}},
                                         "extra_steps": [(BlastParameterPtr('robot'), "give-object", {"tts-text": "Coffee Cup"}),],}, 
                          "action__pr2-cupholder__coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__coffee-run__failure"}),
            #Now we get back the bag
            BlastCodeStep(None, "PLAN", {"world_limits": {"robot-location": 
                                                          {BlastParameterPtr('robot'): BlastParameterPtr('person_location')}},
                                         "extra_steps": [(BlastParameterPtr('robot'), "grab-object", {"tts-text": "Empty Bag"}),],}, 
                          "action__pr2-cupholder__coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__coffee-run__failure"}),
            
            BlastCodeStep(None, "SETROBOTHOLDER", {"holder": BlastParameterPtr('robot', postfix = '.left-arm'),
                                                   "require-preexisting": True, "object-type": "empty_ziplock_1L_bag"}),

            #Set the bag down on the table
            BlastCodeStep(None, "GETOBJECT", {'holder': BlastParameterPtr('robot', postfix = '.left-arm')}, 
                          "action__pr2-cupholder__coffee-run__coffee_money_bag"),

            BlastCodeStep(None, "PLAN", {"world_limits": 
                                         {"place-objects":
                                              [{'object': BlastParameterPtr('action__pr2-cupholder__coffee-run__coffee_money_bag'),
                                                'surfaceposition': BlastParameterPtr('action__pr2-cupholder__coffee-run__bag_location'),
                                                },
                                               ]},}, 
                          "action__pr2-cupholder__coffee-run__plan_return"),
            BlastCodeStep(None, "IF", {'condition': BlastParameterPtr('action__pr2-cupholder__coffee-run__plan_return'),
                                       'label_false': "action__pr2-cupholder__coffee-run__failure"}),
                                                              

            BlastCodeStep(None, "RETURN"),
            BlastCodeStep("action__pr2-cupholder__coffee-run__failure", "FAIL"),
            ]

    return test, code


def make_test_types_world():
    types_world = BlastWorldTypes()
    types_world.add_object_type(ObjectType("arbitrary-object", {}))
    types_world.add_object_type(ObjectType("coffee_cup", 
                                           {"rotation_limit": 0.1,
                                            "accel_z": 0.2,
                                            "accel_x": 1.0,
                                            "accel_y": 1.0,
                                            "bound_d": 0.09398,
                                            "bound_h": 0.15748},
                                           "object_fs/coffee_cup/icon.png"))
    types_world.add_object_tag("coffee_cup", "cupholder_object")
    types_world.add_object_type(ObjectType("empty_ziplock_1L_bag", {"bound_d": 0.1, "bound_h": 0.01},
                                           "object_fs/empty_ziplock_1L_bag/icon.png"))
    types_world.add_object_type(ObjectType("coffee_money_bag", {"bound_d": 0.1, "bound_h": 0.01},
                                           "object_fs/coffee_money_bag/icon.png"))
    
    types_world.add_surface_type(SurfaceType("coffee_shop",
                                             {"default": {"default": True, "accessible": True}},
                                             ["start", "line_*", "end"],))
    types_world.add_surface_type(SurfaceType("table",
                                             {"default": {"default": True, "accessible": True}},
                                             ["location"],
                                             {"location": [ [(0.2540, -0.6985, 0.762), (0.2540,  0.6985, 0.762),
                                                             (1.0668,  0.6985, 0.762), (1.0668, -0.6985, 0.762),
                                                             ], ],
                                              }))
    types_world.add_surface_type(SurfaceType("transparent_heavy_door",
                                             {"default": {"default": True, "accessible": True}},
                                             ["in_entrance", "out_entrance", "in_exit", "out_exit",],))
    types_world.add_surface_type(SurfaceType("elevator",
                                             {"default": {"default": True, "accessible": True}},
                                             ["floor_*",]))
    ATL = 0.0001 #Arm offset tolerance
    types_world.add_robot_type(RobotType("pr2", {"width": 0.668, "height": 0.668, 
                                                 "image": {"image": "robot_fs/pr2/pr2_def.png",
                                                           "priority": 0, #Lowest
                                                           "left-arm": (0.668 , 0),
                                                           "right-arm": (0, 0),
                                                           },       
                                                 "image,left-arm=tucked": {"image": "robot_fs/pr2/pr2_left_tucked.png",
                                                                           "left-arm": ((24 / 48.0 - 0.5) * 0.668, (15 / 48.0 - 0.5) * 0.668),
                                                                           "right-arm": ((41 / 48.0 - 0.5) * 0.668, (11 / 48.0 - 0.5) * 0.668),
                                                                           "priority": 50,
                                                                           },
                                                 "image,right-arm=tucked": {"image": "robot_fs/pr2/pr2_right_tucked.png",
                                                                            "left-arm": ((6 / 48.0 - 0.5) * 0.668, (10 / 48.0 - 0.5) * 0.668),
                                                                            "right-arm": ((17 / 48.0 - 0.5) * 0.668, (20 / 48.0 - 0.5) * 0.668),
                                                                            "priority": 50,
                                                                            },
                                                 "image,left-arm=tucked,right-arm=tucked": {"image": "robot_fs/pr2/pr2_both_tucked.png",
                                                                                            "left-arm": ((24 / 48.0 - 0.5) * 0.668, (15 / 48.0 - 0.5) * 0.668),
                                                                                            "right-arm": ((17 / 48.0 - 0.5) * 0.668, (20 / 48.0 - 0.5) * 0.668),
                                                                                            "priority": 100,
                                                                                            },
                                                 },
                                         {"left-arm": {"mass-limit": 2.5}, 
                                          "right-arm": {"mass-limit": 2.5},
                                          },
                                         {"torso": {False: (["torso",], [ATL,], [0.0,]), "up": [0.3,], "down": [0.0,],
                                                    "MIN": [0.0,], "MAX": [0.3,], },
                                          "left-arm": {False: (["shoulder_pan", "shoulder_lift", "upper_arm_roll",
                                                                "elbow_flex", "forearm_roll", "r_wrist_flex",
                                                                "wrist_roll", "gripper"],
                                                               [ATL, ATL, ATL, ATL, ATL, ATL, ATL, ATL],
                                                               [False, False, False, False, False, False, False, False]),
                                                               #[0.06024, 1.248526, 1.789070, -1.683386, 
                                                               # -1.7343417, -0.0962141, -0.0864407, 0.0]),
                                                       "gripped": [None, None, None, None, None, None, None, 0.0],
                                                       "released": [None, None, None, None, None, None, None, 0.1],
                                                       "tucked": [0.06024, 1.248526, 1.789070, -1.683386, 
                                                                  -1.7343417, -0.0962141, -0.0864407, None],
                                                       "untucked": [0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0, None],
                                                       },
                                          "right-arm": {False: (["shoulder_pan", "shoulder_lift", "upper_arm_roll",
                                                                 "elbow_flex", "forearm_roll", "r_wrist_flex",
                                                                 "wrist_roll", "gripper"],
                                                                [ATL, ATL, ATL, ATL, ATL, ATL, ATL, ATL],
                                                                [False, False, False, False, False, False, False, False]),
                                                                #[-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                                #  -1.4175, -1.8417, 0.21436, 0.0]),
                                                       "gripped": [None, None, None, None, None, None, None, 0.0],
                                                       "released": [None, None, None, None, None, None, None, 0.1],
                                                        "tucked": [-0.023593, 1.1072800, -1.5566882, -2.124408,
                                                                    -1.4175, -1.8417, 0.21436, None],
                                                        "untucked": [-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0, None],
                                                        },
                                          "head": {False: (["pan", "tilt"], [ATL, ATL], [0, 0]), "level": [0.0, 0.0]},
                                          "tilt-laser": False,
                                          }))

    types_world.add_robot_type(RobotType("pr2-cupholder", {"image": 
                                                           {"image": "robot_fs/pr2-cupholder/pr2_def.png",
                                                            "priority": 0, #Lowest
                                                            "left-arm": (0.668 , 0),
                                                            "right-arm": (0, 0),
                                                            "cupholder": ((4.5 / 48.0 - 0.5) * 0.668, ((31 + 5) / 48.0 - 0.5) * 0.668),
                                                            },       
                                                           "image,left-arm=tucked": 
                                                           {"image": "robot_fs/pr2-cupholder/pr2_left_tucked.png",
                                                            "left-arm": ((24 / 48.0 - 0.5) * 0.668, (15 / 48.0 - 0.5) * 0.668),
                                                            "right-arm": ((41 / 48.0 - 0.5) * 0.668, (11 / 48.0 - 0.5) * 0.668),
                                                            "cupholder": ((4.5 / 48.0 - 0.5) * 0.668, ((31 + 5) / 48.0 - 0.5) * 0.668),
                                                            "priority": 50,
                                                            },
                                                           "image,right-arm=tucked":
                                                               {"image": "robot_fs/pr2-cupholder/pr2_right_tucked.png",
                                                                "left-arm": ((6 / 48.0 - 0.5) * 0.668, (10 / 48.0 - 0.5) * 0.668),
                                                                "right-arm": ((17 / 48.0 - 0.5) * 0.668, (20 / 48.0 - 0.5) * 0.668),
                                                            "cupholder": ((4.5 / 48.0 - 0.5) * 0.668, ((31 + 5) / 48.0 - 0.5) * 0.668),
                                                                "priority": 50,
                                                                },
                                                           "image,left-arm=tucked,right-arm=tucked": 
                                                           {"image": "robot_fs/pr2-cupholder/pr2_both_tucked.png",
                                                            "left-arm": ((24 / 48.0 - 0.5) * 0.668, (15 / 48.0 - 0.5) * 0.668),
                                                            "right-arm": ((17 / 48.0 - 0.5) * 0.668, (20 / 48.0 - 0.5) * 0.668),
                                                            "cupholder": ((4.5 / 48.0 - 0.5) * 0.668, ((31 + 5) / 48.0 - 0.5) * 0.668),
                                                            "priority": 100,
                                                            },},
                                         {"cupholder": {}}, {}, types_world.get_robot("pr2")))

    a, c = make_test_actions()
    [types_world.add_action_type(x) for x in a]
    types_world.add_script(c)
    return types_world

###############################################################################


def make_table_top_world(root_path, make_objects = True):
    world = BlastWorld(make_test_types_world())
    clarkcenterfirstfloor = BlastMap("clarkcenterfirstfloor", root_path, "maps/clarkcenterfirstfloor.pgm", 20.0)
    world.append_map(clarkcenterfirstfloor)

    world.append_surface(BlastSurface("table_1", 
                                      {"location": BlastPt(20.742, 18.390, -2.737, clarkcenterfirstfloor.map),},
                                      world.types.get_surface("table"),))
    world.append_surface(BlastSurface("table_2", 
                                      {"location": BlastPt(30.742, 18.390, -2.737, clarkcenterfirstfloor.map),},
                                      world.types.get_surface("table")))

    stair4 = BlastRobot("stair4", 
                        #BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                        BlastPt(12.000, 40.957, 0.148, clarkcenterfirstfloor.map),
                        world.types.get_robot("pr2-cupholder"))
    world.append_robot(stair4)

    world.take_action("stair4", "tuck-both-arms", {}) #To debug with arms tucked.
    world.take_action("stair4", "move", {"end": BlastPt(20.742, 18.390, -2.737, clarkcenterfirstfloor.map)}) #To debug with arms tucked.

    if make_objects:
        world.add_surface_object("table_1", world.types.get_object("coffee_cup"), BlastPos(0.6602, 0.0, 0.762, 0.0, 0.0, 0.0))
        world.add_surface_object("table_1", world.types.get_object("coffee_cup"), BlastPos(0.4602, 0.0, 0.762, 0.0, 0.0, 0.0))
    
    return world
    
    
    
def make_test_world(root_path):
    world = BlastWorld(make_test_types_world())

    clarkcenterfirstfloordoor = BlastMap("clarkcenterfirstfloordoor", root_path, "maps/clarkcenterfirstfloordoor.pgm", 20.0)
    world.append_map(clarkcenterfirstfloordoor)
    clarkcenterfirstflooroutside = BlastMap("clarkcenterfirstflooroutside", root_path, "maps/clarkcenterfirstflooroutside.pgm", 20.0)
    world.append_map(clarkcenterfirstflooroutside)
    clarkcenterfirstfloor = BlastMap("clarkcenterfirstfloor", root_path, "maps/clarkcenterfirstfloor.pgm", 20.0)
    world.append_map(clarkcenterfirstfloor)
    clarkcenterbasementelevator = BlastMap("clarkcenterbasementelevator", root_path, "maps/clarkcenterbasementelevator.pgm", 20.0)
    world.append_map(clarkcenterbasementelevator)
    clarkcenterfirstfloorelevator = BlastMap("clarkcenterfirstfloorelevator", root_path, "maps/clarkcenterfirstfloorelevator.pgm", 20.0)
    world.append_map(clarkcenterfirstfloorelevator)
    clarkcentersecondfloorelevator = BlastMap("clarkcentersecondfloorelevator", root_path, "maps/clarkcentersecondfloorelevator.pgm", 20.0)
    world.append_map(clarkcentersecondfloorelevator)
    clarkcenterthirdfloorelevator = BlastMap("clarkcenterthirdfloorelevator", root_path, "maps/clarkcenterthirdfloorelevator.pgm", 20.0)
    world.append_map(clarkcenterthirdfloorelevator)
    clarkcenterthirdflooroutside = BlastMap("clarkcenterthirdflooroutside", root_path, "maps/clarkcenterthirdflooroutside.pgm", 20.0)
    world.append_map(clarkcenterthirdflooroutside)
    clarkcenterpeetscoffee = BlastMap("clarkcenterpeetscoffee", root_path, "maps/clarkcenterpeetscoffee.pgm", 20.0)
    world.append_map(clarkcenterpeetscoffee)
                                 
    world.append_surface(BlastSurface("salisbury_table", 
                                      {"location": BlastPt(12.420, 39.077, 3.4165, clarkcenterfirstfloor.map),},
                                      world.types.get_surface("table")))
    world.append_surface(BlastSurface("clarkfirstfloordoor", 
                                      {"in_entrance": BlastPt(21.280, 26.643, 0.334, clarkcenterfirstfloordoor.map),
                                       "in_exit": BlastPt(20.656, 18.456, 0.350, clarkcenterfirstfloor.map),
                                       "out_entrance": BlastPt(20.742, 18.390, -2.737, clarkcenterfirstfloor.map),
                                       "out_exit": BlastPt(21.221, 26.481, -2.855, clarkcenterfirstfloordoor.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkfirstflooroutsidedoor", 
                                      {"in_entrance": BlastPt(45.255, 49.637, 1.494, clarkcenterfirstflooroutside.map),
                                       "in_exit": BlastPt(21.280, 26.643, 0.334, clarkcenterfirstfloordoor.map),
                                       "out_entrance": BlastPt(21.221, 26.481, -2.855, clarkcenterfirstfloordoor.map),
                                       "out_exit": BlastPt(45.427, 49.668, -1.735, clarkcenterfirstflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkfirstfloorelevatordoor", 
                                      {"in_entrance": BlastPt(95.682, 62.549, 2.640, clarkcenterfirstflooroutside.map),
                                       "in_exit": BlastPt(49.733, 13.540, 2.619, clarkcenterfirstfloorelevator.map),
                                       "out_entrance": BlastPt(49.654, 13.662, -0.501, clarkcenterfirstfloorelevator.map),
                                       "out_exit": BlastPt(95.647, 62.677, -0.709, clarkcenterfirstflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkthirdfloorelevatordoor", 
                                      {"in_entrance": BlastPt(53.317, 26.607, -3.016, clarkcenterthirdflooroutside.map),
                                       "in_exit": BlastPt(50.452, 12.200, 3.014, clarkcenterthirdfloorelevator.map),
                                       "out_entrance": BlastPt(50.452, 12.200, -0.125, clarkcenterthirdfloorelevator.map),
                                       "out_exit": BlastPt(53.561, 26.425, 0.101, clarkcenterthirdflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkthirdpeetscoffeedoor", 
                                      {"in_entrance": BlastPt(64.375, 33.672, 1.275, clarkcenterthirdflooroutside.map),
                                       "in_exit": BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                                       "out_entrance": BlastPt(56.659, 14.061, 3.100, clarkcenterpeetscoffee.map),
                                       "out_exit": BlastPt(64.258, 33.537, -1.990, clarkcenterthirdflooroutside.map)},
                                      world.types.get_surface("transparent_heavy_door")))
    world.append_surface(BlastSurface("clarkelevator", 
                                      {"floor_1": BlastPt(49.218, 19.462, -3.126, clarkcenterfirstfloorelevator.map),
                                       "floor_2": BlastPt(48.569, 17.927, -2.926, clarkcentersecondfloorelevator.map),
                                       "floor_3": BlastPt(48.569, 17.927, -2.926, clarkcenterthirdfloorelevator.map),
                                       #"floor_B": BlastPt(48.569, 17.927, -2.926, clarkcenterbasementelevator.map),
                                       },
                                      world.types.get_surface("elevator")))
    coffee_pickup = BlastSurface("clark_peets_coffee_shop", 
                                 {"start": BlastPt(56.869, 14.529, 2.511, clarkcenterpeetscoffee.map),
                                  "line_0": BlastPt(58.834, 13.630, -0.697, clarkcenterpeetscoffee.map),
                                  "line_1": BlastPt(58.782, 13.704, 2.412, clarkcenterpeetscoffee.map),
                                  "line_2": BlastPt(57.840, 14.623, 2.497, clarkcenterpeetscoffee.map),
                                  "line_3": BlastPt(57.667, 15.643, 1.731, clarkcenterpeetscoffee.map),
                                  "line_4": BlastPt(57.476, 16.429, 1.731, clarkcenterpeetscoffee.map),
                                  "line_5": BlastPt(58.521, 17.789, 0.816, clarkcenterpeetscoffee.map),
                                  "line_6": BlastPt(58.521, 17.789, -0.687, clarkcenterpeetscoffee.map),
                                  "end": BlastPt(58.521, 17.789, -0.687, clarkcenterpeetscoffee.map),
                                  },
                                 world.types.get_surface("coffee_shop"))
    world.append_surface(coffee_pickup)

    world.add_surface_object("salisbury_table", world.types.get_object("empty_ziplock_1L_bag"), BlastPos(0.6602, 0.3, 0.762, 0.0, 0.0, 0.0))
    
    stair4 = BlastRobot("stair4", 
                        #BlastPt(55.840, 14.504, -0.331, clarkcenterpeetscoffee.map),
                        BlastPt(12.000, 40.957, 0.148, clarkcenterfirstfloor.map),
                        world.types.get_robot("pr2-cupholder"))
    world.append_robot(stair4)

    world.take_action("stair4", "tuck-both-arms", {}) #To debug with arms tucked.
    world.gc_objects()
    return world

def run_test():
    world = make_test_world(".")
    
    #Set at the shop with the initial bag
    initial_bag = BlastObject(world.types.get_object("coffee_money_bag"), None, "stair4.right-arm")
    world.append_object(initial_bag)
    world.robots["stair4"].holders["left-arm"] = BlastObjectRef(initial_bag.uid)
    world.robots["stair4"].location = BlastPt(55.840, 14.504, -0.331, "clarkcenterpeetscoffee")
    
    print "Tuck arms:", world.take_action("stair4", "tuck-both-arms", {})
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    if not world.equal(world):
        return False
    
    res = world.take_action("stair4", "move", {"end": BlastPt(56.869, 14.529, 2.511, "clarkcenterpeetscoffee")})
    print "Action result:", res
    if not res:
        return False
    if not world.equal(world):
        return False
    
    print '-'*130
    print "                                                            MID-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    if not world.equal(world):
        return False
    
    res = world.take_action("stair4", "buy-coffee", {"shop": world.surfaces["clark_peets_coffee_shop"] })
    print "Action result:", res
    if not res:
        return False
    
    
    print '-'*130
    print "                                                            POST-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)


    res = world.take_action("stair4", "stash-cupholder", {}, debug = True)
    print "Action result:", res
    if not res:
        return False

    print '-'*130
    print "                                                            POST-STASH"
    print '-'*130
    print world.to_text()
    print '-'*130
    print world.equal(world)

    res = world.take_action("stair4", "tuck-both-arms", {})
    if not res:
        return False
    print "Tuck arms:", res
    return True

def clone_world_test(world):
    clone = world.copy()
    print '-'*130
    print "                                                            CLONE"
    print '-'*130
    print clone.to_text()
    print '-'*130
    print "Equal", clone.equal(world)
    
    print "Move:", world.enumerate_action("stair4", "move", {})
    print "Coffee:", world.enumerate_action("stair4", "buy-coffee", {})


def elevator_test():
    world = make_test_world(".")
    res = world.take_action("stair4", "tuck-both-arms", {})
    print "Tuck arms:", res
    if not res:
        return False
    world.robots["stair4"].location = world.surfaces["clarkelevator"].locations["floor_1"].copy()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130

    #a = (True, {'elevator': ["clarkelevator"], 
    #            'infloor': (['elevator'], {'clarkelevator': 
    #                                       [Pt(49.218, 19.462, -3.126, "clarkcenterfirstfloorelevator")]}), 
    #            'outfloor': (['elevator'], {'clarkelevator': 
    #                                        [Pt(49.218, 19.462, -3.126, "clarkcenterfirstfloorelevator"), 
    #                                         Pt(48.569, 17.927, -2.926, "clarkcentersecondfloorelevator"), 
    #                                         Pt(48.569, 17.927, -2.926, "clarkcenterthirdfloorelevator")]})})

    r = world.enumerate_action("stair4", "elevator", {})

    print "Elevator:", r

    if r[0] != True: return False
    if type(r[1]) != type({}): return False
    if not "elevator" in r[1]: return False
    if not "infloor" in r[1]: return False
    if not "outfloor" in r[1]: return False

    if len(r[1]["outfloor"][0]) != 1: return False
    if len(r[1]["elevator"]) != 1: return False
    if len(r[1]["infloor"][0]) != 1: return False
    
    if r[1]["outfloor"][0][0] != "elevator": return False
    if str(r[1]["elevator"][0]) != "Surface:clarkelevator": return False
    if r[1]["infloor"][0][0] != "elevator": return False
    
    if len(r[1]["infloor"][1]['clarkelevator']) != 1: return False
    if len(r[1]["outfloor"][1]['clarkelevator']) != 3: return False
    if r[1]["infloor"][1]['clarkelevator'][0].map != "clarkcenterfirstfloorelevator": return False

    return True


def arms_test():
    world = make_test_world(".")
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130

    r = world.take_action("stair4", "tuck-both-arms", {})
    print "Tuck arms:", r
    if not r: return False

    
    print '-'*130
    print "                                                            POST-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    return True
    
def torso_test():
    world = make_test_world(".")
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    
    r = world.take_action("stair4", "torso", {"height": 0.3})
    print "Lift", r
    if not r: return False
    
    
    print '-'*130
    print "                                                            POST-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130


    a = world.enumerate_action("stair4", "torso", {})
    print "Enumerate action:", a
    if a[0] != True: return False
    if not 'height' in a[1]: return False
    if not 0.0 in a[1]['height']: return False
    if not 0.3 in a[1]['height']: return False
    if len(a[1]['height']) != 2: return False

    world.action_robot_pose("stair4", "torso", {})

    return True

def pick_and_place_test():
    world = make_table_top_world(".")
    world.gc_objects()
    
    print '-'*130
    print "                                                            PRE-EXEC"
    print '-'*130
    print world.to_text()
    print '-'*130
    
    print world.enumerate_action("stair4", "table-pick-left", {}) #Fixme untested
    uid = world.surfaces["table_1"].objects[0].uid
    r = world.take_action("stair4", "table-pick-left", 
                          {"table": "table_1", 
                           "object": "BlastObjectRef(" + str(uid) + ")"}, 
                          debug=True)
    if not r: return False
    print r

    print '-'*130
    print "                                                            POST-PICK"
    print '-'*130
    print world.to_text()
    print '-'*130
    
    print world.enumerate_action("stair4", "table-place-left", {}) #Fixme: untested
    r = world.take_action("stair4", "table-place-left",
                          {"table": "table_1", "position": 
                           "table_1, Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)"},
                          debug=True)
    if not r: return False
    print r
    
    
    print '-'*130
    print "                                                            POST-PLACE"
    print '-'*130
    print world.to_text()
    print '-'*130
    r = world.take_action("stair4", "table-coffee-scan", {"table": "table_1"}, debug=True)
    if not r: return False
    print r
    
    print '-'*130
    print "                                                            POST-SCAN"
    print '-'*130
    print world.to_text()
    print '-'*130


    sc = world.get_scan_actions()
    print "Scan actions", sc, "must include coffee_cup, empty_ziplock_1L_bag"
    if not "coffee_cup" in sc: return False
    if not "empty_ziplock_1L_bag" in sc: return False
    return True

if __name__ == '__main__':
    #torso_test()
    run_test()
    #elevator_test()
    #arms_test()
    #pick_and_place_test()
