

import blast_action, blast_world_test, blast_planner

test_list = [('BLAST ACTION TEST MAIN', blast_action.test_main),
             ('BLAST ACTION TEST PLACE', blast_action.test_place),
             ('BLAST WORLD TEST RUN TEST', blast_world_test.run_test),
             ('BLAST WORLD TEST ELEVATOR TEST', blast_world_test.elevator_test),
             ('BLAST WORLD TEST ARMS TEST', blast_world_test.arms_test),
             ('BLAST WORLD TEST TORSO TEST', blast_world_test.torso_test),
             ('BLAST WORLD TEST PICK AND PLACE TEST', blast_world_test.pick_and_place_test),
             ('BLAST PLANNER COFFEE HUNT TEST', blast_planner.coffee_hunt_test),
             ('BLAST PLANNER RUN TEST', blast_planner.run_test),
             ]

test_list_c = test_list
fail = False
while test_list_c != []:
    name, test = test_list_c[0]
    test_list_c = test_list_c[1:]
    print name
    r = None
    try:
        r = test()
    except:
        print "Exception in test:"
        import traceback
        traceback.print_exc()
        r = None
    if r:
        print name, "SUCCEED"
    else:
        print name, "FAIL"
        fail = True
        break

while test_list_c != []:
    name, test = test_list_c[0]
    test_list_c = test_list_c[1:]
    print name, "UNTESTED"

if fail:
    print "A test failed"
else:
    print "All tests passed"
