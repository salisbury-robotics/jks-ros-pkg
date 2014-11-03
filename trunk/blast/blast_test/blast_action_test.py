import blast
from blast_action import *

def test_main():
    import blast_world_test
    man = BlastManager(["test_actions",], blast_world_test.make_test_world())
    prog_id = man.plan_action("stair4", "coffee-run", {"person_location": 
                                                       blast_world.BlastPt(17.460, 38.323, -2.330,
                                                                           "clarkcenterfirstfloor"), 
                                                       "shop": "clark_peets_coffee_shop"}, {})
    if prog_id == None:
        print "Failed to plan", prog_id
        man.stop()
        return False
    if man.wait_for_program(prog_id) == False:
        print "Action execution failed"
        man.stop()
        return False
    man.stop()
    a = "0a74cbe0c1cd7804120143ffe687a63bf8baec2f00d11e47583d075b7b3fc0350a5bcf3be0a1e4d6977904"
    a = a + "dde7e04531ac7662d74ade2749085a3deb79df8e417fa7d2bd93a92d744e22058b29c0012b2f282516"
    if a != man.world.world.get_hex_hash_state():
        return False
    return True

def test_place():
    import blast_world_test
    w = open("table_1_objects.txt", "w")
    w.close()
    w = open("table_2_objects.txt", "w")
    w.write("coffee_cup\n")
    w.close()

    man = BlastManager(["test_actions",], blast_world_test.make_table_top_world())

    print "-"*120
    print " "*60, "BEFORE PICK"
    print man.world.world.to_text()
    print "-"*120

    if not man.plan_hunt("stair4", "cupholder", "coffee_cup"):
        return False

    print "-"*120
    print " "*60, "AFTER PICK"
    print man.world.world.to_text()
    print "-"*120

    #man.plan_action("stair4", "unstash-cupholder", {})
    object_uid = man.world.world.robots["stair4"].holders["cupholder"].uid
    print "Hold onto the uid of", object_uid
    r = man.plan_action("stair4", "table-place-left", {"table": "table_1", 
                                                       "position": ("table_1", "Pos(0.6602, 0.10398, 0.762, 0.0, 0.0, 0.0)")},
                        {"robot-holders": {"stair4": {"left-arm": object_uid}}})
    if not r:
        return False

    print "-"*120
    print " "*60, "AFTER PLACE"
    print man.world.world.to_text()
    print "-"*120

    a = "937e356e0aebbd33554a3380547260698f20b9ca7dc2c6f23361598c994f85648b069fa200933a1f2a7c8f352"
    a = a + "d13d8ca3db1b0a1dc50f28e238cd4177368f587817290159b54892eb02743b464ca71a606d13a1d"
    if a != man.world.world.get_hex_hash_state():
        print "Invalid state, failed test"
        return False
    return True

def test_multi_robot():
    import blast_world_test
    man = BlastManager(["test_actions",], blast_world_test.make_table_top_world(False))


    w = open("table_1_objects.txt", "w")
    w.close()
    w = open("table_2_objects.txt", "w")
    w.write("coffee_cup\n")
    w.write("coffee_cup\n")
    w.close()

    stair5 = blast_world.BlastRobot("stair5", 
                                    blast_world.BlastPt(10.000, 40.957, 0.148, "clarkcenterfirstfloor"),
                                    man.world.world.types.get_robot("pr2-cupholder"))
    man.world.world.append_robot(stair5)
    man.world.world.take_action("stair5", "tuck-both-arms", {}) #To debug with arms tucked.

    print man.world.world.to_text()

    if not man.plan_hunt("stair4", "cupholder", "coffee_cup"):
        return False


    #print man.world.world.to_text()

    #if not man.plan_hunt("stair5", "cupholder", "coffee_cup"):
    #    return False

    print man.world.world.to_text()

    return True


if __name__ == '__main__':
    print test_main()
    #print test_place()
    #print test_multi_robot()

