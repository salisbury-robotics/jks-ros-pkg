#--------------------------------------------------------------------
#Copyright (c) 2015
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without 
#modification, are permitted provided that the following conditions 
#are met:
#  1. Redistributions of source code must retain the above copyright 
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above 
#     copyright notice, this list of conditions and the following 
#     disclaimer in the documentation and/or other materials 
#     provided with the distribution.
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
#COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
#INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
#HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
#ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#--------------------------------------------------------------------

import blast
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
