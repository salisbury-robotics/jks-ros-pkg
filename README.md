This repository contains code from the Salisbury Robotics lab at 
Stanford University. See http://sr.stanford.edu for full details 
on our Lab.

# Contents

## BLAST

BLAST is a BSD-Licensed system that provides an extra layer on top
of ROS. The technology behind BLAST was developed on the basis of 
lessons learned from the PR2 coffee purchasing project. The system
contains a web interface, a task-level planner, and a ROS bridge
as well as an object model. The system is still under development,
so it does not really work in practice.

You can run it by typing: ``python blast_api_test.py --sim`` in 
``blast/blast_test``. Once you have the system running, browse
``http://localhost:5000/`` to see the web interface. You can click
on a map (choose clarkcenterfirstfloor) to open the top down view.
Choose "follow robot" to see the robot. To try some actions, try
clicking "Plan Action" and then selecting "coffee-run" to plan the
coffee run. Note this will take a long time because the planner is 
currently very slow.

## Bosch Arm

This contains a great deal of code for an old robot arm. If one 
comes into possesion of this system, then it may be useful. It is
unlikely to be used otherwise.

## PR2 Coffee

This codebase contains code for the PR2 Coffee purchasing project,
which is a project in which PR2 was programmed to buy us coffee from
a nearby coffee shop. A paper about the project and video of the 
entire extravaganza can be found at: 
http://http://web.stanford.edu/group/sailsbury_robotx/cgi-bin/salisbury_lab/?page_id=793

There is a non-trivial amount of code here and much of it is contrived
for the specific scenarios found in the video. As a result it is unlikely
to be useful except as a recipe book for other projects. However, the 
components individualy work and can potentially be re-used, despite the
admittedly poor documentation.

### Multi-Map Navigation

The most reusable of the components by far. This system allows a PR2 (or
other ROS Robot) to navigate across multiple 2D maps. This approach allows
the system to handle multi-story buildings or to have maps too large to
fit in memory at once. The maps are loaded into the ROS map store and then
accessed by choosing the starting map (choosen by the parameter server). The
system the links the maps with "transitions" which are specified by a YAML
file. Goals are specified as a map name + location, and the system does a
Dikjstra's algorithm to find the shortest path through the maps. The system
shells out to move_base to do the on-map navigation, and then potentially
executes a transistion action to navigate through the transition between
the maps. See ``stage_multi_map_navigation`` and ``multi_map_navigation_pr2``
for examples, with the stage system having working launch files.

### RobotPass

Probably the second most reusable system here. RobotPass allows a PR2 to give
and recieve objects from humans. This is used in the coffee project to grasp
the money and the coffee itself. To run it, you first have to launch the gripper
sensor actions:
``<include file="$(find pr2_gripper_sensor_action)/launch/pr2_gripper_sensor_actions.launch" />``
because it uses the gripper accelerometers. To run it, you can launch 
``python robotpass_server.py --test --use-left`` to do a loop of asking
for an object and giving it back. This quickly becomes annoying due to the robot's
TTS, so you can mute the robot by running ``./mute.sh`` and unmute with ``./sound.sh``.
``./say.sh "I am a string, look at me`` executes TTS, and can be used as an example
of easy TTS on PR2.

Important note: the cupholder functions try to place cups on the side of the robot just
above the arm (see video for detail). It is possible to mount a cupholder here (potentially
on both sides of the robot) and store cups. Using this without a cupholder mod will simply
lead to object droppage.

### Elevator

Does not even compile. Is actually taken from the STAIR4 Project, written by Ellen Klingbeil.
Temporarily planned to get it working on PR2 but wanted to write my own elevator system.

### PR2_Specific

Deeply integrated stuff for our specific application. Door_blast is some code that pushes
open the door from the inside, after a laser-scan three axis line-up procedure. Door_drag
pulls the door open from the outside using mechanical alignment. Elevator does the whole
elevator management process. Coffee_grab is a meta script to run all the actions in sequence
for the coffee run task. Also includes code for waiting in line at the coffee shop.

## Sixense Razer Hydra

Old drivers for the razer hydra system. They are mainly here for historical reasons. Instead
use the new, improved system: https://github.com/aleeper/razer_hydra

## Teleop and Haptics

Work with using teleop and haptics in ROS and for the PR2. More updated code can be found at:
https://github.com/aleeper/ros_haptics

## 3RD Party Libraries

Various libraries used to make the other components work. Subject
to their own licensing.
