
import sys, os
my_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_path + "/../blast_client")

import blast_network_bridge

exec_path = my_path + "/../blast_client/blast_action_exec.py"
actions_path = my_path + "/test_actions/"

def action_start(action_robot_type, action_name, action_id, parameters, write_callback, capability_write):
    global actions_path, exec_path
    py_file = actions_path + action_robot_type + "__" + action_name + ".py"
    cmd = ['python', exec_path, py_file, parameters]
    exc = blast_network_bridge.ActionExecutor(cmd, action_id, write_callback, capability_write)
    return exc.get_callback()

def install_action(robot_type, action_name):
    global actions_path
    if not os.path.isfile(actions_path + robot_type + "__" + action_name + ".py"):
        print "Installing", robot_type, action_name, "(test - does nothing)"
        return False
    return True

def capability_cb(cap, fn, param):
    print "Capability!!!!", cap, fn, param
    return "null"

map_store = blast_network_bridge.MapStore(my_path + "/maps_client/")
bnb = blast_network_bridge.BlastNetworkBridge("localhost", 8080, "stair4", "pr2-cupholder",
                                              action_start, install_action, capability_cb, map_store)
bnb.start()
bnb.wait()
bnb.stop()
if bnb.error:
    print "The system terminated with error", bnb.error
