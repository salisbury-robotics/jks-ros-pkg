
import sys, os, threading, time, math, socket, random
my_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(my_path + "/../blast_client")

import blast_network_bridge, blast_action_exec

exec_path = my_path + "/../blast_client/blast_action_exec.py"
actions_path = my_path + "/test_actions/"


def install_action(robot_type, action_name):
    global actions_path
    if not os.path.isfile(actions_path + robot_type + "__" + action_name + ".py"):
        print "Installing", robot_type, action_name, "(test - does nothing)"
        return False
    return True

ATL = 0.0001
state_lock = threading.Lock()
robot_location = None
l_arm_state = [ 0.06024,  1.248526,   1.789070,  -1.683386, -1.7343417, -0.0962141, -0.0864407, 0]
r_arm_state = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175,    -1.8417,     0.21436,   0]
torso_state = [0.0]
head_state = [0.0, 0.0]
force_forwards = 0
force_strafe = 0
force_turn = 0
do_simulation = True
last_sent = None
definitely_sent = None #State definitely sent to server.

def safe_c(x):
    if type(x) == dict:
        return x.copy()
    if type(x) == list:
        return [a for a in x]
    return x

#TODO: get rid of loops
def capability_cb(cap, fn, param):
    global definitely_sent, last_sent, l_arm_state, r_arm_state, torso_state, head_state, robot_location, force_strafe, force_forwards, force_turn, do_simulation
    #print "Capability!!!!", cap, fn, param
    if fn == "START" or fn == "STOP":
        if cap == "driver" and fn == "STOP":
            state_lock.acquire()
            force_forwards = 0
            force_strafe = 0
            force_turn = 0
            state_lock.release()
        print "Start/stop", cap
        return None
    elif cap == "simulator" and fn == "SIMULATE":
        #force_forwards = 0
        #   force_strafe = 0
        state_lock.acquire()
        if do_simulation and robot_location:
            if type(robot_location) == dict:
                robot_location = blast_action_exec.BlastLocation(jsond = robot_location)
            robot_location = robot_location.rotate(force_turn * 0.1).move(force_forwards * 0.1, force_strafe * 0.1)
            robot_location = robot_location.to_dict()
        state_lock.release()
    elif cap == "driver" and fn == "FORCE":
        state_lock.acquire()
        force_forwards = param.get("forward", 0)
        force_strafe = param.get("strafe", 0)
        force_turn = param.get("turn", 0)
        state_lock.release()
    elif cap == "amcl_set_location" and (fn == "VALUE" or fn == "WAIT_VALUE"):
        if type(param) != dict:
            raise Exception("Parameter is not correct in type")
            return None
        if "x" not in param or "y" not in param or "map" not in param or "a" not in param:
            raise Exception("Parameter is not correct in type")
            return None
        robot_location = param
        while True and fn == "WAIT_VALUE":
            time.sleep(0.01)
            state_lock.acquire()
            ds = definitely_sent
            state_lock.release()
            if type(ds) != dict: continue
            if "robot_loc" not in ds: continue
            if type(ds["robot_loc"]) != dict: continue
            if float(ds["robot_loc"]["x"] - param["x"]) > 0.0001: continue
            if float(ds["robot_loc"]["y"] - param["y"]) > 0.0001: continue
            if float(ds["robot_loc"]["a"] - param["a"]) > 0.0001: continue
            if ds["robot_loc"]["map"] != param["map"]: continue
            break
    elif cap == "robot_pub" and fn == "GETSTATE":
        state_lock.acquire()
        definitely_sent = last_sent
        last_sent = {"robot_loc": safe_c(robot_location),
                     "right-arm": safe_c(r_arm_state),
                     "left-arm": safe_c(l_arm_state),
                     "torso": safe_c(torso_state),
                     "head": safe_c(head_state),
                     }
        state_lock.release()
        return last_sent
    elif cap == "driver" and fn == "WAIT_TORSO":
        if type(param) != list:
            raise Exception("Invalid parameter: " + str(param))
        elif len(param) != 1:
            raise Exception("Invalid parameter: " + str(param))
        elif type(param[0]) != float:
            raise Exception("Invalid parameter: " + str(param))
        state_lock.acquire()
        torso_state = safe_c(param)
        state_lock.release()
        while True:
            time.sleep(0.01)
            state_lock.acquire()
            ds = definitely_sent
            state_lock.release()
            #print "Compare to", ds
            if type(ds) != dict: continue
            if "torso" not in ds: continue
            if type(ds["torso"]) != list: continue
            print param[0], ds["torso"][0], param, ds["torso"], abs(ds["torso"][0] - param[0])
            if abs(ds["torso"][0] - param[0]) > 0.0001: continue
            break
    elif (cap == "tuck_arms" and fn == "TUCK_WAIT") or (cap == "driver" and fn == "WAIT_LEFT_ARM"):
        state_lock.acquire()
        ls, rs, gs = [], [], []
        if fn == "TUCK_WAIT":
            ls = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407, None]
            rs = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436, None]
            gs = ls + rs
        elif fn == "WAIT_LEFT_ARM":
            ls = [x for x in param]
            for i in xrange(0, len(ls)):
                if ls[i] == False or ls[i] == None:
                    ls[i] = l_arm_state[i]
            rs = r_arm_state
            gs = param
        l_arm_state = safe_c(ls)
        r_arm_state = safe_c(rs)
        state_lock.release()
        while True:
            time.sleep(0.01)
            state_lock.acquire()
            ds = definitely_sent
            state_lock.release()
            if type(ds) != dict: continue
            if "right-arm" not in ds: continue
            if "left-arm" not in ds: continue
            if type(ds["right-arm"]) != list: continue
            if type(ds["left-arm"]) != list: continue
            failed_match = False
            if fn == "TUCK_WAIT":
                dc = ds["left-arm"] + ds["right-arm"]
            else:
                dc = ds["left-arm"]
            for s, g in zip(dc, gs):
                if g != None and g != False:
                    if abs(s - g) >= ATL:
                        failed_match = True
                        break
            if not failed_match:
                break
    elif (cap == "driver" and fn == "WAIT_ABSOLUTE_LOCATION") or (cap == "move_base" and fn == "DRIVE"):
        state_lock.acquire()
        do_simulation = False
        start_l = robot_location.copy()
        state_lock.release()
        mid = blast_action_exec.BlastLocation(start_l["x"], start_l["y"], start_l["a"], start_l["map"])
        print "Start driver", mid, start_l
        turn_speed = 0.1
        move_speed = 0.1
        step_size = 0.1
        def wrap_angle(a):
            while a > +math.pi:
                a -= math.pi*2
            while a <= -math.pi:
                a += math.pi*2
            return a
        def clamp(a, mi, ma):
            if a < mi: return mi
            if a > ma: return ma
            return a
        def clamp_turn(a):
            return clamp(a, -turn_speed, turn_speed)
        def set_small(a, b):
            if abs(a - b) < 0.00001:
                return b
            return a
        
        while True:
            #print "Update robot to", mid.to_dict(), robot_location, param
            if mid.x == param["x"] and mid.y == param["y"]:
                if mid.a == param["a"]:
                    break
                else:
                    mid = mid.rotate(clamp_turn(wrap_angle(param["a"] - mid.a)))
                    mid = mid.rotateTo(set_small(mid.a, param["a"]))
            else:
                angle_to_target = wrap_angle(math.atan2(param["y"] - mid.y, param["x"] - mid.x))
                mid = mid.rotateTo(set_small(mid.a, angle_to_target))
                #print mid.a, "->", angle_to_target, clamp_turn(wrap_angle(angle_to_target - mid.a))
                if mid.a != angle_to_target:
                    mid = mid.rotate(clamp_turn(wrap_angle(angle_to_target - mid.a)))
                    mid = mid.rotateTo(set_small(mid.a, angle_to_target))
                else:
                    d = math.sqrt((mid.x - param["x"])**2 + (mid.y - param["y"])**2)
                    if d <= move_speed:
                        mid = mid.moveTo(param["x"], param["y"])
                    else:
                        mid = mid.move(move_speed)
            state_lock.acquire()
            robot_location = mid.to_dict()
            state_lock.release()
            time.sleep(0.1)

        print "Done move"
        while True:
            state_lock.acquire()
            robot_location = param.copy()
            state_lock.release()
            
            state_lock.acquire()
            ds = definitely_sent
            state_lock.release()
            if type(ds) != dict: continue
            if not "robot_loc" in ds: continue
            rl = ds["robot_loc"]
            if type(rl) != dict: continue
            rl = rl.copy()
            if rl["a"] == param["a"] and rl["x"] == param["x"] and rl["y"] == param["y"]:
                break
        print "Really done"
        do_simulation = True
    else:
        print "Invalid function", cap, fn
        raise Exception("We have executed an invalid capability: " + str(cap) + " " + str(fn))
    return None

map_store = blast_network_bridge.MapStore(my_path + "/maps_client/")
action_store = blast_network_bridge.ActionStore(my_path + "/actions_client/")
bnb = blast_network_bridge.BlastNetworkBridge("localhost", 8080, "stair4", "pr2-cupholder",
                                              install_action, capability_cb, map_store, action_store)
bnb.start()
bnb.wait()
bnb.stop()
if bnb.error:
    print "The system terminated with error", bnb.error
