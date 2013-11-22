import json, mimetypes, Image, StringIO, uuid, time, threading
from flask import Flask, request, session, g, redirect, url_for, abort, render_template, flash, Response
import blast_action, blast_world

DEBUG = True
SECRET_KEY = 'flask_dev_key'

SESSION_TIMEOUT = 10.0


manager = blast_action.BlastManager(["test_world"], blast_world.make_test_world())
world_edit_lock = threading.Lock()
world_edit_session = None



class BlastFs(object):
    def __init__(self, root = "", first_filters = None):
        self.root = root
        self.first_filters = first_filters
    def get_file_name(self, name):
        if self.first_filters != None:
            for f in self.first_filters:
                if name.find(f) == 0:
                    return self.root + name
            return None            
        return self.root + name
    def get_mime_type(self, name):
        fs = self.get_file_name(name)
        if fs == None: return None
        return mimetypes.guess_type(fs)
    def get_file(self, name):
        fs = self.get_file_name(name)
        if fs == None: return None
        return open(self.get_file_name(name), "r")
        

fs = BlastFs("", ["maps/", "robot_fs/"])

def return_json(js):
    if type(js) != type(""): js = json.dumps(js)
    return Response(response=js, status=200, mimetype="application/json")



login_sessions = {}
login_sessions_lock = threading.Lock()

def get_world(world):
    print world
    global login_sessions, login_sessions_lock
    login_sessions_lock.acquire()
    if not "sid" in session: 
        login_sessions_lock.release()
        return False
    if not str(session["sid"]) in login_sessions: 
        login_sessions_lock.release()
        return False
    w = login_sessions[str(session["sid"])]["worlds"].get(world)
    login_sessions_lock.release()
    return w

def lock_world(world):
    global login_sessions, login_sessions_lock, world_edit_session, world_edit_lock
    if world != None:
        return True
    world_edit_lock.acquire()
    if world_edit_session != str(session["sid"]):
        world_edit_lock.release()
        return False
    return True

def unlock_world(world):
    global login_sessions, login_sessions_lock, world_edit_session, world_edit_lock
    if world != None:
        return True
    world_edit_lock.release()
    return True

def lock_world_error(world):
    return "World failed to lock"


login_sessions = {}


permissions = ["view", "edit", "plan"]



def clean_sessions():
    global login_sessions, login_sessions_lock, world_edit_session
    bad_sessions = []
    login_sessions_lock.acquire()
    clean_time = time.time()
    for sid in login_sessions:
        if login_sessions[sid]["last_update"] + SESSION_TIMEOUT < clean_time:
            print "Kill session", sid
            if world_edit_session == sid:
                world_edit_lock.acquire()
                if world_edit_session == sid:
                    world_edit_session = None
                world_edit_lock.release()
            bad_sessions.append(sid)

    for sid in bad_sessions:
        del login_sessions[sid]

    login_sessions_lock.release()


def get_user_permission(session, perm):
    global login_sessions, login_sessions_lock
    login_sessions_lock.acquire()
    if not "sid" in session: 
        login_sessions_lock.release()
        return False
    if not str(session["sid"]) in login_sessions: 
        login_sessions_lock.release()
        return False
    session_d = login_sessions[str(session["sid"])]
    session_d["last_update"] = time.time()
    if perm in permissions:
        login_sessions_lock.release()
        return True
    login_sessions_lock.release()
    clean_sessions()
    return False

def permission_error(request, perm):
    return "You need permission:", perm



app = Flask(__name__)
app.config.from_object(__name__)
app.secret_key = 'A0Zr98j/3yX R~XHH!jmN]LWX/,?RT'

@app.route('/session', methods=["PUT", "GET"])
def session_manage():
    global login_sessions, login_sessions_lock, world_edit_session
    if request.method == "PUT":
        login_sessions_lock.acquire()
        sid = str(time.time()) + "-" + str(uuid.uuid4())
        login_sessions[sid] = {"sid": sid, "last_update": time.time(),
                               "worlds": {None: manager.world, }}
        session["sid"] = sid
        login_sessions_lock.release()
        return "true"
    elif request.method == "GET":
        login_sessions_lock.acquire()
        if not "sid" in session: 
            login_sessions_lock.release()
            return "false"
        if not str(session["sid"]) in login_sessions: 
            login_sessions_lock.release()
            return "false"
        login_sessions[str(session["sid"])]["last_update"] = time.time()
        login_sessions_lock.release()
        clean_sessions()
        ew = world_edit_session #Copy to avoid race conditions so no locking
        return return_json({"sessions": len(login_sessions), "edit_world": ew != None,
                            "edit_other_session": ew != str(session["sid"]) })
    else:
        return "false"
    
    

@app.route('/')
def show_main_page():
    f = open("static/index.html", "r")
    r = f.read()
    f.close()
    return r.replace("${SESSION_TIMEOUT}", str(SESSION_TIMEOUT))

@app.route('/world/<world>/surface')
@app.route('/surface')
@app.route('/world/<world>/surface/<surface>')
@app.route('/surface/<surface>')
def api_surface(world = None, surface = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    if surface:
        return return_json(get_world(world).get_surface(surface))
    return return_json(get_world(world).world.surfaces_keysort)

@app.route('/world/<world>/map')
@app.route('/map')
@app.route('/world/<world>/map/<map>')
@app.route('/map/<map>')
def api_map(world = None, map = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    if map:
        return return_json(get_world(world).get_map(map))
    return return_json(get_world(world).world.maps_keysort)

@app.route('/robot')
@app.route('/world/<world>/robot/<robot>')
@app.route('/world/<world>/robot')
@app.route('/robot/<robot>')
def api_robot(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    if robot:
        rd = get_world(world).get_robot(robot)
        if request.args.get("include_type", "false") == "true":
            rd["robot_type"] = get_world(world).world.types.robots.get(rd["robot_type"]).to_dict()
        return return_json(rd)

    if request.args.get("map", False) == False:
        robots = get_world(world).world.robots_keysort
    else:
        robots = []
        for robot in get_world(world).world.robots_keysort:
            if get_world(world).world.robots[robot].location.map == request.args["map"]:
                robots.append(robot)
    return return_json(robots)

@app.route('/fs/<path:filename>')
def api_fs(filename):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    mime, encoding = fs.get_mime_type(filename)
    f = fs.get_file(filename)
    if f == None: return ""
    r = f.read()
    f.close()
    resp = Response(response=r, status=200, mimetype=mime)
    return resp

@app.route('/fspng/<path:filename>')
def api_fspng(filename, imformat="PNG"):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    f = fs.get_file(filename)
    if f == None: return ""
    output = StringIO.StringIO()
    im = Image.open(f)
    im.save(output, format=imformat)
    r = output.getvalue()
    output.close()
    f.close()
    resp = Response(response=r, status=200, mimetype="image/png")
    return resp


@app.route('/robot/<robot>/location', methods=["GET", "POST"])
@app.route('/world/<world>/robot/<robot>/location', methods=["GET", "POST"])
def api_robot_location(world = None, robot = None):
    if request.method == "GET": perm = "view"
    elif request.method == "POST" and world != None: perm = "plan"
    elif request.method == "POST" and world == None: perm = "edit"
    else: return permission_error(request, "INVALID")
    if not get_user_permission(session, perm): return permission_error(request, perm)
    if request.method == "GET":
        robot = get_world(world).get_robot(robot)
        if robot != None: robot = robot["location"]
        return return_json(robot)
    
    if not lock_world(world):
        return lock_world_error(world)
    r = return_json(get_world(world).set_robot_location(robot, json.loads(request.data)))
    unlock_world(world)
    return r


@app.route('/robot/<robot>/holder', methods=["GET"])
@app.route('/robot/<robot>/holder/<holder>', methods=["GET", "POST"])
def api_robot_holder(world = None, robot = None, holder = None):
    if request.method == "GET": perm = "view"
    elif request.method == "POST" and world != None: perm = "plan"
    elif request.method == "POST" and world == None: perm = "edit"
    else: return permission_error(request, "INVALID")
    if not get_user_permission(session, perm): return permission_error(request, perm)
    if request.method == "GET":
        robot = get_world(world).get_robot(robot)
        if robot != None and holder == None: robot = sorted(robot["holders"].keys())
        if holder != None and robot != None: 
            robot = robot["holders"].get(holder, None)
            if robot != None: robot = get_world(world).world.objects.get(robot.uid, None)
            if robot != None: robot = robot.object_type.name
        return return_json(robot)
    
    if not lock_world(world):
        return lock_world_error(world)
    ot = json.loads(request.data)    
    pre = True
    if type(ot) != type("") and type(ot) != type(u""):
        pre = ot["require_pre_existing_object"]
        ot = ot["object_type"]
    
    r = return_json(get_world(world).set_robot_holder(robot, holder, ot, pre))
    unlock_world(world)
    return r

@app.route('/world/<world>/robot/<robot>/position', methods=["GET"])
@app.route('/world/<world>/robot/<robot>/position/<position>', methods=["GET", "POST"])
@app.route('/robot/<robot>/position', methods=["GET"])
@app.route('/robot/<robot>/position/<position>', methods=["GET", "POST"])
def api_robot_position(world = None, robot = None, position = None):
    if request.method == "GET": perm = "view"
    elif request.method == "POST" and world != None: perm = "plan"
    elif request.method == "POST" and world == None: perm = "edit"
    else: return permission_error(request, "INVALID")
    if not get_user_permission(session, perm): return permission_error(request, perm)
    if request.method == "GET":
        robot = get_world(world).get_robot(robot)
        if robot != None and position == None: robot = sorted(robot["positions"].keys())
        if robot != None and position != None: robot = robot["positions"].get(position, None)
        return return_json(robot)
    else:
        if not lock_world(world):
            return lock_world_error(world)
        if not position in get_world(world).get_robot(robot)["positions"]: return_json(False)
        set_data = json.loads(request.data)
        r = return_json(get_world(world).set_robot_position(robot, position, set_data))
        unlock_world(world)
        return r

@app.route('/robot/<robot>/type', methods=["GET"])
def api_robot_type(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    robot = get_world(world).get_robot(robot)
    if robot != None: robot = get_world(world).world.types.robots.get(robot["robot_type"], None)
    if robot != None: robot = robot.to_dict()
    return return_json(robot)

@app.route('/robot_type/<robot>', methods=["GET"])
def api_robot_type(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    robot = get_world(world).world.types.robots.get(robot, None)
    if robot != None: robot = robot.to_dict()
    return return_json(robot)


@app.route('/edit_world', methods=["GET", "PUT"])
def edit_world(world = None, robot = None):
    global world_edit_session
    if not get_user_permission(session, "edit"): return permission_error(request, "edit")
    if request.method == "GET":
        ew = edit_world_session
        return return_json(rw == str(session["sid"]))
    elif request.method == "PUT":
        state = json.loads(request.data)
        world_edit_lock.acquire()
        if world_edit_session == None and state == True:
            world_edit_session = str(session["sid"])
            world_edit_lock.release()
            return return_json(True)
        elif world_edit_session == str(session["sid"]) and state == False:
            world_edit_session = None
            world_edit_lock.release()
            return return_json(True)
        world_edit_lock.release()
        return return_json(False)
    return return_json(False)

@app.route('/world', methods=["GET", "PUT"])
def api_world(world = None):
    global login_sessions, login_sessions_lock
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    wc = get_world(world)
    login_sessions_lock.acquire()
    if request.method == "GET":
        r = return_json(login_sessions[str(session["sid"])]["worlds"].keys())
    elif request.method == "PUT": 
        world_name = json.loads(request.data)
        manager.worlds["api_" + str(session["sid"]) + "_" + world_name] = wc.copy()
        login_sessions[str(session["sid"])]["worlds"][world_name] = \
            manager.worlds["api_" + str(session["sid"]) + "_" + world_name]
        r = "true"
    else:
        r = "ERRROR"
    login_sessions_lock.release()
    return r


@app.route('/world/<world>/plan/<target>', methods=["GET", "PUT", "DELETE"])
@app.route('/plan/<target>', methods=["GET", "PUT", "DELETE"])
def api_plan(target, world = None):
    global login_sessions, login_sessions_lock
    if request.method == "PUT" or request.method == "DELETE": 
        if not get_user_permission(session, "plan"): return permission_error(request, "plan")
    else:
        if not get_user_permission(session, "view"): return permission_error(request, "view")

    world_edit_lock.acquire()

    if world_edit_session != None:
        world_edit_lock.release()
        return "false"

    start = get_world(world)
    end = get_world(target)
    if start == None or end == None:
        world_edit_lock.release()
        return "null"


    if request.method == "DELETE":
        print request.args.get("start", "FAIL")
        try:
            start_i = int(request.args.get("start", "0"))
        except:
            start_i = 0
        if start_i < 0: start_i = 0
        start.plan_steps[end] = start.plan_steps[end][0:start_i]


    start_clone = start.copy()
    start_clone.real_world = False

    if not end in start.plan_steps:
        start.plan_steps[end] = []

    for i in start.plan_steps[end]:
        start_clone.take_action(i[0], i[1], i[2])


    worked = None
    equal = False
    if request.method == "PUT":     
        locations = []
        for name, robot in end.world.robots.iteritems():
            locations.append(robot.location.copy())


        plan_to_world, time, steps = start_clone.plan(lambda w: w.equal(end.world), {"Pt": locations}, plan_and_return = True)

        if plan_to_world != None:
            worked = True
            start.plan_steps[end].extend(steps)
            equal = True
        else:
            worked = False
            equal = False
    else:
        if request.method == "DELETE": worked = True
        equal = start_clone.world.equal(end.world)
    
    if end in start.plan_steps:
        p = [x for x in start.plan_steps[end]]

        if True:
            actions = {}
            for robot, action, params in p:
                if not robot in actions:
                    actions[robot] = {}
                if not action in actions[robot]:
                    rt, at = start.world.types.get_action_for_robot(start.world.robots[robot].robot_type, action)
                    actions[robot] = at.display
        else:
            actions = None
    else:
        actions = None
        p = None
    world_edit_lock.release()

    return return_json([p, equal, worked, actions])


app.run(debug=DEBUG)








