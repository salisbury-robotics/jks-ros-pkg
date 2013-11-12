import json, mimetypes, Image, StringIO, uuid, time, threading
from flask import Flask, request, session, g, redirect, url_for, abort, render_template, flash, Response
import blast_action, blast_world

DEBUG = True
SECRET_KEY = 'flask_dev_key'


manager = blast_action.BlastManager(["test_world"], blast_world.make_test_world())

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
        

fs = BlastFs("", ["maps/"])

def return_json(js):
    if type(js) != type(""): js = json.dumps(js)
    return Response(response=js, status=200, mimetype="application/json")



login_sessions = {}
login_sessions_lock = threading.Lock()

def get_world(world):
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


login_sessions = {}


permissions = ["view", "edit", "plan"]

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
    return False

def permission_error(request, perm):
    return "You need permission:", perm



app = Flask(__name__)
app.config.from_object(__name__)
app.secret_key = 'A0Zr98j/3yX R~XHH!jmN]LWX/,?RT'

@app.route('/session', methods=["PUT", "GET"])
def session_manage():
    global login_sessions, login_sessions_lock
    login_sessions_lock.acquire()
    if request.method == "PUT":
        sid = str(time.time()) + "-" + str(uuid.uuid4())
        login_sessions[sid] = {"sid": sid, "last_update": time.time(),
                               "worlds": {None: manager.world, }}
        session["sid"] = sid
        login_sessions_lock.release()
        return "true"
    elif request.method == "GET":
        if not "sid" in session: 
            login_sessions_lock.release()
            return "false"
        if not str(session["sid"]) in login_sessions: 
            login_sessions_lock.release()
            return "false"
        login_sessions_lock.release()
        return "true"
    else:
        login_sessions_lock.release()
        return "false"
    
    

@app.route('/')
def show_main_page():
    f = open("static/index.html", "r")
    r = f.read()
    f.close()
    return r

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

@app.route('/world/<world>/')
@app.route('/robot')
@app.route('/world/<world>/robot/<robot>')
@app.route('/robot/<robot>')
def api_robot(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    if robot:
        return return_json(get_world(world).get_robot(robot))
    return return_json(get_world(world).world.robots_keysort)

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
    return return_json(get_world(world).set_robot_location(robot, json.loads(request.data)))


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
    ot = json.loads(request.data)    
    pre = True
    if type(ot) != type("") and type(ot) != type(u""):
        pre = ot["require_pre_existing_object"]
        ot = ot["object_type"]
    return return_json(get_world(world).set_robot_holder(robot, holder, ot, pre))

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
        if not position in get_world(world).get_robot(robot)["positions"]: return_json(False)
        set_data = json.loads(request.data)
        return return_json(get_world(world).set_robot_position(robot, position, set_data))

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


app.run(debug=DEBUG)








