import json, mimetypes, Image, StringIO, uuid, time, threading, sys, os
from flask import Flask, request, session, g, redirect, url_for, abort, render_template, flash, Response
import blast_action, blast_world
import trace_dumper

DEBUG = True
SECRET_KEY = 'flask_dev_key'

SESSION_TIMEOUT = 10.0


manager = None

trace_dumper.trace_start("trace.html")


class BlastFs(object):
    def __init__(self, root = "", first_filters = None):
        self.root = root
        self.first_filters = first_filters
    def get_root(self):
        return self.root
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
        print name, fs
        if fs == None: return None
        try:
            return open(self.get_file_name(name), "r")
        except:
            return None
        
blast_root = os.path.dirname(os.path.abspath(__file__)) + "/"
if len(sys.argv) > 1:
    blast_root = sys.argv[1] + "/"

fs = BlastFs(blast_root, ["maps/", "robot_fs/", "action_fs/", "object_fs/",])

def return_json(js):
    if type(js) != type(""): js = json.dumps(js)
    return Response(response=js, status=200, mimetype="application/json")



login_sessions = {}
login_sessions_lock = threading.Lock()

def get_world(world):
    raise Exception("Dead function")


debug_locks = False

THUNK = lambda: True
class AcquireWorld(object):
    def __init__(self, world, edit, lock, on_exit_function = THUNK):
        self.world = world
        self.edit = edit
        self.lock = lock
        self.dead = False
        self.on_exit_function = on_exit_function #Called after lock release

    def release(self):
        self.dead = True
        self.lock.release()
        self.on_exit_function()
    def __del__(self):
        self.release()

    def alive(self):
        if self.dead:
            raise Exception("Tried to get values on a released world")

    def ised(self):
        self.alive()
        if not self.edit:
            raise Exception("Tried to edit a non-edit world")
    
    def set_error(self, a = None, b = None):
        raise Exception("We cannot set this property")

    def copy(self, copy_on_write_optimize = False):
        return self.world.copy(copy_on_write_optimize = False)
    
    #FIXME: this could be regulated, it is unregulated now.
    def get_types(self):
        self.alive()
        return self.world.types
    types = property(get_types, set_error)
    def get_maps_keysort(self):
        self.alive()
        return self.world.maps_keysort
    maps_keysort = property(get_maps_keysort, set_error)
    def get_robots_keysort(self):
        self.alive()
        return self.world.robots_keysort
    robots_keysort = property(get_robots_keysort, set_error)
    def get_surfaces_keysort(self):
        self.alive()
        return self.world.surfaces_keysort
    surfaces_keysort = property(get_surfaces_keysort, set_error)
    def get_objects_keysort(self):
        self.alive()
        return self.world.objects_keysort
    objects_keysort = property(get_objects_keysort, set_error)

    def get_map(self, mp):
        self.alive()
        m = self.world.get_map(mp)
        if m: m = m.copy()
        return m
    def get_robot(self, name):
        self.alive()
        r = self.world.get_robot(name)
        if r: r = r.copy()
        return r
    def get_surface(self, name):
        self.alive()
        s = self.world.get_surface(name)
        if s: s = s.copy()
        return s
    def get_object(self, uid):
        self.alive()
        o = self.world.get_object(uid)
        if o: o = o.copy()
        return o
    def get_robot_location(self, name):
        self.alive()
        l = self.world.get_robot(name)
        if l: l = l.location
        if l: l = l.copy()
        return l

    def set_robot_location(self, *args, **kwargs):
        self.ised()
        self.world.set_robot_location(*args, **kwargs)
    def set_robot_position(self, *args, **kwargs):
        self.ised()
        self.world.set_robot_position(*args, **kwargs)
    

def acquire_world(world, edit = False, temporary_edit = False):
    #print world
    if temporary_edit: edit = True
    clear = THUNK
    global login_sessions, login_sessions_lock, manager
    if debug_locks: print "Lock sessions start"
    login_sessions_lock.acquire()
    if debug_locks: print "Lock sessions done"
    if not "sid" in session: 
        login_sessions_lock.release()
        return None
    if not str(session["sid"]) in login_sessions: 
        login_sessions_lock.release()
        return None

    if world == None:
        login_sessions_lock.release()
        if temporary_edit:
            r = manager.world.set_editor(str(session["sid"]), True, return_update = True)
            if not r:
                return None
            if r != "PRESET":
                print "Clear preset"
                clear = lambda: manager.world.clear_editor(str(session["sid"]))

        if debug_locks: print "Lock manager start"
        manager.world.lock.acquire()
        if debug_locks: print "Lock manager done"
        if not manager.world.can_edit(str(session["sid"])) and edit:
            print "We cannot edit the initial world"
            manager.world.lock.release()
            return None
        return AcquireWorld(manager.world.world, edit, manager.world.lock, clear)

    #print login_sessions
    w = login_sessions[str(session["sid"])]["worlds"].get(world)
    l = None
    if w:
        if debug_locks: print "Lock session world start"
        l = login_sessions[str(session["sid"])]["worlds_locks"][world]
        l.acquire()
        if debug_locks: print "Lock session world done"
    login_sessions_lock.release()
    if w:
        return AcquireWorld(w, edit, l)
    return None

def lock_world(world):
    raise Exception("Dead function")
def unlock_world(world):
    raise Exception("Dead function")


def release_world(world):
    return True

def foo():
    print "Rel", world
    global login_sessions, login_sessions_lock, manager
    if debug_locks: print "Lock sessions start"
    login_sessions_lock.acquire()
    if debug_locks: print "Lock sessions done"

    if not "sid" in session: 
        login_sessions_lock.release()
        return None
    if not str(session["sid"]) in login_sessions: 
        login_sessions_lock.release()
        return None
    if world != None:
        login_sessions[str(session["sid"])]["worlds_locks"][world].release()
        login_sessions_lock.release()
        return True
    login_sessions_lock.release()
    manager.world.lock.release()
    return True

def error_world(world):
    print "We had an error with world:", world
    return return_json(None)

def lock_world_error(world):
    raise Exception("Dead function")
    return "World failed to lock"


login_sessions = {}


permissions = ["view", "edit", "plan"]



def clean_sessions():
    global login_sessions, login_sessions_lock
    bad_sessions = []
    if debug_locks: print "Lock sessions start"
    login_sessions_lock.acquire()
    if debug_locks: print "Lock sessions done"
    clean_time = time.time()
    for sid in login_sessions:
        if login_sessions[sid]["last_update"] + SESSION_TIMEOUT < clean_time:
            print "Kill session", sid
            manager.world.clear_editor(str(sid))
            bad_sessions.append(sid)

    for sid in bad_sessions:
        del login_sessions[sid]

    login_sessions_lock.release()


def get_user_permission(session, perm):
    global login_sessions, login_sessions_lock
    if debug_locks: print "Lock sessions start"
    login_sessions_lock.acquire()
    if debug_locks: print "Lock sessions done"
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
    global login_sessions, login_sessions_lock
    if request.method == "PUT":
        if debug_locks: print "Lock sessions start"
        login_sessions_lock.acquire()
        if debug_locks: print "Lock sessions done"
        sid = str(time.time()) + "-" + str(uuid.uuid4())
        login_sessions[sid] = {"sid": sid, "last_update": time.time(),
                               "worlds": {}, "worlds_locks": {}}
        session["sid"] = sid
        login_sessions_lock.release()
        return "true"
    elif request.method == "GET":
        if debug_locks: print "Lock sessions start"
        login_sessions_lock.acquire()
        if debug_locks: print "Lock sessions done"
        if not "sid" in session: 
            login_sessions_lock.release()
            return "false"
        if not str(session["sid"]) in login_sessions: 
            login_sessions_lock.release()
            return "false"
        login_sessions[str(session["sid"])]["last_update"] = time.time()
        login_sessions_lock.release()
        clean_sessions()
        e = manager.world.is_editor(str(session["sid"]))
        r = {"sessions": len(login_sessions),
             "edit_world": e == True,
             "world_changeable": manager.world.world_changeable(str(session["sid"])),
             "edit_other_session": e == False}
        print r, e
        return return_json(r)
    else:
        return "false"
    
    

@app.route('/')
def show_main_page():
    my_path = os.path.dirname(os.path.abspath(__file__))
    f = open(my_path + "/static/index.html", "r")
    r = f.read()
    f.close()
    return r.replace("${SESSION_TIMEOUT}", str(SESSION_TIMEOUT))

@app.route('/world/<world>/surface')
@app.route('/surface')
@app.route('/world/<world>/surface/<surface>')
@app.route('/surface/<surface>')
def api_surface(world = None, surface = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    w = acquire_world(world)
    if not w: return error_world(world)
    if surface:
        r = w.get_surface(surface)
        if r:
            r = r.to_dict()
            if request.args.get("include_type", "false") == "true":
                r["surface_type"] = w.types.surfaces.get(r["surface_type"]).to_dict()
            if request.args.get("include_objects", "false") == "true":
                r["objects"] = [];
                for uid in w.objects_keysort:
                    obj = w.get_object(uid)
                    if obj.parent == r['name']:
                        od = obj.to_dict()
                        if request.args.get("include_object_types", "false") == "true":
                            od['object_type'] = w.types.objects.get(od['object_type']).to_dict()
                        r['objects'].append(od)
            r = return_json(r)
    else:
        r = return_json(w.surfaces_keysort)
    release_world(world)
    return r

@app.route('/world/<world>/map')
@app.route('/map')
@app.route('/world/<world>/map/<map>')
@app.route('/map/<map>')
def api_map(world = None, map = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    w = acquire_world(world)
    if not w: return error_world(world)
    if map:
        r = w.get_map(map)
        if r:
            r = return_json(r.to_dict())
    else:
        r = return_json(w.maps_keysort)
    release_world(world)
    return r

@app.route('/robot')
@app.route('/world/<world>/robot/<robot>')
@app.route('/world/<world>/robot')
@app.route('/robot/<robot>')
def api_robot(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    w = acquire_world(world)
    if not w: return error_world(world)
    if robot:
        rd = w.get_robot(robot)
        if rd:
            rd = rd.to_dict()
            if request.args.get("include_type", "false") == "true":
                rd["robot_type"] = w.types.robots.get(rd["robot_type"]).to_dict()
            if request.args.get("include_objects", "false") == "true":
                for holder_name, obj in rd["holders"].iteritems():
                    if obj != None:
                        od = w.get_object(obj).to_dict()
                        if request.args.get("include_object_types", "false") == "true":
                            od['object_type'] = w.types.objects.get(od['object_type']).to_dict()
                        rd["holders"][holder_name] = od
        release_world(world)
        return return_json(rd)

    if request.args.get("map", False) == False:
        robots = w.robots_keysort
    else:
        robots = []
        for robot in w.robots_keysort:
            if w.get_robot_location(robot).map == request.args["map"]:
                robots.append(robot)
    release_world(world)
    return return_json(robots)

@app.route('/fs/<path:filename>')
def api_fs(filename):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    f = fs.get_file(filename)
    if f == None: return Response(response="", status=404)
    r = f.read()
    f.close()
    mime, encoding = fs.get_mime_type(filename)
    resp = Response(response=r, status=200, mimetype=mime)
    return resp

@app.route('/fspng/<path:filename>')
def api_fspng(filename, imformat="PNG"):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    f = fs.get_file(filename)
    if f == None: return Response(response="", status=404, mimetype="image/png")
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

    w = acquire_world(world, edit = (request.method == "POST"))
    if not w: return error_world(world)

    if request.method == "GET":
        robot = w.get_robot(robot)
        if robot != None: 
            robot = robot.location.to_dict()
        release_world(world)
        return return_json(robot)
    
    r = return_json(w.set_robot_location(robot, json.loads(request.data)))

    for_user = str(session["sid"])
    if world == None: for_user == None
    release_world(world)
    queue_load(world, "robot", robot, for_user)
    return r


@app.route('/robot/<robot>/holder', methods=["GET"])
@app.route('/robot/<robot>/holder/<holder>', methods=["GET", "POST"])
def api_robot_holder(world = None, robot = None, holder = None):
    if request.method == "GET": perm = "view"
    elif request.method == "POST" and world != None: perm = "plan"
    elif request.method == "POST" and world == None: perm = "edit"
    else: return permission_error(request, "INVALID")
    if not get_user_permission(session, perm): return permission_error(request, perm)
    w = acquire_world(world, edit = (request.method == "POST"))
    if not w: return error_world(world)
    
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
    for_user = str(session["sid"])
    if world == None: for_user == None
    queue_load(world, "robot", robot, for_user)
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
    w = acquire_world(world, edit = (request.method == "POST"))
    if not w: error_world(world)
    if request.method == "GET":
        robot = w.get_robot(robot)
        if robot != None: robot = robot.to_dict()
        if robot != None and position == None: robot = sorted(robot["positions"].keys())
        if robot != None and position != None: robot = robot["positions"].get(position, None)
        release_world(world)
        return return_json(robot)
    else:
        if not position in w.get_robot(robot).positions:
            release_world(world)
            return return_json(False)
        set_data = json.loads(request.data)
        r = return_json(w.set_robot_position(robot, position, set_data))
        release_world(world)
        for_user = str(session["sid"])
        if world == None: for_user == None
        queue_load(world, "robot", robot, for_user)
        return r
    release_world(world)
    return None

@app.route('/robot/<robot>/type', methods=["GET"])
def api_robot__type(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    w = acquire_world(world)
    if not w: return error_world(world)
    robot = w.get_robot(robot)
    if robot != None: robot = w.types.robots.get(robot["robot_type"], None)
    if robot != None: robot = robot.to_dict()
    return return_json(robot)

@app.route('/robot_type/<robot>', methods=["GET"])
def api_robot_type(world = None, robot = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    w = acquire_world(world)
    if not w: return error_world(world)
    robot = w.types.robots.get(robot, None)
    if robot != None: robot = robot.to_dict()
    return return_json(robot)

@app.route('/action_type/<robot_type>/<action_type>', methods=["GET"])
@app.route('/action_type/<robot_type>', methods=["GET"])
@app.route('/action_type', methods=["GET"])
def action_type(world = None, robot_type = None, action_type = None):
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    w = acquire_world(world)
    if not w: return error_world(world)
    if action_type and robot_type:
        return return_json(w.types.actions[robot_type + "." + action_type].to_dict())
    
    rts = None
    if robot_type:
        rts = []
        rt = w.types.robots.get(robot_type, None)
        while rt:
            rts.append(rt.name)
            rt = rt.parent

    ats = []
    for name, action in w.types.actions.iteritems():
        if rts == None or name.split(".")[0] in rts:
            ats.append(name.split(".") + [action.user,])
    return return_json(ats)

@app.route('/edit_world', methods=["GET", "PUT"])
def edit_world(world = None, robot = None):
    if request.method == "GET":
        if not get_user_permission(session, "view"): return permission_error(request, "view")
        return session_manage()
#return_json(manager.world.is_editor(str(session["sid"])))
    elif request.method == "PUT":
        if not get_user_permission(session, "edit"): return permission_error(request, "edit")
        state = json.loads(request.data)
        if state == True:
            r = manager.world.set_editor(str(session["sid"]), True)
            queue_load(None, "edit-world", None)
            return return_json(r)
        elif state == False:
            r = manager.world.clear_editor(str(session["sid"]))
            queue_load(None, "edit-world", None)
            return return_json(r)
        queue_load(None, "edit-world", None)
        return return_json(False)
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    return return_json(False)

@app.route('/world', methods=["GET", "PUT"])
def api_world(world = None):
    global login_sessions, login_sessions_lock
    if not get_user_permission(session, "view"): return permission_error(request, "view")
    if request.method == "GET":
        if debug_locks: print "Lock sessions start"
        login_sessions_lock.acquire()
        if debug_locks: print "Lock sessions done"
        r = return_json(login_sessions[str(session["sid"])]["worlds"].keys())
        login_sessions_lock.release()
    elif request.method == "PUT": 
        world_name = json.loads(request.data)
        if world_name == None:
            r = "false"
        elif type(world_name) != str and type(world_name) != unicode:
            print "Wrong type for new world name", type(world_name)
            r = "false"
        else:
            world_name = str(world_name)
            w = acquire_world(world)
            if w:
                if debug_locks: print "Lock sessions start"
                login_sessions_lock.acquire()
                if debug_locks: print "Lock sessions done"
                #Create a new clone world
                if not world_name in login_sessions[str(session["sid"])]["worlds"]:
                    login_sessions[str(session["sid"])]["worlds"][world_name] = \
                        w.copy(copy_on_write_optimize = False) #False forces a copy.
                    login_sessions[str(session["sid"])]["worlds_locks"][world_name] = threading.Lock()
                    r = "true"
                else:
                    print "We failed because we already have a clone world"
                    r = "false"
                login_sessions_lock.release()
                release_world(world)
            else:
                print "We failed because we could not find the initial world"
                r = "false"
    else:
        r = return_json(None)
    return r


def rm_unicode(t):
    if t == True or t == False or t == None:
        return t
    if type(t) == int or type(t) == long or type(t) == int:
        return t
    if type(t) == unicode or type(t) == str:
        return str(t)
    if type(t) == dict:
        r = {}
        for k, v in t.iteritems():
            r[rm_unicode(k)] = rm_unicode(v)
        return r
    if type(t) == list or type(t) == tuple:
        a = [rm_unicode(x) for x in t]
        if type(t) == tuple: return tuple(a)
        return a
    raise Exception("Could not clean: " + type(t))

#@app.route('/world/<world>/plan/<target>', methods=["GET", "PUT", "DELETE"])
@app.route('/plan', methods=["GET", "PUT"])
@app.route('/plan/<target>', methods=["GET", "PUT", "DELETE"])
def api_plan(target = None, world = None):
    if request.method == "PUT" or request.method == "DELETE": 
        if not get_user_permission(session, "plan"): return permission_error(request, "plan")
    else:
        if not get_user_permission(session, "view"): return permission_error(request, "view")

    if request.method == "DELETE":
        print "Cancelling program"
        try:
            target = int(target)
        except:
            return return_json(None)
        cr = manager.world.cancel_program(target)
        return return_json(cr)

    if request.method == "PUT":
        try:
            action_data = json.loads(request.data)
        except:
            return return_json(None)
        #try:
        action_data = rm_unicode(action_data)
        #except:
        #    print "Could not remove unicode"
        #    return return_json(None)

        if target == None:
            return return_json(None)
        print "Acquire target world", target
        wend = acquire_world(target)
        if not wend: return error_world(target)
        if action_data == None:
            return return_json(None)
        print "Planning action!"
        ptc = {}
        for n, v in action_data["parameters"].iteritems():
            if type(v) == dict:
                if "x" in v and "y" in v and "a" in v and "map" in v:
                    ptc[n] = blast_world.BlastPt(v['x'], v['y'], v['a'], v['map'])
                else:
                    ptc[n] = v
            else:
                ptc[n] = v
        r = manager.world.plan_action(action_data["robot"], 
                                      action_data["action"].split(".")[-1], 
                                      ptc, {})
        print "Finished action", r
        return return_json(r)

    if target != None:
        return return_json(None)

    #Get all current programs
    programs = manager.world.get_programs()
    previous_plan_length, plan, current_plan = manager.world.get_plan()
    
    return return_json({'programs': programs, 'is_planning': manager.world.get_is_planning(),
                        'previous_plan_length': previous_plan_length, 'plan': plan,
                        'current_plan': current_plan})

feed_lock = threading.Lock()
feed = []
global_feed_alive = True
def queue_load(w, typ, item, for_user = None):
    global feed, feed_lock, global_feed_alive

    if debug_locks: print "Feed lock start"
    feed_lock.acquire()
    if debug_locks: print "Feed lock done"
    while len(feed) > 0:
        if (feed[0][0] > time.time() - 60.0):
            break
        feed = feed[1:]
    feed.append( (time.time(), w, typ, item, for_user) )
    feed_lock.release()

@app.route('/feed/<start>')
def get_feed(start):
    if "sid" not in session:
        return return_json(None)
    try:
        start = float(start)
    except:
        return return_json(None)
    while global_feed_alive:
        #if debug_locks: print "Feed lock start"
        feed_lock.acquire()
        #if debug_locks: print "Feed lock done"
        for step in feed:
            if step[0] > start and (step[4] == None or step[4] == str(session["sid"])):
                feed_lock.release()
                print step
                return return_json(step)
        feed_lock.release()
        time.sleep(0.1)
    return return_json(None)

@app.route('/execute_plan/<target>', methods=["POST"])
def execute_plan(target):
    return False
    global world_edit_session, world_edit_lock
    if not get_user_permission(session, "plan"): return permission_error(request, "plan")
    world_edit_lock.acquire()
    start = get_world(None)
    end = get_world(target)

    if start.post_exec_world:
        start_clone = start.post_exec_world.copy()
    else:
        start_clone = start.copy()
    start_clone.real_world = False

    if not end in start.plan_steps:
        start.plan_steps[end] = []

    print "Taking already taken steps"
    for i in start.plan_steps[end]:
        start_clone.take_action(i[0], i[1], i[2])
    if not start_clone.world.equal(end.world, tolerant=True):
        world_edit_lock.release()
        return return_json(False)

    if world_edit_session != None and world_edit_session != WORLD_EDIT_EXECUTING_PLAN:
        world_edit_lock.release()
        return return_json(False)
    else:
        world_edit_session = WORLD_EDIT_EXECUTING_PLAN

    start.post_exec_world = start_clone
    start.current_plan.extend(start.plan_steps[end])
    start.plan_steps[end] = []

    world_edit_lock.release()
    
    queue_load(None, "edit-world", None)
    queue_load(None, "plan", None)
    
    return return_json(True)


NOTIFICATION_ERROR = 0
NOTIFICATION_WARNING = 1
NOTIFICIATON_FINISHED = 2


notifications_lock = threading.Lock()
notifications = []

def notification(level, message):
    if debug_locks: print "Notification lock acquire"
    notifications_lock.acquire()
    if debug_locks: print "Notification lock done"
    notifications.append((level, message))
    notifications_lock.release()
    queue_load(None, "notification", str(level) + ":" + str(message))

def on_robot_change(robot):
    queue_load(None, "robot", robot)
    #time.sleep(1.0)
def on_surface_change(surface):
    queue_load(None, "surface", surface)
    time.sleep(1.0)

def on_program_change():
    queue_load(None, "plan", None)


def run(a, w):
    global manager, global_feed_alive
    manager = blast_action.BlastManager(a, w)
    manager.on_robot_change = on_robot_change
    manager.world.on_program_changed = on_program_change
    manager.on_surface_change = on_surface_change
    #mthread = ManagerThread()
    #mthread.start()

    app.run(debug=DEBUG, threaded=True, use_reloader=False, use_evalex=False)
    print "Terminating"
    queue_load(None, "terminate", None)
    global_feed_alive = False
    manager.stop()
    #mthread.alive = False
    #mthread.join()

    
if __name__ == '__main__':
    if "--test" in sys.argv:
        my_path = os.path.dirname(os.path.abspath(__file__))
        my_path += "/../blast_test"
        import blast_world_test
        run(["test_actions"], blast_world_test.make_test_world(my_path))



