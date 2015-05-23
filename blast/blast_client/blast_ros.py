
import sys, os, threading, time, math, socket, random, subprocess
import blast_network_bridge, blast_action_exec
import rospy, genpy, actionlib, roslib

roslib.load_manifest('tf')
import tf


class BlastROSError(Exception):
    __slots__ = ['value']
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)


import Image
from StringIO import StringIO

class RosMapPub():
    def __init__(self, sd, map_store):
        self.lock = threading.Lock()
        self.sd = sd
        self.frame_id = sd.get("frame_id", "/map")
        self.display_map = None
        self.map_store = map_store
        self.servs = {}
        self.topics = {}
        roslib.load_manifest("nav_msgs")
        import nav_msgs
        import nav_msgs.srv
        import nav_msgs.msg
        for serv in sd.get("services", []):
            self.servs[serv] = rospy.Service(serv, nav_msgs.srv.GetMap, self.get_map_service)
        for topic in sd.get("topics", []):
            self.topics[topic] = rospy.Publisher(topic, nav_msgs.msg.OccupancyGrid, latch=True)

    def set_map_int(self, mn):
        self.lock.acquire()
        self.display_map = mn
        self.res, dat = self.map_store.get_data(self.display_map)
        imc = Image.open(StringIO(dat))
        im = imc.transpose(Image.FLIP_TOP_BOTTOM)
        self.w, self.h = im.size
        datc = im.getdata()
        OCCT = int(0.65 * 255)
        FREE = int(0.196 * 255)
        def proc_pix(x):
            x = 255 - x
            if x > OCCT: return 100
            if x < FREE: return 0
            return -1
        self.datc = [proc_pix(x) for x in list(datc)]
        
        import nav_msgs
        import nav_msgs.msg
        mp = nav_msgs.msg.OccupancyGrid()
        mp.header.frame_id = self.frame_id
        mp.info.resolution = 1.0/self.res
        mp.info.width = self.w
        mp.info.height = self.h
        mp.info.origin.position.x = 0
        mp.info.origin.position.y = 0
        mp.info.origin.position.z = 0
        mp.info.origin.orientation.x = 0
        mp.info.origin.orientation.y = 0
        mp.info.origin.orientation.z = 0
        mp.info.origin.orientation.w = 0
        mp.data = self.datc
        for tn, p in self.topics.iteritems():
            p.publish(mp)
        self.lock.release()

    def get_map_service(self, req):
        self.lock.acquire()
        import nav_msgs
        import nav_msgs.srv
        occ = nav_msgs.srv.GetMapResponse()
        occ.map.header.frame_id = self.frame_id
        occ.map.info.resolution = 1.0/self.res
        occ.map.info.width = self.w
        occ.map.info.height = self.h
        occ.map.info.origin.position.x = 0
        occ.map.info.origin.position.y = 0
        occ.map.info.origin.position.z = 0
        occ.map.info.origin.orientation.x = 0
        occ.map.info.origin.orientation.y = 0
        occ.map.info.origin.orientation.z = 0
        occ.map.info.origin.orientation.w = 0
        occ.map.data = self.datc
        self.lock.release()
        return occ

    def set_map(self, mn):
        self.set_map_int(mn)

    def __del__(self):
        pass

class BlastRos():
    def __init__(self, node_name, capabilities, map_store, start_ros = False):
        self.start_ros = start_ros
        self.map_store = map_store
        self.roscore = None
        if start_ros:
            self.roscore = subprocess.Popen(['roscore'])

        rospy.init_node(node_name)
        self.listener = tf.TransformListener()
        self.node_name = node_name
        self.capabilities = capabilities
        self.subscribers = {}
        self.last_data = {}
        self.data_lock = threading.Lock()
        self.publishers = {}
        self.msg_const = {}
        self.map_dict = {}
        self.srv_const = {}
        self.srv_proxy = {}
        self.action_const = {}
        self.actions = {}
        self.launch_processes = {}
        
        self.spin_thread_o = threading.Thread(target = self.spin_thread)
        self.spin_thread_o.start()

        self.msg_cache = {}

    def get_msg(self, t):
        if not t in self.msg_cache:
            mf, mt = t.split("/")
            exec("import " + mf + ".msg as rosmsg_" + mf)
            exec("self.msg_cache[t] = rosmsg_" + mf + "." + mt)
        return self.msg_cache[t]()


    def fill_message(self, msg, json):
        if type(json) == unicode:
            return str(json)
        if type(json) in [long, float, bool, int]:
            return json
        if type(json) == dict:
            td = {}
            for k, v in zip(msg.__slots__, msg._get_types()):
                td[k] = v
            for k, v in json.iteritems():
                if not hasattr(msg, k):
                    raise blast_action_exec.BlastRuntimeError("Invalid key for message: " 
                                                              + k + " of " + str(type(msg))
                                                              + " " + str(dir(msg)))
                elif isinstance(getattr(msg, k, v), genpy.rostime.Duration):
                    setattr(msg, k, genpy.rostime.Duration(float(v)))
                elif isinstance(getattr(msg, k, v), genpy.rostime.Time):
                    setattr(msg, k, genpy.rostime.Time(float(v)))
                elif type(getattr(msg, k)) == str:
                    setattr(msg, k, str(v))
                elif type(getattr(msg, k)) in [long, float, bool, int]:
                    setattr(msg, k, v)
                elif type(v) == dict:
                    self.fill_message(getattr(msg, k), v)
                elif (type(v) == list or type(v) == tuple) and td[k] == "duration[]":
                    setattr(msg, k, [genpy.rostime.Duration(float(x)) for x in v])
                elif (type(v) == list or type(v) == tuple) and td[k] == "time[]":
                    setattr(msg, k, [genpy.rostime.Time(float(x)) for x in v])
                elif (type(v) == list or type(v) == tuple) and td[k].find("/") != -1:
                    x = []
                    for a in v:
                        b = self.get_msg(td[k].strip("[]"))
                        x.append(b)
                        self.fill_message(b, a)
                    setattr(msg, k, x)
                elif (type(v) == list or type(v) == tuple):
                    setattr(msg, k, [str(x) if type(x) == unicode else x for x in v])
                else:
                    raise blast_action_exec.BlastRuntimeError("Invalid type for message: " + str(k) + " " + str(type(getattr(msg, k))))
        else:
            raise blast_action_exec.BlastRuntimeError("Invalid type for message: " + str(json) + " " + str(type(msg)))

    def ret_msg(self, msg, k = "ROOT"):
        if type(msg) in [long, float, int, bool] or msg == None:
            return msg
        elif type(msg) in [str, unicode]:
            return str(msg)
        elif type(msg) == list or type(msg) == tuple:
            return [self.ret_msg(s) for s in msg]
        elif isinstance(msg, genpy.Message):
            d = {}
            for k in msg.__slots__:
                v = getattr(msg, k)
                d[str(k)] = self.ret_msg(v, k)
            return d
        elif isinstance(msg, genpy.rostime.Time):
            return {"secs": msg.secs, "nsecs": msg.nsecs}
        else:
            raise BlastROSError("Invalid type for message: " + str(type(msg)) + " from " + k)
        return None
        
    def ros_callback(self, msg, t):
        #print t, "->", msg
        rmsg = self.ret_msg(msg)
        self.data_lock.acquire()
        self.last_data[t] = rmsg
        self.data_lock.release()

    def spin_thread(self):
        rospy.spin()

    def install_capability(self, capability_name, capability_dict):
        #print "SPAM", capability_name
        #TODO: make it so that this actually checks for hash stuff
        return (capability_name in self.capabilities)

    def capability_cb(self, cap, fn, param):
        if not cap in self.capabilities:
            print "Invalid capability", cap
            return None

        if fn == "START":
            print "Starting", cap

            for pkg, fil in self.capabilities[cap]["START"].get("launch-files", []):
                if (pkg, fil) not in self.launch_processes:
                    proc = subprocess.Popen(['roslaunch', pkg, fil])
                    self.launch_processes[(pkg, fil)] = proc
                    
        
            for topic, data in self.capabilities[cap]["START"].get("publishers", {}).iteritems():
                mtype = data["message"]
                mtype_folder = mtype.split(".")[0]
                roslib.load_manifest(mtype_folder)
                exec("import " + mtype_folder + ".msg as rosmsg_" + mtype_folder)
                exec("self.publishers[topic] = rospy.Publisher(topic, rosmsg_" + mtype + ")")
                exec("self.msg_const[topic] = rosmsg_" + mtype)

            for topic, data in self.capabilities[cap]["START"].get("subscribers", {}).iteritems():
                mtype = data["message"]
                mtype_folder = mtype.split(".")[0]
                roslib.load_manifest(mtype_folder)
                exec("import " + mtype_folder + ".msg as rosmsg_" + mtype_folder)
                exec("self.subscribers[topic] = rospy.Subscriber(topic, rosmsg_" + mtype 
                     + ", (lambda s: lambda x: s.ros_callback(x, '" + topic + "'))(self))")
                exec("self.msg_const[topic] = rosmsg_" + mtype)

            action_wait = []
            for topic, data in self.capabilities[cap]["START"].get("actions", {}).iteritems():
                mtype = data["action"]
                mtype_folder = mtype.split(".")[0]
                roslib.load_manifest(mtype_folder)
                exec("import " + mtype_folder + ".msg as rosmsg_" + mtype_folder)
                exec("self.actions[topic] = actionlib.SimpleActionClient(topic, rosmsg_" + mtype +"Action)")
                exec("self.action_const[topic] = (rosmsg_" + mtype + "Goal,)")
                action_wait.append(self.actions[topic])
                
            for topic, data in self.capabilities[cap]["START"].get("services", {}).iteritems():
                mtype = data["message"]
                mtype_folder = mtype.split(".")[0]
                roslib.load_manifest(mtype_folder)
                exec("import " + mtype_folder + ".srv as rossrv_" + mtype_folder)
                exec("self.srv_proxy[topic] = rospy.ServiceProxy(topic, rossrv_" + mtype + ")")
                exec("self.srv_const[topic] = (rossrv_" + mtype
                     + "Request, rossrv_" + mtype + "Response)")
                if data.get("wait", True):
                    rospy.wait_for_service(topic)

            for topic, data in self.capabilities[cap]["START"].get("map", {}).iteritems():
                self.map_dict[cap + "__" + topic] = RosMapPub(data, self.map_store)

            for a in action_wait:
                a.wait_for_server()
            return True
        
        if fn == "STOP":
            print "Stopping", cap
            w_queue = []
            for pkg, fil in self.capabilities[cap]["START"].get("launch-files", []):
                if (pkg, fil) in self.launch_processes:
                    w_queue.append(self.launch_processes[(pkg, fil)])
                    self.launch_processes[(pkg, fil)].terminate()
                    del self.launch_processes[(pkg, fil)]
            
            for topic, data in self.capabilities[cap]["START"].get("subscribers", {}).iteritems():
                del self.subscribers[topic]
                
            for topic, data in self.capabilities[cap]["START"].get("publishers", {}).iteritems():
                del self.publishers[topic]
                del self.msg_const[topic]

            for topic, data in self.capabilities[cap]["START"].get("services", {}).iteritems():
                del self.srv_proxy[topic]
                del self.srv_const[topic]

            for topic, data in self.capabilities[cap]["START"].get("actions", {}).iteritems():
                del self.action_const[topic]
                del self.actions[topic]
                
            for q in w_queue: q.wait()
            return True
    
        if not fn in self.capabilities[cap]:
            print "Invalid fn for", cap, "-", fn
            raise blast_action_exec.BlastRuntimeError("Invalid function: " + fn + " for capability " + cap)
            return None
        
        if self.capabilities[cap][fn]["type"] == "service":
            topic = self.capabilities[cap][fn]["name"]
            msg = self.srv_const[topic][0]()
            self.fill_message(msg, param)
            resp = self.srv_proxy[topic](msg)
            return self.ret_msg(resp)
        elif self.capabilities[cap][fn]["type"] == "pub":
            topic = self.capabilities[cap][fn]["topic"]
            msg = self.msg_const[topic]()
            self.fill_message(msg, param)
            self.publishers[topic].publish(msg)
            #print "Sent", topic, msg
            return True
        elif self.capabilities[cap][fn]["type"] == "setmap":
            serve = self.capabilities[cap][fn]["serve"]
            self.map_dict[cap + "__" + serve].set_map(param)
            return True
        elif self.capabilities[cap][fn]["type"] == "param":
            path = self.capabilities[cap][fn]["path"]
            dv = self.capabilities[cap]["START"]["params"][path]
            if param != None:
                if dv.get("type") == None:
                    raise Exception("Set unsetable parameter")
                elif dv.get("type") == "float":
                    param = float(param)
                elif dv.get("type") == "int":
                    param = int(param)
                elif dv.get("type") == "bool":
                    param = bool(param)
                elif dv.get("type") == "long":
                    param = long(param)
                elif dv.get("type") == "str":
                    param = str(param)
                else:
                    raise Exception("Invalid type for " + path+ ": " + dv.get("type"))
                rospy.set_param(path, param)
            return rospy.get_param(path)
        elif self.capabilities[cap][fn]["type"] == "action-send" or self.capabilities[cap][fn]["type"] == "action-wait":
            topic = self.capabilities[cap][fn]["topic"]
            msg = self.action_const[topic][0]()
            self.fill_message(msg, param)
            if self.capabilities[cap][fn]["type"] == "action-send":
                self.actions[topic].send_goal(msg)
            else:
                self.actions[topic].send_goal_and_wait(msg)
            return True
        elif self.capabilities[cap][fn]["type"] == "action-cancel":
            topic = self.capabilities[cap][fn]["topic"]
            self.actions[topic].cancel_goal()
            return True
        elif self.capabilities[cap][fn]["type"] == "action-result":
            topic = self.capabilities[cap][fn]["topic"]
            if type(param) == float:
                self.actions[topic].wait_for_result(rospy.Duration(param))
            else:
                self.actions[topic].wait_for_result()
            return self.ret_msg(self.actions[topic].get_result())
        elif self.capabilities[cap][fn]["type"] == "tf-last":
             try:
                 (trans,rot) = self.listener.lookupTransform(self.capabilities[cap][fn]["source"],
                                                             self.capabilities[cap][fn]["destination"], 
                                                             rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                 return None
             return [trans, rot, tf.transformations.euler_from_quaternion(rot)]
        elif self.capabilities[cap][fn]["type"] == "sub-last":
            topic = self.capabilities[cap][fn]["topic"]
            data = None
            while True:
                if rospy.is_shutdown():
                    raise BlastROSError("BLAST Node shutdown while ROS was stopping")
                self.data_lock.acquire()
                if topic in self.last_data:
                    data = self.last_data[topic]
                    self.data_lock.release()
                    break
                self.data_lock.release()
                time.sleep(0.0001)
            return data
        elif self.capabilities[cap][fn]["type"] == "rosrun":
            package = self.capabilities[cap][fn]["package"]
            prog = self.capabilities[cap][fn]["prog"]
            args = self.capabilities[cap][fn]["args"]
            proc = subprocess.Popen(["rosrun", package, prog, ] + args)
            proc.wait()
            return True
        else:
            raise blast_action_exec.BlastRuntimeError("Invalid capability type for capability: " + cap
                                                      + " - " + self.capabilities[cap][fn]["type"])

        raise blast_action_exec.BlastRuntimeError("Invalid function: " + fn + " for capability " + cap)
        return None

