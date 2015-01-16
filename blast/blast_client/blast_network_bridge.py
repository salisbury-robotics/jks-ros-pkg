
import socket, threading, time, traceback, hashlib, sys, subprocess, os, json

def enc_str(s):
    return s.replace("%", "%p").replace("\n", "%n").replace(",", "%c")
def dec_str(s):
    return s.replace("%n", "\n").replace("%c", ",").replace("%p", "%")

class MapStore():
    def __init__(self, mdir):
        self.lock = threading.Lock()
        self.maps_dir = mdir
        self.map_cache = {}
        self.map_hash = {}
        self.map_resolution = {}
    
    def write_map(self, map_name, c, res):
        #TODO: validate map name
        self.lock.acquire()
        fn = open(self.maps_dir + "/" + map_name + ".pgm", "w")
        fn.write(c)
        fn.close()
        fn = open(self.maps_dir + "/" + map_name + ".txt", "w")
        fn.write(str(res))
        fn.close()
        if c in self.map_cache:
            del self.map_cache[c]
        if c in self.map_hash:
            del self.map_hash[c]
        if c in self.map_resolution:
            del self.map_resolution[c]
        self.load_map(map_name, no_lock = self.lock)
    
    def load_map(self, map_name, no_lock = False):
        if no_lock != False and no_lock != self.lock:
            raise Exception("You should not set the no_lock parameter")
        elif no_lock == False:
            self.lock.acquire()
        if map_name in self.map_cache and map_name in self.map_hash and map_name in self.map_resolution:
            rv = self.map_cache[map_name], self.map_hash[map_name], self.map_resolution[map_name]
            self.lock.release()
            return rv

        if not os.path.isfile(self.maps_dir + "/" + map_name + ".txt"):
            self.lock.release()
            return None, None, None
        if not os.path.isfile(self.maps_dir + "/" + map_name + ".pgm"):
            self.lock.release()
            return None, None, None
        try:
            print "Read in", map_name
            fn = open(self.maps_dir + "/" + map_name + ".txt", "r")
            res = float(fn.read())
            fn.close()
            fn = open(self.maps_dir + "/" + map_name + ".pgm", "r")
            fd = [] 
            hl = hashlib.sha256()
            hl.update(str(res))
            while True:
                r = fn.read(128)
                if not r: break
                fd.append(r)
                hl.update(r)
            fn.close()
            md = "".join(fd)
            mh = str(hl.digest())
            self.map_cache[map_name] = md
            self.map_hash[map_name] = mh
            self.map_resolution[map_name] = res
            print "Done"
            self.lock.release()
            return md, mh, res
        except:
            print "Could not open", map_name
            traceback.print_exc()
            self.lock.release()
            return None, None, None

class BlastNetworkBridge:
    def __init__(self, host, port, robot_name, robot_type, 
                 action_start, install_action, capability_cb, map_store, require_location = False):
        self.action_start = action_start
        self.install_action = install_action
        self.robot_name = robot_name
        self.robot_type = robot_type
        self.require_location = require_location
        self.map_store = map_store
        self.capability_cb = capability_cb
        self.connection_lock = threading.Lock()
        self.alive = True
        self.host = host
        self.port = port
        self.action_id = 0
        self.thread = None
        self.error = False
        self.connect_fail = False
        self.action_callbacks = {}
        self.capabilities = {}
        self.SHUT_WR = socket.SHUT_WR
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # Connect to server and send data
            self.sock.connect((self.host, self.port))
        except:
            self.connect_fail = True
            self.error = "CONNECT FAIL"

    def capability_write(self, aid, cap, fn, param):
        if not aid in self.action_callbacks:
            return "null"
        do_send = True
        res = "null"
        if cap not in self.capabilities:
            self.capabilities[cap] = set()
        
        #Special logic for the start and stop function.
        if fn == "START":
            if len(self.capabilities[cap]) > 0:
                do_send = False
            if aid in self.capabilities[cap]: #Return false if already started
                res = "false"
            else:
                res = "true"
                self.capabilities[cap].add(aid)
                param = "null"
        elif fn == "STOP":
            if aid not in self.capabilities[cap]:
                res = "false"
                do_send = False
            else:
                self.capabilities[cap].remove(aid)
                res = "true"
                if len(self.capabilities[cap]) > 0:
                    do_send = False
        if do_send:
            res = self.capability_cb(cap, fn, param)
        return res

    def write_data(self, data, terminate_action = None):
        good = True
        print "Writing data", data
        self.connection_lock.acquire()
        if terminate_action != None:
            print "Terminating action", terminate_action
            if terminate_action in self.action_callbacks:
                del self.action_callbacks[terminate_action]
            for cap in self.capabilities:
                if terminate_action in self.capabilities[cap]:
                    self.capabilities[cap].remove(terminate_action)
                    if len(self.capabilities[cap]) == 0:
                        self.connection_lock.release()
                        self.capability_cb(cap, "STOP", "null")
                        self.connection_lock.acquire()
        if not self.error and self.alive:
            try:
                self.sock.send(data)
            except:
                print "Send error!!!"
                traceback.print_exc()
                good = False
        else:
            print "Failed for error or alive", self.error, self.alive
            good = False
        self.connection_lock.release()
        if not good:
            print "We could not write data to the server!"
        print "Done"
        return good

    def thread_runner(self):
        try:
            self.sock.send("BLAST_ROBOT_CONNECT\n")
            buff = ""
            print "Beginning reading"
            while self.alive and not self.error:
                if buff.find("\n") == -1:
                    received = self.sock.recv(2*1024*2*2*2)
                    if received == None or received == False:
                        break
                    buff += received
                    print "STPE", buff
                if buff.find("\n") != -1:
                    packet = buff[0:buff.find("\n")].strip()
                    buff = buff[buff.find("\n")+1:]
                    print "Packet", packet[0:128]
                    self.connection_lock.acquire()
                    if packet.find("ERROR,") == 0:
                        self.error = packet[packet.find(",")+1:]
                    elif packet == "WAIT_NAME":
                        self.sock.send("ROBOT," + self.robot_name + "," + self.robot_type + "\n")
                    elif packet == "VALID_ROBOT":
                        self.sock.send("LIST_MAPS\n")
                    elif packet.find("ACTIONS") == 0:
                        ac_data = packet.split(",")[1:]
                        for i in range(len(ac_data)/2):
                            if not self.install_action(ac_data[i*2], ac_data[i*2+1]):
                                print "Could not install an action"
                                self.error = "Could not install an action: " + str(ac_data[i*2]) \
                                    + " " + str(ac_data[i*2+1]) + " - " + str(ac_data)
                                break
                        #Start the controller
                        if self.error:
                            self.connection_lock.release()
                            break
                        if self.require_location:
                            self.sock.send("WAIT_LOCATION\n")
                        else:
                            self.sock.send("START\n")
                    elif packet.find("MAPS") == 0:
                        map_data = packet.split(",")[1:]
                        is_good = True
                        for i in range(len(map_data)/2):
                            map_name, hash_value = dec_str(map_data[i*2]), dec_str(map_data[i*2+1])
                            map_content, map_hash, map_resolution = self.map_store.load_map(map_name)
                            if not map_content or not map_hash:
                                print "Non-existant file", map_name
                                self.sock.send("GET_MAP," + map_name + "\n")
                                is_good = False
                                break
                            else:
                                if map_hash != hash_value:
                                    print "Invalid hash", map_name
                                    print map_hash
                                    print hash_value
                                    self.sock.send("GET_MAP," + enc_str(map_name) + "\n")
                                    is_good = False
                                    break
                        if is_good:
                            self.sock.send("LIST_ACTIONS\n")
                    elif packet.find("MAP,") == 0:
                        map_data = packet.split(",")
                        map_name = dec_str(map_data[1])
                        map_r = float(dec_str(map_data[2]))
                        map_c = dec_str(map_data[3])
                        self.map_store.write_map(map_name, map_c, map_r)
                        self.sock.send("LIST_MAPS\n")
                    elif packet.find("STARTED") == 0:
                        self.connection_lock.release()
                        break
                    else:
                        self.error = "INVALID PACKET," + packet[0:128]
                    self.connection_lock.release()


            if self.error:
                print "System error, not starting second loop."
            else:
                print "-------------------Starting second loop-----------------------"
                while self.alive and not self.error:
                    if buff.find("\n") == -1:
                        received = self.sock.recv(2*1024*2*2*2)
                        if received == None or received == False:
                            break
                        buff += received
                    if buff.find("\n") != -1:
                        packet = buff[0:buff.find("\n")].strip()
                        buff = buff[buff.find("\n")+1:]
                        self.connection_lock.acquire()
                        def is_int(x):
                            try:
                                int(x)
                                return True
                            except:
                                return False
                        if packet.find("START_ACTION,") == 0:
                            self.sock.send("STARTED_ACTION,"
                                           + str(self.action_id) + "\n")
                            self.action_callbacks[self.action_id] \
                                = self.action_start(packet.split(",")[1].strip(),
                                                    packet.split(",")[2].strip(),
                                                    self.action_id,
                                                    dec_str(packet.split(",")[3]),
                                                    self.write_data,
                                                    self.capability_write)
                            self.action_id += 1
                            self.connection_lock.release()
                        elif is_int(packet.split(",")[0]):
                            aid = int(packet.split(",")[0])
                            data = packet[packet.find(",")+1:]
                            print "Send", aid, "<-", data
                            ac = self.action_callbacks.get(aid, None)
                            self.connection_lock.release()
                            if ac != None:
                                ac(dec_str(data))
                            else:
                                self.error = "INVALID_ACTION," + str(aid)
                        else:
                            self.error = "INVALID_PACKET," + packet
                            self.connection_lock.release()
        except:
            print "There was an exception in reading:"
            traceback.print_exc()
            self.error = "READ EXCEPTION"
        finally:
            self.sock.close()
            self.sock = None
            self.alive = False

    def start(self):
        self.thread = threading.Thread(target=self.thread_runner)
        self.thread.start()

    def wait(self):
        try:
            while self.alive:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        
    def __del__(self):
        self.stop()

    def stop(self):
        self.alive = False
        if self.sock:
            self.sock.shutdown(self.SHUT_WR)
        if self.thread:
            self.thread.join()
            self.thread = None
        

#['python', exec_path, py_file, json.dumps(json_prepare(parameters))],
class ActionExecutor():
    def __init__(self, start_command, action_id, write_callback, capability_callback):
        self.shutdown = False
        self.action_id = action_id
        self.write_callback = write_callback
        self.capability_callback = capability_callback
        self.proc = subprocess.Popen(start_command,
                                     stdout=subprocess.PIPE, 
                                     stdin=subprocess.PIPE,
                                     stderr=sys.stdout) #TODO logging output needs to be handled
        self.thread = threading.Thread(target=self.thread_run)
        self.thread.start()

    def get_callback(self):
        return self.write
    
    def thread_run(self):
        while not self.shutdown:
            result = self.proc.stdout.readline()
            if result != "":
                print "Data", result
                if (result.find("CAPABILITY,") == 0):
                    data = result.split(",")
                    cap = dec_str(data[1])
                    fn = dec_str(data[2])
                    param = json.loads(dec_str(data[3]))
                    d = self.capability_callback(self.action_id, cap, fn, param)
                    self.write(enc_str(d) + "\n") #Write the capability result.
                else:
                    term = None
                    if result.strip().strip(",").strip() == "TERMINATE":
                        print "Recieved an explicit terminate message", self.action_id
                        term = self.action_id
                    self.write_callback(str(self.action_id) + "," + enc_str(result) + "\n", terminate_action = term)
            else:
                if self.proc.poll() != None:
                    print "Action shutdown"
                    self.shutdown = True
                    self.write_callback(str(self.action_id) + ",TERMINATE%n\n", terminate_action = self.action_id)
                time.sleep(0.001)
        print "Thread shutdown"
        
    def write(self, data):
        if data == None:
            if not self.shutdown:
                self.shutdown = True
                self.proc.terminate()
            self.thread.join()
            return
        self.proc.stdin.write(data)
        self.proc.stdin.flush()
        
        


def action_start(action_robot_type, action_name, action_id, parameters, write_callback, capability_write):
    print "Action!!!"
    def my_action(data):
        print data
    return my_action

def capability_callback(capability, command, parameters):
    print "Run capability command:", capability, command, parameter
    return "null"

def install_action(robot_type, action_name):
    print "Installing", robot_type, action_name, "(test - does nothing)"
    return True

if __name__ == '__main__' and len(sys.argv) > 1:
    def write_c(s):
        print "ACTION", s
    act = ActionExecutor(["sleep", "5"], 1, write_c, capability_callback)
    time.sleep(1.0)
    act.write(None)
elif __name__ == '__main__':
    map_store = MapStore("maps/")
    bnb = BlastNetworkBridge("localhost", 8080, "stair4", "pr2-cupholder",
                             action_start, install_action, capability_callback, map_store)
    bnb.start()
    bnb.wait()
    bnb.stop()
    if bnb.error:
        print "The system terminated with error", bnb.error









