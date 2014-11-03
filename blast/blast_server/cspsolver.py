

def is_solved(v):
    return type(v) == list and len(v) == 1

def constrain_to(v, l, accept_none = False):
    #print v, "lock to", l
    if l == None and not accept_none:
        return v
    if type(l) != list:
        l = [l,]
    if v == None:
        return [x for x in l]
    x = []
    for a in v:
        if a in l:
            x.append(a)
        elif type(a) == tuple and len(a) == 2: #a is a position, leading to some complications if it is not in a
            if a[1] == False: #Accept all stuff in l with the same surface
                for ct in l:
                    if type(ct) == tuple and len(ct) == 2 and ct[0] == a[0]:
                        x.append(ct)
            else: #If a is a fixed position, then if there is an arbitrary position on the same surface, accept it
                for ct in l:
                    if type(ct) == tuple and len(ct) == 2 and ct[0] == a[0] and ct[1] == False:
                        x.append(a)
    return x

def constrain_from(v, l):
    if l == None:
        return v
    if type(l) != list:
        l = [l,]
    if v == None:
        return None
    x = []
    for a in v:
        if a not in l:
            x.append(a)
    return x

def atleast_one(x):
    if x == None: return False
    return len(x) >= 1

def propogate(csp, deps, var):
    varc = var.copy()
    changed = False
    for c in csp:
        if c[1] == 'not':
            if is_solved(varc[c[0]]) and is_solved(varc[c[2]]):
                if varc[c[0]][0] == varc[c[2]][0]: #Is a failure
                    return None
            elif is_solved(varc[c[0]]):
                if atleast_one(varc[c[0]]):
                    varc[c[2]] = constrain_to(varc[c[2]], [not varc[c[0]][0]])
            elif is_solved(varc[c[2]]):
                if atleast_one(varc[c[2]]):
                    varc[c[0]] = constrain_to(varc[c[0]], [not varc[c[2]][0]])
        elif c[1] == '&&':
            if is_solved(varc[c[0]]): 
                if varc[c[0]][0] == True: #Loop through all others and require truth
                    for v in c[2:]:
                        if is_solved(varc[v]):
                            if varc[v][0] == False: #All must be true
                                return None
                        else:
                            varc[v] = [True]
                else:
                    all_true = True
                    for v in c[2:]:
                        if is_solved(varc[v]):
                            if varc[v][0] == False: #All must be true
                                all_true = False
                        else:
                            all_true = False
                    if all_true:
                        return None
            elif len(c) == 3: #And reduces to boolean == if we only have on argument
                varc[c[0]] = constrain_to(varc[c[0]], varc[c[2]])
                varc[c[2]] = constrain_to(varc[c[2]], varc[c[0]])
            elif len(c) == 4:
                if is_solved(varc[c[2]]): 
                    if varc[c[2]][0] == False: #Then we must impose that the output == False
                        varc[c[0]] = constrain_to(varc[c[0]], False)
                    else: #Then the output == the other var
                        varc[c[0]] = constrain_to(varc[c[0]], varc[c[3]])
                        varc[c[3]] = constrain_to(varc[c[3]], varc[c[0]])
                if is_solved(varc[c[3]]): 
                    if varc[c[3]][0] == False: #Then we must impose that the output == False
                        varc[c[0]] = constrain_to(varc[c[0]], False)
                    else: #Then the output == the other var
                        varc[c[0]] = constrain_to(varc[c[0]], varc[c[2]])
                        varc[c[2]] = constrain_to(varc[c[2]], varc[c[0]])
        
        elif c[1] == '==':
            if type(c[2]) == str:
                if is_solved(varc[c[0]]) and is_solved(varc[c[2]]):
                    if varc[c[0]][0] != varc[c[2]][0]:
                        return None
                else:
                    varc[c[0]] = constrain_to(varc[c[0]], varc[c[2]])
                    varc[c[2]] = constrain_to(varc[c[2]], varc[c[0]])
            else:
                varc[c[0]] = constrain_to(varc[c[0]], c[2], True)
        
        elif c[1] == '===':
            if is_solved(varc[c[0]]):
                if varc[c[0]][0] == True:
                    if is_solved(varc[c[2]]) and is_solved(varc[c[3]]):
                        if varc[c[2]][0] != varc[c[3]][0]:
                            return None
                    else:
                        varc[c[2]] = constrain_to(varc[c[2]], varc[c[3]])
                        varc[c[3]] = constrain_to(varc[c[3]], varc[c[2]])
                else:
                    if is_solved(varc[c[2]]) and is_solved(varc[c[3]]):
                        if varc[c[2]][0] == varc[c[3]][0]:
                            return None
                    elif is_solved(varc[c[3]]):
                        varc[c[2]] = constrain_from(varc[c[2]], varc[c[3]])
                    elif is_solved(varc[c[2]]):
                        varc[c[3]] = constrain_from(varc[c[3]], varc[c[2]])
        
        elif c[1] == 'extract':
            if is_solved(varc[c[2]]):
                typen = str(type(varc[c[2]][0]))
                if c[3] == 'locations' and typen == "<class 'blast_world.BlastSurface'>":
                     varc[c[0]] = constrain_to(varc[c[0]], varc[c[2]][0].locations[c[4]])
                else:
                    raise Exception("Invalid variable and type for extract: " + str(c) + ": " + typen)
            
        elif c[1] == 'surfaceobject':
            if is_solved(varc[c[2]]):
                varc[c[0]] = constrain_to(varc[c[0]], varc[c[2]][0].objects)

        elif c[1] == 'pos':
            if is_solved(varc[c[2]]):
                varc[c[0]] = constrain_to(varc[c[0]], [(varc[c[2]][0], False),])
        else:
            raise Exception("Invalid constraint for planner: " + str(c))


    return varc

def evaluate(var):
    for n, v in var.iteritems():
        if v == None: return False
        if v == []: return None
        if not is_solved(v): return False
    return True
   

def solvecsp(csp, srt = None, var = None):
    if var == None:
        var = {}
        for c in csp:
            if c[1] == '==':
                if type(c[2]) == str:
                    var[c[2]] = var.get(c[2], None)
            #    if type(c[2]) == list:
            #        s = var.get(c[0], [])
            #        if s == None: s = []
            #        s.extend(c[2])
            #        var[c[0]] = s
            #    elif type(c[2]) == bool or type(c[2]) == float or type(c[2]) == int or type(c[2]) == long:
            #        s = var.get(c[0], [])
            #        if c[2] not in s:
            #            s.append(c[2])
            #        var[c[0]] = s
            #    else:
            #        if type(c[2]) == str:
            #            var[c[2]] = var.get(c[2], None)
            var[c[0]] = var.get(c[0], None)
        #print "NEW"
    #print csp
    #print "->", var
    if srt == None:
        srt = sorted(var.keys())
    
    #Actually do it
    varc = var
    for i in xrange(0, len(csp)):
        varc = propogate(csp, {}, varc)
        if varc == None:
            return []
    #print "--->", varc

    e = evaluate(varc)
    if e == True:
        return [varc,]
    if e == None:
        return []
    sk = None
    for k in srt:
        if not is_solved(varc[k]) and varc[k] != None:
            sk = k
            break
    a = []
    if sk != None:
        o = varc[sk]
        for i in o:
            varc[sk] = [i,]
            a.extend(solvecsp(csp, srt, varc))
    return a
    





