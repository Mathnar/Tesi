from skyfield.api import Topos, load
import numpy as np
import pyproj
import math
import sympy as sym
import time
import networkx as nx
import pygeodesy as p
import pickle

from EarthSat import source,dest


########################################################################################################################    1:18
def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)
    return x,y,z

def distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2) * 1.0)

def isVisible(x1,y1,z1,x2,y2,z2):    #1 = p ; 2 = q                      l = vect[0] ; m = vect[1] ; n = vect[2]
    r = 6378137
    sym.init_printing()
    vect = []
    l = x2 - x1
    m = y2 - y1
    n = z2 - z1
    vect.append(l)
    vect.append(m)
    vect.append(n)
    x,y,z = sym.symbols('x,y,z')
    a = sym.Eq((x-x1)/vect[0],(y-y1)/vect[1])
    b = sym.Eq((y-y1)/vect[1],(z-z1)/vect[2])
    tmp = sym.solve([a, b], (x, y, z))
    d = sym.Eq(x, tmp.get(x))
    e = sym.Eq(y, tmp.get(y))
    c = sym.Eq(x**2+y**2+z**2,r**2)
    tmp = sym.solve([d,e,c],(x,y,z))
    if sym.im(tmp[0][0]):
        return True
    else:
        return False


def load_CP(update, indice):
    mtxt = []
    mtx = []
    matrix =[[0 for x in range(0,32)] for y in range(0,32)]
    if update:
        c_graph = nx.Graph()
        tmp = []


        id=indice
        k = 0           #indice che scorre finché non trova l'indicatore di nuova matrice
        while k != id:
            # print("lines ",lines[k])
            k = k + 1

        base = k + 2
        end = base + 31
        # print("QUAAA" ,base,lines[base], "end ",lines[end])
        r = -1
        for i in range(base, end):
            tmp.append(lines[i].split(' '))
            r = r+1
            for j in range(0, 32):
                ind = i - base
                #print("base ",j," ", tmp[ind][j])
                # print(tmp[ind][j])
                c_graph.add_edge(iridium[ind], iridium[j], weight=float(tmp[ind][j]))
                if str(round(float(tmp[ind][j]),1)) != str(0.0) and str(math.inf) not in tmp[ind][j]:
                    #print("tmp  ",j, "ss ", tmp[ind][j])
                    mtxt.append(j)
            #print("R",r)
            #print("mtxt pre ", mtxt)
            mtx.append(np.array(mtxt))
            #adj[r]=mtxt
            #print("adj",adj)
            mtxt.clear()
            #print("mtxt dopo ", mtxt)
            #print("adj dopo ", adj)
            #print("oo", adj[r])
        return c_graph,mtx


def getInd(now,start):
    ind = 1
    if 10 < now-start < 20:
        ind=36
    elif 20 < now-start < 30:
        ind=71
    elif 30 < now-start < 40:
        ind=106
    elif 40 < now-start < 50:
        ind = 141
    elif 50 < now-start < 60:
        ind = 176
    return ind


def getRl(c_graph,source_ind,dest_ind,ecluded_nodes):
    rl = []  # route list
    rl_weight = []
    load_nb = load_CP(update,ind)[1]

    nb = load_nb[source_ind]
    size = nb.size
   # print("nb type ", type(nb))
    for i in range(0,size-1 ):
        if nb[i] in ecluded_nodes:
            #tmp = nb[i]
      #      print("nb[i] ", nb[i],"i ",i)
      #      print("excluded nodes ", excluded_nodes)
            nb = np.delete(nb,i)
       #     print("nb dopo ", nb)
            size = size-1
    for i in range(0,nb.size):
        rl.append(nx.dijkstra_path(c_graph, iridium[nb[i]], iridium[dest_ind]))
        rl_weight.append(nx.dijkstra_path_length(c_graph, iridium[nb[i]], iridium[dest_ind]))
    return rl,rl_weight,nb
########################################################################################################################
print("\n#########################CgrExe")
stations_url = 'http://celestrak.com/NORAD/elements/iridium.txt'
satellites = load.tle(stations_url, reload=False)
iridium = []
c = 0
k = False


for sat in satellites:
    if k:
        iridium.append(satellites[sat])
        c=c+1
    k = not k


c_graph = nx.Graph()

op = open("file.txt","r")
lines = op.readlines()


vertices = []


for i in range(0,32):
 #   vertices.append(iridium[i])
    c_graph.add_node(iridium[i])


#print(adjM.todense())
#print(source)
#print(dest)
#writeTxt(adjM.todense())#scrivo su prova.txt

i = 0                                                       #trovo indici di Source e Dest
while str(source) != str(iridium[i]) and i<31:
    i=i+1
j = 0
while str(dest) != str(iridium[j]) and j<31:
    j=j+1


print("\n#########################Begin Cgr Routine")
now = time.time()  #inizializzo tempo
#end = now+10

update = True   #inizio var update (forse non serve?)
pre_ind = ind = 1   #inizializzo indice e indice precedente

c_graph = load_CP(update,ind)[0] #prima matrix, gli passo update true e il primo indice del cotact plan
load_nb = load_CP(update,ind)[1]

dest = j
#pre_node = None
myNode = i                          #var che passerà di nodo in nodo fino a dest
route_list = []
excluded_nodes = []
path_weight = 0
while myNode != dest:
    #print("mynode ",myNode)

    route_list.append(myNode)

    excluded_nodes.append(myNode)

   # nb = load_nb[myNode]
    #for h in range(0,nb.size):
        #if nb[h] == pre_node:
            #print("Sono nel for, nell'if; nb[h]=",nb[h],"prenode ",pre_node)
            #nb.pop(h)
    #-print("Carico le rl dei nodi adiacenti a  ", iridium[myNode])

    rl = getRl(c_graph, myNode, j,excluded_nodes)[0]
    rl_weight = getRl(c_graph, myNode, j,excluded_nodes)[1]

    nb = getRl(c_graph, myNode, j,excluded_nodes)[2]

    #-for k in range(0, rl_weight.__len__()):
      #-  print("Peso del percorso che inizia da ", nb[k], rl_weight[k])

    min_rl = nb[rl_weight.index(min(rl_weight))]
    path_weight = path_weight + min(rl_weight)
   #print("Percorso con costo minore (indice) ", min_rl)

    #pre_node = myNode
    myNode = min_rl


print("\nRoute list dell'indice ",ind)
nhop = 0
for q in range (0,route_list.__len__()):
    print("Step ", q," ", iridium[route_list[q]])
    nhop = nhop+1
print("________________________________________________")
print("\n# Hops [cgr] ",nhop)
print("\n Path Weight [cgr] {/10^7} ", path_weight/10000000)
print("________________________________________________")
dj = nx.dijkstra_path(c_graph, iridium[i], iridium[j])
print("\nDijkstra standard ",dj)
print("________________________________________________")
print("\n# Hops [Dijkstra standard] ",dj.__len__())
print("\nPath Weight [Dijkstra standard] {/10^7}",nx.dijkstra_path_length(c_graph, iridium[i], iridium[j])/10000000)
print("________________________________________________")
#d_list = nx.dijkstra_path(c_graph, iridium[i], iridium[j])



#print("Indice ", ind, "Update: ", update, "Sorgente ", iridium[i], "Destinazione ", iridium[j], "Shortest Path ", d_list)
print("\nLoop\n ")

update = False
while time.time()-now < 60:
    ind = getInd(time.time(),now)
    if 0 < (time.time()-now)%10 < 00.1 and ind != pre_ind and ind != 1:
        update = True
        print("\nIndice ", ind, "Update: ", update, "Tempo ", time.time()-now)
        route_list.clear()
        excluded_nodes.clear()
        nhop = 0
        path_weight = 0
        myNode = i # cambiato il cp -> ricalcolo il percorso migliore ripartendo dalla sorgente
        c_graph = load_CP(update, ind)[0]
        #load_nb = load_CP(update, ind)[1]
        while myNode != dest: #dest non cambia mai
            #print("mynode ", myNode)
            route_list.append(myNode)
            excluded_nodes.append(myNode)

            # nb = load_nb[myNode]
            # for h in range(0,nb.size):
            # if nb[h] == pre_node:
            # print("Sono nel for, nell'if; nb[h]=",nb[h],"prenode ",pre_node)
            # nb.pop(h)

            #-print("Carico le rl dei nodi adiacenti a  ", iridium[myNode])
            rl = getRl(c_graph, myNode, j, excluded_nodes)[0]
            rl_weight = getRl(c_graph, myNode, j, excluded_nodes)[1]
            nb = getRl(c_graph, myNode, j, excluded_nodes)[2]

            #for k in range(0, rl_weight.__len__()):
             #   print("Peso del percorso che inizia da ", nb[k], rl_weight[k])

            min_rl = nb[rl_weight.index(min(rl_weight))]
            path_weight = path_weight + min(rl_weight)
            #-print("Percorso con costo minore (indice) ", min_rl)

            # pre_node = myNode
            myNode = min_rl

        print("\nRoute list dell'inidce ", ind)
        for q in range(0, route_list.__len__()):
            print("Step ", q, " ", iridium[route_list[q]])
            nhop = nhop + 1
        print("________________________________________________")
        print("\n# Hops ", nhop)
        print("\n Path Weight [cgr] {/10^7} ", path_weight / 10000000)
        print("________________________________________________")
        dj = nx.dijkstra_path(c_graph, iridium[i], iridium[j])
        print("\nDijkstra standard ", dj)
        print("________________________________________________")
        print("\n# Hops Dijjkstra standard ", dj.__len__())
        print("\nPath Weight [Dijkstra standard] {/10^7}", nx.dijkstra_path_length(c_graph, iridium[i], iridium[j]) / 10000000)
        print("________________________________________________")
        update = False
        pre_ind = ind
        ind = ind+1
