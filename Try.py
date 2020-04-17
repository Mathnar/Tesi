from collections import deque, namedtuple
from skyfield.api import Topos, load
import math
import numpy as np
import pyproj
import math
import sympy as sym
from obspy.geodetics import degrees2kilometers
#from pygeodesy import compassAngle
import pygeodesy as p
from collections import deque, namedtuple

stations_url = 'http://celestrak.com/NORAD/elements/iridium.txt'
satellites = load.tle(stations_url, reload=True)
ts = load.timescale()
t = ts.now()
iridium = []
c = 0
k = False

for sat in satellites:
    if k:
        iridium.append(satellites[sat])
        #print(sat)
        #print("")
        c=c+1
    k = not k



inf = float('inf')
Edge = namedtuple('Edge', 'start, end, cost')


def make_edge(start, end, cost=1):
  return Edge(start, end, cost)


class Graph:
    def __init__(self, edges):
        # let's check that the data is right
        wrong_edges = [i for i in edges if len(i) not in [2, 3]]
        if wrong_edges:
            raise ValueError('Wrong edges data: {}'.format(wrong_edges))

        self.edges = [make_edge(*edge) for edge in edges]

    @property
    def vertices(self):
        return set(
            sum(
                ([edge.start, edge.end] for edge in self.edges), []
            )
        )

    def get_node_pairs(self, n1, n2, both_ends=True):
        if both_ends:
            node_pairs = [[n1, n2], [n2, n1]]
        else:
            node_pairs = [[n1, n2]]
        return node_pairs

    def remove_edge(self, n1, n2, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        edges = self.edges[:]
        for edge in edges:
            if [edge.start, edge.end] in node_pairs:
                self.edges.remove(edge)

    def add_edge(self, n1, n2, cost=1, both_ends=True):
        node_pairs = self.get_node_pairs(n1, n2, both_ends)
        for edge in self.edges:
            if [edge.start, edge.end] in node_pairs:
                return ValueError('Edge {} {} already exists'.format(n1, n2))

        self.edges.append(Edge(start=n1, end=n2, cost=cost))
        if both_ends:
            self.edges.append(Edge(start=n2, end=n1, cost=cost))

    @property
    def neighbours(self):
        neighbours = {vertex: set() for vertex in self.vertices}
        for edge in self.edges:
            neighbours[edge.start].add((edge.end, edge.cost))
        return neighbours

    def dijkstra(self, source, dest):
        assert source in self.vertices, 'Such source node doesn\'t exist'
        distances = {vertex: inf for vertex in self.vertices}
        previous_vertices = {
            vertex: None for vertex in self.vertices
        }
        distances[source] = 0
        vertices = self.vertices.copy()

        while vertices:
            current_vertex = min(
                vertices, key=lambda vertex: distances[vertex])
            vertices.remove(current_vertex)
            if distances[current_vertex] == inf:
                break
            for neighbour, cost in self.neighbours[current_vertex]:
                alternative_route = distances[current_vertex] + cost
                if alternative_route < distances[neighbour]:
                    distances[neighbour] = alternative_route
                    previous_vertices[neighbour] = current_vertex

        path, current_vertex = deque(), dest
        while previous_vertices[current_vertex] is not None:
            path.appendleft(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        if path:
            path.appendleft(current_vertex)
        return path


# Print the graph
def print_graph():
  global graph
  global vertices_no
  for i in range(vertices_no):
    for j in range(vertices_no):
      if graph[i][j] != 0:
        print(vertices[i], " -> ", vertices[j], " edge weight: ", graph[i][j])


#def vectp(x1,y1,z1,x2,y2,z2):
  #  vect = []
   # l = x2-x1
    #m = y2-y1
    #n = z2-z1
    #vect.append(l)
    #vect.append(m)
    #vect.append(n)
    #return vect

print("")
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


def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)
    return x,y,z

def distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2) * 1.0)



graph = Graph([
    ("a", "b", 7),  ("a", "c", 90),  ("a", "f", 14), ("b", "c", 10),
    ("b", "d", 15), ("c", "d", 11), ("c", "f", 2),  ("d", "e", 6),
    ("e", "f", 9)])

#start, end, cost
print(graph.dijkstra("a", "e"))



print("")
print("")
print("Array")
print("")

iridPos = []
irSubP = []

irLat = []
irLon = []
irAlt = []

for i in range(0,32):
    iridPos.append(iridium[i].at(t))

#for i in range(0,32):
  #  print("Iridium", iridium[i], iridPos[i].subpoint().latitude)
   # print("Iridium", iridium[i], iridPos[i].subpoint().longitude)
  #  print("Iridium", iridium[i], iridPos[i].subpoint().elevation.km)

print("")
print("")

for i in range(0,32):
    irSubP.append(iridPos[i].subpoint())

for i in range(0,32):                           #splitto latitudine
    tpLt = str(irSubP[i].latitude).split(' ')
    tpLg = str(irSubP[i].longitude).split(' ')

                                                #estrapolo latitudine in gradi primi e secondi
    deLat = str(tpLt[0]).split('d')
    prLat = str(tpLt[1])[:2]
    seLat = str(tpLt[2]).split('"')

    # sommo gradi con primi e secondi opportunamente trasformati e aggiungo il dato sull'array irLat

    irLat.append(float(deLat[0]) + ((float(prLat))/60) + ((float(seLat[0]))/3600))

                                                # estrapolo longitudine in gradi primi e secondi
    deLon = str(tpLg[0]).split('d')
    prLon = str(tpLg[1])[:2]
    seLon = str(tpLg[2]).split('"')

    irLon.append(float(deLon[0]) + ((float(prLon))/60) + ((float(seLon[0]))/3600))

                                                # estrapolo altitudine

    irAlt.append(irSubP[i].elevation.m)

#for i in range(0,32):
 #   print("latitudine di ", iridium[i], " -> ", irLat[i])
  #  print("longitudine di ", iridium[i], " -> ", irLon[i])
   # print("elevazione di ", iridium[i], " -> ", irAlt[i])

#latitude = (lat * math.pi) / 180

tmpTr = []

xx = []
yy = []
zz = []

print("")

for i in range(0,32):
    tmpTr.append(gps_to_ecef_pyproj(irLat[i],irLon[i],irAlt[i]))
  #  print("Altitudine di ",i," ",irAlt[i])

print("")

for i in range(0,32):
    xx.append(round(tmpTr[i][0],3))
    yy.append(round(tmpTr[i][1],3))
    zz.append(round(tmpTr[i][2],3))

print("")

#for i in range(0,32):
 #   print("x ", round(xx[i],3), "y ", round(yy[i],3), "z ", round(zz[i],3))

print("")
aa = p.haversine(irLat[0],irLon[0],irLat[1], irLon[1], radius=6378137)
cc = p.haversine_(irLat[0]*0.01745,irLat[1]*0.01745,(irLon[0]-irLon[1])*0.01745)*10
dd = p.haversine_(irLat[0]*0.01745,irLat[1]*0.01745,(irLon[0]-irLon[1])*0.01745)*100
bb = p.haversine_(irLat[0]*0.01745,irLat[1]*0.01745,(irLon[0]-irLon[1])*0.01745)

ee = p.haversine_(irLat[0]*0.01745,irLat[2]*0.01745,(irLon[0]-irLon[2])*0.01745)*100
ff = p.haversine_(irLat[0]*0.01745,irLat[3]*0.01745,(irLon[0]-irLon[3])*0.01745)*100

print("")
print("")
print("0->1 ","Lat0 ", irLat[0],"Lat1 ", irLat[1],"Lon 0-1 ", irLon[0]-irLon[1],"Haversin Angolo ",bb,"Haversin Angolo*10 ",cc,"Haversin Angolo*100 ",dd)
print("0->2 ","Lat0 ", irLat[0],"Lat1 ", irLat[2],"Lon 0-1 ", irLon[0]-irLon[2],"Haversin Angolo ",ee)
print("0->3 ","Lat0 ", irLat[0],"Lat1 ", irLat[3],"Lon 0-1 ", irLon[0]-irLon[3],"Haversin Angolo ",ff)
print("")
print("")
print("Lat0 ", irLat[0], "Lon0 ", irLon[0], "Alt ", 0, "Conversione in xyz -> ", "x: ", xx[0],"y: ", yy[0],"z: ", zz[0] )
print("Lat1 ", irLat[1], "Lon1 ", irLon[1], "Alt ", 0, "Conversione in xyz -> ", "x: ", xx[1],"y: ", yy[1],"z: ", zz[1] )
print("Lat2 ", irLat[2], "Lon2 ", irLon[2], "Alt ", 0, "Conversione in xyz -> ", "x: ", xx[2],"y: ", yy[2],"z: ", zz[2] )
print("Lat3 ", irLat[3], "Lon3 ", irLon[3], "Alt ", 0, "Conversione in xyz -> ", "x: ", xx[3],"y: ", yy[3],"z: ", zz[3] )
#print("Lat0 ", irLat[0], "Lon0 ", irLon[0], "Alt ", irAlt[0], "Conversione in xyz -> ", "x: ", xx[0],"y: ", yy[0],"z: ", zz[0] )
#print("Lat1 ", irLat[1], "Lon1 ", irLon[1], "Alt ", irAlt[1], "Conversione in xyz -> ", "x: ", xx[1],"y: ", yy[1],"z: ", zz[1] )
print("QUA ",aa,)


print("")



#CONVERTITE POSIZIONI IN XYZ IN XX YY ZZ

vertices = [] #riempio con add_vertex
vertices_no = 0 #aumento con add_vertex
graph = Graph #riempio con add_vertex
cost = 0


for i in range(0,1):
    for j in range(0,32):
       # print("Ciclo ",i,j)
        if i!=j:
            if (p.haversine_(irLat[i]*0.0174533,irLat[j]*0.0174533,(irLon[i]-irLon[j])*0.0174533)) < 0.785398:         #0.785398
               # print("Angolo minore di 45° ",(p.haversine_(irLat[i]*0.0174533,irLat[j]*0.0174533,(irLon[i]-irLon[j])*0.0174533)))
                if not isVisible(xx[i],yy[i],zz[i],xx[j],yy[j],zz[j]):
                    graph.add_edge(vertices[i], vertices[j], math.inf)
                 #   print("Inf ")

                else:
                    graph.add_edge(vertices[i], vertices[j], round(distance(xx[i], yy[i], zz[i], xx[j], yy[j], zz[j]),3))
                 #   print("Distance ")
            else:
                graph.add_edge(vertices[i],vertices[j],math.inf)
               # print("Inf ")
        else:
            graph.add_edge(vertices[i],vertices[j],0)
          #  print("i=j ")

print("")
print("")

print_graph()
print("Internal representation: ", graph)
print("")
print("")
graph.dijkstra(vertices[0], vertices[2])
graph.dijkstra("a","e")


