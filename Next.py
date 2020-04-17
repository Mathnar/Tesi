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

stations_url = 'https://celestrak.com/NORAD/elements/iridium-NEXT.txt'
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

########################################################################################################################
######################################################################################################################## FUNCTIONS
def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)
    return x,y,z

def distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2) * 1.0)


# Add a vertex to the set of vertices and the graph
def add_vertex(v):
  global graph
  global vertices_no
  global vertices
  if v in vertices:
    print("Vertex ", v, " already exists")
  else:
    vertices_no = vertices_no + 1
    vertices.append(v)
    if vertices_no > 1:
        for vertex in graph:
            vertex.append(0)
    temp = []
    for i in range(vertices_no):
        temp.append(0)
    graph.append(temp)

# Add an edge between vertex v1 and v2 with edge weight e
def add_edge(v1, v2, e):
    global graph
    global vertices_no
    global vertices
    # Check if vertex v1 is a valid vertex
    if v1 not in vertices:
        print("Vertex ", v1, " does not exist.")
    # Check if vertex v1 is a valid vertex
    elif v2 not in vertices:
        print("Vertex ", v2, " does not exist.")
    # Since this code is not restricted to a directed or
    # an undirected graph, an edge between v1 v2 does not
    # imply that an edge exists between v2 and v1
    else:
        index1 = vertices.index(v1)
        index2 = vertices.index(v2)
        graph[index1][index2] = e

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


#def isVisible(x1,y1,z1,x2,y2,z2,r):
   # tmp = rectEq(x1,y1,z1,x2,y2,z2,r)
    #if sym.im(tmp[0][0])!=0 or sym.im(tmp[0][1])!=0 or sym.im(tmp[0][2])!=0 or sym.im(tmp[1][0])!=0 or sym.im(tmp[1][1])!=0 or sym.im(tmp[1][2])!=0:
    #    return False
    #else:
    #    return True

######################################################################################################################## FUNCTIONS
########################################################################################################################

print("")
print("")
print("Array")
print("")

iridPos = []
irSubP = []

irLat = []
irLon = []
irAlt = []

for i in range(0,75):
    iridPos.append(iridium[i].at(t))

#for i in range(0,75):
  #  print("Iridium", iridium[i], iridPos[i].subpoint().latitude)
   # print("Iridium", iridium[i], iridPos[i].subpoint().longitude)
  #  print("Iridium", iridium[i], iridPos[i].subpoint().elevation.km)

print("")
print("")

for i in range(0,75):
    irSubP.append(iridPos[i].subpoint())

for i in range(0,75):                           #splitto latitudine
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

#for i in range(0,75):
 #   print("latitudine di ", iridium[i], " -> ", irLat[i])
  #  print("longitudine di ", iridium[i], " -> ", irLon[i])
   # print("elevazione di ", iridium[i], " -> ", irAlt[i])

#latitude = (lat * math.pi) / 180

tmpTr = []

xx = []
yy = []
zz = []

print("")

for i in range(0,75):
    tmpTr.append(gps_to_ecef_pyproj(irLat[i],irLon[i],irAlt[i]))
    print("Altitudine di ",i," ",irAlt[i])

print("")

for i in range(0,75):
    xx.append(round(tmpTr[i][0],3))
    yy.append(round(tmpTr[i][1],3))
    zz.append(round(tmpTr[i][2],3))

print("")

#for i in range(0,75):
 #   print("x ", round(xx[i],3), "y ", round(yy[i],3), "z ", round(zz[i],3))



#CONVERTITE POSIZIONI IN XYZ IN XX YY ZZ

vertices = [] #riempio con add_vertex
vertices_no = 0 #aumento con add_vertex
graph = [] #riempio con add_vertex
cost = 0

for i in range(0,75):
    add_vertex(iridium[i])

#for i in range(0,75):
 #   print(vertices[i])

for i in range(0,75):
    for j in range(0,75):
       # print("Ciclo ",i,j)
        if i!=j:
            if (p.haversine_(irLat[i]*0.0174533,irLat[j]*0.0174533,(irLon[i]-irLon[j])*0.0174533)) < 0.785398:         #0.785398
               # print("Angolo minore di 45° ",(p.haversine_(irLat[i]*0.0174533,irLat[j]*0.0174533,(irLon[i]-irLon[j])*0.0174533)))
                if not isVisible(xx[i],yy[i],zz[i],xx[j],yy[j],zz[j]):
                 #   print("Inf ")
                    add_edge(vertices[i], vertices[j],math.inf)
                else:
                 #   print("Distance ")
                    add_edge(vertices[i], vertices[j], round(distance(xx[i], yy[i], zz[i], xx[j], yy[j], zz[j]),3))
            else:
               # print("Inf ")
                add_edge(vertices[i], vertices[j], math.inf)
        else:
          #  print("i=j ")
            add_edge(vertices[i], vertices[j],0)

print("")
print("")

print_graph()
print("Internal representation: ", graph)





