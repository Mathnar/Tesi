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
satellites = load.tle(stations_url)
ir7 = satellites['IRIDIUM 7 [-]']
ir5 = satellites['IRIDIUM 5 [-]']
ir4 = satellites['IRIDIUM 4 [-]']
ir914 = satellites['IRIDIUM 914 [-]']
ir16 = satellites['IRIDIUM 16 [-]']
ir911 = satellites['IRIDIUM 911 [-]']
ir920 = satellites['IRIDIUM 920 [-]']
ir921 = satellites['IRIDIUM 921 [-]']
ir26 = satellites['IRIDIUM 26 [-]']
ir17 = satellites['IRIDIUM 17 [-]']
ir22 = satellites['IRIDIUM 22 [-]']
dm1 = satellites['DUMMY MASS 1 [-]']
dm2 = satellites['DUMMY MASS 2 [-]']
ir29 = satellites['IRIDIUM 29 [-]']
ir33 = satellites['IRIDIUM 33 [-]']
ir28 = satellites['IRIDIUM 28 [-]']
ir36 = satellites['IRIDIUM 36 [-]']
ir38 = satellites['IRIDIUM 38 [-]']
ir39 = satellites['IRIDIUM 39 [-]']
ir42 = satellites['IRIDIUM 42 [-]']
ir44 = satellites['IRIDIUM 44 [-]']
ir45 = satellites['IRIDIUM 45 [-]']
ir24 = satellites['IRIDIUM 24 [-]']
ir51 = satellites['IRIDIUM 51 [-]']
ir57 = satellites['IRIDIUM 57 [-]']
ir63 = satellites['IRIDIUM 63 [-]']
ir69 = satellites['IRIDIUM 69 [-]']
ir71 = satellites['IRIDIUM 71 [-]']
ir73 = satellites['IRIDIUM 73 [-]']
ir82 = satellites['IRIDIUM 82 [-]']
ir2 = satellites['IRIDIUM 2 [-]']
ir96 = satellites['IRIDIUM 96 [-]']

iridium = [ir7, ir5, ir4, ir914, ir16, ir911, ir920, ir921, ir26, ir17, ir22, dm1, dm2, ir29, ir33, ir28, ir36,
           ir38, ir39, ir42, ir44, ir45, ir24, ir51, ir57, ir63, ir69, ir71, ir73, ir82, ir2, ir96]

obs = Topos('0 N', '0 W')
print('Satelliti considerati')
print(ir96)
print(ir2)
print()

ts = load.timescale()
t = ts.now()

print("Prova con un satellite")
print("")

ir96pos = ir96.at(t)

print()
print('Posizioni satelliti considerati')
print(ir96pos.position.km)
print(ir96pos.position)

print('/')


sub96i = ir96pos.subpoint()
print('latitudine ', sub96i.latitude)
print('longitudine ', sub96i.longitude)
print('altitudine ', sub96i.elevation.km)#risp alla crosta - se ci sommo raggio terrestre ho la distanza dal centro della terra

x = str(sub96i.latitude).split(' ')
print(x)

tmp = str(x[0]).split('d')
deg = tmp[0]
print("deg", deg)

pri = str(x[1])[:2]
print("pri", pri)

tmp1 = str(x[2]).split('"')
sec = tmp1[0]
print("sec", sec)


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


def vectp(x1,y1,z1,x2,y2,z2):
    vect = []
    l = x2-x1
    m = y2-y1
    n = z2-z1
    vect.append(l)
    vect.append(m)
    vect.append(n)
    return vect

print("")
def rectEq(x1,y1,z1,x2,y2,z2,r):    #1 = p ; 2 = q                      l = vect[0] ; m = vect[1] ; n = vect[2]
    sym.init_printing()
    x,y,z = sym.symbols('x,y,z')
    vect = vectp(x1, y1, z1, x2, y2, z2)
    if vect[0] !=0 and vect[1] !=0 and vect[2] !=0:#tutti diversi da 0
        a = sym.Eq((x-x1)/vect[0],(y-y1)/vect[1])
        b = sym.Eq((y-y1)/vect[1],(z-z1)/vect[2])
        tmp = sym.solve([a, b], (x, y, z))
        d,e,f = 0,0,0
        if tmp.get(x) is not None:
            d = sym.Eq(x,tmp.get(x))
        if tmp.get(y) is not None:
            e = sym.Eq(y, tmp.get(y))
        if tmp.get(z) is not None:
            f = sym.Eq(z, tmp.get(z))
        c = sym.Eq(x**2+y**2+z**2,r**2)
        result = sym.solve([d,e,f,c],(x,y,z))
        return result


def isVisible(x1,y1,z1,x2,y2,z2,r):
    tmp = rectEq(x1,y1,z1,x2,y2,z2,r)
    if sym.im(tmp[0][0])!=0 or sym.im(tmp[0][1])!=0 or sym.im(tmp[0][2])!=0 or sym.im(tmp[1][0])!=0 or sym.im(tmp[1][1])!=0 or sym.im(tmp[1][2])!=0:
        return False
    else:
        return True

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
    print("Altitudine ",irAlt[i])

print("")

for i in range(0,32):
    xx.append(tmpTr[i][0])
    yy.append(tmpTr[i][1])
    zz.append(tmpTr[i][2])

print("")
#print("Lat ", irLat[0], "Lon ", irLon[0], "Alt ", irAlt[0], "Conversione in xyz -> ", "x: ", xx[0],"y: ", yy[0],"z: ", zz[0] )

#CONVERTITE POSIZIONI IN XYZ IN XX YY ZZ

vertices = [] #riempio con add_vertex
vertices_no = 0 #aumento con add_vertex
graph = [] #riempio con add_vertex
cost = 0

for i in range(0,32):
    add_vertex(iridium[i])

for i in range(0,32):
    print(vertices[i])

for i in range(0,32):
    for j in range(0,32):
        if i!=j:
            if not isVisible(xx[i],yy[i],zz[i],xx[j],yy[j],zz[j],6378137):
                add_edge(vertices[i], vertices[j],math.inf)
            else:
                add_edge(vertices[i], vertices[j], distance(xx[i], yy[i], zz[i], xx[j], yy[j], zz[j]))
        else:
            add_edge(vertices[i], vertices[j],0)

print("")
print("")

print_graph()
print("Internal representation: ", graph)





