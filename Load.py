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
import sys as sys


import scipy as sc
from scipy.sparse.csgraph import dijkstra


stations_url = 'http://celestrak.com/NORAD/elements/iridium.txt'
satellites = load.tle(stations_url, reload=True)
ir7 = satellites['IRIDIUM 7 [-]']   #0
ir5 = satellites['IRIDIUM 5 [-]']   #1
ir4 = satellites['IRIDIUM 4 [-]']   #2
ir914 = satellites['IRIDIUM 914 [-]']   #3
ir16 = satellites['IRIDIUM 16 [-]']   #4
ir911 = satellites['IRIDIUM 911 [-]']   #5
ir920 = satellites['IRIDIUM 920 [-]']   #6
ir921 = satellites['IRIDIUM 921 [-]']   #7
ir26 = satellites['IRIDIUM 26 [-]']   #8
ir17 = satellites['IRIDIUM 17 [-]']   #9
ir22 = satellites['IRIDIUM 22 [-]']   #10
dm1 = satellites['DUMMY MASS 1 [-]']   #11
dm2 = satellites['DUMMY MASS 2 [-]']   #12
ir29 = satellites['IRIDIUM 29 [-]']   #13
ir33 = satellites['IRIDIUM 33 [-]']   #14
ir28 = satellites['IRIDIUM 28 [-]']   #15
ir36 = satellites['IRIDIUM 36 [-]']   #16
ir38 = satellites['IRIDIUM 38 [-]']   #17
ir39 = satellites['IRIDIUM 39 [-]']   #18
ir42 = satellites['IRIDIUM 42 [-]']   #19
ir44 = satellites['IRIDIUM 44 [-]']   #20
ir45 = satellites['IRIDIUM 45 [-]']   #21
ir24 = satellites['IRIDIUM 24 [-]']   #22
ir51 = satellites['IRIDIUM 51 [-]']   #23
ir57 = satellites['IRIDIUM 57 [-]']   #24
ir63 = satellites['IRIDIUM 63 [-]']   #25
ir69 = satellites['IRIDIUM 69 [-]']   #26
ir71 = satellites['IRIDIUM 71 [-]']   #27
ir73 = satellites['IRIDIUM 73 [-]']   #28
ir82 = satellites['IRIDIUM 82 [-]']   #29
ir2 = satellites['IRIDIUM 2 [-]']   #30
ir96 = satellites['IRIDIUM 96 [-]']   #31

iridium = [ir7, ir5, ir4, ir914, ir16, ir911, ir920, ir921, ir26, ir17, ir22, dm1, dm2, ir29, ir33, ir28, ir36,
           ir38, ir39, ir42, ir44, ir45, ir24, ir51, ir57, ir63, ir69, ir71, ir73, ir82, ir2, ir96]

obs = Topos('0 N', '0 W')
print('Satelliti considerati')
print(ir96)
print(ir2)
print()

ts = load.timescale(builtin=True)
t = ts.utc(2020, 4, 25, 00, 00, 00)   #utc(2020, 4, 16, 00, 00, 00)
print("Tempo ", ts.now())


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


def writeTxt(grafo):
    for i in range(0,32):
        for j in range(0, 32):
            np.savetxt("prova.txt", grafo)


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

for i in range(0,32):
    print("Appendo ir al tempo ", t)
    print("Ir ", iridium[i].at(t))
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
graph = [] #riempio con add_vertex
cost = 0

for i in range(0,32):
    add_vertex(iridium[i])

print("")

for i in range(0,32):
    print(vertices[i])
    print(t.utc_jpl())

for i in range(0,2):
    for j in range(0,2):
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






print(flo)
#writeTxt(graph)





