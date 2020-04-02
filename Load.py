from skyfield.api import Topos, load
import math
from obspy.geodetics import degrees2kilometers
from pygeodesy import compassAngle


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

print('/')


sub96i = ir96pos.subpoint()
print('latitudine ', sub96i.latitude)
print('longitudine ', sub96i.longitude)
print('altitudine ', sub96i.elevation.km)

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

for i in range(0,32):
    print("Iridium", iridium[i], iridPos[i].position.km)


print("")
print("")

for i in range(0,32):
    irSubP.append(iridPos[i].subpoint())

for i in range(0,32):                           #splitto latitudine
    tpLt = str(irSubP[i].latitude).split(' ')
    tpLg = str(irSubP[i].longitude).split(' ')
    tpAl = str(irSubP[i].elevation).split(' ')

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


    #irLon.append(float(deLon[0]) + ((float(prLon))/60) + ((float(seLon[0]))/3600))



for i in range(0,32):
    print(irLat[i])

print("")          #funge
print(degrees2kilometers(1))


kk = compassAngle(irLat[0],irLon[0],irLat[1],irLon[1])
print("Lat1 ",irLat[0], "Lon1 ", irLon[0], "Lat2 ", irLat[1], "Lon2 ",irLon[1], "Angolo ",kk)




# Distance 3d
def distance(x1, y1, z1, x2, y2, z2):
    d = math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2) * 1.0)


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


vertices = []
vertices_no= 0
graph = []
cost = 0

for i in range(0,32):
    add_vertex(iridium[i])


#for i in range(0,32):
    #print(vertices[i])

for i in range(0,32):
    for j in range(0,32):
        add_edge(vertices[i], vertices[j], j)

#print_graph()
#print("Internal representation: ", graph)



#sub2i = ir2pos.subpoint()
#print('Distanza satelliti considerati')
#distance(tasub.latitude.km, tbsub.latitude.km, tasub.longitude.km, tbsub.longitude.km, tasub.elevation.km, tbsub.elevation.km)

#days = t - ir96.epoch
#print('{:.3f} days away from epoch'.format(days))

#print(satellite.epoch.utc_jpl())

#ts = load.timescale()



#geodesy ritorna angolo tra due punti dal centro della terra
