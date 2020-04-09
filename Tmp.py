# Distance 3d
def distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2) * 1.0)

difference = []
for i in range(0,32):
    temp = iridium[i] - obs
    difference.append((temp.at(t)).position.km)

for i in range(0,32):
    print(difference[i])


################################################################################################################
################################################################################################################


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
vertices_no = 0
graph = []
cost = 0

for i in range(0,32):
    add_vertex(iridium[i])


#for i in range(0,32):
    #print(vertices[i])

for i in range(0,32):
    for j in range(0,32):
        add_edge(vertices[i], vertices[j], distance(difference[i][0],difference[i][1],difference[i][2],difference[j][0],difference[j][1],difference[j][2]))

print("")

print_graph()
print("Internal representation: ", graph)



#print("")
#aa = p.haversine(irLat[0],irLon[0],irLat[1], irLon[1], radius=6371008.77141)

#print(aa)


#print("")          #funge
#print(degrees2kilometers(1))


#kk = compassAngle(irLat[0],irLon[0],irLat[1],irLon[1])
#print("Lat1 ",irLat[0], "Lon1 ", irLon[0], "Lat2 ", irLat[1], "Lon2 ",irLon[1], "Angolo ",kk)
