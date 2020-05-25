from skyfield.api import load
import numpy as np
import pyproj
import math
import sympy as sym

########################################################################################################################
def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)
    return x,y,z

def distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2) * 1.0)

#def writeTxt(grafo):
#    np.savetxt(f, grafo)




def closestSatTo_P(px,py,pz,when):
    stations_url = 'http://celestrak.com/NORAD/elements/iridium.txt'
    satellites = load.tle(stations_url, reload=True)
    t = when
    iridium = []
    c = 0
    k = False

    for sat in satellites:
        if k:
            iridium.append(satellites[sat])
            c = c + 1
        k = not k

    iridPos = []
    irSubP = []

    irLat = []
    irLon = []
    irAlt = []

    for i in range(0, 32):
        iridPos.append(iridium[i].at(t))

    for i in range(0, 32):
        irSubP.append(iridPos[i].subpoint())

    for i in range(0, 32):  # splitto latitudine
        tpLt = str(irSubP[i].latitude).split(' ')
        tpLg = str(irSubP[i].longitude).split(' ')
        deLat = str(tpLt[0]).split('d')
        prLat = str(tpLt[1])[:2]
        seLat = str(tpLt[2]).split('"')
        irLat.append(float(deLat[0]) + ((float(prLat)) / 60) + ((float(seLat[0])) / 3600))
        deLon = str(tpLg[0]).split('d')
        prLon = str(tpLg[1])[:2]
        seLon = str(tpLg[2]).split('"')
        irLon.append(float(deLon[0]) + ((float(prLon)) / 60) + ((float(seLon[0])) / 3600))
        irAlt.append(irSubP[i].elevation.m)

    tmpTr = []
    xx = []
    yy = []
    zz = []

    for i in range(0, 32):
        tmpTr.append(gps_to_ecef_pyproj(irLat[i], irLon[i], irAlt[i]))

    for i in range(0, 32):
        xx.append(round(tmpTr[i][0], 3))
        yy.append(round(tmpTr[i][1], 3))
        zz.append(round(tmpTr[i][2], 3))

    vertices = []

    for i in range(0, 32):
        vertices.append(iridium[i])

    jump = True
    min = iridium[0]
    min_d = distance(px, py, pz, xx[0], yy[0], zz[0])
    for i in range(0, 32):
        if not jump:
            tmp = distance(px, py, pz, xx[i], yy[i], zz[i])
            if tmp < min_d:
                min = iridium[i]
                min_d = tmp
        jump = False
    return min


########################################################################################################################
print("\n#########################EarthSatExe")
flo = gps_to_ecef_pyproj(43.769562,11.255814,0)
hkg = gps_to_ecef_pyproj(22.302711,114.177216,0)

flo_x = round(flo[0],3)
flo_y = round(flo[1],3)
flo_z = round(flo[2],3)

hkg_x = round(hkg[0],3)
hkg_y = round(hkg[1],3)
hkg_z = round(hkg[2],3)

ts = load.timescale()
start = ts.utc(2020, 5, 7, 00, 00, 00)
end = ts.utc(2020, 5, 7, 00, 1, 00)
source = closestSatTo_P(flo_x,flo_y,flo_z,start)
dest = closestSatTo_P(hkg_x,hkg_y,hkg_z,end)

print("Nodo sorgente rilevato: ",source)
print("Nodo destinazione rilevato: ",dest)
