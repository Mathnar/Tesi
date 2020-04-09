

##################################################################################################################################
##################################################################################################################################

import math

import pyproj

coords = [
  (37.4001100556,  -79.1539111111,  208.38),
  (37.3996955278,  -79.153841,  208.48),
  (37.3992233889,  -79.15425175,  208.18),
  (37.3989114167,  -79.1532775833,  208.48),
  (37.3993285556,  -79.1533773333,  208.28),
  (37.3992801667,  -79.1537883611,  208.38),
  (37.3992441111,  -79.1540981944,  208.48),
  (37.3992616389,  -79.1539428889,  208.58),
  (37.3993530278,  -79.1531711944,  208.28),
  (37.4001223889,  -79.1538085556,  208.38),
  (37.3992922222,  -79.15368575,  208.28),
  (37.3998074167,  -79.1529132222,  208.18),
  (37.400068,  -79.1542711389,  208.48),
  (37.3997516389,  -79.1533794444,  208.38),
  (37.3988933333,  -79.1534320556,  208.38),
  (37.3996279444,  -79.154401,  208.58),
]

def gps_to_ecef_pyproj(lat, lon, alt):
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, lon, lat, alt, radians=False)

    return x, y, z

def gps_to_ecef_custom(lat, lon, alt):
    rad_lat = lat * (math.pi / 180.0)
    rad_lon = lon * (math.pi / 180.0)

    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    v = a / math.sqrt(1 - e2 * math.sin(rad_lat) * math.sin(rad_lat))

    x = (v + alt) * math.cos(rad_lat) * math.cos(rad_lon)
    y = (v + alt) * math.cos(rad_lat) * math.sin(rad_lon)
    z = (v * (1 - e2) + alt) * math.sin(rad_lat)

    return x, y, z

def run_test():

    for pt in coords:
        print('pyproj', gps_to_ecef_pyproj(pt[0], pt[1], pt[2]))
        print('custom', gps_to_ecef_custom(pt[0], pt[1], pt[2]))


run_test()