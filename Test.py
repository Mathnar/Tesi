def makecubelimits(axis, centers=None, hw=None):
    lims = ax.get_xlim(), ax.get_ylim(), ax.get_zlim()
    if centers == None:
        centers = [0.5*sum(pair) for pair in lims]

    if hw == None:
        widths  = [pair[1] - pair[0] for pair in lims]
        hw      = 0.5*max(widths)
        ax.set_xlim(centers[0]-hw, centers[0]+hw)
        ax.set_ylim(centers[1]-hw, centers[1]+hw)
        ax.set_zlim(centers[2]-hw, centers[2]+hw)
        print("hw was None so set to:", hw)
    else:
        try:
            hwx, hwy, hwz = hw
            print("ok hw requested: ", hwx, hwy, hwz)

            ax.set_xlim(centers[0]-hwx, centers[0]+hwx)
            ax.set_ylim(centers[1]-hwy, centers[1]+hwy)
            ax.set_zlim(centers[2]-hwz, centers[2]+hwz)
        except:
            print("nope hw requested: ", hw)
            ax.set_xlim(centers[0]-hw, centers[0]+hw)
            ax.set_ylim(centers[1]-hw, centers[1]+hw)
            ax.set_zlim(centers[2]-hw, centers[2]+hw)

    return centers, hw

TLE = """1 43205U 18017A   18038.05572532 +.00020608 -51169-6 +11058-3 0  9993
2 43205 029.0165 287.1006 3403068 180.4827 179.1544 08.75117793000017"""
L1, L2 = TLE.splitlines()

from skyfield.api import Loader, EarthSatellite
from skyfield.timelib import Time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

halfpi, pi, twopi = [f*np.pi for f in (0.5, 1, 2)]
degs, rads = 180/pi, pi/180

load = Loader('~/Documents/fishing/SkyData')
data = load('de421.bsp')
ts   = load.timescale()

planets = load('de421.bsp')
earth   = planets['earth']

Roadster = EarthSatellite(L1, L2)

print(Roadster.epoch.tt)
hours = np.arange(0, 3, 0.01)

time = ts.utc(2018, 2, 7, hours)

Rpos    = Roadster.at(time).position.km
Rposecl = Roadster.at(time).ecliptic_position().km

print(Rpos.shape)

re = 6378.

theta = np.linspace(0, twopi, 201)
cth, sth, zth = [f(theta) for f in (np.cos, np.sin, np.zeros_like)]
lon0 = re*np.vstack((cth, zth, sth))
lons = []
for phi in rads*np.arange(0, 180, 15):
    cph, sph = [f(phi) for f in (np.cos, np.sin)]
    lon = np.vstack((lon0[0]*cph - lon0[1]*sph,
                     lon0[1]*cph + lon0[0]*sph,
                     lon0[2]) )
    lons.append(lon)

lat0 = re*np.vstack((cth, sth, zth))
lats = []
for phi in rads*np.arange(-75, 90, 15):
    cph, sph = [f(phi) for f in (np.cos, np.sin)]
    lat = re*np.vstack((cth*cph, sth*cph, zth+sph))
    lats.append(lat)

if True:
    fig = plt.figure(figsize=[10, 8])  # [12, 10]

    ax  = fig.add_subplot(1, 1, 1, projection='3d')

    x, y, z = Rpos
    ax.plot(x, y, z)
    for x, y, z in lons:
        ax.plot(x, y, z, '-k')
    for x, y, z in lats:
        ax.plot(x, y, z, '-k')

    centers, hw = makecubelimits(ax)

    print("centers are: ", centers)
    print("hw is:       ", hw)

    plt.show()

r_Roadster = np.sqrt((Rpos**2).sum(axis=0))
alt_roadster = r_Roadster - re

if True:
    plt.figure()
    plt.plot(hours, r_Roadster)
    plt.plot(hours, alt_roadster)
    plt.xlabel('hours', fontsize=14)
    plt.ylabel('Geocenter radius or altitude (km)', fontsize=14)
    plt.show()










    ######################plot
    from mpl_toolkits.basemap import Basemap
    import matplotlib.pyplot as plt
    import imageio

    for i in range(0, 330, 20):
        my_map = Basemap(projection='ortho', lat_0=0, lon_0=i, resolution='l', area_thresh=1000.0)
        my_map.bluemarble()
        my_map.etopo()
        name = str(i)
        path = '/path/to/your/directory/' + name
        plt.savefig(path + '.png')
        plt.show()
        plt.clf()
        plt.cla()
        plt.close()




        ######        ######        ######        ######        ######        ######        ######        ######        ######        ######



        # tryy = rectEq(-1,2,-1,1,4,-1,1)
        # print("tryy ", tryy)

        # 5091082.5313572 : c1 = sym.Symbol('c1')

        def rectEq2(x1, y1, z1, x2, y2, z2, r):  # 1 = p ; 2 = q
            sym.init_printing()
            x, y, z = sym.symbols('x,y,z')
            t = sym.Symbol('t')
            vect = vectp(x1, y1, z1, x2, y2, z2)
            print(vect)
            if vect[0] != 0 and vect[1] != 0 and vect[2] != 0:
                print("Entro in caso l,m,n diversi da 0")
                a = sym.Eq((x1 + t * (x2 - x1)) ** 2 + (y1 + t * (y2 - x1)) ** 2 + (z1 + t * (z2 - z1)) ** 2, r ** 2)
                result = sym.solve([a], (x, y, z))
                return result


        doggo = rectEq2(5091082.5313572, 1013216.3395964877, 4943045.661622145, 4726826.686803615, 194164.32677549776,
                        5382534.598367236, 6378137)
        print("Doggo ", doggo)
        print("")
        pippo = rectEq2(5091082.5313572, 1013216.3395964877, 4943045.661622145, -2797274.5151061076, 5061417.945485286,
                        -4202378.6294492, 6378137)
        print("Pippo ", pippo)

        print("")
        pluto = rectEq(5091082.5313572, 1013216.3395964877, 4943045.661622145, -2797274.5151061076, 5061417.945485286,
                       -4202378.6294492, 6378137)
        print("Pluto ", pluto)



        #####        #####        #####        #####        #####        #####        #####        #####        #####        #####        #####        #####

        # punti da considerare: #
        # posizione satellite con x y z : xx[0],yy[0],zz[0]
        # posizio

        print("")
        print("")
        # print("altitudine[km] ",irAlt[0]/1000, "orizzonte[km] ", p.horizon(irAlt[0])/1000)

        a = np.array([[3, 1], [1, 2]])
        b = np.array([9, 8])
        x = np.linalg.solve(a, b)

        print(x)

        print("Lat ", irLat[29], "Lon ", irLon[29], "Alt ", irAlt[29], "Conversione in xyz -> ", "x: ", xx[29], "y: ",
              yy[29], "z: ", zz[29])
        print("Lat ", irLat[28], "Lon ", irLon[28], "Alt ", irAlt[28], "Conversione in xyz -> ", "x: ", xx[28], "y: ",
              yy[28], "z: ", zz[28])
        print("Lat ", irLat[20], "Lon ", irLon[20], "Alt ", irAlt[20], "Conversione in xyz -> ", "x: ", xx[20], "y: ",
              yy[20], "z: ", zz[20])

        flat = 43.769562
        flon = 11.255814
        flel = 800000

        plat = 48.856613
        plon = 2.352222
        plel = 800000

        print("")
        print("")
        fx, fy, fz = gps_to_ecef_pyproj(flat, flon, flel)
        print("Fx: ", fx, "Fy ", fy, "Fz ", fz, "GpsFx ", gps_to_ecef_pyproj(flat, flon, flel))
        px, py, pz = gps_to_ecef_pyproj(plat, plon, plel)
        print("Px: ", px, "Py ", py, "Pz ", pz, "GpsPx ", gps_to_ecef_pyproj(plat, plon, plel))
        print("")
        aa = gps_to_ecef_pyproj(1, 1, -6378137)
        print("zero ", aa)

        print("")
        print("")
        print("")
        print("")



        ###########################        ###########################        ###########################        ###########################        ###########################

        elif vect[0] == 0 and vect[1] != 0 and vect[2] != 0:  # l=0
        a = sym.Eq(x, x1)
        b = sym.Eq((y - y1) / vect[1], (z - z1) / vect[2])
        c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
        result = sym.solve([a, b, c], (x, y, z))
        return result
    elif vect[0] != 0 and vect[1] == 0 and vect[2] != 0:  # m=0
    a = sym.Eq(y, y1)
    b = sym.Eq((x - x1) / vect[0], (z - z1) / vect[1])
    c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
    result = sym.solve([a, b, c], (x, y, z))
    return result
elif vect[0] != 0 and vect[1] != 0 and vect[2] == 0:  # n=0
    a = sym.Eq(z, z1)
    b = sym.Eq((x - x1) / vect[0], (y - y1) / vect[1])
    c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
    result = sym.solve([a, b, c], (x, y, z))
    return result
elif vect[0] == 0 and vect[1] == 0 and vect[2] != 0:  # l=m=0
    a = sym.Eq(x, x1)
    b = sym.Eq(y, y1)
    c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
    result = sym.solve([a, b, c], (x, y, z))
    return result
elif vect[0] == 0 and vect[1] != 0 and vect[2] == 0:  # l=n=0
    a = sym.Eq(x, x1)
    b = sym.Eq(z, z1)
    c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
    result = sym.solve([a, b, c], (x, y, z))
    return result
elif vect[0] != 0 and vect[1] == 0 and vect[2] == 0:  # m=n=0
    a = sym.Eq(y, y1)
    b = sym.Eq(z, z1)
    c = sym.Eq(x ** 2 + y ** 2 + z ** 2, r ** 2)
    result = sym.solve([a, b, c], (x, y, z))
    return result