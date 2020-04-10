
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