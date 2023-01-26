import math
import numpy as np

def llh2ecef(lat, lon, height):
    #Source: pynmeagps github helper functions
    #Identical to Vallado, pg 173
    #lat: float,
    #lon: float,
    #height: float
    #) -> tuple:
    #Convert lat lon to ECEF
    #lat lon in deg
    #height is alt in meters
    
    a = 6378137.0      #WGS84_SMAJ_AXIS,
    f = 298.257223563  #WGS84_FLATTENING,
    
    lat, lon = [c * math.pi / 180 for c in (lat,lon)]
    f = 1/f
    e2 = f * (2-f)
    a2 = a**2
    b2 = a2 * (1-e2)
    
    N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    x = (N + height) * math.cos(lat)*math.cos(lon)
    y = (N + height) * math.cos(lat)*math.sin(lon)
    z = ((b2 / a2)*N+height) * math.sin(lat)   
    
    return x,y,z

def ecef2ned(lat, lon):
    #lat: float
    #lon: float
    #) -> np.array:
    #return rotation matrix from ECEF to NED
    #lat lon in deg
    phi = lat * (math.pi/180)
    lam = lon * (math.pi/180)
    sp = math.sin(phi)
    sl = math.sin(lam)
    cp = math.cos(phi)
    cl = math.cos(lam)
    R = np.array([[-1*sp*cl, -1*sp*sl,    cp],
                  [-1*sl   ,    cl   ,     0],
                  [-1*cp*cl, -1*cp*sl, -1*sp]])
    return R