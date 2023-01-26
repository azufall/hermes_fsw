from gps import *
import time
import serial
import adafruit_bno055
import array
import math

#init ---------------------------------------------
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
uart = serial.Serial("/dev/ttyAMA0")
IMU = adafruit_bno055.BNO055_UART(uart)
#state is [LAT LON ALT V_VELO H_VELO HEADING ROTATION_RATE SERVO] 
state = array('d', [0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0])

lat = None
lon = None
alt = None
time = None
time_init = None
delta_t = None

alpha_mag = 0.7
alpha_gyr = 0.7
mag_gain = array('d', [1.0 1.0 1.0])
gyr_gain = array('d', [1.0 1.0 1.0])
mag_off  = array('d', [0.0 0.0 0.0])
gyr_off  = array('d', [0.0 0.0 0.0])
mag = None
gyr = None
mag_cali = array('d', [0.0 0.0 0.0])
gyr_cali = array('d', [0.0 0.0 0.0])

servo = 0.0
ctl_mode = 3
kp = 0.01  ###TODO: update with realistic values
kd = 0.20
#waypoint structure: time (s), lat (deg.ddddd), lon (deg.ddddd), alt (m)
waypoints = array('d', [[  0.0 38.55278 -121.71326 15] ...
                        [ 60.0 38.55278 -121.71271 15] ...
                        [120.0 38.55220 -121.71348 15] ...
                        [180.0 38.55216 -121.71267 15]])
#convert waypoints to ECEF
for i in range(4):
    (x,y,z) = llh2ecef(lat=array[i][1], lon=array[i][2], height=array[i][3])
    waypoints[i][1] = x
    waypoints[i][2] = y
    waypoints[i][3] = z
    
num_waypoints = 4
outer = 0.000135
inner = 0.000090
###TODO: convert limits to feet
#pi = 3.1415926536

file_name = "GPS_test_0.txt"
file = open(file_name, "w")  #append data to txt file
file.write("Lat,Lon,Ctl_mode, etc...") #TODO: decide data log structure
file.close()

count = 0
while count < 100

    #read sensors -------------------------------------
    mag_raw = IMU.magnetic     #3-tuple, in uT
    gyr_raw = IMU.gyro         #3-tuple, in deg/s

    #TODO: restructure this so we get valid data each time.
    #TODO: manage so buffer doesn't fill up
    try:
        report = gpsd.next()
        if report['class'] == 'TPV':     
            lat = getattr(report,'lat',0.0)
            lon = getattr(report,'lon',0.0)
            alt = getattr(report,'alt',0.0)
            time = getattr(report,'time','')
            v_velo = getattr(report,'climb')
            h_velo = 5.0  #TODO find attribute for horizontal velocity and calculate it
            #print(getattr(report,'lat',0.0),"\t")
            #print(getattr(report,'lon',0.0),"\t")
            #print(getattr(report,'time',''),"\t")
            #print(getattr(report,'alt','nan'),"\t\t")
            #print(getattr(report,'climb','nan'),"\t")
        time.sleep(0.05)

    #sensor processing -------------------------------
    if time_init is None:
        time_init = time
    else:
        delta_t = time - time_init
    

    for i in range(3):
        mag_cali[i] = mag_gain[i] * (mag_raw[i] - mag_offset[i])
    if mag is not None:
        mag = alpha_mag*mag_cali + (1-alpha_mag)*mag
    #    mag_prev = mag
    else:
        mag = mag_raw
    #    mag_prev = mag

    for i in range(3):
        gyr_cali[i] = gyr_gain[i] * (gyr_raw[i] - gyr_offset[i])
    if gyr_prev is not None:
        gyr = alpha_gyr*gyr_cali + (1-alpha_gyr)*gyr
    #    gyr_prev = gyr
    else:
        gyr = gyr_raw
    #    gyr_prev = gyr

    gps_valid = (getattr(report,'lat',0.0) > 20.0)  #we know we will be in the northern hemisphere
    heading = math.atan2(mag[1] / mag[0])
    heading_rate = gyr[2]
    #TODO: adjust heading with GPS info

    #calculate state ------------------------------------------------
    ###TODO: implement Kalman filter or similar
    state[0] = lat
    state[1] = lon
    state[2] = alt
    state[3] = v_velo
    state[4] = h_velo
    state[5] = heading
    state[6] = heading_rate
    state[7] = servo

    #determine waypoint --------------------------------------------------
    if delta_t is not None:
        for i in range(num_waypoints):
            if delta_t > waypoints[i][0]:
                wypt_x = waypoints[i][1]
                wypt_y = waypoinys[i][2]
    else:
        wypt_x = 0
        wypt_y = 0
            
    #control ----------------------------------------------------------
    if not gps_valid:
        ctl_mode = 3   #spiral
        h_error = 0
        hd_error = (360 * hori_velo) / (2 * outer * math.pi) - heading_rate
    else:
        dx = wypt_x - lat
        dy = wypt_y - lon  ###TODO: convert to feet
        dist = math.sqrt(dx^2+dy^2)
        if dist > outer:
            ctl_mode = 1  #track
            h_error = (180/pi) * math.atan2(dy/dx) - heading
            hd_error = -1 * heading_rate
        else if dist < inner:
            ctl_mode = 2  #orbit
            h_error = (180/pi) * math.atan2(dy/dx) - heading - 80 #TODO: see if a value closer to 90 is better
            hd_error = (360 * hori_velo) / (2 * dist * math.pi) - heading_rate
    servo = kp * h_error + kd * hd_error #TODO: update this to be around servo center value

    #TODO: command servo


    #log data -------------------------------------------------------------
    file = open(file_name, "a")  #append data to txt file
    file.write()  #TODO: write data to file
    file.close()
    
    count += 1
    
    
    
    
def llh2ecef(
    #Source: pynmeagps github helper functions
    #Identical to Vallado, pg 173
    lat: float,
    lon: float,
    height: float
    a: float = 6378137.0 #WGS84_SMAJ_AXIS,
    f: float = 298.257223563 #WGS84_FLATTENING,
    ) -> tuple:
    #Convert lat lon to ECEF.
    #lat lon in deg
    # height is alt in meters
    
    lat, lon = [c * pi / 180 for c in (lat,lon)]
    f = 1/f
    e2 = f * (2-f)
    a2 = a**2
    b2 = a2 * (1-e2)
    
    N = a / sqrt(1 - e2 * sin(lat) ** 2)
    x = (N + height) * cos(lat)*cos(lon)
    y = (N + height) * cos(lat)*sin(lon)
    z = ((b2 / a2)*N+height) * sin(lat)   
    
    return x,y,z

#TODO: write ECEF to NED conversion
def ecef2ned()