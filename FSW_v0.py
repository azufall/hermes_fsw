from gps import *
import time
#import serial
import board
import busio
import adafruit_bno055
#import array
import math
import numpy as np
import csv
from fsw_helpers import *

#init ---------------------------------------------
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
i2c = busio.I2C(board.SCL, board.SDA)
IMU = adafruit_bno055.BNO055_I2C(i2c)
#state is [LAT(deg), LON(deg), ALT(m), V_VELO(m/s), H_VELO(m/s), HEADING(deg), ROTATION_RATE(deg/s), SERVO(deg)] 
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

lat = None
lon = None
alt = None
time_init = None
delta_t = None
gps_valid = False
gps_time = None
v_velo = 0
h_velo = 0

alpha_mag = 0.7
alpha_gyr = 0.7
mag_gain = np.array([1.0, 1.0, 1.0])
gyr_gain = np.array([1.0, 1.0, 1.0])
mag_offset = np.array([0.0, 0.0, 0.0])
gyr_offset = np.array([0.0, 0.0, 0.0])
mag = None
gyr = None
mag_cali = np.array([0.0, 0.0, 0.0])
gyr_cali = np.array([0.0, 0.0, 0.0])

servo = 0.0
ctl_mode = 3
kp = 0.01  ###TODO: update with realistic values
kd = 0.20
#waypoint structure: time (s), lat (deg.ddddd), lon (deg.ddddd), alt (m)
waypoints = np.array([[  0.0, 38.55278, -121.71326, 15], 
                      [ 60.0, 38.55278, -121.71271, 15], 
                      [120.0, 38.55220, -121.71348, 15], 
                      [180.0, 38.55216, -121.71267, 15]])
#convert waypoints to ECEF
for i in range(4):
    (x,y,z) = llh2ecef(lat=waypoints[i][1], lon=waypoints[i][2], height=waypoints[i][3])
    waypoints[i][1] = x
    waypoints[i][2] = y
    waypoints[i][3] = z
    
num_waypoints = 4
outer = 15.0
inner =  5.0

file_name = "GPS_test_2.csv"
header = ['Count','GPS_time','DeltaT','Lat','Lon','Ctl_mode','Heading','Heading_rate','Heading_error','Heading_rate_erro','Servo']
with open(file_name, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)

#file = open(file_name, "w")  #append data to txt file
#file.write("Count,GPS_time,Lat,Lon,Ctl_mode,Heading,Heading_rate,Heading_error,Heading_rate_erro,Servo")
#file.close()

print("init done")

count = 0
while count < 300:

    #read sensors -------------------------------------
    mag_raw = IMU.magnetic     #3-tuple, in uT
    gyr_raw = IMU.gyro         #3-tuple, in deg/s

    #TODO: restructure this so we get valid data each time.
    #TODO: manage so buffer doesn't fill up
    try:
        for i in range(3):
            report = gpsd.next()
            if report['class'] == 'TPV':     
                lat    = getattr(report,'lat',0.0)
                lon    = getattr(report,'lon',0.0)  #in degrees
                alt    = getattr(report,'alt',0.0)  #in meters
                gps_time = getattr(report,'time','')  #seconds since the Unix epoch? Jan 1 1970. Aprox 1675708560. Careful of precision errors!
                v_velo = getattr(report,'climb')  #this is in m/s
                h_velo = getattr(report,'speed')  #speed over ground, m/s
                heading_gps = getattr(report,'track') #heading, degrees from North
                #TODO: convert v_velo and h_velo to m/s
            
                #print(getattr(report,'lat',0.0),"\t")
                #print(getattr(report,'lon',0.0),"\t")
                #print(getattr(report,'time',''),"\t")
                #print(getattr(report,'alt','nan'),"\t\t")
                #print(getattr(report,'climb','nan'),"\t")
            time.sleep(0.01)
    except:
        v_velo = 0
        h_velo = 0
    #sensor processing -------------------------------
    if time_init is None:
        time_init = gps_time
    else:
        delta_t = 1.5 #TODO: fix gps_time string conversion: gps_time - time_init
    
    for i in range(3):
        mag_cali[i] = mag_gain[i] * (mag_raw[i] - mag_offset[i])
    if mag is not None:
        mag = alpha_mag*mag_cali + (1-alpha_mag)*mag
    #    mag_prev = mag
    else:
        mag = mag_cali
    #    mag_prev = mag

    for i in range(3):
        gyr_cali[i] = gyr_gain[i] * (gyr_raw[i] - gyr_offset[i])
    if gyr is not None:
        gyr = alpha_gyr*gyr_cali + (1-alpha_gyr)*gyr
    #    gyr_prev = gyr
    else:
        gyr = gyr_cali
    #    gyr_prev = gyr

    if lat is not None:
        gps_valid = (lat > 20.0)  #we know we will be in the northern hemisphere
    
    heading = (180/math.pi) * math.atan2(mag[1], mag[0])
    heading_rate = gyr[2]  #TODO: double check gyro is deg/s
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
                wypt_y = waypoints[i][2]
                wypt_z = waypoints[i][3]
    else:
        wypt_x = 0
        wypt_y = 0
        wypt_z = 0
            
    #control ----------------------------------------------------------
    if not gps_valid:
        ctl_mode = 3   #spiral
    else:
        (xi, yi, zi) = llh2ecef(lat, lon, alt)
        dir_ecef = np.array([[wypt_x-xi], [wypt_y-yi], [wypt_z-zi]])
        R_ecef2ned = ecef2ned(lat, lon)
        dir_ned = np.matmul(R_ecef2ned, dir_ecef)
        dx = dir_ned[1] # east distance, in m  
        dy = dir_ned[0] #north distance, in m
        dist = math.sqrt(dx**2+dy**2) #distance, in m
        if dist > outer:
            ctl_mode = 1  #track
        if dist < inner:
            ctl_mode = 2  #orbit
            
    if ctl_mode < 1.5:      #track
        h_error = (180/math.pi) * math.atan2(dy,dx) - heading
        hd_error = -1 * heading_rate
    elif ctl_mode < 2.5: #orbit
        h_error = (180/math.pi) * math.atan2(dy,dx) - heading - 80 #TODO: see if a value closer to 90 is better
        hd_error = (360 * h_velo) / (2 * dist * math.pi) - heading_rate
    else:                   #spiral
        h_error = 0
        hd_error = (360 * h_velo) / (2 * outer * math.pi) - heading_rate
    
    servo = kp * h_error + kd * hd_error
    #TODO: map servo cmd
    #TODO: command servo

    #log data -------------------------------------------------------------
    # Count,GPS_time,dt,Lat,Lon,Ctl_mode,Heading,Heading_rate,Heading_error,Heading_rate_erro,Servo
    data = [count, gps_time, delta_t, lat, lon, ctl_mode, heading, heading_rate, h_error, hd_error, servo]
    with open(file_name, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(data)
    #file = open(file_name, "a")  #append data to txt file
    #file.write() 
    #file.close()
    
    count += 1
    time.sleep(0.1)
     