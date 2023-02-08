from gps import *
import time
import threading
#import serial
import board
import busio
import adafruit_bno055
#import array
import math
import numpy as np
import csv
import RPi.GPIO as gpio
from fsw_helpers import *
#from GPS_Poller import *
import os

gpsd = None #create global variable for gpsd

class GPS_Poller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        global gpsd #bring gpsd object in scope
        gpsd = gps(mode=WATCH_ENABLE) #start info stream
        self.current_value = None
        self.running = True
    
    def run(self):
        global gpsd
        while gpsp.running:
            gpsd.next() #loop through and grab each set of gps readings so buffer doesn't fill

if __name__ == '__main__':
    #init ---------------------------------------------
    gpsp = GPS_Poller() #create thread
    #gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
    i2c = busio.I2C(board.SCL, board.SDA)
    IMU = adafruit_bno055.BNO055_I2C(i2c)
    #state is [LAT(deg), LON(deg), ALT(m), V_VELO(m/s), H_VELO(m/s), HEADING(deg), ROTATION_RATE(deg/s), SERVO(deg)] 
    state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    lat = None
    lat_prev = None
    lon = None
    alt = None
    time_init = None
    delta_t = None
    gps_valid = False
    gps_time = None
    v_velo = 0
    h_velo = 0

    alpha_mag = 0.5
    alpha_gyr = 0.3
    gyr_limit = 90
    mag_gain = np.array([1.0, 1.0, 1.0])
    gyr_gain = np.array([180/math.pi, 180/math.pi, 180/math.pi])
    mag_offset = np.array([0.0, 0.0, 0.0])
    gyr_offset = np.array([0.0, 0.0, 0.0])
    mag = None
    gyr = None
    mag_cali = np.array([0.0, 0.0, 0.0])
    gyr_cali = np.array([0.0, 0.0, 0.0])

    servo = 0.0
    servo_pin = 17  #pin 11, GPIO 17
    gpio.setmode(gpio.BCM)
    gpio.setup(servo_pin, gpio.OUT)
    servo_min = 7.5
    servo_max = 8.8
    servo_mid = 0.5*(servo_min + servo_max)
    s = gpio.PWM(servo_pin, 50) # GPIO 17 for PWM with 50Hz
    s.start(2.5) # Initialization
    ctl_mode = 3
    kp = 0.01  ###TODO: update with realistic values
    kd = 0.20
    #waypoint structure: time (s), lat (deg.ddddd), lon (deg.ddddd), alt (m)
    #waypoints = np.array([[  0.0, 38.55278, -121.71326, 15],
    #                       300.0, 38.55278, -121.71271, 15], 
    #                      [600.0, 38.55220, -121.71348, 15], 
    #                      [900.0, 38.55216, -121.71267, 15]])
    waypoints = np.array([[  0.0, 38.551797, -121.713620, 20],  #manhole cover right outside 2900 Spafford 
                          [300.0, 38.552102, -121.713626, 20],  #manhole cover north, alongside Spafford 
                          [600.0, 38.551797, -121.713620, 20], 
                          [900.0, 38.552102, -121.713626, 20]])
    num_waypoints = 4
    #convert waypoints to ECEF
    for i in range(num_waypoints):
        (x,y,z) = llh2ecef(lat=waypoints[i][1], lon=waypoints[i][2], height=waypoints[i][3])
        waypoints[i][1] = x
        waypoints[i][2] = y
        waypoints[i][3] = z
    
    outer =  8.0
    inner =  4.0
    
    file_name = "GPS_test_Feb_8"
    trial = 1
    while os.path.exists(file_name + str(trial) + ".csv"):
        trial += 1
    file_name = file_name + str(trial) + ".csv"
    header = ['Count','GPS_time','DeltaT','Lat','Lon','Ctl_mode','Heading','Heading_rate','Heading_error','Heading_rate_erro','Servo']
    with open(file_name, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)

    #file = open(file_name, "w")  #append data to txt file
    #file.write("Count,GPS_time,Lat,Lon,Ctl_mode,Heading,Heading_rate,Heading_error,Heading_rate_erro,Servo")
    #file.close()

    print("init done")
    time.sleep(60.0)
    count = 0
    lat_count = 0
    
    try:
        gpsp.start() #start the Poller

        while True: #count < 600:

            #read sensors -------------------------------------
            #TODO TRY EXCEPT
            try:
                mag_raw = IMU.magnetic     #3-tuple, in uT
                gyr_raw = IMU.gyro         #3-tuple, in deg/s
            except:
                mag_raw = mag  #TODO, update this
                gyr_raw = gyr
            #TODO: restructure this so we get valid data each time.
            #TODO: manage so buffer doesn't fill up

            lat = gpsd.fix.latitude
            lon = gpsd.fix.longitude
            gps_time = gpsd.fix.time
            alt = gpsd.fix.altitude
            h_velo = gpsd.fix.speed
            v_velo = gpsd.fix.climb
            heading_gps = gpsd.fix.track

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
                if abs(gyr_raw[i]) > gyr_limit:
                    gyr_raw[i] = gyr[i]
                gyr_cali[i] = gyr_gain[i] * (gyr_raw[i] - gyr_offset[i])
            if gyr is not None:
                gyr = alpha_gyr*gyr_cali + (1-alpha_gyr)*gyr
            #    gyr_prev = gyr
            else:
                gyr = gyr_cali
            #    gyr_prev = gyr

            if lat is not None:
                if lat_prev is not None:
                    if lat-lat_prev < 10^-9:
                        lat_count += 1
                    else:
                        lat_count = 0
                gps_valid = (lat > 20.0) and (lat_count < 25)  #we know we will be in the northern hemisphere
    
            heading = (180/math.pi) * math.atan2(mag[1], mag[0])
            heading_rate = -1*gyr[2]
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
            if count is not None:
                wypt_x = waypoints[0][1] #default in case something goes haywire
                wypt_y = waypoints[0][2]
                wypt_z = waypoints[0][3]
                for i in range(num_waypoints):
                    if count > waypoints[i][0]:
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
                dE = dir_ned[1] # east distance, in m  
                dN = dir_ned[0] #north distance, in m
                dist = math.sqrt(dE**2+dN**2) #distance, in m
                if dist > outer:
                    ctl_mode = 1  #track
                if dist < inner:
                    ctl_mode = 2  #orbit
            
            if ctl_mode < 1.5:      #track
                h_error = (-180/math.pi)*wrapToPi(math.atan2(dN,dE)-math.pi/2) - heading
                hd_error = -1 * heading_rate
            elif ctl_mode < 2.5: #orbit
                h_error = (-180/math.pi)*wrapToPi(math.atan2(dN,dE)-math.pi/2) - heading - 80 #TODO: see if a value closer to 90 is better
                hd_error = (360 * h_velo) / (2 * dist * math.pi) - heading_rate
            else:                   #spiral
                h_error = 90
                hd_error = (360 * h_velo) / (2 * outer * math.pi) - heading_rate
            h_error = (180/math.pi)*wrapToPi((math.pi/180)*h_error)
            #servo = (kp * h_error + kd * hd_error) + 0.5*(servo_min + servo_max)
            if   h_error >  20: servo = servo_max
            elif h_error >  10: servo = 0.5*(servo_mid + servo_max)
            elif h_error < -20: servo = servo_min
            elif h_error < -10: servo = 0.5*(servo_mid + servo_min)
            else:               servo = servo_mid
            s.ChangeDutyCycle(servo)
            #TODO: map servo cmd

            #log data -------------------------------------------------------------
            # Count,GPS_time,dt,Lat,Lon,Ctl_mode,Heading,Heading_rate,Heading_error,Heading_rate_erro,Servo
            data = [count, gps_time, delta_t, lat, lon, ctl_mode, heading, heading_rate, h_error, hd_error, servo]
            with open(file_name, 'a', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
            #file = open(file_name, "a")  #append data to txt file
            #file.write() 
            #file.close()
            #print(data)
            count += 1
            time.sleep(0.23)
     
    except (KeyboardInterrupt, SystemExit):
        print("\nKilling thread")
        gpsd.running = False
        gpsd.join()  #execute thread
    print("Done. \nExited.")