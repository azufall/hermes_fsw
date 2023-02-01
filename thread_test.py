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
    gyr_gain = np.array([180/math.pi, 180/math.pi, 180/math.pi])
    mag_offset = np.array([0.0, 0.0, 0.0])
    gyr_offset = np.array([0.0, 0.0, 0.0])
    mag = None
    gyr = None
    mag_cali = np.array([0.0, 0.0, 0.0])
    gyr_cali = np.array([0.0, 0.0, 0.0])
    
    file_name = "Thread_test_Jan31_1.csv"
    header = ['Count','Lat','Lon','Mag','Gyr']
    with open(file_name, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        
    print("init done")
    time.sleep(1.0)
    count = 0
    
    try:
        gpsp.start() #start the daemon

        while count < 100:

            #read sensors -------------------------------------
            mag_raw = IMU.magnetic     #3-tuple, in uT
            gyr_raw = IMU.gyro         #3-tuple, in deg/s

            lat = gpsd.fix.latitude
            lon = gpsd.fix.longitude
            gps_time = gpsd.fix.time
            alt = gpsd.fix.altitude
            h_velo = gpsd.fix.speed
            v_velo = gpsd.fix.climb
            heading_gps = gpsd.fix.track
            
            data = [count, lat, lon, mag_raw, gyr_raw]
            with open(file_name, 'a', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data)
    
            count += 1
            time.sleep(0.2)
            print(data)
     
    except (KeyboardInterrupt, SystemExit):
        print("\nKilling thread")
        gpsd.running = False
        gpsd.join()  #execute thread
    print("Done. \nExited.")