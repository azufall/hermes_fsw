from gps import *
import time
import serial
import adafruit_bno055
import array

#init ---------------------------------------------
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)
uart = serial.Serial("/dev/ttyAMA0")
IMU = adafruit_bno055.BNO055_UART(uart)
#state is [LAT LON ALT HEADING ROTATION_RATE SERVO] 
state = array('d', [0.0 0.0 0.0 0.0 0.0 0.0])

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
        #print(getattr(report,'lat',0.0),"\t")
        #print(getattr(report,'lon',0.0),"\t")
        #print(getattr(report,'time',''),"\t")
        #print(getattr(report,'alt','nan'),"\t\t")
        #print(getattr(report,'climb','nan'),"\t")
    time.sleep(0.05)

#sensor processing -------------------------------
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
#TODO: adjust heading with GPS info

#calculate state ------------------------------------------------