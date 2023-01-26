#log gyroscope data
import time
#import serial
import board
import busio
import adafruit_bno055
import csv
print("import done")

#uart = serial.Serial("/dev/ttyAMA0")
#uart = serial.Serial("/dev/serial0")
i2c = busio.I2C(board.SCL, board.SDA)

#IMU = adafruit_bno055.BNO055_UART(uart)
IMU = adafruit_bno055.BNO055_I2C(i2c)

print("open log")
file_name = "gyr_log_0.csv"
header = ['count','magx','magy','magz']
with open(file_name, 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(header)
    
print("start collection")
count = 0
while count < 100:
    #mag_raw = IMU.magnetic     #3-tuple, in uT
    gyr_raw = IMU.gyro         #3-tuple, in deg/s

    #log data -------------------------------------------------------------
    # Count,mag_x,mag_y,mag_z
    data = [count, gyr_raw[0], gyr_raw[1], gyr_raw[2]]
    with open(file_name, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(data)
    
    count += 1
    
    time.sleep(0.3)

print("done")