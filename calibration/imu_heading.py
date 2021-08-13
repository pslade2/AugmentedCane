import numpy as np
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
import time

# setup the imu settings
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
time.sleep(0.5)
d2r = 3.14159/180.0 # degrees to radians
r2d = 1.0/d2r # radians to degrees
offset_vec = np.array([-31.85, -9.2, 534.15])
scale_vec = np.array([0.9871977, 0.97198879, 1.04360902])
dec_angle = -13.0

def readCompass(dec_angle, offset_vec, scale_vec):
    # read in IMU data --> later correct for orientation
    mag_input = np.array([fxos.magnetometer[0],fxos.magnetometer[1],fxos.magnetometer[2]])
    mag_corr = (mag_input-offset_vec)*scale_vec
    heading = r2d*np.arctan2(mag_corr[2],  mag_corr[1])
    heading += dec_angle
    if heading < 0.0:
        heading += 360.0
    if heading > 360.0:
        heading -= 360.0
    return int(heading)

while(1):
    current_heading = readCompass(dec_angle, offset_vec, scale_vec) # compute current heading
    print(current_heading)
    time.sleep(0.5)
