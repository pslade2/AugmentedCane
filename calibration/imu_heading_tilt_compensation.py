import numpy as np
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
import time
from scipy import signal
# compensating for tilt by using low-pass of accelerometer measurements
# swapping x and z directions following this: http://students.iitk.ac.in/roboclub/2017/12/21/Beginners-Guide-to-IMU.html

# setup the imu settings
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
time.sleep(0.5)
d2r = 3.14159/180.0 # degrees to radians
r2d = 1.0/d2r # radians to degrees
offset_vec = np.array([-31.85, -9.2, 534.15])
scale_vec = np.array([0.9871977, 0.97198879, 1.04360902])
dec_angle = -13.0

# defining low pass filter and array for data
sample_freq = 20 # in Hz
time_interval = 1.0/sample_freq
start_time = time.time()
len_accel = 40
accel_arr = np.zeros((3, len_accel))
wn = 6 # critical frequency of low-pass
filt_order = 4
b,a = signal.butter(filt_order, wn, fs=sample_freq) # params for low-pass filter
int_cnt = 0

def readCompass(dec_angle, offset_vec, scale_vec, b, a, accel_arr):
    mag = np.array([fxos.magnetometer[0],fxos.magnetometer[1],fxos.magnetometer[2]])
    mag_corr = (mag-offset_vec)*scale_vec

    accel = np.array([fxos.accelerometer[0],fxos.accelerometer[1],fxos.accelerometer[2]])
    accel_arr[:,:-1] = accel_arr[:,1:] # shift data over
    accel_arr[:,-1] = accel # add new data
    filt_arr = signal.filtfilt(b, a, accel_arr)
    accel_ref = np.mean(filt_arr[:,-5:], axis=-1) # try just using these last values or maybe average over last few?

    pitch = np.arctan2(accel_ref[2], np.sqrt(accel_ref[0]**2 + accel_ref[1]**2))
    roll = np.arctan2(accel_ref[1], np.sqrt(accel_ref[0]**2 + accel_ref[2]**2))
    mag_z = mag[2]*np.cos(pitch) + mag[1]*np.sin(roll)*np.sin(pitch) + mag[0]*np.cos(roll)*np.sin(pitch)
    mag_y = mag[1]*np.cos(roll) - mag[0]*np.sin(roll)
    heading = r2d*np.arctan2(-mag_y, mag_z)
    #heading = r2d*np.arctan2(mag_corr[2],  mag_corr[1])
    heading += dec_angle
    if heading < 0.0:
        heading += 360.0
    if heading > 360.0:
        heading -= 360.0
    return int(heading), accel_arr

while(1):
    if (time.time() - start_time) > time_interval:
        start_time = time.time()
        current_heading, accel_arr = readCompass(dec_angle, offset_vec, scale_vec, b, a, accel_arr) # compute current heading
        int_cnt += 1
        if int_cnt%sample_freq == 0: # print every second
            print(current_heading)
