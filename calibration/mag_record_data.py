import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
import time
import numpy as np
import os
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
fxas = adafruit_fxas21002c.FXAS21002C(i2c)
cur_dir = os.getcwd()
start_time = time.time()
init_time = time.time()
cur_time = start_time
imu_delay = 0.01 # delay in seconds between IMU updates
imu_hz = int(1.0/imu_delay)
run_time = 30 # number of seconds to run code for
data_cnt = 0
mag_array = np.zeros((imu_hz*run_time+1, 3))
running_code = True
print("Starting IMU data saving...")
while(running_code):
    cur_time = time.time()
    if cur_time > (start_time + imu_delay):
        data_cnt += 1
        start_time = cur_time
        # update at 100 hz
        #gyr = [d2g*fxas.gyroscope[0], d2g*fxas.gyroscope[1], d2g*fxas.gyroscope[2]]
        #accel = [fxos.accelerometer[0], fxos.accelerometer[1], fxos.accelerometer[2]]
        mag_array[data_cnt,:] = [fxos.magnetometer[0],fxos.magnetometer[1],fxos.magnetometer[2]]
        #Q = madgwick.updateMARG(gyr, accel, mag, Q)
        #Q = madgwick.updateIMU(gyr, accel, Q)
        #ypr = ahrs.common.orientation.q2euler(Q)*180/3.14159
        if data_cnt >= int(run_time*(1.0/imu_delay)): # finished
            np.save(cur_dir + '/imu_cal_file.npy', mag_array)
            running_code = False
            print("Saving data and exiting...")
