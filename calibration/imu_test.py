import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
import time
import ahrs
import numpy as np
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
fxas = adafruit_fxas21002c.FXAS21002C(i2c)
madgwick = ahrs.filters.Madgwick(Dt = 0.1)    # Madgwick's attitude estimation using default values

start_time = time.time()
cur_time = start_time
imu_delay = 0.1 # delay in seconds between IMU updates
Q = np.array([1.0, 0.0, 0.0, 0.0])
ypr = np.zeros(3)
d2g = ahrs.common.DEG2RAD
plot_counter = 0
gyr = np.zeros(3)
accel = np.zeros(3)
mag = np.zeros(3)

while(1):
    cur_time = time.time()

    if cur_time > (start_time + imu_delay):
        plot_counter += 1
        start_time = cur_time
        # update at 100 hz
        gyr = [d2g*fxas.gyroscope[0], d2g*fxas.gyroscope[1], d2g*fxas.gyroscope[2]]
        accel = [fxos.accelerometer[0], fxos.accelerometer[1], fxos.accelerometer[2]]
        mag = [fxos.magnetometer[0],fxos.magnetometer[1],fxos.magnetometer[2]]
        #Q = madgwick.updateMARG(gyr, accel, mag, Q)
        Q = madgwick.updateIMU(gyr, accel, Q)
        ypr = ahrs.common.orientation.q2euler(Q)*180/3.14159
        if plot_counter%20 == 0: # print at a slower rate
            print(Q, ypr[0])
