import numpy as np
import math
import cane_functions as cf
import os
import RPi.GPIO as IO
import time
import board
import busio
import signal
import adafruit_fxos8700
import adafruit_fxas21002c

def exit_function(signal, frame):
    print('Exiting...')
    p_L.stop()
    p_R.stop()
    time.sleep(0.3)
    sys.exit(0)
    
# intialize exit function
signal.signal(signal.SIGINT, exit_function)

# Init motors
IO.setwarnings(False)
IO.setmode (IO.BCM)
m_pin_R = 12
m_pin_L = 13
pwm_freq = 100
IO.setup(m_pin_L,IO.OUT)
IO.setup(m_pin_R,IO.OUT)
p_L = IO.PWM(m_pin_L,pwm_freq)
p_R = IO.PWM(m_pin_R,pwm_freq)
p_L.start(0)
p_R.start(0)
motor_gain = 0.43 # for angles, 0.035 for based on distance
hallway_bump_gain = 0.15
motor_max = 60 # max speed
motor_min = 20 # min speed where motion occurs w/ friction
motor_command = 1
m_avg = 25 # parameters for sweeping
m_amp = 25
m_per = 1.8
m_bias = 0.0
m_offset = 2.4 # scalar to speed up obstacle avoidance
deadband = 0

# setup the imu settings
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
time.sleep(0.5)
offset_vec = np.array([-31.85, -9.2, 534.15])
scale_vec = np.array([0.9871977, 0.97198879, 1.04360902])
#fxas = adafruit_fxas21002c.FXAS21002C(i2c)
day_cnt = 20 # days into the year
dec_angle = -23.45*np.cos(2*3.14159/365*(day_cnt+10))
d2r = 3.14159/180.0 # degrees to radians
r2d = 1.0/d2r 

# init the messages
dt = 0.05 # ctrl period in seconds
time_start = time.time()
obj_angle = 0.0
new_obj_angle = 0.0
side = "L"
obj_dist = 100.0
goal_angle = 0.0
start = True
#np.save(cwd+'/obj_angle.npy', obj_angle)
cwd = os.getcwd() + '/obj_angs/'
obj_cnt = len(os.listdir(cwd))
heading = np.zeros(8)

while(True):
    cur_time = time.time()
    if cur_time >= time_start + dt: # update motor command
        heading[:-1] = heading[1:]
        heading[-1] = cf.readCompassGPS(dec_angle, offset_vec, scale_vec, fxos.magnetometer, r2d) # compute current heading# measure current angle from IMU
        current_heading = int(np.mean(heading))
        if obj_cnt != len(os.listdir(cwd)):
            obj_cnt = len(os.listdir(cwd))
            time.sleep(0.02)
            new_obj_angle = np.load(cwd+'obj_angle_'+str(obj_cnt-1)+'.npy', allow_pickle=True)
            if new_obj_angle != obj_angle: # new angle
                obj_angle = new_obj_angle
                print(new_obj_angle)
                if not start:
                    goal_angle = obj_angle + current_heading# cur_angle
                else:
                    start = False
                    goal_angle = obj_angle + current_heading# cur_angle
        if start:
            des_turn = 0.0
        else:
            des_turn = goal_angle - current_heading
            
            motor_command = cf.calcDesiredTurn(0.0, des_turn, motor_max, motor_min, motor_gain, deadband)
            print(round(current_heading,2), round(goal_angle,2), round(motor_command,2))
            cf.update_motors(motor_command, p_L, p_R)
