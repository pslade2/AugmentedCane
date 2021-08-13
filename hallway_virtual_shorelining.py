## TODO: try and make less brittle by using more sections of lidar for just slight turns rather than whole 90 degree turns
## use more sections on the right to improve stability of shorelining, i.e. doorways cause big issues right now

#!/usr/bin/env python3
# to run on raspberry pi use, add sudo if run into issues with serial: python3 assist_cane_hallway_test.py
from rplidar import RPLidar
import RPi.GPIO as IO
import numpy as np
import time
import serial
import struct
import signal
import sys
import board
import busio
import adafruit_fxos8700 # accel and magnetometer
# import adafruit_fxas21002c # currently don't need gyro
import cane_functions as cf

# stops the motors  called when ctrl-c is hit
def exit_function(signal, frame):
    print('Exiting...')
    p_L.stop()
    p_R.stop()
    lidar.stop()
    lidar.stop_motor()
    time.sleep(2.0)
    lidar.disconnect()
    time.sleep(2.0)
    sys.exit(0)
# intialize exit function
signal.signal(signal.SIGINT, exit_function)

# intialize serial and rplidar
PORT_NAME = '/dev/ttyUSB0'
run_flag = True # starts the program, keeps it running
time.sleep(.5) # rest to start serial
lidar = RPLidar(PORT_NAME)
iterator = lidar.iter_scans(max_buf_meas=120)
time.sleep(.5) # pause to start lidar scanning

# distance and lidar thresholds
hallway_steering_thresh = 700.0 # if any lidar reading less than this, move away from that side
forward_thresh = 1500.0 # only look to turn if less than 2 meters of room ahead
right_thresh = 900.0 # keep the right wall 1m away at all times
right_check_thresh = 2000.0 # always turn right if at least this much space
crash_thresh = 400.0 # if turning into a wall then stop turning
angle_offset = -45.0 # because lidar turned 45 degrees to left
forward_range_low = -5.0+angle_offset # in degrees
forward_range_upper = 5.0+angle_offset # in degrees
left_range_low = 85.0+angle_offset #in degrees
left_range_upper = 95.0+angle_offset #in degrees
right_range_low = -95.0+angle_offset #in degrees
right_range_upper = -85.0+angle_offset #in degrees

# discretized lidar array setup
array_sections = 19 # always have odd value here
num_front_sect = 2 # use the middle section +/- this many sections for checking front collision
array_start = -90.0
array_stop = 90.0
arr_width = (array_stop - array_start)/array_sections
lidar_ang_arr = np.zeros((array_sections,2)) # start then stop angle values
lidar_arr_mean = np.zeros(array_sections) # goes from right to left
distances = np.zeros(array_sections)
for i in range(array_sections):
    start_ang = array_start+i*arr_width
    stop_ang = start_ang + arr_width
    lidar_arr_mean[i] = int((start_ang + stop_ang)/2.0)
    lidar_ang_arr[i,:] = [start_ang+angle_offset,stop_ang+angle_offset]
mid_ind = array_sections//2
lidar_arr_mean -= lidar_arr_mean[mid_ind]
lidar_arr_mean = -1*lidar_arr_mean

# define motor pins, initialize, and constants
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

# defining pushbutton GPIO
button_pin = 18
IO.setup(button_pin, IO.IN)
if cf.read_pushbutton(button_pin): # if not holding button on startup
    engage_sweep = True
else:
    engage_sweep = False

# defining imu setup and constants
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
# fxas = adafruit_fxas21002c.FXAS21002C(i2c)
dec_angle = -13.0 # update based on month (or use gps info)!
d2r = 3.14159/180.0 # degrees to radians
r2d = 1.0/d2r # radians to degrees
offset_vec = np.array([-31.85, -9.2, 534.15]) # correcting error in magnetometer
scale_vec = np.array([0.9871977, 0.97198879, 1.04360902]) # correcting error in magnetometer
loop_num = 20 # how many loops before print state info
deg_turn = 65.0 # degrees to turn left or right
turn_threshold = 10.0 # range of degrees (+/-) within to stop turning
lidar_gain = 0.98 # NOT USED RN: multiplier raised to the power of distance intervals, to prevent accidental Turning

# GPS placeholder
target_heading = 0.0
current_heading = 0.0
deadband = 0

### Initializing state variables
while(run_flag):
    # stop motors and check reset button
    p_L.ChangeDutyCycle(0) # zero motors
    p_R.ChangeDutyCycle(0)
    button_state = cf.read_pushbutton(button_pin)
    time.sleep(0.1)
    # restart time values
    loop_time = time.time()
    loop_cnt = 0
    # state machine values
    forward_distance = 0.0
    left_distance = 0.0
    right_distance = 0.0
    goal_yaw = 0.0
    cur_yaw = 0.0
    state = 1

    ### Continually run code for hallway tracking
    while(button_state): # run until reset button pushed
        ## UPDATE CUR_YAW WITH IMU
        cur_yaw = cf.readCompass(dec_angle, offset_vec, scale_vec, fxos.magnetometer, r2d)
        ## READ AND UPDATE LIDAR
        lidar_scan = next(iterator)
        distances = cf.compute_lidar_array(lidar_scan, lidar_ang_arr, distances)
        ## CHECK STATE MACHINE
        motor_command = cf.steer_towards_open(distances, lidar_gain, hallway_bump_gain, hallway_steering_thresh, mid_ind, motor_gain, motor_max, motor_min, lidar_arr_mean, forward_thresh)
        #motor_command = cf.obstacle_state_machine(distances, mid_ind, motor_gain, motor_max, deadband, target_heading, current_heading, motor_min, lidar_arr_mean, forward_thresh)
        #state, goal_yaw, motor_command = cf.shoreline_state_machine(forward_thresh, right_thresh, right_check_thresh, crash_thresh, forward_distance, left_distance, right_distance, motor_max, motor_min, motor_gain, cur_yaw, goal_yaw, state, motor_command, deg_turn, turn_threshold)
        if motor_command == 0 and engage_sweep: ## CHECK SWEEPING
            motor_command = cf.sweep_sine(m_avg, m_amp, m_bias, m_per)
        ## UPDATE MOTOR COMMAND
        cf.update_motors(motor_command, p_L, p_R)
        ## CHECK RESET BUTTON
        button_state = cf.read_pushbutton(button_pin)
        ## PRINT STATE INFORMATION
        loop_cnt += 1
        if loop_cnt % loop_num == 0: # print how fast it is running
            loop_delay = (time.time() - loop_time)/loop_num
            print(np.round(distances[4:-4],0))
            print(np.round(lidar_arr_mean[4:-4],1))
            print("state:",state, ' ', "goal yaw:", goal_yaw, ' ', "cur yaw:",cur_yaw, ' ', "fwd:",round(distances[mid_ind]), ' ', "Min:", round(np.amin(distances)),' ', "Min ind:",np.argmin(distances)-mid_ind,' ','Mot:',motor_command)
            # print("Loop is running at: ", round(loop_delay,3), " seconds or ", round(1.0/loop_delay,1), "hz")
            loop_time = time.time()
