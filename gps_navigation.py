### TODO add a function to see if actually closer to further point than current waypoint and skip to that one.
import time
import adafruit_gps
import serial
import signal
import numpy as np
import board
import busio
import adafruit_fxos8700
#import adafruit_fxas21002c
import RPi.GPIO as IO
import cane_functions as cf
from rplidar import RPLidar
import os
import sys
import gpxpy
import gpxpy.gpx

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
obstacle_det = True
audio_feedback = int(sys.argv[2])
compass_correct = 6.0 # add on to improve compass heading estimates
audio_ang_offset = 90.0

if audio_feedback:
    print("Starting audio feedback mode")


# initialize the saving folder
subj = str(sys.argv[1])
save_dir = os.getcwd() + '/gps_data/' + subj# + '/'# + str(run_num)
try:
    os.mkdir(save_dir)
except:
    print("Save dir exists: ", save_dir)
files = os.listdir(save_dir)
run_num = len(files)+1
save_dir = save_dir + '/' + str(run_num) + '/'
os.mkdir(save_dir)

# load the gps data as a list
#gps_file = os.getcwd() + '/gps_data/gpx/test_path.gpx' #path1.gpx'
#gpx_f = open(gps_file, 'r')
#gpx = gpxpy.parse(gpx_f)
#points = gpx.tracks[0].segments[0].points
#waypoint_list = []
#for p in points:
#    waypoint_list.append([p.latitude, p.longitude])
#waypoint_list = np.load(os.getcwd() + '/gps_data/gpx/path' + str(sys.argv[3]) + '.npy')
#waypoint_list = cf.load_waypts('GPS','3')#'SXX','31')
waypoint_list = np.load(os.getcwd() + '/gps_data/path2.npy')

uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)     # Use UART/pyserial
# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000') # Set update rate to once a second.
# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()

# intialize serial and rplidar
PORT_NAME = '/dev/ttyUSB0'
run_flag = True # starts the program, keeps it running
time.sleep(.5) # rest to start serial
if not audio_feedback:
    lidar = RPLidar(PORT_NAME)
    iterator = lidar.iter_scans(max_buf_meas=120)
time.sleep(.5) # pause to start lidar scanning

# initialize the gps settings
d2r = 3.14159/180.0 # degrees to radians
r2d = 1.0/d2r # radians to degrees
initialized = False
main_rate = 1.0 # rate in hz
dec_angle = 0.0 # declination angle https://www.pveducation.org/pvcdrom/properties-of-sunlight/declination-angle
waypoint_dist_tolerance = 5 # tolerance in meter to waypoint; once within this tolerance, we'll advance to the nect waypoint
heading_tolerance = 10 # tolerance +/- (in degrees) within which we don't attempt to turn to intercept targetHeading
# waypoints lat/long in degree decimal
#waypoint_list = [[37.426995365893596,-122.17254054883],[37.42721666379759,-122.1724720452093],[37.42757960628986, -122.17234587409479]]
number_waypoints = len(waypoint_list)
waypnt_cnt = 0
cur_waypoint = waypoint_list[waypnt_cnt]
reached_last_waypoint = False
# GPS heading is 0 deg at N, with  0 --> 360 going clockwise from 12 to 12

target_heading = 0.0
cur_heading = 0.0
comp_heading = 0.0
head_vec = np.zeros(5)
deadband = 6 # within +/- degrees for compass to turn off

# setup the imu settings
i2c = busio.I2C(board.SCL, board.SDA)
fxos = adafruit_fxos8700.FXOS8700(i2c)
#acc = adafruit_fxos8700.FXAS21002C(i2c)
time.sleep(0.5)
offset_vec = np.array([-26.0,1.0,541.15])#[-31.85, -9.2, 534.15])
scale_vec = np.array([1.0264612954186412,0.9668898809523808,1.008537058595266])#[0.9871977, 0.97198879, 1.04360902])
#fxas = adafruit_fxas21002c.FXAS21002C(i2c)

# parameters for lidar
forward_thresh = 1700.0 # only look to turn if less than 2 meters of room ahead
angle_offset = 0.0 # because lidar turned 45 degrees to left

# discretized lidar array setup
array_sections = 19 # always have odd value here
num_front_sect = 3 # use the middle section +/- this many sections for checking front collision
array_start = -90.0
array_stop = 90.0
arr_width = (array_stop - array_start)/array_sections
lidar_ang_arr = np.zeros((array_sections,2)) # start then stop angle values
lidar_arr_mean = np.zeros(array_sections) # goes from right to left
distances = np.ones(array_sections)*3000.0
for i in range(array_sections):
    start_ang = array_start+i*arr_width
    stop_ang = start_ang + arr_width
    lidar_arr_mean[i] = int((start_ang + stop_ang)/2.0)
    lidar_ang_arr[i,:] = [start_ang+angle_offset,stop_ang+angle_offset]
mid_ind = array_sections//2
lidar_arr_mean -= lidar_arr_mean[mid_ind]
lidar_arr_mean = -1*lidar_arr_mean

# setup the motor specs
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
motor_max = 60
motor_min = 20
motor_gain = 0.43 # approx 80 degree error to kick max motor = (35/80)
motor_command = 1
m_avg = 25 # parameters for sweeping
m_amp = 25
m_per = 1.5
m_bias = 0.0
m_offset = 2.4 # scalar to speed up obstacle avoidance

# defining pushbutton GPIO
button_pin = 18
IO.setup(button_pin, IO.IN)
button_state = True
if cf.read_pushbutton(button_pin): # if not holding button on startup
    engage_sweep = True
else:
    engage_sweep = False
    
# audio state toggle
playing_audio = False # start this way
prev_pa_flag = False
audio_start_time = time.time()
start_time = time.time()
if audio_feedback:
    audio_delay = 10.0 
else:
    audio_delay = 2.3 # time in seconds between audio allowed to play
prev_pt = [0.0, 0.0]
gps_cnt = 0
gps_file_cnt = 0
gps_save_thresh = 20 # every 20 gps points save the file
gps_data = []
gps_hist = np.zeros((5,2))
gps_comp_ind = 4 # which index of the history to use
comp_weight = 0.5 # weighting on GPS, 1-X weight on compass
accel_hist = np.zeros((10,3))
obst_rec = False
waypnt_cnt = 4
gps_time = time.time()
while not reached_last_waypoint:
    while(not button_state): # stay here until button released
        p_L.ChangeDutyCycle(0) # zero motors
        p_R.ChangeDutyCycle(0)
        button_state = cf.read_pushbutton(button_pin)
        time.sleep(0.1)
        gps.update()

    button_state = cf.read_pushbutton(button_pin)
    if gps_time < time.time() - 1.0: # only update gps at 1hz
        gps.update() # call fast
    
    if not audio_feedback:
        lidar_scan = next(iterator)
        distances = cf.compute_lidar_array(lidar_scan, lidar_ang_arr, distances)
        #print(distances[mid_ind])
        #print(distances)
    #if initialized:
    # update the imu heading
    if audio_feedback:
        cur_heading = cf.readCompassGPS(dec_angle, offset_vec, scale_vec, fxos.magnetometer, r2d) + audio_ang_offset# compute current heading
    else:
        cur_heading = cf.readCompassGPS(dec_angle, offset_vec, scale_vec, fxos.magnetometer, r2d) # compute current heading
    accel_hist[1:,:] = accel_hist[:-1,:]
    accel_hist[0,:] = fxos.accelerometer
    head_vec[:-1] = head_vec[1:]
    head_vec[-1] = cur_heading
    if np.sum(gps_hist) == 0.0:
        gps_hist[:,:] = [gps.latitude, gps.longitude]
    else:
        gps_hist[1:,:] = gps_hist[:-1,:]
        gps_hist[0,:] = [gps.latitude, gps.longitude]
    gps_head = cf.courseToWaypoint(gps_hist[gps_comp_ind,:], gps_hist[0,:], d2r, r2d)
    comp_heading = cur_heading#comp_weight*gps_head + (1-comp_weight)*cur_heading # np.mean(head_vec)
    
    # check if there's an obstacle in the way, notify with audio, and steer around it
    if obstacle_det and not audio_feedback:
        playing_audio, motor_command = cf.obstacle_state_machine(playing_audio, distances, mid_ind, m_offset, motor_gain, motor_max, deadband, target_heading, comp_heading, motor_min, lidar_arr_mean, forward_thresh)
        motor_command = 0
        if not obst_rec:
            obst_rec = playing_audio
    # check when to allow audio again
    playing_audio, prev_pa_flag, audio_start_time = cf.wait_audio(playing_audio, prev_pa_flag, audio_start_time, audio_delay)
    if obst_rec:
        distances += 2500.0
    # otherwise update the motor commands based on heading
    if gps.has_fix and motor_command == 0 and not audio_feedback:
        motor_command = cf.calcDesiredTurn(comp_heading, target_heading, motor_max, motor_min, motor_gain, deadband)
    
    if audio_feedback and gps.has_fix:
        turn = cf.bound_angle(target_heading - comp_heading)
        turn_int = int(turn - turn%10)
        if not playing_audio and turn_int != 0:
            playing_audio = True
            cf.play_preloaded_sound(str(turn_int)+".mp3") # let the person know they are turning to follow GPS
    else:
        cf.update_motors(motor_command, p_L, p_R)
    # TODO: add sweeping function for outdoor use, maybe avg the GPS measurements to make it safer

    # run gps waypoint updates at slower rate
    current = time.monotonic() # update timer to check rate
    if current - last_print >= main_rate:
        last_print = current
        if not initialized: # store the initializing info
            if not gps.has_fix:
                print('Waiting for fix...')
                #cf.update_motors(0.0, p_L, p_R)
                continue
            cf.initialization_print(gps) # print starting info
            day_cnt = (gps.timestamp_utc.tm_mon - 1)*30 +  gps.timestamp_utc.tm_mday
            dec_angle = -23.45*np.cos(np.deg2rad(1)*360/365*(day_cnt+10)) + compass_correct
            print("Day cnt: ", day_cnt, "  Declination angle (deg): ", dec_angle)
            initialized = True
        else: # main loop to run the code
            if not gps.has_fix:
                print('Lost fix...')
                continue
            # update distance
            cur_pt = list(np.mean(gps_hist, axis=0))#[gps.latitude, gps.longitude]
            dist_to_target = cf.distanceToWaypoint(cur_pt, cur_waypoint, d2r)
            #print(cur_pt, cur_waypoint)
            # check to see if we have reached the current waypoint
            if (dist_to_target <= waypoint_dist_tolerance):
                waypnt_cnt += 3
                if waypnt_cnt >= number_waypoints: # done
                    reached_last_waypoint = True # exit code
                    break # exiting loops
                cur_waypoint = np.zeros(2)
                for i in range(4):
                    cur_waypoint += waypoint_list[waypnt_cnt-i]
                cur_waypoint = list(cur_waypoint/4.0)
                #print(cur_pt, cur_waypoint)
                #cur_waypoint = waypoint_list[waypnt_cnt]
                dist_to_target = cf.distanceToWaypoint(cur_pt, cur_waypoint, d2r)
                
                target_heading = cf.courseToWaypoint(cur_pt, cur_waypoint, d2r, r2d)
                #if abs(target_heading - comp_heading) > 30 and not playing_audio:
                #    playing_audio = True
                #    cf.play_preloaded_sound("gps.mp3") # let the person know they are turning to follow GPS
            else: # update the course to the next waypoint
                target_heading = cf.courseToWaypoint(cur_pt, cur_waypoint, d2r, r2d)
            
            if cur_pt != prev_pt: # add to store data
                gps_cnt += 1
                gps_data.append([time.time()-start_time, cur_pt, cur_waypoint, dist_to_target, comp_heading, target_heading, fxos.magnetometer, np.mean(accel_hist,axis=0), obst_rec])
                if obst_rec:
                    obst_rec = False
                prev_pt = cur_pt
                if gps_cnt >= gps_save_thresh:
                    gps_file_cnt += 1
                    np.save(save_dir+str(gps_file_cnt)+".npy", gps_data)
                    gps_data = []
                    gps_cnt = 0
            # print status update
            #print(fxos.magnetometer)
            print("Waypnt: ", waypnt_cnt, " Distance (m): ", np.round(dist_to_target,1), " Target heading: ", np.round(target_heading,1), " Cur heading: ", comp_heading, "M", np.round(motor_command,1))

# Finished running
print('Reached final waypoint!')
try:
    gps_file_cnt += 1
    np.save(save_dir+str(gps_file_cnt)+".npy", gps_data)
except:
    print("Couldn't save last file for some reason")

p_L.stop()
p_R.stop()
if audio_feedback:
    cf.play_preloaded_sound("gps_done.mp3")
else:
    lidar.stop()
    lidar.stop_motor()
    time.sleep(2.0)
    cf.play_preloaded_sound("gps_done.mp3")
    lidar.disconnect()
time.sleep(6.0)
