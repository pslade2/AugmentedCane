# Main file for low-level functions for cane control

from rplidar import RPLidar
import RPi.GPIO as IO
import numpy as np
import time
import serial
import struct
import signal
import sys
import ahrs
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
import os
import subprocess

# wait a certain amount of time before playing a new audio sample to not overload the person
def wait_audio(playing_audio, prev_pa_flag, audio_start_time, audio_delay):
    if playing_audio != prev_pa_flag:
        audio_start_time = time.time()
        prev_pa_flag = True
    if (time.time() >= audio_start_time + audio_delay) and prev_pa_flag: # reset flags and can play again
        prev_pa_flag = False
        playing_audio = False
    return playing_audio, prev_pa_flag, audio_start_time    

# start playing audio file without waiting for it to finish
# preloaded sounds (all with .mp3 at end): gps, gps_turn, hello, obstacle, obstacle_left, obstacle_right
def play_preloaded_sound(sound_file, save_dir='/sound_files/'):
    cwd = os.getcwd()
    process = subprocess.Popen(['mpg321',cwd+save_dir+sound_file])
    #time.sleep(0.05) # necessary?

# bound angle between 0 and 360 degrees
def bound_angle(angle):
    if angle < -180.0:
        angle += 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle

# check if push button is pressed, normally 1 --> 0 when pressed
def read_pushbutton(button_pin):
    button_input = IO.input(button_pin)
    return button_input

# read the IMU yaw angle from magnetometer
def readCompass(dec_angle, offset_vec, scale_vec, raw_mag, r2d):
    # read in IMU data --> later correct for orientation
    mag_input = np.array([raw_mag[0],raw_mag[1],raw_mag[2]])
    mag_corr = (mag_input-offset_vec)*scale_vec
    heading = r2d*np.arctan2(mag_corr[2],  mag_corr[1])
    heading += dec_angle
    heading = bound_angle(heading)
    return int(heading)

# pass in offset (minimum control val), change in motor speed from offset (duty cycle %), and period (s) of the sine oscillation for the motor
# m_bias allows turning while doing S control
# good default values: m_avg=25, m_amp=25, m_per=1.8, m_bias=0.0
def sweep_sine(m_avg, m_amp, m_bias, m_per):
    cur_time = time.time()
    sine_m_cmnd = m_amp*np.sin(2*3.14159*cur_time/m_per)
    sine_m_cmnd += np.sign(sine_m_cmnd)*m_avg # add offset to get rid of deadband
    return sine_m_cmnd

# drive motors based on motor_command
def update_motors(motor_command, p_L, p_R):
    if motor_command > 0: # turn right
        p_L.ChangeDutyCycle(0)
        p_R.ChangeDutyCycle(motor_command)
    elif motor_command < 0:
        p_R.ChangeDutyCycle(0)
        p_L.ChangeDutyCycle(-motor_command)
    else:
        p_L.ChangeDutyCycle(0)
        p_R.ChangeDutyCycle(0)

def compute_lidar_array(lidar_scan, lidar_ang_arr, prev_distances):
    offsets = np.array([((360.0 - meas[1]), meas[2]) for meas in lidar_scan])
    angles = offsets[:,0]
    for ang_ind in range(len(angles)): # pull in angle data from lidar and adjust
        if(angles[ang_ind] > 180):
            angles[ang_ind] = angles[ang_ind] - 360
    distances = offsets[:,1]
    num_sections,_ = lidar_ang_arr.shape
    dist_arr = np.zeros(num_sections)
    for i in range(num_sections):
        lower_range = lidar_ang_arr[i,0]
        upper_range = lidar_ang_arr[i,1]
        if (lower_range > min(angles) and upper_range < max(angles)): # find data points within lower/upper bound
            data_index = [idx for idx in range(len(angles)) if ((angles[idx] > lower_range) and (angles[idx] < upper_range))]
            if(len(data_index) != 0):
                dist_arr[i] = np.mean(distances[data_index])

def compute_lidar_array(lidar_scan, lidar_ang_arr, prev_distances):
    offsets = np.array([((360.0 - meas[1]), meas[2]) for meas in lidar_scan])
    angles = offsets[:,0]
    for ang_ind in range(len(angles)): # pull in angle data from lidar and adjust
        if(angles[ang_ind] > 180):
            angles[ang_ind] = angles[ang_ind] - 360
    distances = offsets[:,1]
    num_sections,_ = lidar_ang_arr.shape
    dist_arr = np.zeros(num_sections)
    for i in range(num_sections):
        lower_range = lidar_ang_arr[i,0]
        upper_range = lidar_ang_arr[i,1]
        if (lower_range > min(angles) and upper_range < max(angles)): # find data points within lower/upper bound
            data_index = [idx for idx in range(len(angles)) if ((angles[idx] > lower_range) and (angles[idx] < upper_range))]
            if(len(data_index) != 0):
                dist_arr[i] = np.mean(distances[data_index])
                if dist_arr[i] <= 1.0: # check for bad values
                    dist_arr[i] = prev_distances[i]
            else:
                dist_arr[i] = prev_distances[i]
    return dist_arr

# state machine for simple hallway code
def state_machine_arr(distances, mid_ind, num_front_sect, lidar_arr_mean, turn_right, turn_left, forward_thresh, min_adjust_dist, max_adjust_dist, single_side_adjust, motor_slight_flag, cur_yaw, goal_yaw, state, m_cmnd,  motor_off, slight_right, slight_left, deg_turn = 90.0, turn_threshold = 5.0):
    if state == 1: # wait for initial lidar readings to be non-zero
        if all(distances): # check all are non-zero
            state = 2
            print("Lidar initialized, starting hallway state machine...")
    elif state == 2: # drive straight, do PID with motor to track center
        state, goal_yaw, m_cmnd, motor_slight_flag = check_state_arr(distances, mid_ind, num_front_sect, lidar_arr_mean, forward_thresh, min_adjust_dist, max_adjust_dist, single_side_adjust, motor_slight_flag, cur_yaw,  motor_off, slight_right, slight_left, turn_right, turn_left, deg_turn)
    elif state == 3: # turn right deg_turn degrees
        state = check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold)
    elif state == 4: # turn left deg_turn degrees
        state = check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold)

    return state, goal_yaw, m_cmnd, motor_slight_flag

def check_state_arr(distances, mid_ind, num_front_sect, lidar_arr_mean, forward_thresh, min_adjust_dist, max_adjust_dist, single_side_adjust, motor_slight_flag, cur_yaw, motor_off, slight_right, slight_left, turn_right, turn_left, turn_deg = 90.0):
    # small adjustments to keep people in middle of hallway
    forward_distance = np.amin(distances[mid_ind-num_front_sect:mid_ind+num_front_sect+1])
    right_distance = distances[0]
    left_distance = distances[-1]
    if (forward_distance > forward_thresh):
        if abs(right_distance - left_distance) > min_adjust_dist and abs(right_distance - left_distance) < max_adjust_dist and right_distance < single_side_adjust: # recenter
            if right_distance > left_distance: # turn right
                m_cmnd, motor_slight_flag = pulse_motor(slight_right, motor_slight_flag, motor_off)
            else:
                m_cmnd, motor_slight_flag = pulse_motor(slight_left, motor_slight_flag, motor_off)
        else:
            m_cmnd = motor_off
            motor_slight_flag = False
        return 2, -1.0, m_cmnd, motor_slight_flag # keep state the same
    else: # hard turn to whichever array index is largest
        max_arr_ind = np.argmax(distances)
        turn_deg = lidar_arr_mean[max_arr_ind]
        print("start turn ", turn_deg, distances)
        if turn_deg < 0.0: # turn right (state 3)
            turn_deg = -turn_deg
            m_cmnd = turn_right
            motor_slight_flag = False
            goal_yaw = bound_angle(cur_yaw - turn_deg)
            return 3, goal_yaw , m_cmnd, motor_slight_flag
        else: # turn left (state 4)
            m_cmnd = turn_left
            motor_slight_flag = False
            goal_yaw = bound_angle(cur_yaw + turn_deg)
            return 4, goal_yaw, m_cmnd, motor_slight_flag

# compute the forward, right, and left distances for the heuristic use
def compute_lidar_distances(lidar_scan, forward_distance, left_distance, right_distance, forward_range_low, forward_range_upper, left_range_upper, left_range_low, right_range_upper, right_range_low):
    offsets = np.array([((360.0 - meas[1]), meas[2]) for meas in lidar_scan])
    angles = offsets[:,0]
    for ang_ind in range(len(angles)): # pull in angle data from lidar and adjust
        if(angles[ang_ind] > 180):
            angles[ang_ind] = angles[ang_ind] - 360
    distances = offsets[:,1]

    if (forward_range_low > min(angles) and forward_range_upper < max(angles)): # find data points within lower/upper bound
        data_index = [idx for idx in range(len(angles)) if ((angles[idx] > forward_range_low) and (angles[idx] < forward_range_upper))]
        if(len(data_index) != 0 ):
            forward_distance = np.mean(distances[data_index])

    if(left_range_upper < max(angles) and left_range_low > min(angles)):
        data_index = [idx for idx in range(len(angles)) if ((angles[idx] > left_range_low) and (angles[idx] < left_range_upper))]
        if(len(data_index) != 0 ):
            left_distance = np.mean(distances[data_index])

    if(right_range_upper < max(angles) and right_range_low > min(angles)):
        data_index = [idx for idx in range(len(angles)) if ((angles[idx] > right_range_low) and (angles[idx] < right_range_upper))]
        if(len(data_index) != 0):
            right_distance = np.mean(distances[data_index])

    return forward_distance, left_distance, right_distance

# if doing a slight turn, oscillate between off and on
def pulse_motor(turn_command, motor_slight_flag, motor_off):
    m_ret = motor_off
    if motor_slight_flag:
        m_ret = motor_off
    else:
        m_ret = turn_command
    motor_slight_flag = not motor_slight_flag
    return m_ret, motor_slight_flag

# state machine checking if forward threshold to initiate turn has been reached
# otherwise do a deadband control to keep the person centered
def check_state(forward_thresh, min_adjust_dist, max_adjust_dist, single_side_adjust, motor_slight_flag, forward_distance, left_distance, right_distance, cur_yaw, motor_off, slight_right, slight_left, turn_right, turn_left, turn_deg = 90.0):
    # small adjustments to keep people in middle of hallway
    if (forward_distance > forward_thresh) or (forward_distance > right_distance) and (forward_distance > left_distance):
        if abs(right_distance - left_distance) > min_adjust_dist and abs(right_distance - left_distance) < max_adjust_dist and right_distance < single_side_adjust: # recenter
            if right_distance > left_distance: # turn right
                m_cmnd, motor_slight_flag = pulse_motor(slight_right, motor_slight_flag, motor_off)
            else:
                m_cmnd, motor_slight_flag = pulse_motor(slight_left, motor_slight_flag, motor_off)
        else:
            m_cmnd = motor_off
            motor_slight_flag = False
        return 2, -1.0, m_cmnd, motor_slight_flag # keep state the same
    else: # hard turn to whichever side has more room
        if right_distance > left_distance: # turn right (state 3)
            m_cmnd = turn_right
            motor_slight_flag = False
            goal_yaw = bound_angle(cur_yaw - turn_deg)
            return 3, goal_yaw , m_cmnd, motor_slight_flag
        else: # turn left (state 4)
            m_cmnd = turn_left
            motor_slight_flag = False
            goal_yaw = bound_angle(cur_yaw + turn_deg)
            return 4, goal_yaw, m_cmnd, motor_slight_flag

# pass in cur and goal yaw angles clipped to -180 to 180
## TODO add PID to slow down motor as it approaches the correct angle
def check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold = 5.0):
    angle_diff = abs(cur_yaw - goal_yaw)
    if (angle_diff <= turn_threshold) or (angle_diff > 360.0-turn_threshold):
        state = 2
    return state

# state machine for simple hallway code
def brittle_state_machine(turn_right, turn_left, forward_thresh, min_adjust_dist, max_adjust_dist, single_side_adjust, motor_slight_flag, forward_distance, left_distance, right_distance, cur_yaw, goal_yaw, state, m_cmnd,  motor_off, slight_right, slight_left, deg_turn = 90.0, turn_threshold = 5.0):
    if state == 1: # wait for initial lidar readings to be non-zero
        if (forward_distance != 0.0) and (left_distance != 0.0) and (right_distance != 0.0):
            state = 2
            print("Lidar initialized, starting hallway state machine...")
    elif state == 2: # drive straight, do PID with motor to track center
        state, goal_yaw, m_cmnd, motor_slight_flag = check_state(forward_thresh, min_adjust_dist, max_adjust_dist, single_side_adjust, motor_slight_flag, forward_distance, left_distance, right_distance, cur_yaw,  motor_off, slight_right, slight_left, turn_right, turn_left, deg_turn)
    elif state == 3: # turn right deg_turn degrees
        state = check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold)
    elif state == 4: # turn left deg_turn degrees
        state = check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold)

    return state, goal_yaw, m_cmnd, motor_slight_flag

def check_crash(lidar_distance, crash_thresh, state):
    if lidar_distance < crash_thresh:
        state = 2
    return state

# state machine to turn to steer towards most open space
# TODO calibrate the amount to turn it down
def steer_towards_open(distances, lidar_gain, hallway_bump_gain, hallway_steering_thresh, mid_ind, motor_gain, motor_max, motor_min, lidar_arr_mean, forward_thresh):
    num_checks = (len(distances)-1)//2
    # TODO first apply a gain to turn down really large turn angles in case they are not appropriate?
    if any(distances[mid_ind-2:mid_ind+2] < forward_thresh):
        max_ind = np.argmax(distances)
        lidar_turn = lidar_arr_mean[max_ind]
        m_deadband = lidar_turn*motor_gain # proportional control
        m_cmnd = np.clip(m_deadband + np.sign(m_deadband)*motor_min , -motor_max, motor_max) # add deadband offset and clip
    elif any(distances < hallway_steering_thresh):
        min_val = np.amin(distances)
        dist_ind = np.argmin(distances)
        if dist_ind < mid_ind: # right side hitting, so turn left (negative)
            hallway_dist = -(hallway_steering_thresh - min_val)
        else:
            hallway_dist = (hallway_steering_thresh - min_val)
        m_deadband = hallway_dist*hallway_bump_gain
        m_cmnd = np.clip(m_deadband + np.sign(m_deadband)*motor_min , -motor_max, motor_max) # add deadband offset and clip
    else:
        m_cmnd = 0

    return m_cmnd

# state machine to turn to avoid obstacles in front of you
# TODO integrate target adn current heaidings
# TODO increase gain based on closest point to colliding?
def obstacle_state_machine(playing_audio, distances, mid_ind, m_offset, motor_gain, motor_max, deadband, target_heading, current_heading, motor_min, lidar_arr_mean, forward_thresh):
    num_checks = (len(distances)-1)//2

    for i in range(num_checks):
        m_cmnd = 0
        if i == 0 and (distances[mid_ind] > forward_thresh):
            m_cmnd = 0
            break
        else: # check the next indices for closest to center to turn towards
            if (distances[mid_ind + i] > forward_thresh) or (distances[mid_ind - i] > forward_thresh):
                if not playing_audio: # say that obstacle detected ahead
                    play_preloaded_sound("obstacle.mp3")
                    playing_audio = True
                #head_err = target_heading - current_heading
                # in future add weighting between heading error and obstacle avoidance?
                if distances[mid_ind + i] > forward_thresh: # turn left
                    lidar_turn = lidar_arr_mean[mid_ind+i]*m_offset # proportional control
                else: # turn right
                    lidar_turn = lidar_arr_mean[mid_ind-i]*m_offset
                m_deadband = lidar_turn*motor_gain# proportional control # head_err*motor_gain + 
                m_cmnd = np.clip(m_deadband + np.sign(m_deadband)*motor_min, -motor_max, motor_max) # add deadband offset and clip
                break
    return playing_audio, m_cmnd

# state machine for simple hallway code
def shoreline_state_machine(forward_thresh, right_thresh, right_check_thresh, crash_thresh, forward_distance, left_distance, right_distance, motor_max, motor_min, motor_gain, cur_yaw, goal_yaw, state, m_cmnd, deg_turn = 90.0, turn_threshold = 5.0):
    if state == 1: # wait for initial lidar readings to be non-zero
        if (forward_distance != 0.0) and (left_distance != 0.0) and (right_distance != 0.0):
            state = 2
            print("Lidar initialized, starting hallway state machine...")
    elif state == 2: # drive straight, do PID with motor to track center
        state, goal_yaw, m_cmnd = shoreline_check_state(forward_thresh, right_thresh, right_check_thresh, forward_distance, left_distance, right_distance, cur_yaw, motor_max, motor_min, motor_gain, deg_turn)
    elif state == 3: # turn right deg_turn degrees
        state = check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold)
        state = check_crash(right_distance, crash_thresh, state)
    elif state == 4: # turn left deg_turn degrees
        state = check_quad_turn(cur_yaw, goal_yaw, state, turn_threshold)
        state = check_crash(left_distance, crash_thresh, state)
    return state, goal_yaw, m_cmnd

# state machine doing PID to maintain distance to right wall and checking for turns
# TODO, use left distance to help improve something?

# MAKE it more robust to forward threshold, use multiple checks of angles do smaller turns based on which angle triggers threshold
# use a counter for forward thresh occurances and see if that can smooth response
# If its turning left or right and the corresponding distance is less than a threshold stop the turn so it doesn't keep running into a wall

def shoreline_check_state(forward_thresh, right_thresh, right_check_thresh, forward_distance, left_distance, right_distance, cur_yaw, motor_max, motor_min, motor_gain, turn_deg = 90.0):
    # small adjustments to keep people fixed distance from right wall
    if (forward_distance > forward_thresh):# and (forward_distance > right_distance) and (forward_distance > left_distance):
        right_err = right_distance - right_thresh # maybe use motor min to prevent deadband?
        m_deadband = right_err*motor_gain # proportional control
        m_cmnd = np.clip(m_deadband + np.sign(m_deadband)*motor_min , -motor_max, motor_max) # add deadband offset and clip
        return 2, -1.0, m_cmnd # keep state the same
    else: # hard turn to whichever side has more room
        if right_distance > right_check_thresh: # turn right (state 3)
            m_cmnd = motor_max
            goal_yaw = bound_angle(cur_yaw - turn_deg)
            return 3, goal_yaw, m_cmnd
        else: # turn left (state 4)
            m_cmnd = -motor_max
            goal_yaw = bound_angle(cur_yaw + turn_deg)
            return 4, goal_yaw, m_cmnd

########### GPS FUNCTIONS ##################

def distanceToWaypoint(cur_point, cur_waypoint, d2r):
    currentLat = cur_point[0]
    currentLong = cur_point[1]
    targetLat = cur_waypoint[0]
    targetLong = cur_waypoint[1]
    delta = d2r*(currentLong - targetLong)
    sdlong = np.sin(delta)
    cdlong = np.cos(delta)
    lat1 = d2r*(currentLat)
    lat2 = d2r*(targetLat)
    slat1 = np.sin(lat1)
    clat1 = np.cos(lat1)
    slat2 = np.sin(lat2)
    clat2 = np.cos(lat2)
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
    delta = delta**2
    delta += (clat2 * sdlong)**2
    delta = delta**0.5
    denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
    delta = np.arctan2(delta, denom)
    distanceToTarget =  delta * 6372795 
    return distanceToTarget

# get course in degrees (North = 0, West = 270) from current pos to targetLat
# because earth is not a sphere may be off slightly
def courseToWaypoint(cur_point, cur_waypoint, d2r, r2d):
    currentLat = cur_point[0]
    currentLong = cur_point[1]
    targetLat = cur_waypoint[0]
    targetLong = cur_waypoint[1]
    dlon = d2r*(targetLong - currentLong)
    cLat = d2r*(currentLat)
    tLat = d2r*(targetLat)
    a1 = np.sin(dlon) * np.cos(tLat)
    a2 = np.sin(cLat) * np.cos(tLat) * np.cos(dlon)
    a2 = np.cos(cLat) * np.sin(tLat) - a2
    a2 = np.arctan2(a1, a2)
    if (a2 < 0.0):
        a2 += 2*3.14159 # add 2 pi to make it positive
    targetHeading = r2d*(a2)
    return targetHeading

# return the heading of the person in degrees
def readCompassGPS(dec_angle, offset_vec, scale_vec, raw_mag, r2d):
    mag_input = np.array([raw_mag[0],raw_mag[1],raw_mag[2]])
    mag_corr = (mag_input-offset_vec)*scale_vec
    heading = r2d*np.arctan2(-mag_corr[2],  mag_corr[1]) # need to negative sign to go 0-->360 CW
    heading += dec_angle
    if heading < 0.0:
        heading += 360.0
    if heading > 360.0:
        heading -= 360.0
    return int(heading)

# turn the person until they are within the threshold
def calcDesiredTurn(c_head, t_head, max_motor, min_motor, motor_gain, deadband):
    head_err = t_head - c_head
    if abs(head_err) < deadband or abs(head_err) > 360.0-deadband: # close enough, turn off motor
        m_cmnd = 0
    else:
        # proportional command
        p_cmnd = np.clip(abs(head_err)*motor_gain + min_motor , -max_motor, max_motor) # add deadband offset and clip

        if abs(head_err) < 180:
            if head_err > 0: # turn right
                m_cmnd = p_cmnd # turn right
            else:
                m_cmnd = -p_cmnd
        else:
            if head_err > 180: # target is 360, cur is 10 --> turn left
                m_cmnd = -p_cmnd
            else:
                m_cmnd = p_cmnd
    return m_cmnd

def initialization_print(gps): # print once to get info
    print('Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}'.format(
        gps.timestamp_utc.tm_mon,   # Grab parts of the time from the
        gps.timestamp_utc.tm_mday,  # struct_time object that holds
        gps.timestamp_utc.tm_year,  # the fix time.  Note you might
        gps.timestamp_utc.tm_hour,  # not get all data like year, day,
        gps.timestamp_utc.tm_min,   # month!
        gps.timestamp_utc.tm_sec))
    print('Latitude: {0:.6f} degrees'.format(gps.latitude))
    print('Longitude: {0:.6f} degrees'.format(gps.longitude))
    print('Fix quality: {}'.format(gps.fix_quality))
    if gps.satellites is not None:
        print('# satellites: {}'.format(gps.satellites))
    if gps.altitude_m is not None:
        print('Altitude: {} meters'.format(gps.altitude_m))
    if gps.speed_knots is not None:
        print('Speed: {} knots'.format(gps.speed_knots))
    if gps.track_angle_deg is not None:
        print('Track angle: {} degrees'.format(gps.track_angle_deg))
    if gps.horizontal_dilution is not None:
        print('Horizontal dilution: {}'.format(gps.horizontal_dilution))
    if gps.height_geoid is not None:
        print('Height geo ID: {} meters'.format(gps.height_geoid))
