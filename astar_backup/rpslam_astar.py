#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
                 
Copyright (C) 2018 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE            = '/dev/ttyUSB0'


# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 20

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
try:
    from roboviz import MapVisualizer
    headless = False
except:
    headless = True
import astar_funct as af
import numpy as np
import cane_functions as cf
import os
import RPi.GPIO as IO
import signal
from picamera import PiCamera
import time
import sys
import warnings

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

if __name__ == '__main__':
    # intialize exit function
    signal.signal(signal.SIGINT, exit_function)
    warnings.filterwarnings("ignore")
    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
    # Set up a SLAM display
    if not headless:
        viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
    # Initialize an empty trajectory
    trajectory = []
    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it
    next(iterator)
    viz_rate = 2
    viz_cnt = 0
    # set up folder to save astar images
    save_dir = os.getcwd() + "/viz"
    save_fold = save_dir + '/run_'+str(len(os.listdir(save_dir))+1)
    os.mkdir(save_fold)
    
    # set up camera
    camera = PiCamera()
    camera.resolution = (320,240)#(608, 608)
    #camera.framerate = 80
    
    # set up motor
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

    #cwd = os.getcwd()
    save_images = False
    resolution = 50 #grid squares per meter from raw image
    person_radius = 0.2#12 #length of person, in meters
    canny_sigma = 2 #sigma for canny edges
    input_width = 500 #n columns in input image
    input_height = 500 #n rows in input image
    step_size = 0.25 #smallest movement unit in meters. min is 0.02.
    step_size_grid = 3 #smallest movement in pixels in downsized image (higher = better resolution)
    viz_astar = True
    done = False
    start_time = time.time()
    while not done:
        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(iterator)]
        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles    = [360.0-item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles    = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(previous_distances, scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = slam.getpos()
        # Get current map bytes as grayscale
        viz_cnt += 1
        mapc = np.copy(mapbytes)
        curr_map = np.reshape(mapc, [input_height, input_width])
        
        if viz_cnt%viz_rate == 0:
            slam.getmap(mapbytes)
            # Display map and robot pose, exiting gracefully if user closes it
            if not headless:
                if not viz.display(x/1000., y/1000., theta, mapbytes):
                    exit(0)
        
        if len(curr_map.shape) == 2 and (viz_cnt%viz_rate == 0):
            pos = (x/1000.-5, y/1000.-5) # need to convert this to pixels and subtract center? #(150,30)
            head = af.correct_heading(theta + 180, convert=False)
            goal = (-4, 0) # start going straight, need to convert to pixels #(240,270)
            # save maps and astar viz to debug offline?
            target_heading, path, done = af.comp_astar(camera, viz_astar, curr_map, resolution, person_radius, canny_sigma, step_size, step_size_grid, position=pos, heading=head, goal=goal)
            
            # TODO: return the angle to steer here to close the loop - ADD IMU
            if target_heading != -100:
                target_heading = af.correct_heading(target_heading, convert = True)
                motor_command = -cf.calcDesiredTurn(head, target_heading, motor_max, motor_min, motor_gain, deadband)
            else:
                motor_command = 0
            cf.update_motors(motor_command, p_L, p_R)
            print ("T",np.round(time.time()-start_time,1),"Pos",np.round(pos,1), "Goal",np.round(goal,1), "Head", np.round(head,1),"Targ", np.round(target_heading, 1), "Mot", np.round(motor_command,1))
            if target_heading < 0:
                print(path)
        else:
            #print('Cur map size', len(mapbytes))
            pass
        
 
    # Shut down the lidar connection
    print("Reached goal!")
    lidar.stop()
    lidar.disconnect()
