# Code to take SLAM images, process them and run a-star, and give navigation info

import numpy as np
import math
from numpy import genfromtxt
import matplotlib.pyplot as plt
import os
import sys
from scipy import signal
from scipy import ndimage
import pandas as pd
from heapq import *
#import skimage as sk
import skimage.feature
import skimage.transform
import itertools
import numpy.lib.recfunctions as rf
import time

################## Rudimentary helper functions and process variables ##################
def m_to_input_width_cells(n, resolution):
    """
    Given a length in meters, return result in grid spaces of the original image size
    """
    return int(n * resolution)

def m_to_cells(n, down_factor, resolution):
    """
    Given a length in meters, return result in grid spaces of the original image size
    n: length in meters
    """
    return int(n * resolution * down_factor)

def input_xy_to_xy(pos, down_factor):
    """
    Given a position in (x, y) grid units of the original image, return position in grid units
    of the downsampled image
    """
    return (int(pos[0] * down_factor), int(pos[1] * down_factor))

def xy_to_input_xy(pos, down_factor):
    """
    Convert xy position in downsampled grid to xy position in original image
    """
    return (pos[0] / down_factor, pos[1] / down_factor)

def xy_to_rc(coord, map_height):
    """
    Convert xy coordinates to rc coordinates on the downsampled map.
    x, y have origin in bottom left. r, c coordinates used to index
    arrays have origin in top left.

    coord: pass as a tuple.
    """
    x, y = coord
    r = map_height
    r -= (y + 1)
    return int(r), int(x)

def cells_to_m(n, down_factor):
    """
    Given distance in grid cells, return result in meters. Rounding errors
    from downsampling and upsampling will occur.
    n: number of cells in downsampled grid
    """
    return (n / resolution) / down_factor

def get_move_radius_set(step_size_grid):
    out = []
    #calculate up to (inclusive) halfway point and reflect the result
    for x in range(0, math.ceil(step_size_grid/2) + 1):
        y = round(math.sqrt(step_size_grid**2 - x**2))
        #0-index placed in only 4 spots
        if x == 0:
            out = out + [(x, y), (x, -y), (y, x), (-y, x)]
        elif x == y:
            out = out + [(x, y), (x, -y), (-x, y), (-x, -y)]
        else:
            #reflect across vertical/horizontal axes (flip signs)
            out = out + [(x, y), (x, -y), (-x, y), (-x, -y)]
            #reflex across 45-degree axes (flip indices)
            out = out + [(y, x), (-y, x), (y, -x), (-y, -x)]
    return out

#######################################################################################
####################### A STAR HELPER FUNCTIONS ##################################
#later - inline this function call. do after selecting best heuristic
def heuristic(a, b):
    """
    Return straight-shot distance from a to b
    """
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

#Uses downsampled map
def is_legal(pos, processed_map, map_height):
    """
    Checks if the passed position (NOT actual, current position) is legal,
    i.e. inside grid bounds and not on an obstacle
    """
    r, c = xy_to_rc(pos, map_height)
    w, h = processed_map.shape
    if r < 0 or r >= h:
        return False
    if c < 0 or c >= w:
        return False
    return not processed_map[r, c]

############################ PLOTTING METHODS ########################################
def get_axis_ticks(map_height, map_width, n_labels=10):
    """
    Generate tick positions and labels to show meter units on images
    n_labels is number of text labels to show on the axis
    Positions are indices in map for ticks
    Labels are list of text to attach to ticks
    Use plt.xticks(x_positions, x_labels)

    Returns: x_positions, x_labels, y_positions, y_labels
    """
    #range spanning map size in meters, 1 tick per meter
    x = np.arange(0, int(input_width / resolution))
    y = np.arange(0, int(input_height / resolution))

    # step between consecutive labels
    step_x = int(x.shape[0] / n_labels)
    step_y = int(y.shape[0] / n_labels)

    # ticks go at every meter interval
    x_positions = np.arange(0, map_width, map_width/max(x))
    y_positions = np.arange(0, map_height, map_height/max(y))

    #labels is a subset of the full list of ticks
    x_labels = x[::step_x] # labels you want to see
    y_labels = y[::step_y]
    return x_positions, x_labels, y_positions, y_labels


def show_downsampled_map(camera, curr_map, map_to_plot, position, goal, path, plot_r=False, plot_path=False, plot_goal=False, close_set=None, open_set=None):
    """
    Given the agent and its position, and scan radius, display the agent
    and the currently known map.
    Note that tick positions and labels MUST be set globally before!

    plot_r: True if you want to plot radius of latest scan
    plot_path: True if you want to plot path from agent to goal
    plot_goal: True if you want to plot goal marker
    close: 'close set' from astar - just pass what it returns, method handles shape
    open: 'open heap' from astar - just pass, method handles data structure
    """
    fig, ax = plt.subplots()
    plt.imshow(np.flipud(map_to_plot), origin='lower', cmap='gray')
    #plt.imshow(map_to_plot, origin='lower', cmap='gray')

    #agent and goal
    x, y = position
    plt.scatter(x, y, color='r')
    plt.scatter(goal[0], goal[1], color='g', marker='x')

    if open_set is not None:
        open_set = np.array(open_set)[:, 2]
        x_open = []
        y_open = []
        for i in range(open_set.shape[0]):
            x_open.append(open_set[i][0])
            y_open.append(open_set[i][1])
        ax.scatter(x_open, y_open, color='aqua', marker='o', s=(72./fig.dpi)**2)
    if close_set is not None:
        close_set = np.array(close_set)
        ax.scatter(close_set[:, 0], close_set[:, 1], color='green', marker='.')
    if plot_path and path is not None:
        plt.plot(path[:, 0], path[:, 1], ls='--', color='r')
    #Set ticks
    #plt.xticks(x_positions, x_labels)
    #plt.yticks(y_positions, y_labels)
    #plt.show()
    save_dir = os.getcwd() + "/viz"
    save_fold = save_dir + '/run_'+str(len(os.listdir(save_dir)))
    
    #os.mkdir(save_fold)
    #print('save',save_dir)
    num_files = len(os.listdir(save_fold))//5
    if len(str(num_files)) == 1:
        num_files = '0'+str(num_files)
        
    fn = save_fold + '/map_' + str(num_files) + '.png'
    plt.savefig(fn, dpi = 30)
    #print("saved fig", fn)
    plt.close()
    
    # save the original map
    fig, ax = plt.subplots()
    #plt.imshow(np.flipud(map_to_plot), origin='lower', cmap='gray')
    plt.imshow(curr_map, origin='lower', cmap='gray')
    fn = save_fold + '/origmap_' + str(num_files) + '.png'
    plt.savefig(fn, dpi = 30)
    #print("saved fig", fn)
    plt.close()
    
    # take a picture with camera
    #camera.capture(save_fold + '/frame_' + str(num_files) + '.jpg')

    # save map data
    fn = save_fold + '/mapraw_' + str(num_files) + '.npy'
    np.save(fn, map_to_plot)
    fn = save_fold + '/origmapraw_' + str(num_files) + '.npy'
    np.save(fn, curr_map)
    
    # save the current position, orientation, and path
    fn = save_fold + '/state_' + str(num_files) + '.npy'
    np.save(fn, (position, goal, path, time.time()))

######################################################################

#step 2: thicken borders by person radius
def process_map(map_height, map_width, person_radius, filtered_map):
    """
    Updates the map by expanding borders outward by robot radius
    (to prevent squeezing through tiny gaps).
    Also updates downsampled map.
    Requires observed_map to be set; it should have 0s for open areas and
    1s/Trues for obstacles. It may have -1s for unknowns, which will be
    replaced with open areas.
    """
    #refresh the filtered map, cancel -1s to 0s
    processed_map = np.copy(filtered_map) + (filtered_map == -1)
    #assume radius is a square, we will apply this filter on top
    r_box = np.ones((person_radius*2, person_radius*2), dtype=np.bool_)

    for r in range(map_height):
        for c in range(map_width):
            out_top_left = r <= person_radius or c <= person_radius
            out_bottom = r >= map_height - person_radius
            out_right = c >= map_width - person_radius
            if out_top_left or out_bottom or out_right:
                processed_map[r, c] = True
            elif filtered_map[r, c] == True:
                processed_map[r-person_radius:r+person_radius, c-person_radius:c+person_radius] = r_box
    processed_map = np.flipud(processed_map)
    return processed_map

#step 3: compute a*
def astar(position, goal, move_radius_set, processed_map, map_height, max_iterations, step_size_grid):
    """
    Runs astar.
    When it finds a path it returns the path as a list of tuple coordinates. 
    If no path is found, it returns False, so you can test if path before plotting. 
    """
    if goal is None:
        print("Goal is not set")
        return None
    
    close_set = set()
    came_from = {}
    gscore = {position:0}
    fscore = {position:heuristic(position, goal)}

    #heap will contain pairs (fscore, neighbor coord pair)
    oheap = []
    counter = itertools.count()
    niters = 0
    heappush(oheap, (fscore[position], -next(counter), position))

    while oheap and niters < max_iterations:
        #current = heappop(oheap)[2]
        pop = heappop(oheap)
        current = pop[2]
        niters += 1

        if heuristic(current, goal) <= step_size_grid:
            #print("Oheap len: " + str(len(oheap)))
            #print("Closed set len: " + str(len(list(close_set))))
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            #print(niters)
            return np.array(data)
            #return (self.next_move(), list(close_set), oheap)

        close_set.add(current)
        for i, j in move_radius_set:
            neighbor = current[0] + i, current[1] + j
            if not is_legal(neighbor, processed_map, map_height):
                continue

            #already seen better route to this node
            #previously used gscore.get(neighbor,0) but this seems redundant
            #if neighbor is in close_set, it has a g_score
            tentative_g_score = gscore[current] + step_size_grid
            if neighbor in close_set and tentative_g_score >= gscore[neighbor]:
                continue

            #seems redundant to check for this, this is the exact negation of the above
            #if we use gscore.get(neighbor, 10000) it could be more efficient
            #because it tentative < 10000 --> true if neighbor doesn't have a gscore yet
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [k[2] for k in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], -next(counter), neighbor))
    print("open set empty")
    return None#, list(close_set), oheap)

def get_trajectory(position, path, down_factor):
    """
    Given the stored path, position, and header, compute the
    trajectory to the next waypoint.
    Returns dx, dy, angle. Angle starts 3'clock and goes counter-
    clockwise, units in radians. Dx, dy units in meters.
    """
    #last item in path is the next waypoint
    waypoint = path[-1]
    dx = waypoint[0] - position[0]
    dy = waypoint[1] - position[1]

    unit_vector = [dx, dy] / np.linalg.norm([dx, dy])
    angle = np.arccos(np.dot(unit_vector, [1, 0]))
    return (cells_to_m(dx, down_factor), cells_to_m(dy, down_factor), angle)

def update_heading(h):
    """
    Given new heading, update class heading attribute
    h: heading angle, starts 3'clock and goes counter-clockwise,
    units in radians
    """
    heading = h % 2*math.pi

def update_position(x, y, down_factor):
    """
    Given new position coordinates, update class heading attribute.
    x: units in SLAM input grid squares. starts from bottom left,
    unlike r, c indices
    y: same as above
    """
    position = input_xy_to_xy((x, y), down_factor)

def comp_astar(camera, viz_comp, curr_map, resolution, person_radius, canny_sigma, step_size, step_size_grid, position=(150,30), heading=0, goal=(240,270), max_iterations=3000):
    """
    curr_map: map. must be anumpy array with 2 dimensions.
    resolution: number of cells per meter. e.g. 500x500 map depicting 10x10 meters has resolution 50. 
    person_radius: radius of person in meters. 
    canny_sigma: sigma value for smoothing before canny edge detector. Lower = more false positives, higher = more false negatives
    input_width: width of the input map
    input_height: height of the input map
    step_size: length of a single step, in meters. 
    step_size_grid: size of a single step, in cells, in downsampled map
    position: coordinates of the current user position, as (x, y) in the ORIGINAL map units
    heading: angle of the user. count counter-clockwise starting at 3 o'clock
    goal: coordinates of the goal, as (x, y) in the ORIGINAL map units
    max_iterations: number of iterations to try finding a map before giving up
    """
    #clip the grid step size between 1 and the grid step size in the original map
    step_size_grid = max(1, min(step_size_grid, m_to_input_width_cells(step_size, resolution)))
    #downfactor of 0.5 means downsampled map has half the length of the original
    down_factor = 1/(m_to_input_width_cells(step_size, resolution) // step_size_grid)
    #map_height = int(input_height * down_factor)
    #map_width = int(input_width * down_factor)
    map_width = int(curr_map.shape[0] * down_factor)
    map_height = int(curr_map.shape[1] * down_factor)
    person_radius = m_to_cells(person_radius, down_factor, resolution)
    #goal = input_xy_to_xy(goal, down_factor)

    #position = input_xy_to_xy(position, down_factor)
    move_radius_set = get_move_radius_set(step_size_grid)
    path = np.zeros((1, 2))
    #Downsample the map
    # Threshold image to get only sharp edges
    # Canny uses a filter to blend edges

    curr_map_orig = skimage.transform.rescale(curr_map, down_factor, multichannel=False, anti_aliasing=True)
    curr_map = np.copy(curr_map_orig)
    c_thresh = 0.4 # make smaller if too many obstacles that shouldn't be there -- 0.3 is good value too
    thresh_inds = curr_map > c_thresh
    curr_map[thresh_inds] = 1.0
    thresh_inds2 = curr_map <= c_thresh
    curr_map[thresh_inds2] = 0.0
    filtered_map = skimage.feature.canny(curr_map, sigma=canny_sigma, low_threshold=None, high_threshold = 0.3)
    #x_positions, x_labels, y_positions, y_labels = get_axis_ticks()
    processed_map = process_map(map_height, map_width, person_radius, filtered_map)
    position = m_to_pix(position)
    #print(position, goal)
    goal = m_to_pix(goal)
    done = False
    #print(position, goal, processed_map.shape)
    path = astar(position, goal, move_radius_set, processed_map, map_height, max_iterations, step_size_grid)
    true_vec = path != None
    if type(true_vec) != bool:
        if len(path.shape) == 1:
            last_pt = path
        else:
            last_pt = path[-1,:]
        try:
            diff = last_pt - position
            des_heading = -np.arctan2(diff[0],  diff[1])
        except:
            des_heading = -100
            path = None
            done = True
        #print(position, last_pt, diff, des_heading)
        if viz_comp:
            show_downsampled_map(camera, curr_map_orig, processed_map, position, goal, path, plot_path=True, plot_goal=True)
    else:
        des_heading = -100
        
    return des_heading, path, done

def m_to_pix(pos, scale = 6.25):
    pos2 = np.array(pos)*scale*2 + scale*10 # map the meters to pixels --> 10 to 50
    return tuple(pos2)

def correct_heading(heading, convert=True): # pass in raw radians
    if convert:
        d2r = 3.14159/180.0 # degrees to radians
        r2d = 1.0/d2r # radians to degrees
        heading = r2d*heading + 90 # need to negative sign to go 0-->360 CW
    if heading < 0.0:
        heading += 360.0
    if heading > 360.0:
        heading -= 360.0
    return heading

### EXAMPLE USING FUNCTION ABOVE
# cwd = os.getcwd()
# image_path = cwd+"/map_list3.npy"
# map_video = np.load(image_path)
# resolution = 50 #grid squares per meter from raw image
# person_radius = 0.25 #length of person, in meters
# canny_sigma = 6 #sigma for canny edges
# input_width = 500 #n columns in input image
# input_height = 500 #n rows in input image
# step_size = 0.2 #smallest movement unit in meters. min is 0.02.
# step_size_grid = 2 #smallest movement in pixels in downsized image (higher = better resolution)
# position=(1,0)#(50,30)
# heading=0
# goal=(-1,-1)#(240,270)
# viz_astar = True
# n_scans = int(map_video.size / (input_width ** 2))
# map_video = np.reshape(map_video, [n_scans, input_height, input_width])
# 
# for i in range(0, 52): #range(n_scans):
#     curr_map = np.copy(map_video[i, :, :])
#     print(len(curr_map))
#     des_heading, path = comp_astar(viz_astar, curr_map, resolution, person_radius, canny_sigma, step_size, step_size_grid, position, heading, goal)
#     print(des_heading, correct_heading(des_heading, convert=True), path)

### EXAMPLE FOR SINGLE CALL
#variables for testing / change later
# show_images = False
# cwd = os.getcwd()
# image_path = cwd+"/map_list3.npy"
#
# #content variables
# resolution = 50 #grid squares per meter from raw image
# person_radius = 0.25 #length of person, in meters
# canny_sigma = 6 #sigma for canny edges
# input_width = 500 #n columns in input image
# input_height = 500 #n rows in input image
# step_size = 0.2 #smallest movement unit in meters. min is 0.02.
# step_size_grid = 2 #smallest movement in pixels in downsized image (higher = better resolution)
#
#
# #step 0: set key parameters. This section has to be changed.
# #open the image - change this
# map_video = np.load(image_path)
# n_scans = int(map_video.size / (input_width ** 2))
# map_video = np.reshape(map_video, [n_scans, input_height, input_width])
# curr_map = np.copy(map_video[n_scans-1, :, :])
#
# #get current position and heading. using toy data for now
# #using "trig" angles that assumes 3 o'clock is 0 degrees and goes counterclockwise
# position = (150, 30)
# heading = 0
# goal = (240, 270)
#
# #ensure is at least 1, and at most same num steps as original step size (avoids up-sizing)
# step_size_grid = max(1, min(step_size_grid, m_to_input_width_cells(step_size)))
# down_factor = 1/(m_to_input_width_cells(step_size) // step_size_grid)
# map_height = int(input_height * down_factor)
# map_width = int(input_width * down_factor)
#
# person_radius = m_to_cells(person_radius)
# goal = input_xy_to_xy(goal)
# position = input_xy_to_xy(position)
#
# move_radius_set = get_move_radius_set()
# path = np.zeros((1, 2))
# print(position)
# print(goal)
#
# #step 1: downsample (to speed up A*) and filter
# #This handles transformation from 0-255 to 0-1
# curr_map = skimage.transform.rescale(curr_map, down_factor, multichannel=False, anti_aliasing=True)
#
# filtered_map = skimage.feature.canny(curr_map, sigma=canny_sigma)
#
# x_positions, x_labels, y_positions, y_labels = get_axis_ticks()
#
# processed_map = process_map()
#
# if show_images:
#     plt.gray()
#     plt.imshow(np.flipud(curr_map), origin='lower')
#     plt.show()
#     plt.close()
#
#     plt.gray()
#     plt.imshow(np.flipud(filtered_map), origin='lower')
#     plt.show()
#     plt.close()
#
#     show_downsampled_map(processed_map)
#
# path = astar()
# print("Path:",path)
# show_downsampled_map(processed_map, plot_path=True, plot_goal=True)
#
# print(get_trajectory())
