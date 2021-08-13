# run yolo and create audio to inform distances to key objects
from picamera import PiCamera
from subprocess import Popen, PIPE
import threading
from time import sleep
import os, fcntl
import cv2
import select
import shutil
import time
import numpy as np
import math
iframe = 0

# parameters for detection
key_object_list = ['stop sign']#, 'person', 'car']
confidence_theshold = 30 # confidence (%) of object before trusting yolo
obj_conf = 0
obj_type = ''
obj_box = np.zeros(4)
obj_dist = 0
obj_angle = 0
lr_bias = 0.0
lr_weight = 0.0

def roundup(x): # round to nearest ten digit
    return int(math.ceil(x / 10.0)) * 10

def process_yolo(text, key_object_list, confidence_theshold):
    text_lines = stdout.splitlines()
    box = np.zeros(4) # center (x,y) + box (width, height)
    obj_type = 'none'
    best_obj_conf = 0
    best_obj_ind = 0
    if len(text_lines) >= 2: # make sure enough lines
        num_boxes = int(text_lines[1]) # determine number of objects
        num_objects = len(text_lines) - 3 - num_boxes
        for i in range(num_objects): # loop through each object
            line_ind = 2+i+num_boxes # start at first object line
            obj_txt = text_lines[line_ind]
            obj_list = obj_txt.split(':')
            obj = str(obj_list[0])
            obj_conf = int(obj_list[1][1:-1])
            # check for objects we care about [stop sign, person, car ..?]
            # and check if they have good enough certainty
            if (obj in key_object_list) and (obj_conf >= confidence_theshold) and (obj_conf >= best_obj_conf):
                best_obj_conf = obj_conf
                best_obj_ind = i
                obj_type = obj
                pix_line_ind = line_ind - num_objects
                pix_line = text_lines[pix_line_ind]
                # find the pixel center and box size
                box[3] = float(pix_line[-9:-1])
                box[2] = float(pix_line[-18:-10])
                box[1] = float(pix_line[-40:-33])
                box[0] = float(pix_line[-49:-42])
                print(pix_line[-9:-1], pix_line[-18:-10], pix_line[-40:-33], pix_line[-49:-42])
    return obj_type, best_obj_conf, box

# compute distance and relative location of object
# horizontal field of view (hor_fov) is 62.2 degrees for rpi camera V2
def compute_distance(box, lr_bias, lr_weight, hor_fov=62.2):
    angle = int((box[0]-0.5)*hor_fov)
    if angle > 0:
        side = 'right'
    else:
        angle = -angle
        side = 'left'
    obj_height = box[3]
    dist = obj_height*lr_weight + lr_bias
    return dist, angle, side

# camera = PiCamera()
# camera.resolution = (608, 608)
# camera.capture('frame.jpg')
sleep(0.1)
#spawn darknet process
yolo_proc = Popen(["./darknet",
                   "detect",
                   "./cfg/yolov3-tiny.cfg",
                   "./yolov3-tiny.weights",
                   "-thresh","0.1"],
                   stdin = PIPE, stdout = PIPE)
fcntl.fcntl(yolo_proc.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
stdout_buf = ''

cwd = os.getcwd()
photo_directory = '/home/pi/Desktop/lidar_cane/yolo_pics/train/stop_sign/'
#stop_sign_folders = os.listdir(photo_directory) # get folders of stop signs
#stop_sign_folders.sort()
stop_sign_folders = ['1']
dist_folders = ['2.5','5','7.5','10']
ang_folders = ['0','30','60']
########## INIT RUN TO NOT BE OFF BY ONE
try:
    # copy frame to the dark_NN_folder as frame.jpg   
    yolo_proc.stdin.write(b'frame.jpg\n')
    stdout_buf = ''
    # take in images at a fixed rate, pass through yolo
    select.select([yolo_proc.stdout], [], [])
    stdout = yolo_proc.stdout.read().decode('utf-8')
    #print(stdout) # printing out object categories and confidence levels
    stdout_buf = stdout
    if 'Enter Image Path' in stdout_buf:
        try:
           im = cv2.imread('predictions.png')
           cv2.imshow('yolov3-tiny',im)
           # copy predictions to the saved folder and rename
           #shutil.copyfile(cwd+'/predictions.png', final_dir+photo[:-4]+'_yolo.png')
           key = cv2.waitKey(5) 
        except Exception as e:
            print("Error:", e)
except Exception as e:
    print("Error:", e)
##############

# make matrix [features, ss_num, ss_dists, ss_angles, cnts] to store values
# features = [confidence, box (4 vals), ..?]
stop_sign_data = np.zeros((5, len(stop_sign_folders), len(dist_folders), len(ang_folders)))

for j, ss_fold in enumerate(stop_sign_folders): # ss num
#     dist_folders = os.listdir(photo_directory+ss_fold+'/')
#     dist_folders.sort()
    for k, dist_fold in enumerate(dist_folders): # dist dir
#         ang_folders = os.listdir(photo_directory+ss_fold+'/'+dist_fold+'/')
#         ang_folders.sort()
        for l, ang_fold in enumerate(ang_folders): # ang fold
            final_dir = photo_directory+ss_fold+'/'+dist_fold+'/'+ang_fold+'/'
            photos = os.listdir(final_dir)
            for photo in photos: # remove old yolo photos
                if photo[-8:-4] == 'yolo':
                    os.remove(final_dir + photo) # remove
            photos = os.listdir(final_dir)
            photos.sort()
            ind_cond_data = np.zeros((5, len(photos)-1))
            for i, photo in enumerate(photos):
                try:
                    # copy frame to the dark_NN_folder as frame.jpg
                    shutil.copyfile(final_dir+photo, cwd+'/frame.jpg')
    
                    yolo_proc.stdin.write(b'frame.jpg\n')
                    stdout_buf = ''
                    # take in images at a fixed rate, pass through yolo
                    select.select([yolo_proc.stdout], [], [])
                    stdout = yolo_proc.stdout.read().decode('utf-8')
                    #print(stdout) # printing out object categories and confidence levels
                    stdout_buf = stdout
                    if 'Enter Image Path' in stdout_buf:
                        try:
                           im = cv2.imread('predictions.png')
                           cv2.imshow('yolov3-tiny',im)
                           # copy predictions to the saved folder and rename
                           if i != 0: # don't copy first one because it's wrong
                               shutil.copyfile(cwd+'/predictions.png', final_dir+photos[i-1][:-4]+'_yolo.png')
                           key = cv2.waitKey(5) 
                        except Exception as e:
                            print("Error:", e)
                        #camera.capture('frame.jpg')
                        # copy frame to the dark_NN_folder as frame.jpg
                        #shutil.copyfile(final_dir+photo, cwd+'/frame.jpg')
#                         
                        #yolo_proc.stdin.write(b'frame.jpg\n')
                        #stdout_buf = ''
                    if len(stdout.strip())>0 and i!=0:
                        #print('get %s' % stdout) # printing frame jpeg, time, num objects, box info
                        # searching if any objects of interest meet criteria
                        obj_type, best_obj_conf, box = process_yolo(stdout, key_object_list, confidence_theshold)
                        if obj_type != 'none': # found object
                            ind_cond_data[0,i-1] = best_obj_conf
                            ind_cond_data[1:,i-1] = box
                            # compute distance and play text with audio
                            #obj_dist, obj_angle, side = compute_distance(box, lr_bias, lr_weight)
                            print(obj_type, best_obj_conf)#, obj_dist, obj_angle, side)
                            #time.sleep(4) # slow down rate here?
                except Exception as e:
                    print("Error:", e)
            # avg the results from that batch of photos
            rm_inds = []
            for i in range(ind_cond_data.shape[1]):
                if ind_cond_data[0,i] == 0:
                    rm_inds.append(i)
            # remove bad indeces
            ind_cond_proc = np.delete(ind_cond_data, rm_inds)
            try:
                ind_cond_avg = np.mean(ind_cond_data, axis=1) # take avg
                stop_sign_data[:, j, k, l] = ind_cond_avg
            except:
                print("Not enough samples to compute mean for "+ss_fold+" "+dist_fold+" "+ang_fold)
                stop_sign_data[:, j, k, l] = np.zeros(5)
# save the final matrix to the
np.save(photo_directory+'../../train_data.npy', stop_sign_data)
