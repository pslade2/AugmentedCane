# run with sudo python yolo_...
# first run python3 workers.py

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
#from gtts import gTTS
#from pygame import mixer
import numpy as np
import math
#import cane_functions as cf
import os
#import RPi.GPIO as IO
import signal
import sys
from multiprocessing import Process, Queue
#import workers
iframe = 0
# cwd = os.getcwd()
# def clear(q):
#     try:
#         while True:
#             q.get_nowait()
#     except:
#         pass
# 
# # initializing workers
# q = Queue() # queue for IMU commanded angles
# b = Queue()
# imuProc = Process(target=workers.IMU_steering(q, b))
# imuProc.start()

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
    #print(text)
    box = np.zeros(4) # center (x,y) + box (width, height)
    obj_type = 'none'
    best_obj_conf = 0
    best_obj_ind = 0
    if len(text_lines) >= 2: # make sure enough lines
        num_boxes = int(text_lines[1]) # determine number of objects
        num_objects = len(text_lines) - 3 - num_boxes
        sizes = np.zeros(num_boxes)
        for i in range(num_boxes):#num_objects): # loop through each object
            box_ind = i+2
            pix_line = text_lines[box_ind]
            box[3] = float(pix_line[-9:-1])
            box[2] = float(pix_line[-18:-10])
            box[1] = float(pix_line[-40:-33])
            box[0] = float(pix_line[-49:-42])
            #print(box)
            sizes[i] = float(box[3]*box[2])
        
        stop_ind = np.argmax(sizes)
        #print(sizes, stop_ind)
        pix_line = text_lines[2+stop_ind]
        box[3] = float(pix_line[-9:-1])
        box[2] = float(pix_line[-18:-10])
        box[1] = float(pix_line[-40:-33])
        box[0] = float(pix_line[-49:-42])
        
        for i in range(num_objects):
            line_ind = 2+i+num_boxes # start at first object line
            obj_txt = text_lines[line_ind]
            obj_list = obj_txt.split(':')
            obj = str(obj_list[0])
            #print(obj)
            obj_conf = int(obj_list[1][1:-1])
            # check for objects we care about [stop sign, person, car ..?]
            # and check if they have good enough certainty
            if (obj in key_object_list) and (obj_conf >= confidence_theshold) and (obj_conf >= best_obj_conf):
                best_obj_conf = obj_conf
                best_obj_ind = i
                obj_type = obj
#                 pix_line_ind = line_ind - num_boxes
#                 print(pix_line_ind, line_ind,  num_objects)
#                 pix_line = text_lines[pix_line_ind]
#                 print(pix_line)
#                 # find the pixel center and box size
#                 box[3] = float(pix_line[-9:-1])
#                 box[2] = float(pix_line[-18:-10])
#                 box[1] = float(pix_line[-40:-33])
#                 box[0] = float(pix_line[-49:-42])
#                 #print('x pos:',np.round(box[0],3), 'y pos:',np.round(box[1],3), 'w:',np.round(box[3],3), 'h:','x pos:',np.round(box[0],3))
    #print(obj_type, best_obj_conf)
    return obj_type, best_obj_conf, box
            

# compute distance and relative location of object
# horizontal field of view (hor_fov) is 62.2 degrees for rpi camera V2
def compute_distance(box, lr_bias, lr_weight, hor_fov=62.2):
    angle = int((box[0]-0.5)*hor_fov)
    if angle > 0:
        side = 'right'
    else:
        #angle = -angle
        side = 'left'
    obj_height = box[3]
    obj_width = box[2]
    
    dist = 0.99654809*obj_width - 38.56318845*obj_height + 8.613978322524407
    #obj_height*lr_weight + lr_bias
    return dist, angle, side
    
# Init camera
camera = PiCamera()
camera.resolution = (608, 608)
camera.capture('frame.jpg')
sleep(0.1)
#spawn darknet process
yolo_proc = Popen(["./darknet",
                   "detect",
                   "./cfg/yolov3-tiny.cfg",
                   "./yolov3-tiny.weights",
                   "-thresh","0.3"],
                   stdin = PIPE, stdout = PIPE)
fcntl.fcntl(yolo_proc.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
stdout_buf = ''

photo_directory = '/home/pi/Desktop/lidar_cane/yolo_pics/test/'
num_runs = len(os.listdir(photo_directory)) # make
save_cnt = 0
save_dir = photo_directory + 'run_' + str(num_runs+1) + '/'
cwd = os.getcwd() + '/obj_angs'
try:
    shutil.rmtree(cwd)
except:
    pass
sleep(0.2)
os.mkdir(cwd)

# creating directory to save pics
save_pics = os.getcwd() + '/m8_60' # change this pics name for different test cases
pic_cnt = 0
try:
    os.mkdir(save_pics)
except:
    pass

start_time = time.time()
while True:
    try:
        # take in images at a fixed rate, pass through yolo
        select.select([yolo_proc.stdout], [], [])
        stdout = yolo_proc.stdout.read().decode('utf-8')
        #print(stdout) # printing out object categories and confidence levels
        stdout_buf = stdout
        if 'Enter Image Path' in stdout_buf:
            try:
               im = cv2.imread('predictions.png')
               cv2.imshow('yolov3-tiny',im)
               key = cv2.waitKey(5) 
            except Exception as e:
                print("Error:", e)
            pic_dir = save_pics + '/frame_' + str(save_cnt) +'.jpg'
            stamp_dir = save_pics + '/stamp_' + str(save_cnt)
            camera.capture(pic_dir)#'frame.jpg')
            np.save(stamp_dir, time.time())
            # TODO: how to save backup, by incrementing the number of name of pic
            yolo_proc.stdin.write(b''+pic_dir+'\n')#b'frame.jpg\n')
            stdout_buf = ''
        if len(stdout.strip())>0:
            #print('get %s' % stdout) # printing frame jpeg, time, num objects, box info
            # searching if any objects of interest meet criteria
            obj_type, best_obj_conf, box = process_yolo(stdout, key_object_list, confidence_theshold)
            if obj_type != 'none': # found object
                # compute distance and play text with audio
                obj_dist, obj_angle, side = compute_distance(box, lr_bias, lr_weight)
                # np.save([obj_type, best_obj_conf, obj_dist, obj_angle, side, time.time()], 'data_'+str(cnt)+'.npy')
                # steer towards the stop sign                np.savetxt(cwd+'obj_angle_'+save_cnt, obj_angle)
                np.save(cwd+'/obj_angle_'+str(save_cnt), obj_angle)
                np.save(save_pics + '/obj_angle_'+str(save_cnt), [box, obj_angle, obj_dist])

                save_cnt += 1
                #np.save(cwd+'/obj_angle.npy', obj_angle)
                #q.put([obj_angle, side, obj_dist])
                print ("T",np.round(time.time()-start_time,1),"Angle",np.round(obj_angle,1), "Conf",np.round(best_obj_conf,1))#, "Mot", np.round(motor_command,1))

    except Exception as e:
        print("Error:", e)
