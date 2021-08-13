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
from gtts import gTTS
from pygame import mixer
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

# initialize code and tell that it is starting now
def play_mp3(mixer_obj, mp3_file, text_string):
    tts = gTTS(text_string, lang='en-uk', slow=False)
    tts.save(mp3_file)
    mixer_obj.music.load(mp3_file)
    mixer_obj.music.play()

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

mp3_file = 'file.mp3'
mixer.init()
play_mp3(mixer, mp3_file, "Hello. Starting audio for yolo object recognition.")

camera = PiCamera()
camera.resolution = (608, 608)
camera.capture('frame.jpg')
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
            camera.capture('frame.jpg')
            yolo_proc.stdin.write(b'frame.jpg\n')
            stdout_buf = ''
        if len(stdout.strip())>0:
            #print('get %s' % stdout) # printing frame jpeg, time, num objects, box info
            # searching if any objects of interest meet criteria
            obj_type, best_obj_conf, box = process_yolo(stdout, key_object_list, confidence_theshold)
            if obj_type != 'none': # found object
                # compute distance and play text with audio
                obj_dist, obj_angle, side = compute_distance(box, lr_bias, lr_weight)
                if obj_angle < 10.0:
                    speech_string ="There is a "+obj_type+" "+str(int(obj_dist))+" meters ahead."
                else:
                    speech_string = "There is a "+obj_type+" "+str(int(obj_dist))+" meters ahead and "+str(roundup(obj_angle))+" degrees to the "+side+"."
                play_mp3(mixer, mp3_file, speech_string)
                print(obj_type, best_obj_conf, obj_dist, obj_angle, side)
                time.sleep(4) # slow down rate here?
    except Exception as e:
        print("Error:", e)
