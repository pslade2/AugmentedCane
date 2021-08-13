import time
import adafruit_gps
import serial
import signal
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import natsort

subj = 'GPS'
run = '3' #'20'
folder = os.getcwd() + '/' + subj + '/' + run + '/'
num_files = os.listdir(folder)
num_files = natsort.natsorted(num_files)

def load_waypts(subj, run):
    folder = os.getcwd() + '/' + subj + '/' + run + '/'
    num_files = os.listdir(folder)
    num_files = natsort.natsorted(num_files)
    init_len = 1000
    cur = np.zeros((init_len,2))
    gps_cnt = 0
    for i, f in enumerate(num_files):
        temp_data = np.load(folder+f, allow_pickle=True)
        lf = len(temp_data)
        for j in range(lf):
            cur[gps_cnt+j,:] = temp_data[j][1]
        gps_cnt += lf
        
    cur = cur[:gps_cnt,:]
    return cur

# print(load_waypts(subj,run))
# print("DONE")
# init_len = 1000
# cur = np.zeros((init_len,2))
# target = np.zeros((init_len,2))
# times = np.zeros(init_len)
# gps_cnt = 0
# for i, f in enumerate(num_files):
#     temp_data = np.load(folder+f, allow_pickle=True)
#     lf = len(temp_data)
#     for j in range(lf):
#         times[gps_cnt+j] = temp_data[j][0]
#         cur[gps_cnt+j,:] = temp_data[j][1]
#         target[gps_cnt+j,:] = temp_data[j][2]
#     gps_cnt += lf
# times = times[:gps_cnt]
# cur = cur[:gps_cnt,:]
# target = target[:gps_cnt,:]

plt.figure()
runs = ['1','2','3']
colors = ['b','g','r']
cur_vecs = []
for j in range(3):
    cur = load_waypts(subj,runs[j])
    cur_vecs.append(cur)
    gps_cnt = cur.shape[0]
    for i in range(gps_cnt):
        plt.scatter(cur[i,0], cur[i,1], c=colors[j], alpha = i/gps_cnt)

comb = []
ind1 = 81
start_list = cur_vecs[2][:ind1]
comb =  np.concatenate((start_list,cur_vecs[0][84:182], np.flip(start_list,axis=0)), axis=0)#+ start_list.reverse() 
comb2 =  np.concatenate((start_list,cur_vecs[0][84:132]), axis=0)#+ start_list.reverse() 

plt.scatter(comb[:,0], comb[:,1], c='k')#, alpha = i/gps_cnt)
plt.scatter(comb2[:,0], comb2[:,1], c='m')#, alpha = i/gps_cnt)

print(comb[0,:] - comb[-1,:])
#plt.scatter(target[:,0], target[:,1], c='r')
plt.xlim([min(cur[:,0]), max(cur[:,0])])
plt.ylim([min(cur[:,1]), max(cur[:,1])])

np.save(os.getcwd() + '/path1.npy', comb)
np.save(os.getcwd() + '/path2.npy', comb2)

# loading another

plt.show()



    