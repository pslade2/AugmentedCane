import numpy as np
import sys
import os
import matplotlib.pyplot as plt

cur_dir = os.getcwd()
mag_data = np.load(cur_dir + '/imu_cal_file.npy')
mag_data = mag_data[1:,:]

# plot fig 1
plt.figure()
plt.plot(mag_data[:,0],mag_data[:,1])
plt.plot(mag_data[:,1],mag_data[:,2])
plt.plot(mag_data[:,0],mag_data[:,2])
plt.show()

# hard iron offset?

offset_x = (np.amax(mag_data[:,0]) + np.amin(mag_data[:,0]))/2.0
offset_y = (np.amax(mag_data[:,1]) + np.amin(mag_data[:,1]))/2.0
offset_z = (np.amax(mag_data[:,2]) + np.amin(mag_data[:,2]))/2.0

corr_mag_data = mag_data
corr_mag_data[:,0] = corr_mag_data[:,0] - offset_x
corr_mag_data[:,1] = corr_mag_data[:,1] - offset_y
corr_mag_data[:,2] = corr_mag_data[:,2] - offset_z

plt.figure()
plt.plot(corr_mag_data[:,0],corr_mag_data[:,1])
plt.plot(corr_mag_data[:,1],corr_mag_data[:,2])
plt.plot(corr_mag_data[:,0],corr_mag_data[:,2])
plt.show()

# soft iron offset?
avg_delta_x = (np.amax(corr_mag_data[:,0]) - np.amin(corr_mag_data[:,0]))/2.0
avg_delta_y = (np.amax(corr_mag_data[:,1]) - np.amin(corr_mag_data[:,1]))/2.0
avg_delta_z = (np.amax(corr_mag_data[:,2]) - np.amin(corr_mag_data[:,2]))/2.0
avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z)/3.0
scale_x = avg_delta/avg_delta_x
scale_y = avg_delta/avg_delta_y
scale_z = avg_delta/avg_delta_z
print("Offset terms: ", offset_x, offset_y, offset_z)
print("Scaling terms: ", scale_x, scale_y, scale_z)
soft_corr_data = corr_mag_data

soft_corr_data[:,0] = soft_corr_data[:,0]*scale_x
soft_corr_data[:,1] = soft_corr_data[:,1]*scale_y
soft_corr_data[:,2] = soft_corr_data[:,2]*scale_z

plt.figure()
plt.plot(soft_corr_data[:,0],soft_corr_data[:,1])
plt.plot(soft_corr_data[:,1],soft_corr_data[:,2])
plt.plot(soft_corr_data[:,0],soft_corr_data[:,2])
plt.show()
