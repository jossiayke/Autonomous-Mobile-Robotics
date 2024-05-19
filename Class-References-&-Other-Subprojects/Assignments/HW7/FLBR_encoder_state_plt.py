#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

# Question 1

# 1) Load imu data as string
encoder_states = np.genfromtxt("d:\\Documents\\UMD_Docs\\Grad-school\\ENPM701\\HW7\\FLBR_left_encoder_states.txt", dtype=str)

# f = open("FLBR_straight_encoder_states.txt", "r")
# encoder_states = f.read()

# 2) Plot the pitch angle raw data

# Obtain shape of data/array
size = encoder_states.shape

# Extract raw data
enc_BR = np.int64(encoder_states[:,0].astype(np.float64))
enc_FL = np.int64(encoder_states[:,1].astype(np.float64))

# Create horizontal x axis for plot
x = np.linspace(0,size[0],num=size[0],endpoint=False,dtype=int)

# Plot pitch angle with respect to horizontal step 0 - imusize
fig, (ax1, ax2) = plt.subplots(2,1)
#fig.add_axes()
#ax1 = fig.add_subplot(211)
#ax2 = fig.add_subplot(212)

# fig, axes = plt.subplots(nrows=2, ncols=1)

fig.suptitle('Motor Encoder Analysis')

ax1.plot(x,enc_BR,ls='solid', color='blue',linewidth=2, label='BR-encoder-raw-data')
ax2.plot(x,enc_FL,ls='solid', color='red',linewidth=2, label='FL-encoder-raw-data')

# 3) Label the axes, title, legend
ax1.set_ylabel('Back Right Encoder State')
ax2.set_ylabel('Front Left Encoder State')

# axis.set(title="Motor Encoder Analysis",
#        ylabel="Encoder State",
#        xlabel="GPIO Input Reading")
ax1.set_xlim(0,3000)
ax2.set_xlim(0,3000)
plt.savefig('FLBR-left-path-encoder-state-plot.png')
plt.show()
plt.close()

