import numpy as np
import matplotlib.pyplot as plt

# Question 1

# 1) Load imu data as string
encoder_states = np.genfromtxt("encoder_states.txt", dtype=str)

# 2) Plot the pitch angle raw data

# Obtain shape of data/array
size = encoder_states.shape

# Extract pitch angle raw data (5th column)
pitch = encoder_states[:,4].astype(np.int64)

# Create horizontal x axis for plot
x = np.linspace(0,size[0],num=size[0],endpoint=False,dtype=int)

# Plot pitch angle with respect to horizontal step 0 - imusize
fig, ax1 = plt.subplots()
ax1.plot(x,pitch,ls='solid', color='red',linewidth=2, label='pitch-raw-data')

# 3) Label the axes, title, legend
ax1.set(title="Motor Encoder Analysis",
       ylabel="Encoder State",
       xlabel="GPIO Input Reading")

plt.savefig('encoder-state-plot.png')
plt.show()
plt.close()