import numpy as np
import matplotlib.pyplot as plt


# 1) Load imu data as string
picamdata = np.genfromtxt("hw4data.txt", dtype=str)

# Obtain shape of data/array
size = picamdata.shape

# Extract raw data and convert to milliseconds for better resolution
perf = picamdata[:].astype(np.float64) * 1000

# Create horizontal x axis for plot
x = np.linspace(0,size[0],num=size[0],endpoint=False,dtype=int)

# Plot perf angle with respect to horizontal step 0 - imusize
fig, ax1 = plt.subplots()
ax1.plot(x,perf,ls='solid', color='red',linewidth=2, label='picam-img-det-raw-data')

# 3) Label the axes, title, legend
ax1.set(title="Object Tracking Processing Time",
        ylabel="Processing Time [msec]",
        xlabel="Frame")
plt.show()
fig.savefig('obj_trck_prcs_time.png')
plt.close()

# Plot histogram of data
num_bins = size[0]
fig, ax = plt.subplots()

# the histogram of the data
# n, bins, patches = ax.hist(perf,num_bins, density=True)
ax.hist(perf,bins=num_bins)
ax.set_xlabel('Processing Time [msec]')
ax.set_ylabel('Number of Frames')
ax.set_title('Object Tracking: Processing Time')

# Tweak spacing to prevent clipping of ylabel
fig.tight_layout()
plt.show()
fig.savefig('hist-obj_trck_prcs_time.png')
plt.close()