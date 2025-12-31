#!/usr/bin/env python3
## coding: UTF-8
# Copyright (c) 2023 Hiroyuki Okada
# This software is released under the MIT License.
# http://opensource.org/licenses/mit-license.php

import sounddevice as sd
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

device_list = sd.query_devices()
print(device_list)

#sd.default.device = [0, 0] # Input, Outputデバイス指定

def callback(indata, frames, time, status):
    # indata.shape=(n_samples, n_channels)
    global plotdata
    data = indata[::downsample, 0]
    shift = len(data)
    plotdata = np.roll(plotdata, -shift, axis=0)
    plotdata[-shift:] = data


def update_plot(frame):
    """This is called by matplotlib for each plot update.
    """
    global plotdata
    line.set_ydata(plotdata)
    return line,

downsample = 10
length = int(1000 * 44100 / (1000 * downsample))
plotdata = np.zeros((length))

fig, ax = plt.subplots()
#fig.canvas.set_window_title("Mic Input Monitor")
line, = ax.plot(plotdata)
ax.set_ylim([-1.0, 1.0])
ax.set_xlim([0, length])
ax.yaxis.grid(True)
fig.tight_layout()

stream = sd.InputStream(
        channels=1,
        dtype='float32',
        callback=callback
    )
ani = FuncAnimation(fig, update_plot, interval=30, blit=True)
with stream:
    plt.show()


