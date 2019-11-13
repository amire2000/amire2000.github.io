---
layout: post
title: Matplotlib dynamic graph 
categories: python
tags: [matplotlib, dynamic]
public: true
image: dynamic-graph.png
description: Examples andd methods to plot dynamic graph
---

# Figure parts
![](/images/2019-11-13-07-17-06.png)
- image from matplotlib user guide

# Simple Example

```python
import matplotlib.pyplot as plt
import time
import random
 
#random data
ysample = random.sample(range(-50, 50), 100)
 
xdata = []
ydata = []
 
plt.show()
 
axes = plt.gca()            #get current axes from current figure
axes.set_ylim(-50, +50)
line, = axes.plot(xdata, ydata, 'r-')
 
for i in range(100):
    xdata.append(i)
    ydata.append(ysample[i])
    line.set_xdata(xdata)
    line.set_ydata(ydata)
    axes.set_xlim(i-10, i)
    plt.draw()
    plt.pause(0.01)
    time.sleep(0.5)
 
plt.show()
```

## Animation method example
```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

fig1 = plt.figure()

data = np.random.rand(2, 25)
l, = plt.plot([], [], 'r-')
plt.xlim(0, 1)
plt.ylim(0, 1)
plt.xlabel('x')
plt.title('test')
line_ani = animation.FuncAnimation(fig1, update_line, 25, fargs=(data, l),
                                   interval=50, blit=True)
plt.show()
```

# Reference
-[matplotlib user guide](https://matplotlib.org/3.1.0/tutorials/introductory/usage.html#sphx-glr-tutorials-introductory-usage-py)