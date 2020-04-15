---
layout: post
title: Matlab python integration
categories: matlab
tags: [python]
description:
public: true
---

## Setup
- Create Virtual env
- Install


```bash
python3 -m venv matsim
source matsim/bin/activate

# <matlab_path>/extern/engines/python
# ~/MATLAB/R2018b/extern/engines/python
(matsim)python setup.py install
```

### Check
```bash
python
>>> import matlab.engine
>>> eng = matlab.engine.start_matlab()
>>> x=4.0
>>> eng.workspace["y"]=x
>>> a = eng.eval("sqrt(y)")
>>> print(a)
2.0
```

# Simulink
variables that are needed to be seen by Python must be sent to Workspace through a ‘To Workspace’ Block


# Reference
- [Controlling a Simulink Model by a Python Controller](https://medium.com/@soutrikbandyopadhyay/controlling-a-simulink-model-by-a-python-controller-2b67bde744ee)

```python
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import psutil
import threading
import time

class Model():
    def __init__(self):
        self.x, self.y = [], []
        self.counter = 0

    def read(self):
        self.counter += 1
        self.x.append(self.counter)
        self.y.append(psutil.cpu_percent())

    def run(self):
        while True:
            if self.counter > 100:
                print("Model fetch data end")
                break
            
            self.read()
            time.sleep(1/10)

    def start(self):
        t = threading.Thread(target=self.run)
        t.setDaemon(True)
        t.start()


class Controller():
    def __init__(self, viewer, model):
        self.model = model
        self.viewer = viewer
        anim = animation.FuncAnimation(self.viewer.fig,
                self.animate,
                init_func=self.viewer.init,
                frames=10, interval=300, blit=True)
        self.model.start()
        self.viewer.show()



    def animate(self, f_id):
        self.viewer.ax.set_xlim(left=max(0, self.model.counter-5), right=self.model.counter)
        self.viewer.line.set_data(self.model.x, self.model.y)
        return self.viewer.line,

class Viewer():
    def __init__(self):
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(0, 10), ylim=(0, 100))
        self.line, = self.ax.plot([], [], lw=2)

    def init(self):
        self.line.set_data([], [])
        return self.line,        

    def show(self):
        plt.show()

if __name__ == "__main__":
    model = Model()
    viewer = Viewer()
    controller = Controller(viewer, model)
```