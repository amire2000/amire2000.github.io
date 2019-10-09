---
layout: post
title: Signal running thread to stop
categories: python
tags: [multithread]
public: true
description: Simple code using Event to signal running thread to stop
---

```python
import time 
import threading 
  
class Task():
    def __init__(self):
        self._flag = threading.Event()

    def start(self):
        t = threading.Thread(target=self._run)
        t.start()

    def stop(self):
        self._flag.set()

    def _run(self):
        while (True):
            if self._flag.isSet():
                print("safe exit")
                break
            print ("hello")
            time.sleep(1)

if __name__ == "__main__":
    t1 = Task()
    t1.start() 
    time.sleep(3) 
    t1.stop() 
```