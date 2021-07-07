---
layout: post
title: zmq poller
categories: python
tags: [zmq]
image: zmq.png
description: zmq poller
public: true
---


```python
import multiprocessing
import zmq
import time
from random import Random
import logging

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
log = logging.getLogger()

def rep():
    
    context = zmq.Context()
    socket56 = context.socket(zmq.REP)
    socket56.bind("tcp://*:5556")
    socket57 = context.socket(zmq.REP)
    socket57.bind("tcp://*:5557")
    poller = zmq.Poller()
    poller.register(socket56, zmq.POLLIN)
    poller.register(socket57, zmq.POLLIN)

    while True:
        socks = dict(poller.poll())
        if socket56 in socks and socks[socket56] == zmq.POLLIN:
            data = socket56.recv_string(zmq.DONTWAIT)
            socket56.send_string(data + " echo 56")

        if socket57 in socks and socks[socket57] == zmq.POLLIN:
            data = socket57.recv_string(zmq.DONTWAIT)
            socket57.send_string(data + " echo 57")


def req(client_id):
    """Send data
    """
    time.sleep(1)
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    url = f"tcp://127.0.0.1:{client_id}"
    socket.connect(url)
    time.sleep(1)
    for i in range(5):
        msg = f"send: {i}"
        socket.send_string(msg)
        message = socket.recv()
        log.info(message)
        time.sleep(Random().randint(1, 3))
    
    
    
if __name__ == "__main__":
    p_req = multiprocessing.Process(target=req, args=(5556,))
    p_rep = multiprocessing.Process(target=rep)
    p_rep.start()
    p_req.start()

    p_req57 = multiprocessing.Process(target=req, args=(5557,))
    p_req57.start()

    p_req.join()
    p_rep.join()
    p_req57.join()


```