---
layout: post
title: Python zmq pub/sub using asyncio
categories: python
tags: [asyncio, zmq, pub-sub]
description: Using python asyncio in zmq
image: zmq_asyncio.png
public: true
---

> message with topic and data always need to be send multipart in asyncio

## demo code

```python
import asyncio
import zmq.asyncio
from zmq.asyncio import Context
import traceback
import logging


# set message based on language
class HelloWorld:
    def __init__(self):
        self.lang = 'eng'
        self.msg = "Hello World"

    def change_language(self):
        if self.lang == 'eng':
            self.lang = 'jap'
            self.msg = "Hello Sekai"
        else:
            self.lang = 'eng'
            self.msg = "Hello World"

    def msg_pub(self):
        self.change_language()
        return self.msg



# manages message flow between publishers and subscribers
class HelloWorldMessage:
    def __init__(self, url='127.0.0.1', port='5555'):
        self.url = "tcp://{}:{}".format(url, port)
        self.ctx = Context.instance()
        # init hello world publisher obj
        self.hello_world = HelloWorld()
        # activate publishers / subscribers
        asyncio.get_event_loop().run_until_complete(asyncio.wait([
            self.hello_world_pub(),
            self.hello_world_sub()
        ]))

    # generates message "Hello World" and publish to topic 'world'
    async def hello_world_pub(self):
        pub = self.ctx.socket(zmq.PUB)
        pub.connect(self.url)
        await asyncio.sleep(.3)
        try:
            while True:
                # ask for message
                msg = self.hello_world.msg_pub()
                # slow down message publication
                await asyncio.sleep(.5)
                # multipart: topic, message; async always needs `send_multipart()`?
                await pub.send_multipart([b'world', msg.encode('utf-8')])

        except Exception as e:
            print("Error with pub world")
            logging.error(traceback.format_exc())

        finally:
            # TODO disconnect pub/sub
            pass

    async def hello_world_sub(self):
        print("Setting up world sub")
        sub = self.ctx.socket(zmq.SUB)
        sub.bind(self.url)
        sub.setsockopt(zmq.SUBSCRIBE, b'world')
        print("World sub initialized")

        try:
            while True:
                [topic, msg] = await sub.recv_multipart()
                # process message
                print("sub get: " + msg.decode('utf-8'))

        except Exception as e:
            print("Error with sub world")
            logging.error(traceback.format_exc())

        finally:
            # TODO disconnect pub/sub
            pass

    
if __name__ == '__main__':
    HelloWorldMessage()
```
# Reference 
- [Eventloops and PyZMQ](https://pyzmq.readthedocs.io/en/latest/eventloop.html)