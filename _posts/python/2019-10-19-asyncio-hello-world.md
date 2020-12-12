---
layout: post
title: Python asyncio hello world
categories: python
tags: [asyncio]
image: event_loop.png
description: python asyncio first step, basic terms, event loop, task and coroutine
public: true
---
Python asyncio is a library for efficient single-thread concurrent applications. async programing fit for I/O operation tasks

Async operations all run in the same thread, but they yield to one another as needed

Async is about **concurrency**, while threads and multiprocessing are about **parallelism**. Concurrency involves dividing time efficiently among multiple tasks at once. Parallelism involves multiple agents processing multiple tasks side by side.

# Python asyncio
- Python added two keywords, async and await, for creating async operations

### Coroutines
functions prefixed with the `async` keyword become asynchronous functions, also known as coroutines
- Coroutines can only be called from other async functions
- Coroutines can use another keyword, await, which allows a coroutine to wait for results from another coroutine without blocking. Until results come back from the awaited coroutine, Python switches freely among other running coroutines

## Demo1

```python
import asyncio

async def job():
    print("start")
    await asyncio.sleep(2)
    print("stop")

async def start():
    await asyncio.gather(job(), job())
    
if __name__ == "__main__":
    asyncio.run(start())
    print("end")
```
- `asyncio.run()` is used to launch an async function from the non-asynchronous part of our code  
- asyncio.gather() takes one or more coroutines (async functions), run them all and wait for all of them to finish

## Demo2
```python
import asyncio

tasks = []

async def job(i):
    print(f"start: {i}")
    await asyncio.sleep(2)
    print(f"stop: {i}")

async def start():
    for i in range(5):
        tasks.append(asyncio.create_task(job(i)))
    await asyncio.gather(*tasks)
    
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(start())
    print("end")
```

- get_event_loop
- run_until_complete
- create_task
  
The **event loop** ultimately runs scheduled callback

## Demo3
- using future with callback

```python
import asyncio

async def job(i):
    print(f"start: {i}")
    await asyncio.sleep(2)
    print(f"stop: {i}")
    return "job done"

def got_result(future):
    print(future.result())

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    task = loop.create_task(job(1))
    task.add_done_callback(got_result)
    loop.run_until_complete(task)
    print("end")
```
- `create_task`: creates a task from the coroutine
- `add_done_callback`: adds a callback to our task
- `run_until_complete`: runs the event loop until the task is realized. As soon as it has value, the loop terminates
- `run_forever`: alternative function to `run_until_complete` don't exit on task's ends
  
&nbsp;  
&nbsp;  
&nbsp;  


## Demo4
using `ensure_future` method instead `create_task`

> [ensure_future vs create_task](https://stackoverflow.com/questions/36342899/asyncio-ensure-future-vs-baseeventloop-create-task-vs-simple-coroutine)

```python
import asyncio

async def job(i):
    print(f"start: {i}")
    await asyncio.sleep(2)
    print(f"stop: {i}")
    return "job done"

def got_result(future):
    print(future.result())

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    task = asyncio.ensure_future(job(1))
    task.add_done_callback(got_result)
    loop.run_forever()
```

# Reference
- [Get started with async in Python](https://www.infoworld.com/article/3454442/get-started-with-async-in-python.html)