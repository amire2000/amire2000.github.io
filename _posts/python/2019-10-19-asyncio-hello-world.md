---
layout: post
title: Python asyncio hello world
categories: python
tags: [asyncio]
image: event_loop.png
description: python asyncio first step, basic terms, event loop, task and coroutine
public: true
---

**Concurrency**: Two threads running on signal core CPU
**Parallelism**: Two threads running simultaneously on different core of multi core CPU
**Asynchronous**: Is a programming concept, When execute some `Task` (like networking or other IO) and decide to do some other work instead of waiting.

# Python asyncio
- event loop: manage the execution of tasks
- task: subroutine wait in the event loop
- coroutine: Like subroutine but it's save the state, allow to pause and resumed its execution from the saved state.
- future: represent the result of a task, this result may be exception .


# example

```python
import asyncio

async def count():
    print("One")
    await asyncio.sleep(1)
    print("Two")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(asyncio.wait([
        count(), 
        count(), 
        count()
    ]))
    loop.close()
```

> The `await asyncio.sleep(1)` simulate io function
> The `await` pause the current function run and give back control to `event_loop`
# Reference
- [Coroutines and Tasks](https://docs.python.org/3/library/asyncio-task.html#asyncio.ensure_future)
- [asyncio.ensure_future vs. BaseEventLoop.create_task vs. simple coroutine?](https://stackoverflow.com/questions/36342899/asyncio-ensure-future-vs-baseeventloop-create-task-vs-simple-coroutine)
- [Asyncio.gather vs asyncio.wait](https://stackoverflow.com/questions/42231161/asyncio-gather-vs-asyncio-wait/42246632)
  
async/await pattern

