---
layout: post
title: Using inspect module to analyze my code
categories: [python]
tags: [inspect, analyze]
public: true
---

```python
import inspect
import sample
import json

db = {}

def learn(key, obj):
    for name, data in inspect.getmembers(obj):
        if name.startswith('__'):
            continue
        key = key + "_" + name
        db[key]=json.loads(data.__doc__)


for key, data in inspect.getmembers(sample, inspect.isclass):
    print('{} : {!r}'.format(key, data))
    learn(key, data)

print(json.dumps(db))
```

## Code under inspection
```python
import json

def decorator(source, dest, msg):
    def _decorator(f):
        def __decorator():
            f()
        
        data = {
            "src": source,
            "dst": dest,
            "msg": msg
        }
        __decorator.__doc__ = json.dumps(data)
        __decorator.__name__ = f.__name__
        return __decorator
    return _decorator

class MyClass():
    @decorator("B", "C", "msg1")
    def method1(self):
        """class method"""
        pass

class MyClass1():
    @decorator("A", "B", "msg")
    def method1(self):
        """class method"""
        pass
```