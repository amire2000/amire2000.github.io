---
layout: post
title: NamedTuple
categories: cook_python
tags: [cookbook]
image: cookbook.png
description: tuple on steroids
public: true
---

NamedTuple act like a tuple but allow access values using `name` or `index`

## New syntax 
using `from typing import NamedTuple`

```python
class class_name(NamedTuple):
    field1: datatype
    field2: datatype
```

## Old syntax
using `import collections` module

```python
class_name = collections.namedtuple('class_name', ['field1', 'field2'])
```

&nbsp;  
&nbsp;  
## Demo

```python
# Declare
>>> from typing import NamedTuple
>>> class Point(NamedTuple):
...     x: int
...     y: int
... 
# Defined
>>> p1 = Point(10, 10)
>>> p1.x
10
>>> p1[0]
10
# Tuple are read only
>>> p1.x = 2
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
AttributeError: can't set attribute


```

## Demo 2
- With default values

```python
>>> class PointF(NamedTuple):
...     x:float=0.0
...     y:float=1.0
... 
>>> p2 = PointF()
>>> p2.y
1.0
```

## Demo 3
- Create nametuple from iterable or dictionary

```python
# From list
>>> p4 = Point._make([20,30])
>>> p4.x
20
>>> p4.y
30

# From dict
>>> p5 = Point(**{'x':50, 'y':100})
>>> p5.x
50

```