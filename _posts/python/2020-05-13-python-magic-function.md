---
layout: post
title: Python Magic functions
categories: python
tags: [magic function]
description: 
public: true
---



# __add__, __iadd__, __radd__

```python
class A():
    def __init__(self, v):
        self.v = v

    def __radd__(self, other):
        if type(other) == int:
            return A(self.v + other)

    def __add__(self, other):
        if type(other) == int:
            return A(self.v + other)
        
        if isinstance(self, A):
            sum = self.v + other.v
            return A(sum)

        raise NotImplementedError("Add operation for this type not implemented")

    def __iadd__(self, val):
        self.v += val
        return self

    def __str__(self):
        return str(self.v)

if __name__ == "__main__":
    a = A(1)
    print (f"using __iadd__: {a+5}")
    print (f"using __radd__: {5+a}")
    print (f"using __add__: {a+A(5)}")
    print (f"using __add__: {sum([A(1), A(2), A(3)])}")
```