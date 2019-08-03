---
layout: post
title: Python ctypes 
categories: python
tags: [python, ctypes]
image: ctypes.png
description: Using python ctypes to call c/cpp libraries
public: true
---
ctypes allows to call functions exposed from `C` shared libraries or `extern C` in `CPP` world


# Hello ctypes
- c file or cpp with `extern C` section
- compile as so library
- load using ctypes
- call shared library function

&nbsp;  
&nbsp;  
&nbsp;  
## c library
```c
#include <stdio.h>

void hello_world(){
    printf("hello ctypes\n");
}
```

## Compile
```
gcc -c -Wall -Werror -fpic cpy.c
gcc -shared -o libcpy.so cpy.o
```

## python ctypes wrapper
```python
import ctypes
# load so files
lib = ctypes.cdll.LoadLibrary('./libcpy.so')
# call so function
lib.hello_world()
```

&nbsp;  
&nbsp;  
&nbsp;  
# Ctypes and CPP class
- using g++ to compile
- C extern section mapping between the cpp class to python wrapper
  - ctype understand only c 


## cpp library
```cpp
#include <iostream>

class Foo
{
public:
    void bar()
    {
        std::cout << "Hello cpp" << std::endl;
    }
};

extern "C"
{
    #return cpp object 
    Foo *Foo_new() { return new Foo(); }

    #call cpp object method
    void Foo_bar(Foo *foo) { foo->bar(); }
}
```

## compile
```
g++ -c -fPIC foo.cpp -o foo.o
g++ -shared -Wl,-soname,libfoo.so -o libfoo.so  foo.o
```

## python ctypes wrapper
```python
import ctypes
# load so files
lib = ctypes.cdll.LoadLibrary('./libfoo.so')

class Foo(object):
    def __init__(self):
        self.obj = lib.Foo_new()

    def bar(self):
        lib.Foo_bar(self.obj)

if __name__ == "__main__":
    f = Foo()
    f.bar()
```

# More examples
- Passing string
- Passing and return arguments from python code
- Return structure

## passing string 
- python (part of the Foo class)
  
```python
def print_me(self):
        lib.Foo_print_me(self.obj, data)
```

- cpp and c extern

```c
void print_me(const std::string &input)
{
    std::cout << input << std::endl;
}

...

extern "C"
{
    void Foo_print_me(Foo *foo, char const *cstr)
    {
        #convert to std::string
        std::string str(cstr);
        foo->print_me(str);
    }
}
```

## passing arguments
- python (part of Foo class)

```python
# part of python Foo class
def add_ret(self, a, b):
    return lib.Foo_add_ret(self.obj, a, b)
```

```c
// part of Foo class
int add_ret(int a, int b)
{
    return a + b;
}

....

extern "C"
{
    int Foo_add_ret(Foo *foo, int a, int b)
    {
        return foo->add_ret(a, b);
    }
}
```

## Return structure
- Using python class to map c struct

```python
class Point(ctypes.Structure):
    _fields_ = [('x', ctypes.c_int), ('y', ctypes.c_int)]

if __name__ == "__main__":
    # daclera return type
    lib.get_point.restype = Point
    p = lib.get_point()
    print (p.x, p.y)
```

```c
// using extern if compile with g++
extern "C"
{
    typedef struct
    {
        int x;
        int y;
    } Point;

    Point get_point()
    {
        Point p = {2 ,3};
        printf("get point: %d\n", p.y);
        return p;
    }
}
```

## c callback

```python
import ctypes
lib = ctypes.cdll.LoadLibrary('./libfoo.so')

def cb(a):
    print("cb call:{}".format(a))

if __name__ == "__main__":
    c_cb = ctypes.CFUNCTYPE(None, ctypes.c_int)(cb)
    lib.reg(c_cb)
```

```c
extern "C"
{
    void reg(void (*func)(int)){
            func(10);
        }
}
```

# Makefile
```
all: foo.so

clean:
	rm -f *.so *.o 
	
foo.so: foo.o
	g++ -shared -Wl,-soname,libfoo.so -o libfoo.so  foo.o

foo.o: foo.cpp
	g++ -c -fPIC foo.cpp -o foo.o
```

# VSCode task
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "build",
            "type": "shell",
            "options": {
                "cwd": "ctype_example"
            },
            "command": "make",
            "args": [
                "all"
            ],
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "clean",
            "type": "shell",
            "options": {
                "cwd": "ctype_example"
            },
            "command": "make",
            "args": [
                "clean"
            ]
        }
    ]
}
```