---
layout: post
title: Smart Pointers part 1 - unique_ptr
categories: cpp
tags: [pointers]
public: true
description: Introduction to SmartPointer, unique_ptr, shared_ptr, weak_ptr, create and move between ownership
image: smart-pointers.png
---

Smart Pointer provide convenient way of managing memory allocation and control objects life time

# Content
- Construction
  - demo
- Usage
  - ownership and move
  - demos
    - pass unique_ptr to function
    - return unique_ptr from function

# unique_ptr

There are two kinds of unique_ptr
- hold scaler
- hold array 

# Construction

```cpp
//c++11
//scaler
std::unique_ptr<int> p1(new int(10));
//array of 10 element
std::unique_ptr<int[]> p2(new int[10]());

//c++14 using make_unique
//scaler initialize int value to 10
auto p1 = make_unique<int>(10);
//array of 10 element
auto p2 = make_unique<int[]>(10)
```

## demo
- Create unique_ptr for `Obj` class 


```cpp
//c++14
//g++ demo1.cpp -std=c++14 -o demo1
#include <iostream>
#include <memory>   //std::unique_ptr
#include <utility>  //std::move

using namespace std;

class Obj{
class Obj{
private:
    std::string data = "smart";
public:
    Obj(){
        cout << "constructor" << endl;
    }

    ~Obj(){
        cout << "destructor" << endl;
    }

    void print_me(){
        cout << data << endl;
    }
};


void foo(){
    cout << "foo start" << endl;
    auto p1 = make_unique<Obj>();
    cout << "foo end" << endl;
}

int main(int argc, char const *argv[])
{
    foo();
    return 0;
}
```

# usage
- using `->`, `*` like traditional pointers

> traditional pointers know call `raw pointers`

- get raw pointer using smart pointer `get` method
  
```cpp
Obj* raw = p1.get();
raw->print_me();
```

# ownership and move
- moving unique pointer between function

> A unique_ptr variable is said to be the owner of the object it points to.

To pass ownership we use `std::move` from `utility` header

## demo (pass to func)
- move unique_ptr to function
  
> According to C++ guidelines a function should take a smart pointer as parameter only to manipulate object life time

```cpp
void foo(unique_ptr<Obj> obj){
    cout << "foo start" << endl;
    cout << "foo end" << endl;
    //obj pointer will be release when foo get out of scope
}

//get raw ptr by ref
void bar(Obj& obj){
    cout << "-----------" << endl;
    cout << "address of: " << &obj << endl;
    obj.print_me();
    cout << "by ref" << endl;
}

//get raw pointer
void bar(Obj* obj){
    cout << "-----------" << endl;
    cout << "address of: " << obj << endl;
    obj->print_me();
    cout << "raw pointer" << endl;
}

int main(int argc, char const *argv[])
{
    //p1 own the pointer 
    auto p1 = make_unique<Obj>();
    //to pass pointer to function
    //we must changed pointer ownership with move
    foo(std::move(p1));
    
    //get raw pointer from smart
    auto ptr = p1.get();
    cout << "address of: " << ptr << endl;
    //pass by raw pointer
    bar(ptr);
    //pass by ref
    bar(*ptr);
    return 0;
}
```

## demo (return from func)
- Return unique_ptr from function

```cpp
unique_ptr<Obj> foo(){
    cout << "foo start" << endl;
    auto p1 = make_unique<Obj>();
    cout << "foo end" << endl;
    return std::move(p1);
}

int main(int argc, char const *argv[])
{
    auto p1 = foo();
    cout << "back to main" << endl;
    return 0;
}
```


# Reference
- [Move smart pointers in and out functions in modern C++](https://www.internalpointers.com/post/move-smart-pointers-and-out-functions-modern-c)