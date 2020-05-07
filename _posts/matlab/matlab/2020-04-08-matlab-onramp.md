---
layout: post
title: Matlab onramp
categories: matlab
tags: [hello]
---

## Extracting multiple elements
```
density([1 3 6])
```

## Changing Values
> 1 base indexing

- Change first element `a(1) = 11`


## Array 
multiple: `.*`

## logical index
```
a = (1:10)
>> a
a =
     1     2     3     4     5     6     7     8     9    10

>> a<5
ans =
   1   1   1   1   0   0   0   0   0   0

>> a(a<5)
ans =

     1     2     3     4
```
# Reference
- [MATLAB Onramp](https://www.mathworks.com/learn/tutorials/matlab-onramp.html)