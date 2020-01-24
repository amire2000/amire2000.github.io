---
layout: post
title: Matlab tips and recipes
categories: cheat
tags: [mantlab]
public: true
image: matlab.png
---

## Matrix and vector
```bash
x=[1, 2, 3]     # 1*3 (row) vector
x=[1; 2; 3]     # 3*1 (column) vector
x=[1, 2: 2, 4]  # 2*2 matrix
x=[start:spacing:end]   # vector ends on last value within range
x=x'            # Transpose
x=[1:10]'       # column vector , spacing 1

# element wise
x.*y
x./y
x.^y
```

## Logical
```
&   And
|   Or
~   Not
```

## Relational Operators
```
==      Equal to
~=      Not Equal to
```