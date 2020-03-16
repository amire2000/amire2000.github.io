---
layout: post
title: VSCode CMake and google test
categories: cpp
tags: [cmake, gtest]
description: Using VSCode with google tests and CMake
public: true
---

# Project struct
```
├── build
├── CMakeLists.txt
├── lib
│   └── googletest
├── src
│   ├── CMakeLists.txt
│   └── main.cpp
└── tst
    ├── CMakeLists.txt
    └── main.cpp
```
&nbsp;  
&nbsp;  
&nbsp;  
# VSCode config
- [CMake tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
- [GoogleTestAdapter](https://marketplace.visualstudio.com/items?itemName=DavidSchuldenfrei.gtest-adapter)
  
&nbsp;  
&nbsp;  
&nbsp;  
## Root Folder
### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.12)
project(tst_demo VERSION 0.1.0)

include(CTest)
enable_testing()

add_subdirectory(lib/googletest)
add_subdirectory(src)
add_subdirectory(tst)
```

## tst folder
### CMakeLists.txt
```cmake
add_executable(main_tst main_test.cpp)
target_link_libraries(main_tst
    gtest_main
)

add_test(
    NAME main_tst
    COMMAND main_tst
)
```

### main_test.cpp
```cpp
#include <gtest/gtest.h>


TEST(main_test, test1){
    EXPECT_TRUE(false);
}
```

## VSCode
### launch.json
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/build/tst/main_tst",
            "args": [],
            "stopAtEntry": false,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# Run Test
- Press `f7` to build (cmake tools key)
- Run tests from `tool bar`

![](/images/2020-03-16-06-47-07.png)

- Run test from explorer
  - Set Debug config first

![](/images/2020-03-16-06-48-37.png)
&nbsp;  
&nbsp;  
&nbsp;  

## Test Fixtures
Placeholder for code and data used by multiple tests

>SetUp and TearDown called for each test

```cpp
#include <gtest/gtest.h>
#include <iostream>

using std::cout; 

class DemoTest : public ::testing::Test
{
  protected:

  virtual void SetUp()
  {
      cout << "setup\n";
  }

  virtual void TearDown()
  {
      cout << "tear down\n";
  }
};

TEST_F(DemoTest, test1){
    EXPECT_TRUE(false);
}

TEST_F(DemoTest, test2){
    EXPECT_TRUE(true);
}
```
&nbsp;  
&nbsp;  
&nbsp;  
# Reference
- [Introduction to Google Test and CMake](https://youtu.be/Lp1ifh9TuFI)
- [Googletest Primer](https://github.com/abseil/googletest/blob/master/googletest/docs/primer.md)