---
layout: post
title: CMake Build for c++
categories: cpp
tags: [cmake, build]
public: True
image: cmake.png
description: Using cmake to build project using vscode
---
CMake is a tool that generated build scripts for build system
On Linux `cmake` scripts generated `Makefile` by default
CMake base project contain script file named `CMakeLists.txt`  
CMake best practice is to separate build folder from src folder  
- Clean with `rm -rf *` without worry
- Good for `.gitignore`

&nbsp;  
&nbsp;  
&nbsp;  
# Basic hello CMake
## VSCode
Install CMake [extension] for(https://marketplace.visualstudio.com/items?itemName=twxs.cmake)
  - color syntax
  - snippets
  - and more

## CMake
### Minimum commands
- `cmake_minimum_required`: sets minimum version of cmake to be used
- `project`: Sets the name of the project, and stores it in the variable PROJECT_NAME
- `add_subdirectory`: adds a subdirectory to build Hierarchy 
- `add_executable`: adds executable target with given name
- `message`: prints given message

## Basic usage 
### Project struct
```
├── build
├── CMakeLists.txt
├── README.md
└── src
    ├── CMakeLists.txt
    └── hello.cpp
```

#### Root CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.10)
project(hello)

add_subdirectory(src)
```

#### Src CMakeLists.txt
```cmake
message(Start)                  # print message to console
add_executable(app hello.cpp)
# Add install target
# Run with make install
install(TARGETS app DESTINATION ${PROJECT_SOURCE_DIR}/bin)
```

## Build and Compile
From build sub folder

```bash
# from build folder
cmake ..
# or with debug symbols
cmake -DCMAKE_BUILD_TYPE=Debug
make
# or using make install to run install target
make install
```
## Using CMake Generators
Generator `make` file or `ninja`

```bash
#make
cmake -G "Unix Makefiles" ..
cmake -j4
# Ninja
cmake -G "Ninja" ..
# ninja default jobs: 10
ninja
```

&nbsp;
&nbsp;
&nbsp;
## VSCode task
- cmake task
  - > add debug flage with `DCMAKE_BUILD_TYPE`
- make install
  - depend on `cmake` task
- clean
  
```json
{
    "type": "shell",
    "label": "cmake",
    "command": "cmake",
    "args": [
        "-DCMAKE_BUILD_TYPE=Debug",
        ".."
    ],
    "options": {
        "cwd": "${workspaceFolder}/build"
    }
},
{
    "type": "shell",
    "label": "cmake make and install",
    "dependsOn":["cmake"],
    "command": "make",
    "args": [
        "install",
        // "VERBOSE=1"
    ],
    "options": {
        "cwd": "${workspaceFolder}/build"
    },
    "group": {
        "kind": "build",
        "isDefault": true
    }
},
{
    "type": "shell",
    "label": "Clean",
    "command": "rm -rf ${workspaceFolder}/build/*;rm -rf ${workspaceFolder}/bin/* "
}
```
## ninja tasks
- cmake ninja: Generated Ninja 
- Ninja and install: Build with `ninja` and run install target


```json
{
    "type": "shell",
    "label": "cmake ninja",
    "command": "cmake",
    "args": [
        "-G",
        "Ninja",
        ".."
    ],
    "options": {
        "cwd": "${workspaceFolder}/build"
    }
},
{
    "type": "shell",
    "label": "Ninja and install",
    "dependsOn": [
        "cmake ninja"
    ],
    "command": "ninja",
    "args": [
        "install",
    ],
    "options": {
        "cwd": "${workspaceFolder}/build"
    }
}
```
&nbsp;
&nbsp;
&nbsp;
# Find Package
cmake `find_package` command set the external library and include header
- <package name>_INCLUDE_DIRS / <package name>_INCLUDE_DIR
- <package name>_LIBS

Package definition locate :
```bash
/usr/lib/cmake
/usr/local/lib/cmake
```

```
nlohmann-json-dev: /usr/include/json.hpp
nlohmann-json-dev: /usr/include/nlohmann/json.hpp
nlohmann-json-dev: /usr/lib/cmake/nlohmann_jsonConfig.cmake
nlohmann-json-dev: /usr/lib/cmake/nlohmann_jsonConfigVersion.cmake
nlohmann-json-dev: /usr/lib/cmake/nlohmann_jsonTargets.cmake
```

```cmake
find_package( OpenCV REQUIRED )
if (OpenCV_FOUND)
    message(STATUS ${OpenCV_DIR})
endif()
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( app main.cpp )
target_link_libraries( app ${OpenCV_LIBS} )
```

&nbsp;
&nbsp;
&nbsp;
# CMake as a language
## Variables
```cmake
set(Foo "foo data")
set(FooExt "Extension ${Foo}") 
# Built in variables
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W Wall")
```

## Lists
```cmake
set(FOO a b c)
message(STATUS ${FOO})
#abc
list(APPEND FOO x y z)
message(STATUS ${FOO})
#abcxyz
list(LENGTH FOO result)
message(STATUS ${result})
#6
```

## loop
- for loop

```cmake
# loop items in list
set(FOO a b c)
foreach(i ${FOO})
    message(STATUS ${i})
endforeach()

# loop files in folder
file (GLOB SRC "src/*.cpp")
foreach(f ${SRC})
    message(STATUS ${f})
endforeach()
```

## Condition
- string compression [cmake doc](https://cmake.org/cmake/help/git-master/command/if.html)
- `NOT` keyword
- `else()` 


```cmake
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    message("Not GNU Compiler")
else()
    message("GNU Compiler")
endif()
```
&nbsp;
&nbsp;
&nbsp;
# CMake Tips
### 1) Set Cpp standard
```
set(CMAKE_CXX_STANDARD 11)
```
&nbsp;
&nbsp;
### Set linker flags
- static for example

```
set(CMAKE_EXE_LINKER_FLAGS "-static")
```
