---
layout: post
title: Python cpp bindings
categories: python
tags: [python, cpp, bindings]
---

## pybind11
### install
```bash
sudo  apt install python3-pip
sudo pip3 install pytest
```
```bash
git clone https://github.com/pybind/pybind11
cd pybind11
mkdir build
cd build
cmake ..
make check -j 4
sudo make install 
#/usr/local/include/
```

### first app
```
├── CMakeLists.txt
├── setup.py
└── src
    └── main.cpp
```
#### src/main.cpp
```cpp
#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(cmake_example, m) {
    m.def("add", &add);
    
     m.attr("__version__") = VERSION_INFO;
}
```
> VERSION_INFO defined by setup.py
#### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 2.8.12)
project(cmake_example)

find_package(pybind11 REQUIRED)
pybind11_add_module(cmake_example src/main.cpp)
```

#### setup.py
```python
import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """
    setup entry point
    """
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
        build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)

setup(
    name='cmake_example',
    version='0.0.1',
    author='dev',
    author_email='dev@gmail.com',
    description='A test project using pybind11 and CMake',
    long_description='',
    ext_modules=[CMakeExtension('cmake_example')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)

```
#### Install (run setup)

```bash
# From project parent folder (pybind11_t project root folder)
pip3 install -v ./pybind11_t
#install .so into ~/.local/lib/python3.6/site-packages
```

#### Check 
> sudo pip3 install ipython

```bash
ipython3
```
```python
In [1]: import cmake_example                                                                                          

In [2]: cmake_example.add(1,2)                                                                                        
Out[2]: 3
In [3]: cmake_example.__version__                                                                                     
Out[3]: '0.0.1'
```


#### callbacks
- cpp
```cpp
#include <pybind11/pybind11.h>
#include <iostream>
namespace py = pybind11;

void run_test(const py::object& f){
    py::object pyr = f(5);
   int r = pyr.cast<int>();
   std::cout << "result: " << r << std::endl;
}

PYBIND11_MODULE(cmake_example, m) {
    m.def("run_test", &run_test);
}
```
- run setup 
- python
```python
In [2]: import cmake_example as m                                                                                     

In [3]: m.__version__                                                                                                 
Out[3]: '0.0.2'

In [4]: def test(x): 
   ...:     return 2*x 
   ...:                                                                                                               

In [5]: m.run_test(test)                                                                                              
result: 10

```
## References
- [Python - C++ bindings](https://www.hardikp.com/2017/12/30/python-cpp/)
- [pybind11 github](https://github.com/pybind/pybind11)
- [pybind11](https://pybind11.readthedocs.io/en/latest/basics.html)
- [pybind11 example with cmake](https://github.com/pybind/cmake_example)
- [pybind11/tests/test_callbacks.cpp](https://github.com/pybind/pybind11/blob/master/tests/test_callbacks.cpp)
- [pybind11 basic callback, incompatible function signature error](https://stackoverflow.com/questions/47474961/pybind11-basic-callback-incompatible-function-signature-error)
## To read
-[Building and testing a hybrid Python/C++ package](http://www.benjack.io/2017/06/12/python-cpp-tests.html)


