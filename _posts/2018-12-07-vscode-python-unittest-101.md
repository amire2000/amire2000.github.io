---
layout: post
title: VSCode python unittest 101
categories: vscode 
tags: [vscode, python, unittest]
---

### Guidelines
- Import from `unittest`
- All test methods mast have `test` string in there names
- Test Class inherit from `unittest.TestCase`
- Test Class start with `Test` string
  
### Enable unittest (settings.json)
```json
"python.unitTest.unittestEnabled": true
```

### Project structure
```
├── src
│   ├── inc_dec.py
│   └── __init__.py
└── tests
    ├── __init__.py
    └── test_ine_dec.py
```

### code under test
```python
def increment(x):
    return x+1

def decrement(x):
    return x-1
```

### unittest code
```python
import unittest
from src import inc_dec

class Test_TestIncrementDecrement(unittest.TestCase):
    def test_increment(self):
        self.assertEqual(inc_dec.increment(3), 4)

if __name__ == '__main__':
    unittest.main()
```

## Run and ..
> command -> python:Discover Unit Tests

![](/images/2018-12-07-14-02-25.png)

## View unittest result from 

Output -> Python TestLog

![](/images/2018-12-07-17-34-23.png)

## Run unit test from command line
```bash
# from root folder
# run all tests
python -m unittest tests.test_ine_dec

#run TestCase
python -m unittest tests.test_ine_dec.Test_TestIncrementDecrement

# run specific test
python -m unittest tests.test_ine_dec.Test_TestIncrementDecrement.test_increment
```