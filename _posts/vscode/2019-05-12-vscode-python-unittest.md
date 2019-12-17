---
layout: post
title: Python Unittest
categories: vscode
tags: [python, unittest]
image: python-unit-test.png
public: True
description: Using python unittest framework 
---
# Content
- Unittest template
- Test fixture
- VSCode unittest setting and extension
- Project structure



# Unittest template
```python
import unittest

class TestSum(unittest.TestCase):
    def test_method(self):
        # Arrange
        # Act
        # Assert

if __name__ == '__main__':
    unittest.main()
```

# Test fixture
- `setUp()` \ `tearDown()`: method run before and aftter every test
- `setUpClass()` / `tearDownClass()` – before and after a class of tests
- `setUpModule()` / `tearDownModule()` – before and after a module of tests

# VSCode unittest setting and extension
- unittest framework settings
```json
    "python.testing.unittestArgs": [
        "-v",
        "-s",
        "./test",
        "-p",
        "test*.py"
    ],
    "python.testing.pyTestEnabled": false,
    "python.testing.nosetestsEnabled": false,
    "python.testing.unittestEnabled": true,
    "python.testing.cwd": "/src"
```
- unittest args:
  - -v: verbose
  - -s: test start folder
  - ./test 
  - -p: test method name pattern
  - test*.py 

- Enable only unittest framework
- `python.testing.cwd`: code under tests

# Run
## VSCode
![](/images/2019-05-12-18-37-41.png)


# Project structure
```
project/
│
├── my_app/
│   └── __init__.py
│
└── tests/
```

> Set `"python.testing.cwd": "/my_app"`


# Reference
- [Python Unit Testing – Structuring Your Project](https://www.patricksoftwareblog.com/python-unit-testing-structuring-your-project/)