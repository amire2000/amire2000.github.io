---
layout: post
title: Python pylint and VSCode
categories: python
tags: [vscode,  pylint]
image: pylint.svg
public: true
description: Keep your code standard and helping to find errors
---
Linting highlights syntactical and stylistic errors in your Python source code. return code feedback and suggestion how to fix the code  

# Install linter
```bash
#python2
sudo pip install pylint
#python3
sudo pip3 install pylint
```

# Run / usage
- Linting run automatically when file are saved.
- From command `Python: Run Linting`

## Pylint result
In `Output` panel select `python` from combo

![](/images/2018-12-25-21-57-34.png)

## Pylint
Pylint messages fall into this categories
- E: Error (red underline) Code bug
- W: Warning 
- R: Refactor ()
- C: Convention (green underline)

By default only errors and warnings are shown.

&nbsp;  
&nbsp;  
&nbsp;  
# Settings and Control
## .pylintrc
- create
  ```
  pylint --generate-rcfile > .pylintrc
  ```

### Demo: .pylintrc usage 
- project structure
```
├── config
│   └── __init__.py
└── utils
    └── __init__.py
```
- utils __init__ file import from config 
  
![](/images/2018-12-25-21-33-24.png)

- Fix the `E0401` error, we need to tell pylint where to look for modules

  - Create `.pylintrc` file
  - Set `init-hook` under `MASTER`
  
```init
[MASTER]
init-hook='import sys; sys.path.append("<project source root folder></project>")'
```
- Disabled `C0111` Convention by add `disable` entry
    - Example disabled C0111 and W0401 Convention and Warning
  
```
[MASTER]
disable=C0111,W0401
```
&nbsp;  
&nbsp;  
&nbsp;  
# Tips
## disabled pylint warning 
```python
def calc(arg_a, arg_b)
    print arg_a
    
```
**pylint output result**
```
3,16,warning,W0613:Unused argument 'arg_b'
```

- Disable pylint warning for current line

```python
def calc(arg_a, arg_b):#pylint: disable=unused-argument
    print arg_a
```

- Disable the warring globally from `.pylintrc`

```
[MESSAGES CONTROL]
disable=unused-argument
```

## OpenCV (cv2) no-member
from `.pylintrc` at [MASTER]

```
[MASTER]
extension-pkg-whitelist=cv2
```

&nbsp;  
&nbsp;  
&nbsp;  
## Reference
- [Pylint](https://www.pylint.org/)
- [VSCode linting](https://code.visualstudio.com/docs/python/linting)