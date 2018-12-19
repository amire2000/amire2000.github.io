---
layout: post
title: VSCode settings and tips
categories: vscode
tags: [vscode]
---

## Extensions


| Auto close tag      | ![](/images/2018-12-19-06-53-19.png) | Automatically add html/xml close tag                           |
| Bookmarks           | ![](images/2018-12-19-06-56-03.png)  | Mark Link and jump to them                                     |
| Code runner         | ![](/images/2018-12-19-07-07-33.png) | Run Current code                                               |
| Code Spell checker  | ![](/images/2018-12-19-07-08-18.png) | Spell checker                                                  |
| Git lens            | ![](images/2018-12-19-07-08-57.png)  | Supercharge the Git capabilities built into Visual Studio Code |
| Google Test Adapter | ![](/images/2018-12-19-07-12-48.png) | Easily run GoogleTest from VsCode                              |
| JSON Tools          | ![](/images/2018-12-19-07-14-36.png) |                                                                |
| Paste Image         | ![](/images/2018-12-19-07-16-19.png) |                                                                |
| Path Intellisense   | ![](/images/2018-12-19-07-16-44.png) |                                                                |
| Projects Manager    | ![](/images/2018-12-19-07-17-55.png) |                                                                |
| Text Table          | ![](/images/2018-12-19-07-19-55.png) |                                                                |
| Todo+               | ![](/images/2018-12-19-07-18-45.png) |                                                                |
| XML Tools           | ![](/images/2018-12-19-07-15-36.png) |                                                                |




### Language Ext.

| Python | ![](/images/2018-12-19-07-05-05.png) | inting, Debugging (multi-threaded, remote), Intellisense, code formatting, refactoring, unit tests, |
| C++    | ![](/images/2018-12-19-07-05-43.png) | C/C++ IntelliSense, debugging, and code browsing.                                                   |


## Python
> VSCode intellisense not working

- Set Python path and extraPath if needed
```json
"python.pythonPath": "/usr/bin/python",
    "python.autoComplete.extraPaths": [
       "${workspaceFolder}/customModule"
    ]
```